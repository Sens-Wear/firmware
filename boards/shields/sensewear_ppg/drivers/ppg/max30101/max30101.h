/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30101.h
 * @brief SenseWear MAX30101 PPG (pulse-oximeter / heart-rate) sensor API.
 *
 * @defgroup sensewear_max30101 SenseWear MAX30101 PPG sensor
 * @ingroup io_interfaces
 * @{
 *
 * The MAX30101 is the optical front end on the SenseWear PPG daughter board. It
 * drives Red, IR, and Green LEDs and streams photoplethysmography samples through
 * an internal FIFO. This driver probes the part, programs its acquisition
 * configuration, drains the FIFO, and turns hardware interrupt conditions into
 * stable software events for higher-level policy code.
 *
 * @section sensewear_max30101_timestamp_semantics Timestamp semantics
 *
 * The MAX30101 FIFO records do not carry a Unix epoch timestamp. The driver
 * captures the interrupt arrival time with rtc_get_timestamp_us() and uses that
 * as the batch anchor when draining the FIFO. Each decoded PPG sample is then
 * assigned an interpolated timestamp in microseconds since the Unix epoch based
 * on the configured sampling rate.
 *
 * Like the other SenseWear board drivers, the MAX30101 routes every transfer
 * through the board's @ref sensewear_sys_i2c ownership wrapper rather than
 * calling Zephyr's I2C API directly:
 *
 * @code{.text}
 * application / PPG bridge
 *          |
 *          | max30101_config(), max30101_irq_handler(), max30101_enable_sampling(), ...
 *          v
 * MAX30101 driver
 *          |
 *          | sys_i2c_lock(), transfer(s), sys_i2c_release()
 *          v
 * SenseWear shared system I2C bus
 * @endcode
 *
 * Every high-level hardware operation acquires the shared bus once and calls
 * sys_i2c_release() on exit. Private register helpers assume that ownership has
 * already been acquired.
 *
 * @section sensewear_max30101_devicetree Devicetree representation
 *
 * The sensor is a standard I2C child of the shared system bus, declared by the
 * `sensewear_ppg` shield overlay:
 *
 * @code{.dts}
 * &sys_i2c_peripheral {
 *     max30101: max30101@57 {
 *         compatible = "sensewear,daughter-i2c-device", "i2c-device";
 *         reg = <0x57>;
 *         label = "MAX30101";
 *         status = "okay";
 *         vin-supply = <&tpsm83102>;
 *     };
 * };
 * @endcode
 *
 * The node is also wired into the board aliases `sensewear-daughter` and
 * `sensewear-ppg`, both pointing at this instance.
 *
 * `reg` supplies the target address used by SYS_I2C_DT_SPEC_GET(). `vin-supply`
 * names the shared daughter-connector rail (VDD_DAUGHTER, the `tpsm83102`
 * regulator) and is mandatory: the driver resolves it from devicetree at build
 * time (a missing `vin-supply` is a build error) and owns the regulator handle
 * directly, so none is passed in at run time. The driver validates the rail
 * voltage and enables the rail itself when acquisition starts. The MAX30101 INT
 * line is not a dedicated devicetree GPIO: it is wired to the daughter-board
 * connector and obtained at run time from the @ref sensewear_daughter_if arbiter
 * (line ::daughter_if_GPIO1).
 *
 * @section sensewear_max30101_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call max30101_init() to verify the shared bus, resolve and validate the
 *    devicetree supply rail, probe the part, and claim and configure the
 *    daughter-board interrupt line. The rail is enabled later, when acquisition
 *    starts.
 * 2. Call max30101_config() to apply the acquisition configuration (NULL selects
 *    the SenseWear defaults).
 * 3. Start acquisition with max30101_enable_wrist_hr_sampling() or
 *    max30101_enable_sampling().
 * 4. On interrupt, call max30101_irq_handler() from thread context to decode the
 *    interrupt sources, drain the FIFO into the internal sample buffer, and
 *    publish events. When samples are drained the handler publishes
 *    ::max30101_event_FifoDataReady; its `v_param` carries the sample count and
 *    its `p_param` points at an array of ::max30101_sample_t held in the driver
 *    context. Consumers read the samples directly from that event payload.
 *
 * Initialization and configuration are deliberately separate. A successful probe
 * does not imply that the acquisition parameters have been applied.
 *
 * @section sensewear_max30101_interrupts Interrupt and event model
 *
 * The driver does not depend on Bluetooth or any consumer subsystem. It only
 * decodes hardware activity into events and accumulates decoded samples in an
 * internal buffer that is republished with each ::max30101_event_FifoDataReady.
 *
 * The daughter-board interrupt callback posts ::max30101_Irq from ISR context.
 * A consumer then calls max30101_irq_handler() from thread context, which reads
 * the interrupt status registers, performs the action for each asserted source,
 * and publishes one decoded `max30101_event_*` identifier per source. When FIFO
 * data is drained the decoded samples are written to the internal buffer and
 * ::max30101_event_FifoDataReady is published with the sample count in `v_param`
 * and a pointer to the sample array in `p_param`. All events are delivered
 * through the shared @ref sensewear_device_driver_events manager.
 *
 * @section sensewear_max30101_example Typical usage
 *
 * @code{.c}
 * if (max30101_init() != 0) {
 *     // Sensor absent, bus unavailable, or interrupt line could not be claimed.
 *     return;
 * }
 *
 * if (max30101_config(NULL) != 0) {
 *     return;
 * }
 *
 * // false: interrupt once per FIFO almost-full batch; true: per sample.
 * max30101_enable_wrist_hr_sampling(false);
 * @endcode
 */

#ifndef MAX30101_H_
#define MAX30101_H_

#include "max30101_registers.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

/**
 * @brief Maximum time allowed for acquiring the shared I2C bus.
 * @details Expressed in milliseconds and passed to K_MSEC() by the driver.
 *          Applications may override it before including this header.
 */
#ifndef MAX30101_I2C_TIMEOUT
#define MAX30101_I2C_TIMEOUT (100)
#endif

/**
 * @brief Driver-level MAX30101 event identifiers.
 * @details This is a software event namespace rather than a hardware register
 *          encoding. ::max30101_Irq is the raw INT-pin assertion posted from the
 *          interrupt callback; the `max30101_event_*` values map decoded interrupt
 *          status bits to stable identifiers; ::max30101_event_FifoDataReady
 *          reports that FIFO samples were drained and are available to consumers.
 */
enum max30101_event_type {
	max30101_event_Invalid = -1,			   /**< No valid event. */
	max30101_Irq = 0,						   /**< INT pin assertion detected (posted from ISR). */
	max30101_event_PowerReady,				   /**< Power-ready interrupt (PWR_RDY). */
	max30101_event_Proximity,				   /**< Proximity threshold interrupt (PROX_INT). */
	max30101_event_AmbientLightCancelOverflow, /**< Ambient-light-cancel overflow (ALC_OVF). */
	max30101_event_DieTemperatureReady, /**< Die-temperature conversion ready (DIE_TEMP_RDY). */
	max30101_event_FifoDataReady,		/**< FIFO samples were drained and are available. */
	max30101_event_Count,				/**< Number of valid event identifiers. */
};

/**
 * @brief High-level MAX30101 acquisition configuration.
 * @details This software datatype maps enum and amplitude selections onto the
 *          FIFO_Configuration, ModeConfiguration, SpO2Configuration, multi-LED
 *          control, and per-LED pulse-amplitude registers during max30101_config().
 *          It is not a raw register image.
 */
struct max30101_config_t {
	/** Interrupt-enable selection programmed into InterruptEnable1/2. */
	union max30101_interrupt_enable_t interrupts;
	/** FIFO averaging, roll-over, and almost-full threshold. */
	union max30101_fifo_configuration_t fifo_config;
	/** Operating mode (heart-rate / SpO2 / multi-LED). */
	union max30101_mode_configuration_t mode_config;
	/** Sample rate, pulse width, and ADC range. */
	union max30101_spo2_configuration_t spo2_config;
	/** Multi-LED time-slot assignment. */
	union max30101_multi_led_mode_control_t multi_led_config;
	/** Red LED pulse amplitude (LED1_PA). */
	uint8_t red_led_pulse_amplitude_config;
	/** IR LED pulse amplitude (LED2_PA). */
	uint8_t ir_led_pulse_amplitude_config;
	/** Green LED pulse amplitude (LED3_PA). */
	uint8_t green_led_pulse_amplitude_config;
	/** Proximity-mode LED pulse amplitude (ProxModeLED_PA). */
	uint8_t proximity_led_pulse_amplitude_config;
	/** Proximity interrupt threshold (ProxIntThreshold). */
	uint8_t proximity_int_threshold;
	/** Supply-rail voltage in microvolts, validated against the active LEDs. */
	int32_t ppg_voltage_uv;
};

/**
 * @brief One decoded multi-channel PPG sample produced by the driver.
 * @details The interrupt handler unpacks each FIFO record into this datatype and
 *          appends it to the internal sample buffer that is published with
 *          ::max30101_event_FifoDataReady. Channels that are not active in the
 *          current mode are reported as zero. Counts are raw 18-bit ADC values
 *          right-justified in the 32-bit fields.
 */
struct max30101_ppg_sample_t {
	uint64_t timestamp; /**< Acquisition timestamp in microseconds since the Unix epoch. */
	uint32_t ir;		/**< IR channel counts (slot 1), or 0 if inactive. */
	uint32_t red;		/**< Red channel counts (slot 2), or 0 if inactive. */
	uint32_t green;		/**< Green channel counts (slot 3), or 0 if inactive. */
};

/**
 * @brief Initialize and probe the MAX30101.
 *
 * Verifies that the shared bus is ready, loads the SenseWear default
 * configuration, and validates the supply rail resolved from the devicetree
 * `vin-supply` phandle (if the rail is already enabled its voltage must be in
 * range). It then reads the PART_ID register to confirm a device responds, puts
 * the device into shutdown, and claims and configures the daughter-board
 * interrupt line. The rail is not enabled here; the driver powers it when
 * acquisition starts. This function does not program the acquisition
 * configuration.
 *
 * @retval 0 The shared bus was ready and the sensor responded (or the driver was
 *         already initialized).
 * @retval -EIO The bus was unavailable, the rail voltage was out of range, the
 *         probe failed, or the interrupt line could not be claimed.
 */
int max30101_init(void);

/**
 * @brief Report whether the MAX30101 is initialized and configured.
 *
 * @retval true The sensor has been detected and its acquisition configuration was applied.
 * @retval false Initialization, probing, or configuration has not succeeded.
 */
bool max30101_is_ready(void);

/**
 * @brief Populate the SenseWear default acquisition configuration.
 * @details Selects multi-LED mode with the IR, Red, and Green channels in FIFO
 *          slots 1-3, the SenseWear FIFO averaging / roll-over / almost-full
 *          settings, the default per-LED pulse amplitudes, the FIFO-almost-full
 *          interrupt, and the default supply-rail voltage. This is the same
 *          configuration applied when max30101_config() is called with NULL.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void max30101_get_default_config(struct max30101_config_t* config);

/**
 * @brief Program the MAX30101 acquisition parameters.
 *
 * The complete register sequence is protected by one shared-I2C ownership scope.
 * Passing NULL selects the SenseWear defaults.
 *
 * @param config Configuration to apply, or NULL for the defaults.
 * @retval 0 All configuration registers were written and ownership released.
 * @retval -EINVAL The driver was not initialized or the configuration is out of range.
 * @retval -EIO Locking failed or a register transfer failed.
 */
int max30101_config(struct max30101_config_t* config);

/**
 * @brief Return the printable name for a MAX30101 event identifier.
 *
 * @param event_id Event identifier from enum max30101_event_type.
 * @return Constant string for the event, or "Unknown" when @p event_id is invalid.
 */
const char* max30101_event_name(uint32_t event_id);

/**
 * @brief Handle a MAX30101 interrupt.
 *
 * Reads InterruptStatus1/2 under one ownership scope and performs the action for
 * each asserted source: power-ready, proximity, ambient-light-cancel overflow,
 * and die-temperature-ready publish their decoded `max30101_event_*` identifier;
 * a new-data or almost-full condition drains the FIFO into the internal sample
 * buffer and publishes ::max30101_event_FifoDataReady (sample count in `v_param`,
 * sample-array pointer in `p_param`). The sample timestamps are back-filled from
 * the interrupt anchor returned by max30101_last_irq_timestamp() and the current
 * sampling rate.
 *
 * Call from thread context in response to ::max30101_Irq.
 *
 * @retval 0 The interrupt status was read and all asserted sources handled.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EIO Locking failed or the status read failed.
 */
int max30101_irq_handler(void);

/**
 * @brief Return the most recent MAX30101 IRQ time.
 * @details The timestamp is captured from rtc_get_timestamp_us() in the GPIO
 *          interrupt callback and is expressed in microseconds since the Unix
 *          epoch. FIFO sample timestamps are interpolated from this anchor.
 */
time_t max30101_last_irq_timestamp(void);
/**
 * @brief Return the number of LED channels active in the current mode.
 *
 * @return Active LED count, or -1 if the sensor is not configured.
 */
int max30101_get_led_count(void);

/**
 * @brief Return the effective per-channel sampling rate in hertz.
 *
 * @return Sampling rate, or a negative value if the sensor is not configured.
 */
float max30101_get_sampling_rate(void);

/**
 * @brief Enable wrist heart-rate acquisition (multi-LED mode, three LEDs).
 *
 * @param per_sample_irq Selects the FIFO notification cadence: true arms PPG_RDY
 *        so the device interrupts on every new sample, false arms A_FULL so it
 *        interrupts once per FIFO almost-full batch.
 * @retval 0 Multi-LED acquisition was enabled.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EBUSY Acquisition is already running.
 * @retval -EINVAL The LED supply could not be powered, or a negative errno
 *         propagated from max30101_config().
 */
int max30101_enable_wrist_hr_sampling(bool per_sample_irq);

/**
 * @brief Enable a sampling operation mode.
 *
 * @param mode Operating mode to enable.
 * @param per_sample_irq Selects the FIFO notification cadence: true arms PPG_RDY
 *        so the device interrupts on every new sample, false arms A_FULL so it
 *        interrupts once per FIFO almost-full batch.
 * @retval 0 The mode was enabled.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EBUSY Acquisition or proximity detection is already running.
 * @retval -EINVAL The mode is invalid or the LED supply could not be powered.
 * @retval -EIO A configuration transfer failed.
 */
int max30101_enable_sampling(enum max30101_operation_mode_type mode, bool per_sample_irq);

/**
 * @brief Stop an active sampling operation.
 *
 * Clears the sampling state. If proximity detection was running alongside the
 * acquisition the sensor is reprogrammed back into proximity-detection mode;
 * otherwise the device is placed in shutdown and its LED supply is powered off.
 *
 * @retval 0 Sampling was stopped, or no sampling was active.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EBUSY The sensor is both sampling and detecting proximity, which is an
 *         ambiguous state that this call refuses to resolve.
 * @retval -EIO Reprogramming proximity-detection mode failed.
 */
int max30101_disable_sampling(void);

/**
 * @brief Enable proximity detection (single IR channel).
 *
 * @retval 0 Proximity detection was enabled.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EBUSY Proximity detection or acquisition is already running.
 * @retval -EIO A configuration transfer failed or the LED supply could not be powered.
 */
int max30101_enable_proximity(void);

/**
 * @brief Stop proximity detection.
 *
 * @retval 0 Proximity detection was stopped, or none was active.
 * @retval -EAGAIN The sensor is not configured.
 * @retval -EIO Reprogramming the sensor failed.
 *
 * @note Declared for API symmetry with max30101_enable_proximity(); not yet
 *       implemented in this driver revision.
 */
int max30101_disable_proximity(void);

/**
 * @brief Put the MAX30101 into shutdown (low-power) mode.
 */
void max30101_shutdown(void);

/** @} */

#endif /* MAX30101_H_ */
