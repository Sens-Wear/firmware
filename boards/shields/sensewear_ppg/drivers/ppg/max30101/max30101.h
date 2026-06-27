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
 * Like the other SenseWear board drivers, the MAX30101 routes every transfer
 * through the board's @ref sensewear_sys_i2c ownership wrapper rather than
 * calling Zephyr's I2C API directly:
 *
 * @code{.text}
 * application / PPG bridge
 *          |
 *          | max30101_config(), max30101_irq_handler(), max30101_read_stream(), ...
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
 *         compatible = "i2c-device";
 *         reg = <0x57>;
 *         label = "MAX30101";
 *         status = "okay";
 *     };
 * };
 * @endcode
 *
 * `reg` supplies the target address used by SYS_I2C_DT_SPEC_GET(). The MAX30101
 * INT line is not a dedicated devicetree GPIO: it is wired to the daughter-board
 * connector and obtained at run time from the @ref sensewear_daughter_if arbiter
 * (line ::daughter_if_GPIO1).
 *
 * @section sensewear_max30101_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call max30101_init() to verify the shared bus, probe the part, and claim
 *    and configure the daughter-board interrupt line.
 * 2. Call max30101_config() to apply the acquisition configuration (NULL selects
 *    the SenseWear defaults).
 * 3. Start acquisition with max30101_enable_wrist_hr_sampling() or
 *    max30101_enable_sampling().
 * 4. On interrupt, call max30101_irq_handler() from thread context to decode the
 *    interrupt sources, drain the FIFO into the internal sample stream, and
 *    publish events. Read the decoded samples with max30101_read_stream().
 *
 * Initialization and configuration are deliberately separate. A successful probe
 * does not imply that the acquisition parameters have been applied.
 *
 * The convenience entry points ppg_sensor_init() and ppg_set_streaming_enabled()
 * wrap this lifecycle for application use.
 *
 * @section sensewear_max30101_interrupts Interrupt and event model
 *
 * The driver does not depend on Bluetooth or any consumer subsystem. It only
 * decodes hardware activity into events and accumulates decoded samples in an
 * internal stream that consumers drain at their own pace.
 *
 * The daughter-board interrupt callback posts ::max30101_Irq from ISR context.
 * A consumer then calls max30101_irq_handler() from thread context, which reads
 * the interrupt status registers, performs the action for each asserted source,
 * and publishes one decoded `max30101_event_*` identifier per source. When FIFO
 * data is drained the decoded samples are pushed to the internal stream and
 * ::max30101_event_FifoDataReady is published. All events are delivered through
 * the shared @ref sensewear_device_driver_events manager.
 *
 * @section sensewear_max30101_example Typical usage
 *
 * @code{.c}
 * if (!max30101_init()) {
 *     // Sensor absent, bus unavailable, or interrupt line could not be claimed.
 *     return;
 * }
 *
 * if (!max30101_config(NULL)) {
 *     return;
 * }
 *
 * max30101_enable_wrist_hr_sampling();
 * @endcode
 */

#ifndef MAX30101_H_
#define MAX30101_H_

#include "max30101_registers.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
 *          interrupt callback; the `max30101_irq_*` values map decoded interrupt
 *          status bits to stable identifiers; ::max30101_FifoDataRead reports
 *          that FIFO samples were drained and are available to consumers.
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
};

/**
 * @brief One decoded multi-channel PPG sample produced by the driver.
 * @details The interrupt handler unpacks each FIFO record into this datatype and
 *          appends it to the internal sample stream. Channels that are not active
 *          in the current mode are reported as zero. Counts are raw 18-bit ADC
 *          values right-justified in the 32-bit fields.
 */
struct max30101_sample_t {
	uint64_t unix_ms; /**< Acquisition timestamp in milliseconds. */
	uint32_t ir;	  /**< IR channel counts (slot 1), or 0 if inactive. */
	uint32_t red;	  /**< Red channel counts (slot 2), or 0 if inactive. */
	uint32_t green;	  /**< Green channel counts (slot 3), or 0 if inactive. */
};

/**
 * @brief Initialize and probe the MAX30101.
 *
 * Verifies that the shared bus is ready, reads the PART_ID register to confirm a
 * device responds, and claims and configures the daughter-board interrupt line.
 * This function does not program the acquisition configuration.
 *
 * @retval true The shared bus was ready and the sensor responded.
 * @retval false The bus was unavailable, the probe failed, or the interrupt line
 *         could not be claimed.
 */
bool max30101_init(void);

/**
 * @brief Report whether max30101_init() successfully detected the sensor.
 *
 * @retval true The sensor is available.
 * @retval false Initialization has not succeeded.
 */
bool max30101_is_ready(void);

/**
 * @brief Populate the SenseWear default acquisition configuration.
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
 * @retval true All configuration registers were written and ownership released.
 * @retval false The driver was not initialized, locking failed, or a transfer failed.
 */
bool max30101_config(struct max30101_config_t* config);

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
 * a new-data or almost-full condition drains the FIFO, appends the decoded
 * samples to the internal stream, and publishes ::max30101_event_FifoDataReady.
 *
 * Call from thread context in response to ::max30101_Irq.
 */
void max30101_irq_handler(void);

/**
 * @brief Copy decoded samples out of the internal sample stream.
 *
 * Removes up to @p max_samples of the oldest decoded samples from the stream the
 * interrupt handler fills. Non-blocking: returns immediately with whatever is
 * currently available.
 *
 * @param samples Destination array. Must not be NULL.
 * @param max_samples Capacity of @p samples in elements.
 * @return Number of samples copied into @p samples.
 */
size_t max30101_read_stream(struct max30101_sample_t* samples, size_t max_samples);

/**
 * @brief Read available samples from the MAX30101 FIFO.
 *
 * @param buffer Destination buffer for the raw FIFO bytes.
 * @param buffer_size Size of @p buffer in bytes.
 * @return Number of bytes read, or a negative value on error.
 */
size_t max30101_read_fifo(void* buffer, size_t buffer_size);

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
 * @retval true Multi-LED acquisition was enabled.
 * @retval false The sensor is unconfigured or busy detecting proximity.
 */
bool max30101_enable_wrist_hr_sampling(void);

/**
 * @brief Enable a sampling operation mode.
 *
 * @param mode Operating mode to enable.
 * @retval true The mode was enabled.
 * @retval false The mode is invalid or could not be enabled.
 */
bool max30101_enable_sampling(enum max30101_operation_mode_type mode);

/**
 * @brief Enable proximity detection (single IR channel).
 *
 * @retval true Proximity detection was enabled.
 * @retval false The sensor is unconfigured.
 */
bool max30101_enable_proximity(void);

/**
 * @brief Put the MAX30101 into shutdown (low-power) mode.
 */
void max30101_shutdown(void);

/**
 * @brief Initialize the MAX30101 for acquisition.
 * @details Convenience wrapper around max30101_init().
 */
void ppg_sensor_init(void);

/**
 * @brief Enable or disable PPG acquisition.
 *
 * @param enabled true to configure the sensor and start sampling; false to place
 *        the sensor in shutdown. While enabled, decoded samples accumulate in the
 *        internal stream and are retrieved with max30101_read_stream().
 */
void ppg_set_streaming_enabled(bool enabled);

/** @} */

#endif /* MAX30101_H_ */
