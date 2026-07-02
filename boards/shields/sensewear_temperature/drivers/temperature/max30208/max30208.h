/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30208.h
 * @brief SenseWear MAX30208 digital temperature sensor API.
 *
 * @defgroup sensewear_max30208 SenseWear MAX30208 temperature sensor
 * @ingroup io_interfaces
 * @{
 *
 * The MAX30208 is the digital temperature sensor on the SenseWear temperature
 * daughter board. This driver probes the part, programs its acquisition
 * configuration (FIFO, interrupts, alarm thresholds, and GPIO modes), paces
 * single-shot conversions, and publishes decoded samples as driver-level events
 * on the shared device-event queue.
 *
 * @section sensewear_max30208_timestamp_semantics Timestamp semantics
 *
 * The MAX30208 does not provide a Unix-epoch timestamp for FIFO records. The
 * driver timestamps each conversion with rtc_get_timestamp_us() when the sample
 * is drained. If multiple samples are present in one drain, they all share that
 * same conversion timestamp.
 *
 * Like the other SenseWear board drivers, the MAX30208 routes every transfer
 * through the board's @ref sensewear_sys_i2c ownership wrapper rather than
 * calling Zephyr's I2C API directly:
 *
 * @code{.text}
 * application / temperature bridge
 *          |
 *          | max30208_config(), max30208_start(), max30208_get_samples(), ...
 *          v
 * MAX30208 driver
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
 * Unlike the PPG daughter board, the temperature daughter board does not expose
 * a regulator or a daughter-connector interrupt line to this driver: the rail is
 * owned by the daughter-board manager and there is no hardware INT pin. The
 * driver therefore depends only on the shared system I2C bus and substitutes a
 * periodic software timer for the missing interrupt line.
 *
 * @section sensewear_max30208_lifecycle Driver lifecycle
 *
 * 1. Call max30208_init() to verify the shared bus, probe the part, and load the
 *    SenseWear default configuration into the driver context. A successful probe
 *    does not imply that the acquisition parameters have been written to the part.
 * 2. Call max30208_config() to apply the acquisition configuration (NULL selects
 *    the SenseWear defaults). The defaults can be inspected or tailored through
 *    max30208_get_default_config(). This step is optional: max30208_start()
 *    applies the defaults automatically if the part has not been configured yet.
 * 3. Call max30208_start() to flush the internal buffer and begin pacing
 *    conversions; this publishes ::max30208_event_SamplingStarted.
 * 4. The driver's sampling timer fires at the configured rate and publishes
 *    ::max30208_TimerIrq from timer context.
 * 5. On ::max30208_TimerIrq the consumer calls max30208_get_samples(), which
 *    performs the conversion, appends it to the internal buffer, and publishes
 *    ::max30208_event_SampleReady (sample count in `v_param`, a pointer to the
 *    ::temperature_sample_t array in `p_param`). Each decoded sample gets the
 *    same conversion timestamp.
 * 6. Call max30208_stop() / max30208_deinit() to halt.
 *
 * Initialization and configuration are deliberately separate: programming the
 * acquisition registers is driven by max30208_config() rather than the probe in
 * max30208_init().
 *
 * @section sensewear_max30208_config Acquisition configuration
 *
 * struct max30208_config_t gathers the register bit-field overlays the driver
 * programs during max30208_config(): the INTERRUPT_ENABLE selection, the
 * FIFO_CONFIG1 almost-full threshold, the FIFO_CONFIG2 roll-over / almost-full /
 * status-clear options, the GPIO_SETUP pin modes, and the high/low temperature
 * alarm thresholds. It is a software datatype, not a raw register image; each
 * member is written to its corresponding register under one shared-bus ownership
 * scope. max30208_get_default_config() fills the structure with the SenseWear
 * defaults (overridable through max30208_config.h), which is the same
 * configuration applied when max30208_config() is called with NULL.
 *
 * @section sensewear_max30208_events Event model
 *
 * The driver does not depend on any consumer subsystem; it only paces hardware
 * activity into events delivered through the shared
 * @ref sensewear_device_driver_events manager:
 *
 * - ::max30208_TimerIrq is posted from the sampling timer each period.
 * - ::max30208_event_SampleReady is posted by max30208_get_samples() when a
 *   conversion completes, carrying the decoded samples in its payload.
 * - ::max30208_event_SamplingStarted / ::max30208_event_SamplingStopped bracket
 *   an acquisition session.
 */

#ifndef MAX30208_H_
#define MAX30208_H_

#include "max30208_registers.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

/**
 * @brief Maximum time allowed for acquiring the shared I2C bus.
 * @details Expressed in milliseconds and passed to K_MSEC() by the driver.
 *          Applications may override it before including this header.
 */
#ifndef MAX30208_I2C_TIMEOUT
#define MAX30208_I2C_TIMEOUT (100)
#endif

/**
 * @brief Driver-level MAX30208 event identifiers.
 * @details A software event namespace rather than a hardware register encoding.
 *          ::max30208_TimerIrq is the periodic sampling tick posted from timer
 *          context; the `max30208_event_*` values report decoded activity. All
 *          values are published through the shared device-event manager.
 */
enum max30208_event_type {
	max30208_event_Invalid = -1,	/**< No valid event. */
	max30208_TimerIrq = 0,			/**< Sampling timer fired (posted from timer context). */
	max30208_event_SampleReady,		/**< A conversion completed; payload carries samples. */
	max30208_event_SamplingStarted, /**< Periodic acquisition has started. */
	max30208_event_SamplingStopped, /**< Periodic acquisition has stopped. */
	max30208_event_Count,			/**< Number of valid event identifiers. */
};

/**
 * @brief One decoded temperature sample produced by the driver.
 */
struct temperature_sample_t {
	time_t timestamp;			/**< Timestamp of the sample in microseconds since the Unix epoch. */
	int32_t temperature_mdeg_c; /**< Temperature in milli-degrees Celsius. */
};

/**
 * @brief High-level MAX30208 acquisition configuration.
 * @details This software datatype gathers the register bit-field overlays the
 *          driver programs during max30208_config() into a single structure. It
 *          is not a raw register image: max30208_config() writes each member to
 *          its corresponding register under one shared-bus ownership scope.
 */
struct max30208_config_t {
	/** Interrupt-enable selection programmed into INTERRUPT_ENABLE (0x01). */
	union max30208_interrupt_enable_register_t interrupts;
	/** FIFO almost-full threshold programmed into FIFO_CONFIG1 (0x09). */
	union max30208_fifo_config1_register_t fifo_config1;
	/** FIFO roll-over / almost-full / status-clear options in FIFO_CONFIG2 (0x0A). */
	union max30208_fifo_config2_register_t fifo_config2;
	/** GPIO pin mode configuration programmed into GPIO_SETUP (0x20). */
	union max30208_gpio_setup_register_t gpio_setup;
	/** High temperature alarm threshold in raw sensor counts (ALARM_HIGH). */
	int16_t alarm_high_counts;
	/** Low temperature alarm threshold in raw sensor counts (ALARM_LOW). */
	int16_t alarm_low_counts;
};

/**
 * @brief Initialize and probe the MAX30208.
 *
 * Verifies that the shared bus is ready, reads PART_ID to confirm a device
 * responds, and loads the SenseWear default configuration into the driver
 * context. It does not write the acquisition parameters to the part; call
 * max30208_config() (or max30208_start(), which configures on demand) for that.
 *
 * @retval 0 The shared bus was ready and the sensor responded (or the driver was
 *         already initialized).
 * @retval -ENODEV The bus was unavailable or the sensor did not respond.
 * @retval -EIO A configuration transfer failed.
 */
int max30208_init(void);

/**
 * @brief Populate the SenseWear default acquisition configuration.
 * @details Selects the SenseWear FIFO almost-full threshold, roll-over and
 *          status-clear behaviour, interrupt-enable defaults, GPIO pin modes, and
 *          the default temperature alarm thresholds. This is the same
 *          configuration applied when max30208_config() is called with NULL.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void max30208_get_default_config(struct max30208_config_t* config);

/**
 * @brief Program the MAX30208 acquisition parameters.
 *
 * The complete register sequence is protected by one shared-I2C ownership scope.
 * Passing NULL selects the SenseWear defaults.
 *
 * @param config Configuration to apply, or NULL for the defaults.
 * @retval 0 All configuration registers were written and ownership released.
 * @retval -EINVAL The driver was not initialized.
 * @retval -EIO Locking failed or a register transfer failed.
 */
int max30208_config(struct max30208_config_t* config);

/**
 * @brief Report whether the MAX30208 is initialized and configured.
 *
 * @retval true The sensor has been detected and its acquisition configuration was applied.
 * @retval false Initialization, probing, or configuration has not succeeded.
 */
bool max30208_is_ready(void);

/**
 * @brief Start periodic temperature acquisition.
 *
 * Initializes the sensor on demand if necessary, applies the default acquisition
 * configuration if the part has not been configured yet, flushes the internal
 * sample buffer, starts the sampling timer at the configured rate, and publishes
 * ::max30208_event_SamplingStarted.
 *
 * @retval 0 Acquisition was started.
 * @return A negative errno propagated from max30208_init() or max30208_config().
 */
int max30208_start(void);

/** @brief Stop periodic acquisition and publish ::max30208_event_SamplingStopped. */
void max30208_stop(void);

/** @brief Stop acquisition and mark the driver uninitialized. */
void max30208_deinit(void);

/**
 * @brief Set the per-sensor sampling rate.
 *
 * @param new_sampling_rate Sampling rate in hertz. Ignored when zero.
 */
void max30208_set_sampling_rate(uint16_t new_sampling_rate);

/**
 * @brief Perform a conversion and drain the latest sample(s).
 *
 * Triggers a single-shot conversion, appends the decoded result to the driver's
 * internal sample buffer, publishes ::max30208_event_SampleReady, and copies the
 * buffered samples into @p samples. Intended to be called by the event consumer
 * in response to ::max30208_TimerIrq. All samples returned from one drain share
 * the same conversion timestamp captured with rtc_get_timestamp_us().
 *
 * @param samples Destination array, or NULL to only refresh the internal buffer
 *        and publish the event.
 * @param max_samples Capacity of @p samples in elements.
 * @retval >=0 Number of samples written to @p samples.
 * @retval -EAGAIN The driver is not initialized or not sampling.
 * @retval -EIO A bus transfer failed.
 * @retval -ETIMEDOUT The conversion did not complete in time.
 */
int max30208_get_samples(struct temperature_sample_t* samples, size_t max_samples);

/**
 * @brief Return the printable name for a MAX30208 event identifier.
 *
 * @param event_id Event identifier from enum max30208_event_type.
 * @return Constant string for the event, or "Unknown" when @p event_id is invalid.
 */
const char* max30208_event_name(uint32_t event_id);

/** @} */

#endif /* MAX30208_H_ */
