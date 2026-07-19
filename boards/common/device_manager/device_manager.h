/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_manager.h
 * @brief SensWear device manager: per-message-type zbus streams.
 *
 * @defgroup senswear_device_manager SensWear device manager
 * @ingroup io_interfaces
 * @{
 *
 * The device manager is the single publisher that turns decoded device activity
 * into application-facing data. It owns one @b zbus channel per message type in
 * @ref senswear_device_driver_messages — each channel is an independent,
 * subscribable @e stream:
 *
 * @code{.text}
 *   device drivers ──▶ device_driver_events queue ──▶ device manager (sole publisher)
 *                                                          │  publish by value
 *              ┌───────────────┬───────────────┬───────────┴───────────┐
 *          IMU streams      PPG stream      touch streams          battery ...
 *              │               │               │                       │
 *        BLE manager      storage/log     app / LED-haptic         app logic
 *        (subscriber)     (subscriber)    (listener/subscriber)    (subscriber)
 * @endcode
 *
 * A stream carries exactly one message type, so a subscriber observes only the
 * data it cares about; filtering is the framework's job, not the consumer's.
 *
 * @section senswear_device_manager_subscribe Subscribing to a stream
 *
 * Consumers register themselves as zbus observers of the streams they want once
 * the producing device is ready (typically after it has been configured). For a
 * queued, own-thread consumer (recommended for the BLE manager and storage):
 *
 * @code{.c}
 * ZBUS_MSG_SUBSCRIBER_DEFINE(my_sub);
 * ret = device_manager_stream_register(device_manager_stream_TouchGesture,
 *                                      &my_sub,
 *                                      K_MSEC(100));
 *
 * const struct zbus_channel *chan;
 * while (!zbus_sub_wait_msg(&my_sub, &chan, &msg, K_FOREVER)) {
 *     if (device_manager_stream_from_channel(chan) == device_manager_stream_TouchGesture) {
 *         const struct touch_gesture_msg_t *g = &msg;
 *         // ...
 *     }
 * }
 * @endcode
 *
 * For a cheap synchronous reaction (LED / haptic feedback) use a listener
 * instead; note listeners run in the publisher's (device-manager) context, so
 * they must not block.
 *
 * @section senswear_device_manager_lifecycle Lifecycle
 *
 * device_manager_init() runs each available device's driver init but does not
 * start any thread. device_manager_start() then spawns the manager's consumer
 * thread, which drains the shared @ref senswear_device_driver_events queue,
 * drives each device's interrupt handler, and publishes decoded messages onto
 * the streams. The whole module is gated by @c CONFIG_SENSWEAR_DEVICE_MANAGER,
 * so it is absent from isolated device/shield test images.
 */

#ifndef SENSWEAR_DRIVERS_COMMON_DEVICE_MANAGER_H_
#define SENSWEAR_DRIVERS_COMMON_DEVICE_MANAGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include <zephyr/zbus/zbus.h>

#include "device_driver_messages.h"

/**
 * @brief Device manager devices with stream-producing drivers.
 * @details Used by the generic per-device configuration entry points. The
 *          configuration pointer type depends on the selected device:
 *          - ::device_manager_device_Bhi360: struct bhi360_config_t
 *          - ::device_manager_device_Bq25180: struct bq25180_config_t
 *          - ::device_manager_device_Bq27427: struct bq27427_config_t
 *          - ::device_manager_device_Max30101: struct max30101_config_t
 *          - ::device_manager_device_Max30208: struct max30208_config_t
 *          - ::device_manager_device_Mtch6102: struct mtch6102_config_t
 *
 *          ::device_manager_device_Tpsm83102 is a devicetree-instantiated
 *          regulator with no configuration surface; it is not configured through
 *          device_manager_configure_device() and only produces a status stream.
 *
 *          Passing NULL to device_manager_configure_device() selects the
 *          device's default configuration.
 */
enum device_manager_device_type {
	device_manager_device_Invalid = 0,
	device_manager_device_Bhi360,
	device_manager_device_Bq25180,
	device_manager_device_Bq27427,
	device_manager_device_Max30101,
	device_manager_device_Max30208,
	device_manager_device_Mtch6102,
	device_manager_device_Tpsm83102,
	device_manager_device_Count,
};

/**
 * @brief Device-manager message streams.
 * @details Runtime observer registration is routed through these values so the
 *          manager can reject subscriptions to streams whose producing device is
 *          not ready. For most devices readiness means the driver has been
 *          initialized and configured; a devicetree-instantiated device such as
 *          the regulator is ready once its Zephyr device is ready.
 */
enum device_manager_stream_type {
	device_manager_stream_Invalid = 0,
	device_manager_stream_ImuQuaternion,
	device_manager_stream_ImuAccel,
	device_manager_stream_ImuGyro,
	device_manager_stream_ImuPedometer,
	device_manager_stream_ImuGesture,
	device_manager_stream_ImuActivity,
	device_manager_stream_Ppg,
	device_manager_stream_Temperature,
	device_manager_stream_Touch,
	device_manager_stream_TouchGesture,
	device_manager_stream_Battery,
	device_manager_stream_Charger,
	device_manager_stream_Regulator,
	device_manager_stream_Count,
};

/**
 * @brief IMU (BHI360) physical-stream configuration.
 * @details Controls the high-rate physical sensors (quaternion, linear
 *          acceleration, gyroscope). Their per-sensor output rates are fixed by
 *          the driver's sensor table; what is exposed here is whether they stream
 *          at all and how often the driver drains the FIFO — which sets the
 *          effective reporting cadence and batch size.
 */
struct device_manager_imu_config_t {
	bool phy_streams_enabled; /**< Sample the physical sensors (quaternion/accel/gyro). */
	uint32_t drain_period_ms; /**< FIFO drain-timer period in milliseconds. */
};

/**
 * @brief PPG (MAX30101) acquisition configuration.
 */
struct device_manager_ppg_config_t {
	bool sampling_enabled; /**< Acquire multi-LED wrist-HR samples. */
	bool per_sample_irq;   /**< Interrupt per sample instead of per FIFO batch. */
};

/**
 * @brief RTC-minute-driven periodic-update configuration.
 * @details Some devices are refreshed on a slow wall-clock cadence rather than by
 *          their own interrupt. The device manager counts ::rtc_event_MinuteAlarm
 *          events and triggers the device update every @c update_period_min
 *          minutes.
 */
struct device_manager_periodic_config_t {
	uint16_t update_period_min; /**< Update every N minutes; 0 disables periodic update. */
};

/**
 * @brief Aggregate device-manager configuration.
 * @details A high-level view of the most important knobs; the manager maps each
 *          field onto the underlying driver calls. Members for devices that are
 *          not built are simply ignored.
 */
struct device_manager_config_t {
	struct device_manager_imu_config_t imu;				 /**< IMU physical streaming. */
	struct device_manager_ppg_config_t ppg;				 /**< PPG acquisition. */
	struct device_manager_periodic_config_t gauge;		 /**< Fuel-gauge refresh cadence. */
	struct device_manager_periodic_config_t temperature; /**< Temperature sampling cadence. */
};

/**
 * @brief Initialize the available managed devices.
 * @details Runs each supported device's driver init. A device that fails to come
 *          up is logged and skipped — the manager still serves the rest — so a
 *          missing or faulty device does not block the others. Most devices must
 *          still be configured (device_manager_configure_device()) before their
 *          streams accept observers; a devicetree-instantiated device such as the
 *          regulator needs no configuration step.
 *
 * This does not start the consumer thread; call device_manager_start() for that.
 *
 * @return 0 always (individual device failures are ignored).
 */
int device_manager_init(void);

/**
 * @brief Start the device-manager consumer thread.
 * @details Spawns the thread that drains the shared device-event queue, drives
 *          each device's interrupt handler, and publishes decoded messages onto
 *          the streams. Safe to call once; subsequent calls are no-ops.
 *
 * @retval 0 The manager was started (or was already running).
 * @retval -ENODEV The shared event queue is not initialized, or a stream already
 *         has observers but its producing device is not ready.
 * @return A negative errno if the consumer thread could not be started.
 */
int device_manager_start(void);

/**
 * @brief Fill a configuration with the SensWear defaults.
 * @param config Destination configuration. Must not be NULL.
 */
void device_manager_get_default_config(struct device_manager_config_t* config);

/**
 * @brief Apply a device-manager configuration.
 * @details Enables/stops IMU physical streams at the requested drain period, and
 *          arms the RTC minute alarm when any minute-driven periodic update is
 *          requested. Fields for devices that are not built are ignored.
 *
 * @param config Configuration to apply, or NULL for the defaults.
 * @retval 0 The configuration was applied.
 * @retval -ENODEV A feature was requested (IMU streaming, or a gauge/temperature
 *         cadence) whose device is not ready; no state is changed.
 * @return A negative errno propagated from a driver call.
 */
int device_manager_config(const struct device_manager_config_t* config);

/**
 * @brief Fill a native driver configuration with defaults for one device.
 * @details @p config must point to the native configuration structure documented
 *          by ::device_manager_device_type for @p device.
 *
 * @retval 0 The default configuration was written.
 * @retval -EINVAL @p device is invalid or @p config is NULL.
 * @retval -ENOTSUP The selected driver is not built.
 */
int device_manager_get_default_device_config(enum device_manager_device_type device, void* config);

/**
 * @brief Configure one managed device through its native driver.
 * @details @p config must be NULL or point to the native configuration structure
 *          documented by ::device_manager_device_type for @p device. On success
 *          the device's streams become eligible for observer registration.
 *
 * @retval 0 The device was configured.
 * @retval -EINVAL @p device is invalid or the configuration is rejected.
 * @retval -EIO The selected driver failed to configure and reports only boolean status.
 * @retval -ENOTSUP The selected driver is not built.
 * @return A negative errno propagated from the native driver where available.
 */
int device_manager_config_device(enum device_manager_device_type device, const void* config);

/**
 * @brief Report whether a stream exists and its producing device is ready.
 * @details Readiness means the driver has been initialized and configured
 *          (reported through its `*_is_ready()`) for most devices; a
 *          devicetree-instantiated device such as the regulator is ready once
 *          its Zephyr device is ready.
 */
bool device_manager_stream_valid(enum device_manager_stream_type stream);

/**
 * @brief Report whether a stream can currently accept observer registration.
 * @details Currently equivalent to device_manager_stream_valid(): a stream
 *          accepts observers exactly when it exists and its producing device is
 *          ready. Exposed separately so registration readiness can diverge from
 *          plain validity in the future without changing callers.
 */
bool device_manager_stream_ready(enum device_manager_stream_type stream);

/**
 * @brief Map a private zbus channel pointer back to its device-manager stream.
 * @details Use this on the channel pointer returned by zbus_sub_wait() or
 *          zbus_sub_wait_msg(). The channel objects are intentionally not
 *          declared in this public header, so build-time observer registration
 *          with ZBUS_CHAN_ADD_OBS() is not available to consumers.
 *
 * @param chan Channel pointer returned by zbus.
 * @return Matching stream, or ::device_manager_stream_Invalid if @p chan does
 *         not belong to the device manager.
 */
enum device_manager_stream_type device_manager_stream_from_channel(const struct zbus_channel* chan);

/**
 * @brief Register an observer for a ready device-manager stream.
 *
 * @retval 0 Observer registered.
 * @retval -EINVAL @p stream is invalid or @p obs is NULL.
 * @retval -ENODEV The stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno from zbus_chan_add_obs().
 */
int device_manager_stream_register(enum device_manager_stream_type stream,
								   const struct zbus_observer* obs,
								   k_timeout_t timeout);

/**
 * @brief Remove an observer from a device-manager stream.
 *
 * @retval 0 Observer removed.
 * @retval -EINVAL @p stream is invalid or @p obs is NULL.
 * @return A negative errno from zbus_chan_rm_obs().
 */
int device_manager_stream_unregister(enum device_manager_stream_type stream,
									 const struct zbus_observer* obs,
									 k_timeout_t timeout);

/**
 * @brief Enable or disable IMU physical streams at run time.
 * @details Controls the high-rate BHI360 physical streams without changing the
 *          rest of the device-manager configuration.
 *
 * @param enabled True to enable quaternion/accel/gyro streams, false to stop them.
 * @param drain_period_ms FIFO drain period to use when enabling streams.
 * @retval 0 The physical-stream state was updated.
 * @retval -ENODEV The IMU is not ready.
 * @retval -ENOTSUP The IMU driver is not built.
 * @return A negative errno propagated from the IMU driver.
 */
int device_manager_set_imu_phy_streams_enabled(bool enabled, uint32_t drain_period_ms);

/**
 * @brief Enable or disable PPG sampling at run time.
 * @details Controls MAX30101 acquisition through the device manager. Enabling
 *          uses the driver's wrist-HR sampling mode and default configuration
 *          when the sensor has not already been configured.
 *
 * Uses the interrupt cadence selected by
 * device_manager_set_ppg_per_sample_irq() when enabling.
 * Repeating the current
 * state is a successful no-op.
 *
 * @param enabled True to enable PPG
 * sampling, false to stop sampling.
 * @retval 0 The sampling state was updated.
 * @retval -ENODEV
 * The PPG sensor is not initialized or present.
 * @retval -ENOTSUP The PPG shield driver is not built.
 * @return A negative errno propagated from the PPG driver.
 */
int device_manager_set_ppg_sampling_enabled(bool enabled);

/**
 * @brief Select the PPG FIFO interrupt cadence.
 * @details The cadence can only be changed
 * while PPG sampling is stopped.
 *
 * @param per_sample_irq True for one interrupt per sample,
 * false for FIFO batch interrupts.
 * @retval 0 The cadence was updated or already had the
 * requested value.
 * @retval -EBUSY PPG sampling is active.
 * @retval -ENOTSUP The PPG shield
 * driver is not built.
 */
int device_manager_set_ppg_per_sample_irq(bool per_sample_irq);

/** @return True when PPG sampling is active according to the device manager. */
bool device_manager_is_ppg_sampling_enabled(void);

/** @return True when per-sample PPG interrupts are selected. */
bool device_manager_is_ppg_per_sample_irq(void);

/**
 * @brief Enable or disable touch-controller acquisition at run time.
 * @details Controls MTCH6102 acquisition through the device manager. Enabling
 *          starts interrupt-driven touch acquisition; disabling stops it.
 *
 * @param enabled True to enable touch acquisition, false to stop acquisition.
 * @retval 0 The acquisition state was updated.
 * @retval -ENOTSUP The touch shield driver is not built.
 * @return A negative errno propagated from the touch driver.
 */
int device_manager_set_touch_sampling_enabled(bool enabled);

/**
 * @brief Update the IMU FIFO-drain timer period at run time.
 * @param period_ms New drain-timer period in milliseconds.
 * @retval 0 The period was updated.
 * @retval -ENODEV The IMU is not ready.
 * @retval -ENOTSUP The IMU driver is not built.
 * @return A negative errno propagated from the IMU driver.
 */
int device_manager_set_imu_drain_period(uint32_t period_ms);

/**
 * @brief Update how often the fuel gauge is refreshed, in minutes.
 * @param minutes Update every N minutes; 0 disables periodic gauge refresh.
 * @retval 0 The cadence was updated.
 * @retval -ENODEV A non-zero cadence was requested but the gauge is not ready.
 */
int device_manager_set_gauge_update_period(uint16_t minutes);

/**
 * @brief Update how often body temperature is sampled, in minutes.
 * @param minutes Update every N minutes; 0 disables periodic temperature sampling.
 * @retval 0 The cadence was updated.
 * @retval -ENODEV A non-zero cadence was requested but the temperature sensor is not ready.
 */
int device_manager_set_body_temperature_update_period(uint16_t minutes);

/**
 * @brief Set the RTC wall-clock time through the SensWear RTC driver.
 * @details Programs the RTC facade with Unix time in seconds since
 *          1970-01-01 00:00:00 UTC. The RTC driver updates
 *          SYS_CLOCK_REALTIME, persists the value, and realigns active RTC
 *          alarms.
 *
 * @param unix_seconds Seconds since the Unix epoch.
 * @retval 0 The RTC time was updated.
 * @retval -EINVAL @p unix_seconds is below the RTC driver's sanity threshold.
 * @retval -ENODEV The RTC driver could not be initialized.
 * @retval -ENOTSUP The RTC driver is not built.
 * @retval -EIO The RTC driver rejected the clock update.
 */
int device_manager_set_rtc_time(time_t unix_seconds);

#if defined(CONFIG_SHIELD_SENSWEAR_HAPTIC)
/**
 * @name Vibration motor control
 * @brief Drive the DRV2605 haptic actuator. The manager owns the real-time
 *        playback buffer, so callers do not have to keep pattern data alive for
 *        the duration of asynchronous playback.
 * @{
 */

/** Maximum number of RTP frames the manager buffers for one haptic pattern. */
#define DEVICE_MANAGER_HAPTIC_RTP_MAX 32
/** Maximum number of ROM waveform-sequencer entries in one pattern. */
#define DEVICE_MANAGER_HAPTIC_SEQ_MAX 8

/**
 * @brief Start a simple vibration.
 * @details Real-time playback of one frame: @p amplitude held for @p duration_ms.
 *
 * @param amplitude   RTP amplitude (0-255).
 * @param duration_ms How long to hold it, in milliseconds.
 * @retval 0 Playback started.
 * @retval -ENODEV The haptic device is not ready.
 * @retval -EBUSY A pattern is already playing.
 * @return A negative errno propagated from the haptic driver.
 */
int device_manager_haptic_vibrate(uint8_t amplitude, uint32_t duration_ms);

/**
 * @brief Start real-time playback of an amplitude/hold pattern.
 * @details The pattern is copied into the manager's buffer, so the caller's
 *          arrays need not outlive the call.
 *
 * @param amplitude Per-frame RTP amplitudes.
 * @param hold_us   Per-frame hold times, in microseconds.
 * @param frames    Number of frames; at most ::DEVICE_MANAGER_HAPTIC_RTP_MAX.
 * @retval 0 Playback started.
 * @retval -EINVAL A pointer was NULL or @p frames was zero.
 * @retval -E2BIG @p frames exceeds ::DEVICE_MANAGER_HAPTIC_RTP_MAX.
 * @retval -ENODEV The haptic device is not ready.
 * @retval -EBUSY A pattern is already playing.
 * @return A negative errno propagated from the haptic driver.
 */
int device_manager_haptic_start_rtp(const uint8_t* amplitude,
									const uint32_t* hold_us,
									size_t frames);

/**
 * @brief Play a ROM waveform sequence from the LRA library.
 * @details Plays pre-programmed effects from the DRV2605 LRA library (the only
 *          library the SensWear haptic driver supports). @p sequence holds
 *          waveform identifiers (1-123); playback stops at the first zero entry,
 *          so a short sequence needs no explicit terminator.
 *
 * @param sequence Waveform identifiers to play in order.
 * @param count    Number of entries; at most ::DEVICE_MANAGER_HAPTIC_SEQ_MAX.
 * @retval 0 Playback started.
 * @retval -EINVAL @p sequence was NULL or @p count was zero.
 * @retval -E2BIG @p count exceeds ::DEVICE_MANAGER_HAPTIC_SEQ_MAX.
 * @retval -ENODEV The haptic device is not ready.
 * @retval -EBUSY A pattern is already playing.
 * @return A negative errno propagated from the haptic driver.
 */
int device_manager_haptic_play_rom(const uint8_t* sequence, size_t count);

/**
 * @brief Stop any active haptic output.
 * @retval 0 Output was stopped.
 * @retval -ENODEV The haptic device is not ready.
 * @return A negative errno propagated from the haptic driver.
 */
int device_manager_haptic_stop(void);

/**
 * @brief Report whether haptic playback (RTP stream or ROM sequence) is active.
 * @details Reads the DRV2605 GO bit to detect ROM playback, so a transient I2C
 *          error is reported as inactive (false).
 */
bool device_manager_haptic_is_active(void);

/** @} */
#endif /* CONFIG_SHIELD_SENSWEAR_HAPTIC */

/** @} */

#endif /* SENSWEAR_DRIVERS_COMMON_DEVICE_MANAGER_H_ */
