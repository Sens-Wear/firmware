/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bhi360.h
 * @brief SenseWear BHI360 smart IMU driver API.
 *
 * @defgroup sensewear_bhi360 SenseWear BHI360 smart IMU
 * @ingroup io_interfaces
 * @{
 *
 * The BHI360 is a 6-axis inertial sensor with integrated firmware that fuses
 * accelerometer, gyroscope, and optional magnetometer data to produce virtual
 * sensors: quaternion orientation, linear acceleration, gravity compensation,
 * step detection, gesture recognition, and activity classification. The device
 * communicates via SPI and generates interrupts on its INT pins when FIFO data
 * becomes available.
 *
 * The driver operates in interrupt-driven mode: GPIO callbacks post raw IRQ
 * events to the central device-driver event queue (see @ref
 * device_driver_events.h). The application consumes those events from thread
 * context, calls bhi360_irq_handler(), and then consumes decoded sensor/meta
 * events posted by the parser callbacks.
 *
 * @section sensewear_bhi360_timestamp_semantics Timestamp semantics
 *
 * Sensor payload timestamps come from the BHY2 FIFO parser callback data. The
 * @c time_stamp value in @c struct bhy2_fifo_parse_data_info is the BHI360
 * firmware's raw FIFO tick counter, measured in 15.625 us ticks since sensor
 * boot. The driver converts that counter to elapsed microseconds before storing
 * it in the payload structs below. These values are not Unix-epoch timestamps.
 * The driver also records a host-minus-sensor offset at IRQ entry for internal
 * time alignment; the payload timestamps themselves remain sensor-boot-relative.
 *
 * IRQ timestamps captured by the driver itself use rtc_get_timestamp_ms() and
 * therefore are milliseconds since the Unix epoch.
 *
 * @section sensewear_bhi360_event_model Event-driven architecture
 *
 * The BHI360 posts the following event types (see @ref bhi360_event_type):
 *
 * - **Irq**: Raw interrupt signal. The application/caller thread should call
 *   bhi360_irq_handler() to read and parse FIFO data, which may generate zero or
 *   more downstream decoded events.
 *
 * - **QuaternionBatch**: A batch of orientation samples (x, y, z, w, accuracy).
 *   @details Posted once per FIFO drain, not per sample. v_param carries the number
 *   of samples collected during the drain; p_param points at the first of that many
 *   contiguous @c struct bhi360_quat_data_t (oldest first). Source: Game rotation
 *   vector wake-up sensor at 100 Hz. The pointed-to array is valid only until
 *   the next drain; bhi360_copy_quaternion() returns a thread-safe snapshot.
 *
 * - **LinearAccelerationBatch**: A batch of X, Y, Z acceleration samples.
 *   @details Posted once per FIFO drain, not per sample. v_param carries the number
 *   of samples; p_param points at that many contiguous @c struct bhi360_lacc_data_t
 *   (oldest first). Source: Accelerometer wake-up sensor at 100 Hz. Valid until
 *   the next drain; bhi360_copy_linear_acceleration() returns a thread-safe snapshot.
 *
 * - **GyroBatch**: A batch of angular velocity X, Y, Z samples.
 *   @details Posted once per FIFO drain, not per sample. v_param carries the number
 *   of samples; p_param points at that many contiguous @c struct bhi360_gyro_data_t
 *   (oldest first). Source: Gyroscope wake-up (GYRO_WU) sensor at 100 Hz. Valid
 *   until the next drain; bhi360_copy_gyro() returns a thread-safe snapshot.
 *
 * - **Pedometer**: Step count and detection state.
 *   @details Sensor ID passed in v_param; data via p_param
 *   (@c struct bhi360_pedometer_data_t*). Source: Step counter low-power (STC_LP)
 *   at 1 Hz. @c timestamp is derived from the BHY2 callback timestamp and is
 *   stored as elapsed microseconds since BHI360 firmware boot. The current
 *   default firmware does not advertise the wake-up low-power step
 *   counter/detector IDs.
 *
 * - **Gesture**: Gesture type and sensor ID.
 *   @details Sensor ID and gesture value packed in v_param as (sensor_id << 8) | value;
 *   data via p_param (@c struct bhi360_gesture_data_t*). Sources: No Motion LP
 *   wake-up, Wrist Gesture Detect LP wake-up, and Wrist Wear LP wake-up.
 *   @c timestamp is derived from the BHY2 callback timestamp and is stored as
 *   elapsed microseconds since BHI360 firmware boot. The current default
 *   firmware does not advertise Any Motion LP wake-up or Significant Motion LP
 *   wake-up.
 *
 * - **Activity**: Activity classification with start/end flags.
 *   @details Sensor ID and activity bits packed in v_param as (sensor_id << 16) | activity;
 *   data via p_param (@c struct bhi360_activity_data_t*). Activity bits indicate
 *   which activity started/ended (still, walking, running, bicycle, vehicle,
 *   tilting). @c timestamp is derived from the BHY2 callback timestamp and is
 *   stored as elapsed microseconds since BHI360 firmware boot. Sources:
 *   Activity Recognition Wear (AR_WEAR_WU) at 1 Hz.
 *
 * - **MetaEvent**: Actionable firmware-generated metadata (sensor mode, reset,
 *   calibration, errors, overflow, etc.).
 *   @details Packed in v_param as (type << 16) | (byte1 << 8) | byte2;
 *   no p_param. Includes events: flush complete, sample rate changed, power mode
 *   changed, algorithm events, sensor status, BSX steps, sensor errors, FIFO
 *   overflow, dynamic range changed, watermark, initialization, transfer cause,
 *   sensor framework, and reset notifications. Routine spacer and initialized
 *   notifications are consumed by the driver and not posted to the application
 *   queue.
 *
 * @section sensewear_bhi360_devicetree Devicetree representation
 *
 * The BHI360 is declared with its INT pins and SPI bus specification:
 *
 * @code{.dts}
 * bhi360: bhi360@1 {
 *     compatible = "bosch,bhi360";
 *     reg = <1>;
 *     spi-max-frequency = <8000000>;
 *     cs-gpios = <&gpio2 5 GPIO_ACTIVE_LOW>;
 *     int-gpios = <&gpio1 9 GPIO_ACTIVE_LOW>;
 *     reset-gpios = <&gpio2 7 GPIO_ACTIVE_LOW>;
 *     gpio0-gpios = <&gpio1 10 GPIO_ACTIVE_HIGH>;
 *     gpio1-gpios = <&gpio2 6 GPIO_ACTIVE_HIGH>;
 *     status = "okay";
 * };
 * @endcode
 *
 * `int-gpios` is the active-low host IRQ. `gpio0-gpios` routes the BHI360 ASDX
 * pin to nRF54L15 P1.10, and `gpio1-gpios` routes the BHI360 ASCX pin to P2.06.
 * These auxiliary pins are exposed to callers through bhi360_get_gpio0() and
 * bhi360_get_gpio1() for custom BHI360 firmware use. Because GPIO0 is on nRF54L15
 * Port 1, it is better suited as an output strobe. GPIO1 is better suited as an
 * MPU input for custom firmware triggers.
 *
 * @section sensewear_bhi360_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call bhi360_init() to verify the shared SPI/GPIO resources and initialize
 *    interrupt handlers.
 * 2. Call bhi360_config() to bind the BHY2 transport hooks, probe the
 *    product ID, upload firmware, configure FIFO/host interrupt routing, and
 *    enable the configured low-rate activity-class event sensors.
 * 3. Start high-rate physical streams later by calling
 *    bhi360_start_phy_sensor_streams().
 * 4. Process BHI360 device events as they arrive via the event queue. On each
 *    bhi360_event_Irq, call bhi360_irq_handler() from caller thread context.
 * 5. Use bhi360_stop_phy_sensor_streams() to stop only the high-rate physical
 *    streams; low-rate event sensors remain configured.
 * 6. Call bhi360_stop() to disable all configured virtual sensors and soft-reset
 *    the device.
 *
 * @section sensewear_bhi360_sensors Configured Sensors
 *
 * The driver uses two requested sensor groups.
 * Sensor availability is firmware-dependent: bhi360_config() only enables
 * sensors advertised by the loaded BHI360 firmware. If the firmware image is
 * changed, update these requested sensor tables to match the virtual sensors
 * exposed by that firmware.
 *
 * **Physical stream sensors** (100 Hz sample rate, optional):
 * - Game Rotation Vector wake-up (GAMERV_WU): Fused orientation as quaternion
 *   (x, y, z, w, accuracy).
 * - Accelerometer wake-up (ACC_WU): Acceleration.
 * - Gyroscope wake-up (GYRO_WU): Angular velocity.
 *
 * The high-rate streams intentionally use wake-up virtual-sensor IDs so their
 * samples are placed in the wake FIFO. They are enabled only when
 * bhi360_start_phy_sensor_streams(period_ms) is called after configuration.
 * They are disabled by bhi360_stop_phy_sensor_streams() without disabling the
 * low-rate event sensors.
 *
 * The wake and non-wake FIFO watermarks are both programmed to 8 bytes during
 * bhi360_config() so host interrupts are generated after small batches
 * instead of waiting for firmware default watermarks.
 *
 * **Activity-class event sensors** (configured by bhi360_config()):
 * - Step Counter LP (STC_LP): Cumulative low-power step count.
 * - Step Detector LP (STD_LP): Step-detected events.
 * - Any Motion LP (ANY_MOTION_LP): Low-power motion event.
 * - No Motion LP wake-up (NO_MOTION_LP_WU): Wake-up no-motion event.
 * - Wrist Gesture Detect LP wake-up, Wrist Wear LP wake-up.
 * - Activity Recognition Wear (AR_WEAR_WU): Wake-up variant for detecting
 *   when user activity changes.
 *
 * Each configured entry provides its own sample rate and report latency. Step
 * Counter LP, Step Detector LP, and Any Motion LP are non-wake streams in the
 * current firmware, so the non-wake FIFO watermark is also kept small.
 *
 * **Meta-Events** (firmware-generated, always enabled):
 * - Flush complete, sample rate changed, power mode changed, algorithm events,
 *   sensor status, BSX calibration steps, sensor errors, FIFO overflow,
 *   dynamic range changed, watermark, firmware initialization (boot/reset
 *   complete), transfer cause, sensor framework, and reset notifications. Only
 *   routine spacer (FIFO padding) packets are filtered in the driver.
 *
 * @section sensewear_bhi360_example Typical usage
 *
 * @code{.c}
 * // Initialize
 * if (!bhi360_init()) {
 *     return;  // Probe failed
 * }
 *
 * // Configure the default low-rate activity-class event sensors.
 * if (!bhi360_config(NULL)) {
 *     return;  // Configuration failed
 * }
 *
 * // Enable high-rate quaternion/accelerometer/gyroscope streams when needed.
 * // The period controls the driver's software event used to request stream drains.
 * if (bhi360_start_phy_sensor_streams(100) != 0) {
 *     return;
 * }
 *
 * // Consume events in a dedicated thread
 * struct device_driver_event_t event;
 * while (device_driver_event_wait(K_FOREVER, &event)) {
 *     if (event.device_id != BHI360_DEVICE_DTS_ID) {
 *         continue;
 *     }
 *
 *     switch ((enum bhi360_event_type)event.event_id) {
 *
 *     case bhi360_event_Irq:
 *         // Drain FIFO and post downstream sensor/meta events
 *         bhi360_irq_handler();
 *         break;
 *
 *     case bhi360_event_QuaternionBatch:
 *         {
 *             uint32_t count = event.v_param;
 *             const struct bhi360_quat_data_t *quat =
 *                 (const struct bhi360_quat_data_t *)(uintptr_t)event.p_param;
 *             for (uint32_t i = 0; i < count; i++) {
 *                 printk("quat[%u] x=%d y=%d z=%d w=%d acc=%u\n", i,
 *                        quat[i].x, quat[i].y, quat[i].z, quat[i].w, quat[i].accuracy);
 *             }
 *         }
 *         break;
 *
 *     case bhi360_event_LinearAccelerationBatch:
 *         {
 *             uint32_t count = event.v_param;
 *             const struct bhi360_lacc_data_t *lacc =
 *                 (const struct bhi360_lacc_data_t *)(uintptr_t)event.p_param;
 *             for (uint32_t i = 0; i < count; i++) {
 *                 printk("lacc[%u] x=%d y=%d z=%d\n", i, lacc[i].x, lacc[i].y, lacc[i].z);
 *             }
 *         }
 *         break;
 *
 *     case bhi360_event_GyroBatch:
 *         {
 *             uint32_t count = event.v_param;
 *             const struct bhi360_gyro_data_t *gyro =
 *                 (const struct bhi360_gyro_data_t *)(uintptr_t)event.p_param;
 *             for (uint32_t i = 0; i < count; i++) {
 *                 printk("gyro[%u] x=%d y=%d z=%d\n", i, gyro[i].x, gyro[i].y, gyro[i].z);
 *             }
 *         }
 *         break;
 *
 *     case bhi360_event_Pedometer:
 *         {
 *             uint32_t sensor_id = event.v_param;
 *             const struct bhi360_pedometer_data_t *ped =
 *                 (const struct bhi360_pedometer_data_t *)(uintptr_t)event.p_param;
 *             printk("pedometer count=%u detected=%u\n",
 *                    ped->step_count, ped->step_detected);
 *         }
 *         break;
 *
 *     case bhi360_event_Gesture:
 *         {
 *             uint32_t sensor_id = (event.v_param >> 8) & 0xFF;
 *             uint8_t gesture_value = event.v_param & 0xFF;
 *             const struct bhi360_gesture_data_t *gest =
 *                 (const struct bhi360_gesture_data_t *)(uintptr_t)event.p_param;
 *             printk("gesture id=%u value=0x%02x\n", sensor_id, gesture_value);
 *         }
 *         break;
 *
 *     case bhi360_event_Activity:
 *         {
 *             uint32_t sensor_id = (event.v_param >> 16) & 0xFFFF;
 *             uint16_t activity = event.v_param & 0xFFFF;
 *             const struct bhi360_activity_data_t *act =
 *                 (const struct bhi360_activity_data_t *)(uintptr_t)event.p_param;
 *             printk("activity id=%u bits=0x%04x\n", sensor_id, activity);
 *         }
 *         break;
 *
 *     case bhi360_event_MetaEvent:
 *         {
 *             uint8_t meta_type = (event.v_param >> 16) & 0xFF;
 *             uint8_t byte1 = (event.v_param >> 8) & 0xFF;
 *             uint8_t byte2 = event.v_param & 0xFF;
 *             printk("meta type=0x%02x byte1=0x%02x byte2=0x%02x\n",
 *                    meta_type, byte1, byte2);
 *         }
 *         break;
 *     }
 * }
 *
 * // Later, stop only the high-rate physical streams. Low-rate event sensors
 * // configured by bhi360_config() continue to run.
 * (void)bhi360_stop_phy_sensor_streams();
 * @endcode
 *
 * @section sensewear_bhi360_isr Interrupt handling
 *
 * GPIO callbacks fire in ISR context and post bhi360_event_Irq to the event
 * queue. The application must call bhi360_irq_handler() from thread context to
 * read the FIFO, decode data, and generate downstream sensor events.
 */

#ifndef BHI360_H_
#define BHI360_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <zephyr/drivers/gpio.h>

#include "bhi3_defs.h"
#include "bhy2.h"

/**
 * @brief BHI360 event type enumeration.
 *
 * Each event type corresponds to a class of data the firmware can produce:
 * interrupt signals, sample streams, or metadata. Event handling passes sensor
 * data in two fields:
 * - v_param: Metadata, sample rate, sensor ID, or packed fields (see each type below).
 * - p_param: Pointer to driver-owned sample buffer (cast to @c uintptr_t);
 *   NULL for meta-events.
 */
enum bhi360_event_type {
	/**
	 * @brief Raw interrupt notification (enum value 0).
	 * @details v_param unused; p_param unused.
	 * The application/caller thread calls bhi360_irq_handler() from thread context
	 * to drain the FIFO and post downstream sensor and meta-events.
	 */
	bhi360_event_Irq = 0,
	/**
	 * @brief Quaternion orientation batch ready (enum value 1).
	 * @details Posted once per bhi360_irq_handler() drain that decoded at least one
	 * Game Rotation Vector wake-up (GAMERV_WU, 100 Hz) sample, not once per sample.
	 * v_param: Number of quaternion samples collected during this drain (uint32_t).
	 * p_param: Pointer to the first element of an array of that many
	 * @c struct bhi360_quat_data_t (x, y, z, w, accuracy), oldest first. The array is
	 * driver-owned and remains valid only until the next drain; use
	 * bhi360_copy_quaternion() for a thread-safe snapshot.
	 */
	bhi360_event_QuaternionBatch,
	/**
	 * @brief Linear acceleration batch ready (enum value 2).
	 * @details Posted once per bhi360_irq_handler() drain that decoded at least one
	 * Accelerometer wake-up (ACC_WU, 100 Hz) sample, not once per sample.
	 * v_param: Number of acceleration samples collected during this drain (uint32_t).
	 * p_param: Pointer to the first element of an array of that many
	 * @c struct bhi360_lacc_data_t (x, y, z), oldest first. The array is driver-owned
	 * and remains valid only until the next drain; use
	 * bhi360_copy_linear_acceleration() for a thread-safe snapshot.
	 */
	bhi360_event_LinearAccelerationBatch,
	/**
	 * @brief Angular velocity batch ready (enum value 3).
	 * @details Posted once per bhi360_irq_handler() drain that decoded at least one
	 * Gyroscope wake-up (GYRO_WU, 100 Hz) sample, not once per sample.
	 * v_param: Number of gyroscope samples collected during this drain (uint32_t).
	 * p_param: Pointer to the first element of an array of that many
	 * @c struct bhi360_gyro_data_t (x, y, z), oldest first. The array is driver-owned
	 * and remains valid only until the next drain; use bhi360_copy_gyro() for a
	 * thread-safe snapshot.
	 */
	bhi360_event_GyroBatch,
	/**
	 * @brief Step count or step detection event (enum value 4).
	 * @details v_param: Sensor ID (uint32_t).
	 * p_param: Pointer to @c struct bhi360_pedometer_data_t (sensor_id, step_count, step_detected).
	 * - Step Counter LP (STC_LP): Cumulative count, step_detected false.
	 * Wake-up low-power step counter/detector sensors are supported by the parser
	 * but are not enabled by the default table because the current firmware does
	 * not advertise them.
	 */
	bhi360_event_Pedometer,
	/**
	 * @brief Gesture detection event (enum value 5).
	 * @details v_param: Packed as (sensor_id << 8) | gesture_value (uint32_t).
	 * p_param: Pointer to @c struct bhi360_gesture_data_t (sensor_id, value).
	 * Sources: No Motion LP wake-up, Wrist Gesture Detect LP wake-up, and Wrist
	 * Wear LP wake-up. All post at 1 Hz.
	 */
	bhi360_event_Gesture,
	/**
	 * @brief Activity classification event (enum value 6).
	 * @details v_param: Packed as (sensor_id << 16) | activity_bits (uint32_t).
	 * p_param: Pointer to @c struct bhi360_activity_data_t (sensor_id, activity).
	 * Activity bits indicate which activity (still, walking, running, bicycle, vehicle, tilt)
	 * started or ended. Source: AR_WEAR_WU. Post at 1 Hz.
	 */
	bhi360_event_Activity,
	/**
	 * @brief Firmware meta-event notification (enum value 7).
	 * @details v_param: Packed as (type << 16) | (byte1 << 8) | byte2 (uint32_t);
	 * p_param: NULL (always).
	 * Type identifies the meta-event (flush, sample rate, power mode, algorithm, status,
	 * BSX calibration, sensor error, FIFO overflow, dynamic range, watermark, init,
	 * transfer cause, sensor framework, reset, spacer). Byte1 and byte2 hold event-specific
	 * data (sensor ID, accuracy level, error code, etc.). Routine spacer and
	 * initialized packets are filtered and not posted.
	 */
	bhi360_event_MetaEvent,
	/** @brief Number of valid event IDs. */
	bhi360_event_Count,
};

/** @brief Pedometer subevents carried in v_param for bhi360_event_Pedometer. */
enum bhi360_pedometer_event_type {
	bhi360_pedometer_event_StepCounter = BHY2_SENSOR_ID_STC,
	bhi360_pedometer_event_StepCounterWakeup = BHY2_SENSOR_ID_STC_WU,
	bhi360_pedometer_event_StepCounterLowPower = BHY2_SENSOR_ID_STC_LP,
	bhi360_pedometer_event_StepCounterLowPowerWakeup = BHY2_SENSOR_ID_STC_LP_WU,
	bhi360_pedometer_event_StepDetector = BHY2_SENSOR_ID_STD,
	bhi360_pedometer_event_StepDetectorWakeup = BHY2_SENSOR_ID_STD_WU,
	bhi360_pedometer_event_StepDetectorLowPower = BHY2_SENSOR_ID_STD_LP,
	bhi360_pedometer_event_StepDetectorLowPowerWakeup = BHY2_SENSOR_ID_STD_LP_WU,
};

/** @brief Gesture subevents carried in bits 15:8 of v_param for bhi360_event_Gesture. */
enum bhi360_gesture_event_type {
	bhi360_gesture_event_Wake = BHY2_SENSOR_ID_WAKE_GESTURE,
	bhi360_gesture_event_Glance = BHY2_SENSOR_ID_GLANCE_GESTURE,
	bhi360_gesture_event_Pickup = BHY2_SENSOR_ID_PICKUP_GESTURE,
	bhi360_gesture_event_WristTilt = BHY2_SENSOR_ID_WRIST_TILT_GESTURE,
	bhi360_gesture_event_TiltDetector = BHY2_SENSOR_ID_TILT_DETECTOR,
	bhi360_gesture_event_StationaryDetector = BHY2_SENSOR_ID_STATIONARY_DET,
	bhi360_gesture_event_MotionDetector = BHY2_SENSOR_ID_MOTION_DET,
	bhi360_gesture_event_SignificantMotion = BHY2_SENSOR_ID_SIG,
	bhi360_gesture_event_SignificantMotionLowPower = BHY2_SENSOR_ID_SIG_LP,
	bhi360_gesture_event_SignificantMotionLowPowerWakeup = BHY2_SENSOR_ID_SIG_LP_WU,
	bhi360_gesture_event_AnyMotionLowPower = BHY2_SENSOR_ID_ANY_MOTION_LP,
	bhi360_gesture_event_AnyMotionLowPowerWakeup = BHY2_SENSOR_ID_ANY_MOTION_LP_WU,
	bhi360_gesture_event_NoMotionLowPowerWakeup = BHI3_SENSOR_ID_NO_MOTION_LP_WU,
	bhi360_gesture_event_WristGestureDetectLowPowerWakeup = BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU,
	bhi360_gesture_event_WristWearLowPowerWakeup = BHI3_SENSOR_ID_WRIST_WEAR_LP_WU,
};

/** @brief Activity source subevents carried in bits 23:16 of v_param for bhi360_event_Activity. */
enum bhi360_activity_event_type {
	bhi360_activity_event_Recognition = BHY2_SENSOR_ID_AR,
	bhi360_activity_event_WearRecognitionWakeup = BHI3_SENSOR_ID_AR_WEAR_WU,
};

/**
 * @brief Activity-class sensor sources supported by bhi360_config().
 * @details This is the supported subset for the current SenseWear BHI360
 *          firmware and parser implementation. Other BHY2 activity-capable
 *          virtual sensors may exist, but they are intentionally not exposed
 *          until the driver has parser and firmware support for them.
 */
enum bhi360_activity_sensor_type {
	bhi360_activity_sensor_AnyMotionLowPower,
	bhi360_activity_sensor_NoMotionLowPowerWakeup,
	bhi360_activity_sensor_WristGestureDetectLowPowerWakeup,
	bhi360_activity_sensor_WristWearLowPowerWakeup,
	bhi360_activity_sensor_WearRecognitionWakeup,
	bhi360_activity_sensor_StepCounterLowPower,
	bhi360_activity_sensor_StepDetectorLowPower,
	bhi360_activity_sensor_Count,
};

/**
 * @brief Configuration for one supported BHI360 activity-class sensor.
 */
struct bhi360_activity_sensor_config_t {
	enum bhi360_activity_sensor_type sensor; /**< Supported activity-class sensor to enable. */
	float sample_rate_hz;					/**< Output data rate in Hz. */
	uint32_t latency_ms;						/**< Report latency in milliseconds; 0 for real-time. */
};

/**
 * @brief BHI360 firmware and low-rate sensor configuration.
 * @details Physical high-rate streams are not controlled here. Configure the
 *          device first, then enable quaternion/accelerometer/gyroscope streams
 *          later with bhi360_start_phy_sensor_streams().
 */
struct bhi360_config_t {
	const struct bhi360_activity_sensor_config_t* activity_sensors;
	size_t activity_sensor_count;
};

/** @brief Activity transition bits carried in bits 15:0 of v_param for bhi360_event_Activity. */
enum bhi360_activity_transition_type {
	bhi360_activity_transition_StillEnded = BHY2_STILL_ACTIVITY_ENDED,
	bhi360_activity_transition_WalkingEnded = BHY2_WALKING_ACTIVITY_ENDED,
	bhi360_activity_transition_RunningEnded = BHY2_RUNNING_ACTIVITY_ENDED,
	bhi360_activity_transition_BicycleEnded = BHY2_ON_BICYCLE_ACTIVITY_ENDED,
	bhi360_activity_transition_VehicleEnded = BHY2_IN_VEHICLE_ACTIVITY_ENDED,
	bhi360_activity_transition_TiltingEnded = BHY2_TILTING_ACTIVITY_ENDED,
	bhi360_activity_transition_StillStarted = BHY2_STILL_ACTIVITY_STARTED,
	bhi360_activity_transition_WalkingStarted = BHY2_WALKING_ACTIVITY_STARTED,
	bhi360_activity_transition_RunningStarted = BHY2_RUNNING_ACTIVITY_STARTED,
	bhi360_activity_transition_BicycleStarted = BHY2_ON_BICYCLE_ACTIVITY_STARTED,
	bhi360_activity_transition_VehicleStarted = BHY2_IN_VEHICLE_ACTIVITY_STARTED,
	bhi360_activity_transition_TiltingStarted = BHY2_TILTING_ACTIVITY_STARTED,
};

/** @brief Meta-event subevents carried in bits 23:16 of v_param for bhi360_event_MetaEvent. */
enum bhi360_meta_event_type {
	bhi360_meta_event_FlushComplete = BHY2_META_EVENT_FLUSH_COMPLETE,
	bhi360_meta_event_SampleRateChanged = BHY2_META_EVENT_SAMPLE_RATE_CHANGED,
	bhi360_meta_event_PowerModeChanged = BHY2_META_EVENT_POWER_MODE_CHANGED,
	bhi360_meta_event_AlgorithmEvents = BHY2_META_EVENT_ALGORITHM_EVENTS,
	bhi360_meta_event_SensorStatus = BHY2_META_EVENT_SENSOR_STATUS,
	bhi360_meta_event_BsxDoStepsMain = BHY2_META_EVENT_BSX_DO_STEPS_MAIN,
	bhi360_meta_event_BsxDoStepsCalib = BHY2_META_EVENT_BSX_DO_STEPS_CALIB,
	bhi360_meta_event_BsxGetOutputSignal = BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL,
	bhi360_meta_event_SensorError = BHY2_META_EVENT_SENSOR_ERROR,
	bhi360_meta_event_FifoOverflow = BHY2_META_EVENT_FIFO_OVERFLOW,
	bhi360_meta_event_DynamicRangeChanged = BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED,
	bhi360_meta_event_FifoWatermark = BHY2_META_EVENT_FIFO_WATERMARK,
	bhi360_meta_event_Initialized = BHY2_META_EVENT_INITIALIZED,
	bhi360_meta_event_TransferCause = BHY2_META_TRANSFER_CAUSE,
	bhi360_meta_event_SensorFramework = BHY2_META_EVENT_SENSOR_FRAMEWORK,
	bhi360_meta_event_Reset = BHY2_META_EVENT_RESET,
	bhi360_meta_event_Spacer = BHY2_META_EVENT_SPACER,
};

/**
 * @brief Quaternion orientation data.
 * @details X, Y, Z, W components of the quaternion with associated accuracy.
 */
struct bhi360_quat_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	int16_t x;		  /**< @brief Quaternion X component (fixed-point). */
	int16_t y;		  /**< @brief Quaternion Y component (fixed-point). */
	int16_t z;		  /**< @brief Quaternion Z component (fixed-point). */
	int16_t w;		  /**< @brief Quaternion W component (fixed-point). */
	uint16_t accuracy; /**< @brief Estimation accuracy. */
};

/**
 * @brief Linear acceleration data (gravity-compensated).
 * @details Acceleration in X, Y, Z axes with gravity removed.
 */
struct bhi360_lacc_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	int16_t x;		  /**< @brief X-axis acceleration (fixed-point). */
	int16_t y;		  /**< @brief Y-axis acceleration (fixed-point). */
	int16_t z;		  /**< @brief Z-axis acceleration (fixed-point). */
};

/**
 * @brief Angular velocity data.
 * @details Rotation rate in X, Y, Z axes.
 */
struct bhi360_gyro_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	int16_t x;		  /**< @brief X-axis angular velocity (fixed-point). */
	int16_t y;		  /**< @brief Y-axis angular velocity (fixed-point). */
	int16_t z;		  /**< @brief Z-axis angular velocity (fixed-point). */
};

/**
 * @brief Pedometer output: step count and detection state.
 * @details Cumulative step count and flag indicating whether a step was
 *          detected in the current interval.
 */
struct bhi360_pedometer_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	uint8_t sensor_id;	 /**< @brief Sensor identifier (firmware-assigned). */
	uint32_t step_count; /**< @brief Cumulative step count. */
	bool step_detected;	 /**< @brief True if a step was detected recently. */
};

/**
 * @brief Gesture event data.
 * @details Identifies which gesture (shake, flip, etc.) was detected.
 */
struct bhi360_gesture_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	uint8_t sensor_id; /**< @brief Sensor identifier (firmware-assigned). */
	uint8_t value;	   /**< @brief Gesture type code. */
};

/**
 * @brief Activity classification data.
 * @details Identifies the current activity (walk, run, etc.) and confidence.
 */
struct bhi360_activity_data_t {
	time_t timestamp; /**< @brief Timestamp in microseconds since BHI360 firmware boot, derived from
						 BHY2 callback_info->time_stamp. */
	uint8_t sensor_id; /**< @brief Sensor identifier (firmware-assigned). */
	uint16_t activity; /**< @brief Activity type and confidence bits. */
};

/**
 * @brief Initialize BHI360 board resources.
 * @details Verifies the shared SPI bus and GPIO resources, sets up GPIO
 *          interrupt handlers, and prepares the driver for configuration.
 * @retval true Initialization completed.
 * @retval false SPI/GPIO resources are unavailable or IRQ setup failed.
 * @pre The SPI bus and GPIO interrupt lines are available.
 */
bool bhi360_init(void);

/**
 * @brief Report whether the BHI360 is initialized and configured.
 * @retval true The driver resources are initialized and firmware/sensors are configured.
 * @retval false Initialization or configuration has not succeeded.
 */
bool bhi360_is_ready(void);

/**
 * @brief Populate the SenseWear default BHI360 configuration.
 * @details The default enables the supported low-rate activity-class sensors at
 *          their historical SenseWear rates and latencies. Physical high-rate
 *          streams are intentionally not enabled by this configuration.
 *
 * @param config Destination configuration. Must not be NULL. The returned
 *        `activity_sensors` pointer refers to driver-owned immutable storage.
 */
void bhi360_get_default_config(struct bhi360_config_t* config);

/**
 * @brief Configure the BHI360 for sensor operation.
 * @details Binds the BHY2 transport hooks, verifies the product ID, uploads
 *          firmware, configures host IRQ/FIFO routing, registers parser
 *          callbacks, and enables the activity-class sensors requested in
 *          @p config. Passing NULL selects bhi360_get_default_config().
 *          Physical stream sensors (GAMERV_WU, ACC_WU, GYRO_WU) are not enabled
 *          here; call bhi360_start_phy_sensor_streams() later when high-rate data
 *          is needed.
 * @param config Configuration to apply, or NULL for the SenseWear defaults.
 * @retval true Firmware upload and sensor configuration completed.
 * @retval false Upload or configuration failed.
 * @pre bhi360_init() has completed successfully.
 */
bool bhi360_config(const struct bhi360_config_t* config);

/**
 * @brief Drain the FIFO and generate sensor events.
 * @details Called from caller thread context in response to bhi360_event_Irq.
 *          Reads FIFO packets, decodes sensor data, and posts corresponding
 *          events (QuaternionBatch, LinearAccelerationBatch, GyroBatch,
 *          Pedometer, etc.) to the device-driver event queue. Meta-events are
 *          also posted for firmware notifications.
 * @retval 0 FIFO processing completed.
 * @retval -ENODEV The driver is not initialized and configured.
 * @retval -EIO Device communication or FIFO parsing failed.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 * @pre Called from thread context, not ISR.
 */
int bhi360_irq_handler(void);

/**
 * @brief Enable high-rate physical sensor streams.
 * @details Enables the physical stream sensor table: Game Rotation Vector
 *          wake-up, Accelerometer wake-up, and Gyroscope wake-up. These sensors
 *          generate QuaternionBatch, LinearAccelerationBatch, and GyroBatch
 *          events after bhi360_irq_handler() drains FIFO data. The driver starts
 *          its software stream timer with @p period_ms; the timer posts a
 *          bhi360_event_Irq into the common event queue so the caller's existing
 *          event consumer can drain the FIFO in thread context. Hardware FIFO
 *          interrupts may also post bhi360_event_Irq.
 *
 *          The physical streams share the same timer used by
 *          bhi360_start_periodic_timer(). If that timer is already running at a
 *          different period, this call fails with -EBUSY.
 *
 *          Calling this function does not affect the low-rate event sensors that
 *          were enabled by bhi360_config().
 * @param period_ms Software stream-drain period in milliseconds.
 * @retval 0 Physical streams were enabled and the stream timer was started.
 * @retval -ENODEV Driver is not initialized and configured.
 * @retval -EBUSY Physical streams are already enabled, or the shared timer is
 *         already running at a different period.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_start_phy_sensor_streams(uint32_t period_ms);

/**
 * @brief Disable high-rate physical sensor streams.
 * @details Stops the software stream timer and disables only the physical stream
 *          sensor table: Game Rotation Vector wake-up, Accelerometer wake-up,
 *          and Gyroscope wake-up. The low-rate event sensors configured by
 *          bhi360_config() remain enabled and can continue to produce events.
 *
 *          The disabled physical streams are flushed as part of the stop path,
 *          so queued stream samples may be discarded. Callers should continue to
 *          service bhi360_event_Irq events and call bhi360_irq_handler() so any
 *          remaining low-rate FIFO data and flush/meta events are parsed.
 * @retval 0 Physical streams were disabled.
 * @retval -ENODEV Driver is not initialized and configured.
 * @retval -EINVAL Physical streams are not currently enabled.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_stop_phy_sensor_streams(void);

/**
 * @brief Start periodic FIFO drain requests.
 * @details Starts the driver's shared FIFO timer without enabling any additional
 *          virtual sensors. Each timer expiry posts bhi360_event_Irq to the
 *          common device-driver event queue; the caller's event consumer must
 *          still call bhi360_irq_handler() from thread context to drain and
 *          parse FIFO data.
 *
 *          Use this when low-rate event sensors need bounded latency even when
 *          BHI360 hardware FIFO interrupts are sparse or watermark-driven. For
 *          high-rate quaternion/accelerometer/gyroscope streaming, prefer
 *          bhi360_start_phy_sensor_streams(), which enables those sensors and
 *          uses the same periodic drain mechanism.
 *
 *          If the shared timer is already running with the same period, this
 *          call succeeds without changing state. If it is running with a
 *          different period, this call fails with -EBUSY.
 * @param period_ms Periodic FIFO drain request interval in milliseconds.
 * @retval 0 Timer is running at @p period_ms.
 * @retval -ENODEV Driver is not initialized and configured.
 * @retval -EBUSY Shared timer is already running at a different period.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_start_periodic_timer(uint32_t period_ms);

/**
 * @brief Stop periodic FIFO drain requests.
 * @details Stops the shared FIFO timer. If high-rate physical streams are
 *          currently enabled, this also disables and flushes the physical stream
 *          sensor table because those streams depend on the same timer for
 *          bounded FIFO draining. Low-rate event sensors configured by
 *          bhi360_config() remain enabled.
 *
 *          A final bhi360_event_Irq is posted after stopping so the caller can
 *          drain any remaining FIFO, flush, or meta packets from thread context.
 * @retval 0 Timer was stopped.
 * @retval -ENODEV Driver is not initialized and configured.
 * @retval -EINVAL Periodic timer is not currently running.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_stop_periodic_timer(void);

/**
 * @brief Copy the most recently decoded quaternion samples out of the driver.
 * @details Copies up to @p max_samples quaternion samples decoded during the
 *          last bhi360_irq_handler() drain into @p out. The driver cache lock is
 *          held during the copy so a concurrent FIFO drain cannot overwrite the
 *          samples mid-read. The cache is replaced wholesale on each drain, so
 *          this returns the latest batch rather than an accumulating history.
 * @param out Destination buffer for up to @p max_samples samples.
 * @param max_samples Capacity of @p out in samples.
 * @return Number of samples copied (>= 0), -EINVAL if @p out is NULL or
 *         @p max_samples is zero, or -ENODEV if the driver is not configured.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_copy_quaternion(struct bhi360_quat_data_t* out, size_t max_samples);

/**
 * @brief Copy the most recently decoded linear-acceleration samples out.
 * @details Behaves like bhi360_copy_quaternion() for the accelerometer stream.
 * @param out Destination buffer for up to @p max_samples samples.
 * @param max_samples Capacity of @p out in samples.
 * @return Number of samples copied (>= 0), -EINVAL on bad arguments, or -ENODEV
 *         if the driver is not configured.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_copy_linear_acceleration(struct bhi360_lacc_data_t* out, size_t max_samples);

/**
 * @brief Copy the most recently decoded gyroscope samples out.
 * @details Behaves like bhi360_copy_quaternion() for the gyroscope stream.
 * @param out Destination buffer for up to @p max_samples samples.
 * @param max_samples Capacity of @p out in samples.
 * @return Number of samples copied (>= 0), -EINVAL on bad arguments, or -ENODEV
 *         if the driver is not configured.
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_copy_gyro(struct bhi360_gyro_data_t* out, size_t max_samples);

/**
 * @brief FIFO control snapshot.
 * @details Watermark and total size, in bytes, of the wake-up and non-wake-up
 *          FIFOs as currently programmed in the firmware. The driver programs
 *          both FIFO watermarks to 8 bytes during configuration.
 */
struct bhi360_fifo_status {
	uint32_t wakeup_watermark;	  /**< @brief Wake-up FIFO watermark in bytes. */
	uint32_t wakeup_size;		  /**< @brief Wake-up FIFO total size in bytes. */
	uint32_t nonwakeup_watermark; /**< @brief Non-wake-up FIFO watermark in bytes. */
	uint32_t nonwakeup_size;	  /**< @brief Non-wake-up FIFO total size in bytes. */
};

/**
 * @brief Read the BHI360 FIFO control (watermarks and sizes).
 * @details Queries the firmware FIFO control parameter over the status channel.
 *
 *          @warning Do NOT call this while sensors are streaming. The driver runs
 *          with the async status channel enabled, so the firmware continuously
 *          pushes status messages into the same channel this parameter read uses;
 *          the read then collides (returns -EIO) and can leave the channel stuck,
 *          breaking subsequent FIFO processing. The driver already logs the FIFO
 *          control once at bhi360_config() time, before the async status
 *          channel is enabled, which is the reliable source for these values.
 *          This call is only safe when sensor streaming is quiesced (for example
 *          after bhi360_stop()).
 *
 *          Shares the BHY2 device SPI access with bhi360_irq_handler(), so it must
 *          be called from thread context; the driver serializes the two with the
 *          cache lock.
 * @param out Destination for the FIFO control snapshot.
 * @retval 0 Snapshot populated.
 * @retval -EINVAL @p out is NULL.
 * @retval -ENODEV The driver is not initialized and configured.
 * @retval -EIO Device read failed (for example, async status pending).
 * @pre bhi360_init() and bhi360_config() have completed successfully.
 */
int bhi360_get_fifo_status(struct bhi360_fifo_status* out);

/**
 * @brief Retrieve the GPIO specification for auxiliary GPIO0.
 * @details GPIO0 is routed from the BHI360 ASDX pin to nRF54L15 P1.10. It is
 *          available for custom BHI360 firmware use and is better suited as an
 *          output strobe because it is connected to nRF54L15 Port 1.
 * @return Pointer to the GPIO0 spec (driver-internal, do not modify).
 * @pre bhi360_init() has completed successfully.
 */
const struct gpio_dt_spec* bhi360_get_gpio0(void);

/**
 * @brief Retrieve the GPIO specification for auxiliary GPIO1.
 * @details GPIO1 is routed from the BHI360 ASCX pin to nRF54L15 P2.06. It is
 *          available for custom BHI360 firmware use and is better suited as an
 *          MPU input for triggering custom firmware actions.
 * @return Pointer to the GPIO1 spec (driver-internal, do not modify).
 * @pre bhi360_init() has completed successfully.
 */
const struct gpio_dt_spec* bhi360_get_gpio1(void);

/**
 * @brief Query the name of a BHI360 event or subevent.
 *
 * @param event_id Event ID from @ref bhi360_event_type.
 * @param v_param Event value parameter. For events with subevents, this is decoded
 * according to the event-specific v_param packing.
 * @return Constant event-name string, or "Unknown" for invalid IDs.
 */
const char* bhi360_event_name(enum bhi360_event_type event_id, uint32_t v_param);

/**
 * @brief Stop BHI360 virtual-sensor output.
 * @details Disables configured virtual sensors, flushes their FIFOs, soft-resets
 *          the BHY2 device context, and marks the driver unconfigured while
 *          preserving initialized board resources.
 * @pre bhi360_init() has completed successfully.
 */
void bhi360_stop(void);

/** @} */

#endif /* BHI360_H_ */
