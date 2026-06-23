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
 * communicates via SPI and generates interrupts on its INT pins when new data
 * or meta-events become available.
 *
 * The driver operates in interrupt-driven mode: GPIO callbacks post events to
 * the central device-driver event queue (see @ref device_driver_events.h).
 * The application consumes these events from thread context, decodes the
 * payloads, and updates application state.
 *
 * @section sensewear_bhi360_event_model Event-driven architecture
 *
 * The BHI360 posts the following event types (see @ref bhi360_event_type):
 *
 * - **Irq**: Raw interrupt signal; the handler calls bhi360_process_irq() to
 *   read and parse FIFO data, which may generate zero or more downstream events.
 *
 * - **Quaternion**: Orientation as x, y, z, w components with accuracy.
 *   @details Sensor ID passed in v_param; sample data via p_param
 *   (@c struct bhi360_quat_data*). Sources: Rotation vector (RV) sensor at 100 Hz.
 *
 * - **LinearAcceleration**: X, Y, Z acceleration with gravity compensation.
 *   @details Sensor ID passed in v_param; data via p_param
 *   (@c struct bhi360_lacc_data*). Sources: Linear acceleration (LACC) sensor
 *   at 100 Hz.
 *
 * - **Gyro**: Angular velocity X, Y, Z.
 *   @details Sensor ID passed in v_param; data via p_param
 *   (@c struct bhi360_gyro_data*). Sources: Gyroscope (GYRO) sensor at 100 Hz.
 *
 * - **Pedometer**: Step count and detection state.
 *   @details Sensor ID passed in v_param; data via p_param
 *   (@c struct bhi360_pedometer_data*). Sources: Step counter (STC, STC_WU, STC_LP,
 *   STC_LP_WU) at 1 Hz and step detector (STD, STD_WU, STD_LP, STD_LP_WU) events.
 *   Counter variants include normal, wake-up, and low-power modes.
 *
 * - **Gesture**: Gesture type and sensor ID.
 *   @details Sensor ID and gesture value packed in v_param as (sensor_id << 8) | value;
 *   data via p_param (@c struct bhi360_gesture_data*). Sources: Wake gesture, glance,
 *   pickup, wrist tilt, tilt detector, stationary/motion detectors, significant motion,
 *   and wrist-wear detectors.
 *
 * - **Activity**: Activity classification with start/end flags.
 *   @details Sensor ID and activity bits packed in v_param as (sensor_id << 16) | activity;
 *   data via p_param (@c struct bhi360_activity_data*). Activity bits indicate which
 *   activity started/ended (still, walking, running, bicycle, vehicle, tilting).
 *   Sources: Activity recognition (AR, AR_WEAR_WU) at 1 Hz.
 *
 * - **MetaEvent**: Firmware-generated metadata (sensor mode, reset, calibration, etc.).
 *   @details Packed in v_param as (type << 16) | (byte1 << 8) | byte2;
 *   no p_param. Includes events: flush complete, sample rate changed, power mode
 *   changed, algorithm events, sensor status, BSX steps, sensor errors, FIFO
 *   overflow, dynamic range changed, watermark, initialization, transfer cause,
 *   sensor framework, and reset notifications.
 *
 * @section sensewear_bhi360_devicetree Devicetree representation
 *
 * The BHI360 is declared with its INT pins and SPI bus specification:
 *
 * @code{.dts}
 * bhi360: imu@0 {
 *     compatible = "sensewear,bhi360";
 *     reg = <0>;
 *     spi-max-frequency = <10000000>;
 *     int0-gpios = <&gpio0 20 GPIO_ACTIVE_HIGH>;
 *     int1-gpios = <&gpio0 21 GPIO_ACTIVE_HIGH>;
 *     status = "okay";
 * };
 * @endcode
 *
 * `int0-gpios` and `int1-gpios` are unconfigured GPIO pins wired to the ASDX
 * and ASCX pins of the MPU. These can be configured in custom firmware as
 * user-defined functions.
 *
 * @section sensewear_bhi360_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call bhi360_init() to probe the device, verify boot loader completion,
 *    and initialize interrupt handlers.
 * 2. Call bhi360_configure() to upload firmware, enable sensors, and configure
 *    virtual-sensor channels.
 * 3. Process BHI360 device events as they arrive via the event queue.
 * 4. Call bhi360_stop() to disable interrupts and put the device into standby
 *    (typically during power-down sequences).
 *
 * @section sensewear_bhi360_sensors Configured Sensors
 *
 * The driver enables three categories of sensors during bhi360_configure():
 *
 * **Base Sensors** (100 Hz sample rate):
 * - Rotation Vector (RV): Fused orientation as quaternion (x, y, z, w, accuracy).
 * - Linear Acceleration (LACC): Gravity-compensated acceleration.
 * - Gyroscope (GYRO): Angular velocity.
 * - Step Counter (STC, STC_WU, STC_LP, STC_LP_WU): Cumulative step count (normal,
 *   wake-up, low-power, low-power wake-up variants).
 * - Step Detector (STD, STD_WU, STD_LP, STD_LP_WU): Binary step-detected events.
 *   All step sensors run at 1 Hz.
 *
 * **Gesture Sensors** (1 Hz sample rate):
 * - Wake Gesture, Glance Gesture, Pickup Gesture, Wrist Tilt Gesture
 * - Tilt Detector, Stationary Detector, Motion Detector, Significant Motion
 * - Significant Motion LP (low-power variant)
 * - Any Motion LP and LP wake-up variants
 * - No Motion LP wake-up, Wrist Gesture Detect LP wake-up, Wrist Wear LP wake-up
 *
 * **Activity Sensors** (1 Hz sample rate):
 * - Activity Recognition (AR): Recognizes still, walking, running, on bicycle,
 *   in vehicle, tilting. Posts start/end flags for each activity.
 * - Activity Recognition Wear (AR_WEAR_WU): Wake-up variant for detecting
 *   when user activity changes.
 *
 * **Meta-Events** (firmware-generated, always enabled):
 * - Flush complete, sample rate changed, power mode changed, algorithm events,
 *   sensor status, BSX calibration steps, sensor errors, FIFO overflow,
 *   dynamic range changed, watermark, firmware initialization, transfer cause,
 *   sensor framework, and reset notifications.
 *
 * @section sensewear_bhi360_example Typical usage
 *
 * @code{.c}
 * // Initialize
 * if (!bhi360_init()) {
 *     return;  // Probe failed
 * }
 *
 * // Configure sensors
 * if (!bhi360_configure()) {
 *     return;  // Configuration failed
 * }
 *
 * // Consume events in a dedicated thread
 * struct device_driver_event_t event;
 * while (device_driver_event_wait(K_FOREVER, &event)) {
 *     if (event.device_id != BHI360_DEVICE_ID) {
 *         continue;
 *     }
 *
 *     switch ((enum bhi360_event_type)event.event_id) {
 *
 *     case bhi360_event_Irq:
 *         // Drain FIFO and post downstream sensor/meta events
 *         bhi360_process_irq();
 *         break;
 *
 *     case bhi360_event_Quaternion:
 *         {
 *             uint32_t sensor_id = event.v_param;
 *             const struct bhi360_quat_data *quat =
 *                 (const struct bhi360_quat_data *)(uintptr_t)event.p_param;
 *             printk("quat x=%d y=%d z=%d w=%d acc=%u\n",
 *                    quat->x, quat->y, quat->z, quat->w, quat->accuracy);
 *         }
 *         break;
 *
 *     case bhi360_event_LinearAcceleration:
 *         {
 *             uint32_t sensor_id = event.v_param;
 *             const struct bhi360_lacc_data *lacc =
 *                 (const struct bhi360_lacc_data *)(uintptr_t)event.p_param;
 *             printk("lacc x=%d y=%d z=%d\n", lacc->x, lacc->y, lacc->z);
 *         }
 *         break;
 *
 *     case bhi360_event_Gyro:
 *         {
 *             uint32_t sensor_id = event.v_param;
 *             const struct bhi360_gyro_data *gyro =
 *                 (const struct bhi360_gyro_data *)(uintptr_t)event.p_param;
 *             printk("gyro x=%d y=%d z=%d\n", gyro->x, gyro->y, gyro->z);
 *         }
 *         break;
 *
 *     case bhi360_event_Pedometer:
 *         {
 *             uint32_t sensor_id = event.v_param;
 *             const struct bhi360_pedometer_data *ped =
 *                 (const struct bhi360_pedometer_data *)(uintptr_t)event.p_param;
 *             printk("pedometer count=%u detected=%u\n",
 *                    ped->step_count, ped->step_detected);
 *         }
 *         break;
 *
 *     case bhi360_event_Gesture:
 *         {
 *             uint32_t sensor_id = (event.v_param >> 8) & 0xFF;
 *             uint8_t gesture_value = event.v_param & 0xFF;
 *             const struct bhi360_gesture_data *gest =
 *                 (const struct bhi360_gesture_data *)(uintptr_t)event.p_param;
 *             printk("gesture id=%u value=0x%02x\n", sensor_id, gesture_value);
 *         }
 *         break;
 *
 *     case bhi360_event_Activity:
 *         {
 *             uint32_t sensor_id = (event.v_param >> 16) & 0xFFFF;
 *             uint16_t activity = event.v_param & 0xFFFF;
 *             const struct bhi360_activity_data *act =
 *                 (const struct bhi360_activity_data *)(uintptr_t)event.p_param;
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
 * @endcode
 *
 * @section sensewear_bhi360_isr Interrupt handling
 *
 * GPIO callbacks for INT0 and INT1 fire in ISR context and post
 * bhi360_event_Irq to the event queue. The application must call
 * bhi360_process_irq() from thread context to read the FIFO, decode
 * data, and generate downstream sensor events.
 */

#ifndef BHI360_H_
#define BHI360_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>

/**
 * @brief BHI360 event type enumeration.
 *
 * Each event type corresponds to a class of data the firmware can produce:
 * interrupt signals, sample streams, or metadata. Event handling passes
 * sensor data in two fields:
 * - v_param: Metadata, sample rate, sensor ID, or packed fields (see each type below).
 * - p_param: Pointer to driver-owned sample buffer (cast to @c uintptr_t);
 *   NULL for meta-events.
 */
enum bhi360_event_type {
	/**
	 * @brief Raw interrupt notification (enum value 0).
	 * @details v_param unused; p_param unused.
	 * The handler calls bhi360_process_irq() from thread context to drain the FIFO
	 * and post downstream sensor and meta-events. Multiple Irq events may occur per
	 * interrupt if the FIFO contains multiple packets.
	 */
	bhi360_event_Irq = 0,
	/**
	 * @brief Quaternion orientation event (enum value 1).
	 * @details v_param: Sensor ID (uint32_t).
	 * p_param: Pointer to @c struct bhi360_quat_data (x, y, z, w, accuracy).
	 * Posted by Rotation Vector (RV) sensor at 100 Hz.
	 */
	bhi360_event_Quaternion,
	/**
	 * @brief Linear acceleration event (enum value 2).
	 * @details v_param: Sensor ID (uint32_t).
	 * p_param: Pointer to @c struct bhi360_lacc_data (x, y, z).
	 * Posted by Linear Acceleration (LACC) sensor at 100 Hz.
	 */
	bhi360_event_LinearAcceleration,
	/**
	 * @brief Angular velocity event (enum value 3).
	 * @details v_param: Sensor ID (uint32_t).
	 * p_param: Pointer to @c struct bhi360_gyro_data (x, y, z).
	 * Posted by Gyroscope (GYRO) sensor at 100 Hz.
	 */
	bhi360_event_Gyro,
	/**
	 * @brief Step count or step detection event (enum value 4).
	 * @details v_param: Sensor ID (uint32_t).
	 * p_param: Pointer to @c struct bhi360_pedometer_data (sensor_id, step_count, step_detected).
	 * - Step Counter (STC, STC_WU, STC_LP, STC_LP_WU): Cumulative count, step_detected false.
	 * - Step Detector (STD, STD_WU, STD_LP, STD_LP_WU): Trigger event, step_detected true.
	 * All post at 1 Hz.
	 */
	bhi360_event_Pedometer,
	/**
	 * @brief Gesture detection event (enum value 5).
	 * @details v_param: Packed as (sensor_id << 8) | gesture_value (uint32_t).
	 * p_param: Pointer to @c struct bhi360_gesture_data (sensor_id, value).
	 * Sources: Wake, glance, pickup, wrist tilt gestures; tilt/stationary/motion detectors;
	 * significant motion; wrist-related sensors. All post at 1 Hz.
	 */
	bhi360_event_Gesture,
	/**
	 * @brief Activity classification event (enum value 6).
	 * @details v_param: Packed as (sensor_id << 16) | activity_bits (uint32_t).
	 * p_param: Pointer to @c struct bhi360_activity_data (sensor_id, activity).
	 * Activity bits indicate which activity (still, walking, running, bicycle, vehicle, tilt)
	 * started or ended. Sources: AR, AR_WEAR_WU. Post at 1 Hz.
	 */
	bhi360_event_Activity,
	/**
	 * @brief Firmware meta-event notification (enum value 7).
	 * @details v_param: Packed as (type << 16) | (byte1 << 8) | byte2 (uint32_t);
	 * p_param: NULL (always).
	 * Type identifies the meta-event (flush, sample rate, power mode, algorithm, status,
	 * BSX calibration, sensor error, FIFO overflow, dynamic range, watermark, init,
	 * transfer cause, sensor framework, reset, spacer). Byte1 and byte2 hold event-specific
	 * data (sensor ID, accuracy level, error code, etc.).
	 */
	bhi360_event_MetaEvent,
};

/**
 * @brief Quaternion orientation data.
 * @details X, Y, Z, W components of the quaternion with associated accuracy.
 */
struct bhi360_quat_data {
	int16_t x;		   /**< @brief Quaternion X component (fixed-point). */
	int16_t y;		   /**< @brief Quaternion Y component (fixed-point). */
	int16_t z;		   /**< @brief Quaternion Z component (fixed-point). */
	int16_t w;		   /**< @brief Quaternion W component (fixed-point). */
	uint16_t accuracy; /**< @brief Estimation accuracy. */
};

/**
 * @brief Linear acceleration data (gravity-compensated).
 * @details Acceleration in X, Y, Z axes with gravity removed.
 */
struct bhi360_lacc_data {
	int16_t x; /**< @brief X-axis acceleration (fixed-point). */
	int16_t y; /**< @brief Y-axis acceleration (fixed-point). */
	int16_t z; /**< @brief Z-axis acceleration (fixed-point). */
};

/**
 * @brief Angular velocity data.
 * @details Rotation rate in X, Y, Z axes.
 */
struct bhi360_gyro_data {
	int16_t x; /**< @brief X-axis angular velocity (fixed-point). */
	int16_t y; /**< @brief Y-axis angular velocity (fixed-point). */
	int16_t z; /**< @brief Z-axis angular velocity (fixed-point). */
};

/**
 * @brief Pedometer output: step count and detection state.
 * @details Cumulative step count and flag indicating whether a step was
 *          detected in the current interval.
 */
struct bhi360_pedometer_data {
	uint8_t sensor_id;	 /**< @brief Sensor identifier (firmware-assigned). */
	uint32_t step_count; /**< @brief Cumulative step count. */
	bool step_detected;	 /**< @brief True if a step was detected recently. */
};

/**
 * @brief Gesture event data.
 * @details Identifies which gesture (shake, flip, etc.) was detected.
 */
struct bhi360_gesture_data {
	uint8_t sensor_id; /**< @brief Sensor identifier (firmware-assigned). */
	uint8_t value;	   /**< @brief Gesture type code. */
};

/**
 * @brief Activity classification data.
 * @details Identifies the current activity (walk, run, etc.) and confidence.
 */
struct bhi360_activity_data {
	uint8_t sensor_id; /**< @brief Sensor identifier (firmware-assigned). */
	uint16_t activity; /**< @brief Activity type and confidence bits. */
};

/**
 * @brief Initialize and probe the BHI360.
 * @details Verifies the SPI bus, waits for boot-loader completion, sets up
 *          GPIO interrupt handlers, and prepares the device for configuration.
 * @retval true Initialization completed and boot loader signaled ready.
 * @retval false SPI unavailable or boot loader failed to complete.
 * @pre The SPI bus and GPIO interrupt lines are available.
 */
bool bhi360_init(void);

/**
 * @brief Configure the BHI360 for sensor operation.
 * @details Uploads the firmware (if not yet loaded), enables desired virtual
 *          sensors (quaternion, linear acceleration, etc.), and configures
 *          output rates and dynamic-range settings.
 * @retval true Firmware upload and sensor configuration completed.
 * @retval false Upload or configuration failed.
 * @pre bhi360_init() has completed successfully.
 */
bool bhi360_configure(void);

/**
 * @brief Drain the FIFO and generate sensor events.
 * @details Called from thread context in response to bhi360_event_Irq. Reads
 *          FIFO packets, decodes sensor data, and posts corresponding events
 *          (Quaternion, Gyro, Pedometer, etc.) to the device-driver event
 *          queue. Meta-events are also posted for firmware notifications.
 * @return Number of events processed, or negative error code on read failure.
 * @pre bhi360_init() and bhi360_configure() have completed successfully.
 * @pre Called from thread context, not ISR.
 */
int bhi360_process_irq(void);

/**
 * @brief Retrieve the GPIO specification for INT0.
 * @return Pointer to the INT0 GPIO spec (driver-internal, do not modify).
 * @pre bhi360_init() has completed successfully.
 */
const struct gpio_dt_spec* bhi360_get_gpio0(void);

/**
 * @brief Retrieve the GPIO specification for INT1.
 * @return Pointer to the INT1 GPIO spec (driver-internal, do not modify).
 * @pre bhi360_init() has completed successfully.
 */
const struct gpio_dt_spec* bhi360_get_gpio1(void);

/**
 * @brief Disable interrupts and put the BHI360 into standby.
 * @details Disables GPIO interrupts and sends the device into a low-power
 *          standby state. Call during system shutdown or when the sensor is
 *          not needed.
 * @pre bhi360_init() has completed successfully.
 */
void bhi360_stop(void);

/** @} */

#endif /* BHI360_H_ */
