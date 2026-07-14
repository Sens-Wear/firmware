/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_driver_messages.h
 * @brief SenseWear device-manager message contract.
 *
 * @defgroup sensewear_device_driver_messages SenseWear device driver messages
 * @ingroup io_interfaces
 * @{
 *
 * This header defines the decoded data structures the SenseWear device manager
 * publishes for the rest of the application to consume (for example over zbus).
 * It is the stable *contract* between the device manager and its subscribers
 * (the BLE manager, storage/logging, app logic, LED/haptic feedback, ...): a
 * subscriber depends only on these types, never on the individual device
 * drivers.
 *
 * @section sensewear_device_driver_messages_design Design
 *
 * The message types are **self-contained plain-old-data**: they intentionally do
 * @b not include any device-driver header, and they flatten each driver's decoded
 * sample into stable fields with explicit units. Two consequences follow:
 *
 * - The contract compiles regardless of which board drivers or shield is
 *   selected, so it can be shared by every subscriber without pulling in
 *   driver code.
 * - The device manager is the single place that translates a driver's native
 *   sample struct (e.g. `struct touch_sensor_sample_t`, `struct
 *   max30101_ppg_sample_t`) into the message below and publishes it @b by @b
 *   value. Only the device manager includes both the driver headers and this
 *   contract; when the device manager is disabled (for isolated device or shield
 *   tests) nothing else references it.
 *
 * @section sensewear_device_driver_messages_time Timestamps
 *
 * Every message carries a @c timestamp in **microseconds since the Unix epoch**.
 * Some devices report a device-local time base instead (the BHI360, for example,
 * timestamps relative to sensor-hub firmware boot); normalizing those to the
 * common epoch is the device manager's responsibility, so subscribers can treat
 * all timestamps uniformly.
 *
 * @section sensewear_device_driver_messages_channels Channels
 *
 * This header defines only the payloads. How they are carried is a device-manager
 * decision: either one channel per data type (each message struct is the channel
 * message), or one channel carrying the tagged ::device_msg_t envelope. The
 * per-type structs and the envelope are both provided so that choice can be made
 * (and revisited) in the device manager without changing the contract.
 *
 * The @c device_id field carries the producing device's generated
 * `<LABEL>_DEVICE_ID` (see @ref sensewear_device_driver_events). It is a
 * build-time identifier, not a stable ABI value; never persist it off-device.
 */

#ifndef SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_MESSAGES_H_
#define SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_MESSAGES_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/**
 * @brief Payload discriminator for the ::device_msg_t envelope.
 * @details Identifies which member of ::device_msg_t.payload is valid. The
 *          per-type message structs can also be used directly as per-channel
 *          messages, in which case this tag is not needed.
 */
enum device_msg_type {
	device_msg_None = 0,	  /**< No/invalid message. */
	device_msg_ImuQuaternion, /**< ::imu_quaternion_msg_t payload. */
	device_msg_ImuAccel,	  /**< ::imu_accel_msg_t payload (linear acceleration). */
	device_msg_ImuGyro,		  /**< ::imu_gyro_msg_t payload. */
	device_msg_ImuPedometer,  /**< ::imu_pedometer_msg_t payload (step counter/detector). */
	device_msg_ImuGesture,	  /**< ::imu_gesture_msg_t payload (wrist/motion gestures). */
	device_msg_ImuActivity,	  /**< ::imu_activity_msg_t payload (activity recognition). */
	device_msg_Ppg,			  /**< ::ppg_msg_t payload. */
	device_msg_Temperature,	  /**< ::temperature_msg_t payload. */
	device_msg_Touch,		  /**< ::touch_msg_t payload (touch position/presence). */
	device_msg_TouchGesture,  /**< ::touch_gesture_msg_t payload (discrete touch gestures). */
	device_msg_Battery,		  /**< ::battery_msg_t payload (fuel gauge). */
	device_msg_Charger,		  /**< ::charger_msg_t payload. */
	device_msg_Regulator,	  /**< ::regulator_msg_t payload (rail on/off + VOUT). */
	device_msg_Count,		  /**< Number of valid message types. */
};

/**
 * @brief IMU gesture types.
 * @details Contract-level mirror of the BHI360 wrist-gesture-detect output. The
 *          values match the sensor-hub encoding. Interpret ::imu_gesture_msg_t.gesture
 *          with this enum when the message comes from the wrist-gesture-detect
 *          source (see @c sensor_id); other gesture sources (no-motion, wrist
 *          wear) report a detection rather than a gesture code.
 */
enum imu_gesture_type {
	imu_gesture_None = 0x00,			 /**< No gesture. */
	imu_gesture_WristShakeJiggle = 0x03, /**< Wrist shake / jiggle. */
	imu_gesture_FlickIn = 0x04,			 /**< Wrist flick in. */
	imu_gesture_FlickOut = 0x05,		 /**< Wrist flick out. */
};

/**
 * @brief IMU activity-recognition types.
 * @details Contract-level mirror of the BHI360 activity classes. The device
 *          manager decodes the sensor hub's packed activity word and publishes
 *          one ::imu_activity_msg_t per activity transition, so each message
 *          names a single activity via this enum rather than a combined bit mask.
 */
enum imu_activity_type {
	imu_activity_Still = 0, /**< Still / stationary. */
	imu_activity_Walking,	/**< Walking. */
	imu_activity_Running,	/**< Running. */
	imu_activity_OnBicycle, /**< On a bicycle. */
	imu_activity_InVehicle, /**< In a vehicle. */
	imu_activity_Tilting,	/**< Tilting. */
};

/**
 * @brief IMU activity transition direction.
 * @details Whether the activity named in an ::imu_activity_msg_t just started or
 *          just ended.
 */
enum imu_activity_transition {
	imu_activity_transition_Ended = 0, /**< The activity ended. */
	imu_activity_transition_Started,   /**< The activity started. */
};

/**
 * @brief Touch gesture types.
 * @details Normalized touch-gesture identifiers decoded from the touch
 *          controller (mirrors the touch driver's gesture events). Used for
 *          ::touch_gesture_msg_t.gesture.
 */
enum touch_gesture_type {
	touch_gesture_None = 0,			 /**< No gesture. */
	touch_gesture_SingleClick,		 /**< Single click / tap. */
	touch_gesture_ClickAndHold,		 /**< Click and hold. */
	touch_gesture_DoubleClick,		 /**< Double click / tap. */
	touch_gesture_DownSwipe,		 /**< Downward swipe. */
	touch_gesture_DownSwipeAndHold,	 /**< Downward swipe and hold. */
	touch_gesture_RightSwipe,		 /**< Rightward swipe. */
	touch_gesture_RightSwipeAndHold, /**< Rightward swipe and hold. */
	touch_gesture_UpSwipe,			 /**< Upward swipe. */
	touch_gesture_UpSwipeAndHold,	 /**< Upward swipe and hold. */
	touch_gesture_LeftSwipe,		 /**< Leftward swipe. */
	touch_gesture_LeftSwipeAndHold,	 /**< Leftward swipe and hold. */
};

/**
 * @brief IMU orientation (quaternion) message.
 * @details Mirrors the BHI360 rotation-vector output; components are fixed-point
 *          as produced by the sensor hub.
 */
struct imu_quaternion_msg_t {
	time_t timestamp;  /**< Microseconds since the Unix epoch. */
	int16_t x;		   /**< Quaternion X component (fixed-point). */
	int16_t y;		   /**< Quaternion Y component (fixed-point). */
	int16_t z;		   /**< Quaternion Z component (fixed-point). */
	int16_t w;		   /**< Quaternion W component (fixed-point). */
	uint16_t accuracy; /**< Estimation accuracy. */
};

/**
 * @brief IMU linear-acceleration message.
 */
struct imu_accel_msg_t {
	time_t timestamp; /**< Microseconds since the Unix epoch. */
	int16_t x;		  /**< X-axis acceleration (fixed-point). */
	int16_t y;		  /**< Y-axis acceleration (fixed-point). */
	int16_t z;		  /**< Z-axis acceleration (fixed-point). */
};

/**
 * @brief IMU angular-rate (gyroscope) message.
 */
struct imu_gyro_msg_t {
	time_t timestamp; /**< Microseconds since the Unix epoch. */
	int16_t x;		  /**< X-axis angular velocity (fixed-point). */
	int16_t y;		  /**< Y-axis angular velocity (fixed-point). */
	int16_t z;		  /**< Z-axis angular velocity (fixed-point). */
};

/**
 * @brief IMU pedometer (step counter / detector) message.
 * @details From the BHI360 step-counter/detector virtual sensors.
 */
struct imu_pedometer_msg_t {
	time_t timestamp;	 /**< Microseconds since the Unix epoch. */
	uint8_t sensor_id;	 /**< Firmware-assigned source virtual-sensor ID. */
	uint32_t step_count; /**< Cumulative step count. */
	bool step_detected;	 /**< True when a step was detected in this update. */
};

/**
 * @brief IMU gesture message.
 * @details From the BHI360 gesture virtual sensors (for example wrist gesture,
 *          wrist wear, no-motion). @c gesture is the firmware gesture-type code.
 */
struct imu_gesture_msg_t {
	time_t timestamp;			   /**< Microseconds since the Unix epoch. */
	uint8_t sensor_id;			   /**< Firmware-assigned source virtual-sensor ID. */
	enum imu_gesture_type gesture; /**< Decoded gesture type (see ::imu_gesture_type). */
};

/**
 * @brief IMU activity-recognition message.
 * @details From the BHI360 activity-recognition virtual sensor. One message
 *          reports a single activity transition; the device manager splits the
 *          sensor hub's packed activity word into one message per set bit.
 */
struct imu_activity_msg_t {
	time_t timestamp;						 /**< Microseconds since the Unix epoch. */
	uint8_t sensor_id;						 /**< Firmware-assigned source virtual-sensor ID. */
	enum imu_activity_type activity;		 /**< Which activity (see ::imu_activity_type). */
	enum imu_activity_transition transition; /**< Whether it started or ended. */
};

/**
 * @brief PPG (optical) message.
 * @details One decoded multi-channel PPG sample. Channels not active in the
 *          current acquisition mode are reported as zero. Counts are raw 18-bit
 *          ADC values right-justified in the 32-bit fields.
 */
struct ppg_msg_t {
	time_t timestamp; /**< Microseconds since the Unix epoch. */
	uint32_t ir;	  /**< IR channel counts, or 0 if inactive. */
	uint32_t red;	  /**< Red channel counts, or 0 if inactive. */
	uint32_t green;	  /**< Green channel counts, or 0 if inactive. */
};

/**
 * @brief Skin-temperature message.
 */
struct temperature_msg_t {
	time_t timestamp;			/**< Microseconds since the Unix epoch. */
	int32_t temperature_mdeg_c; /**< Temperature in milli-degrees Celsius. */
};

/**
 * @brief Touch position / presence message.
 * @details Continuous touch state decoded from the touch controller. Discrete
 *          gestures are reported separately as ::touch_gesture_msg_t.
 */
struct touch_msg_t {
	time_t timestamp;	 /**< Microseconds since the Unix epoch. */
	bool touched;		 /**< True while a touch is present. */
	uint16_t x;			 /**< Touch X coordinate (valid when @c touched). */
	uint16_t y;			 /**< Touch Y coordinate (valid when @c touched). */
	uint8_t touch_state; /**< Raw touch-state register byte. */
};

/**
 * @brief Touch gesture message.
 * @details One discrete touch gesture (tap, double-tap, click-and-hold, swipes,
 *          ...). @c gesture is the decoded identifier from the touch driver's
 *          event enum; @c gesture_state is the raw controller gesture code.
 */
struct touch_gesture_msg_t {
	time_t timestamp;				 /**< Microseconds since the Unix epoch. */
	enum touch_gesture_type gesture; /**< Decoded gesture type (see ::touch_gesture_type). */
	uint8_t gesture_state;			 /**< Raw gesture-state register byte. */
};

/**
 * @brief Battery fuel-gauge message.
 * @details Decoded snapshot from the fuel gauge. Signed currents/powers are
 *          negative on discharge.
 */
struct battery_msg_t {
	time_t timestamp;			  /**< Microseconds since the Unix epoch. */
	int32_t temperature_ddeg_c;	  /**< Battery temperature in tenths of a degree Celsius. */
	int32_t voltage_mv;			  /**< Cell voltage in millivolts. */
	int32_t average_current_ma;	  /**< Average current in milliamperes (signed). */
	int32_t average_power_mw;	  /**< Average power in milliwatts (signed). */
	int32_t state_of_charge_dpct; /**< State of charge in tenths of a percent. */
	int32_t nominal_available_capacity_mah; /**< Nominal available capacity in mAh. */
	int32_t full_capacity_mah;				/**< Full available capacity in mAh. */
	int32_t remaining_capacity_mah;			/**< Remaining capacity in mAh. */
	bool learning_in_progress; /**< True while the gauge is still qualifying capacity. */
};

/**
 * @brief Charger status message.
 * @details Normalized charger conditions decoded from the charger driver. @c
 *          flags carries the driver's complete packed status word for consumers
 *          that need the full bit set; the booleans expose the most commonly used
 *          conditions without requiring the charger driver header.
 */
struct charger_msg_t {
	time_t timestamp;	  /**< Microseconds since the Unix epoch. */
	time_t last_irq_time; /**< Timestamp of the last INT assertion, or -1 if none. */
	uint32_t flags;		  /**< Complete packed charger-state word from the driver. */
	bool power_good;	  /**< VIN is power-good. */
	bool charging;		  /**< Constant-current or constant-voltage charging. */
	bool charged;		  /**< Charge cycle is complete. */
	bool fault;			  /**< At least one charger fault is latched. */
};

/**
 * @brief Voltage-regulator status message.
 * @details Decoded snapshot of a managed regulator rail (for example the
 *          TPSM83102 buck converter). Emitted on rail enable/disable and on
 *          output-voltage setpoint changes. Regulator lifecycle events do not
 *          carry a device timestamp, so @c timestamp is stamped by the device
 *          manager when it translates the event.
 */
struct regulator_msg_t {
	time_t timestamp; /**< Microseconds since the Unix epoch (stamped by the manager). */
	uint32_t vout_uv; /**< Output-voltage setpoint in microvolts. */
	bool enabled;	  /**< True when the rail is enabled. */
};

/**
 * @brief Tagged device-manager message envelope.
 * @details A single value type able to carry any device message, for use on one
 *          shared channel. @c type selects the valid @c payload member and @c
 *          device_id identifies the producing device (its generated
 *          `<LABEL>_DEVICE_ID`).
 */
struct device_msg_t {
	uint32_t device_id;		   /**< Producing device's generated `<LABEL>_DEVICE_ID`. */
	enum device_msg_type type; /**< Selects the valid @c payload member. */
	union {
		struct imu_quaternion_msg_t
			imu_quaternion;				  /**< Valid when @c type == ::device_msg_ImuQuaternion. */
		struct imu_accel_msg_t imu_accel; /**< Valid when @c type == ::device_msg_ImuAccel. */
		struct imu_gyro_msg_t imu_gyro;	  /**< Valid when @c type == ::device_msg_ImuGyro. */
		struct imu_pedometer_msg_t
			imu_pedometer; /**< Valid when @c type == ::device_msg_ImuPedometer. */
		struct imu_gesture_msg_t imu_gesture; /**< Valid when @c type == ::device_msg_ImuGesture. */
		struct imu_activity_msg_t
			imu_activity;	  /**< Valid when @c type == ::device_msg_ImuActivity. */
		struct ppg_msg_t ppg; /**< Valid when @c type == ::device_msg_Ppg. */
		struct temperature_msg_t
			temperature;		  /**< Valid when @c type == ::device_msg_Temperature. */
		struct touch_msg_t touch; /**< Valid when @c type == ::device_msg_Touch. */
		struct touch_gesture_msg_t
			touch_gesture;			  /**< Valid when @c type == ::device_msg_TouchGesture. */
		struct battery_msg_t battery; /**< Valid when @c type == ::device_msg_Battery. */
		struct charger_msg_t charger; /**< Valid when @c type == ::device_msg_Charger. */
		struct regulator_msg_t regulator; /**< Valid when @c type == ::device_msg_Regulator. */
	} payload;
};

/** @} */

#endif // SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_MESSAGES_H_
