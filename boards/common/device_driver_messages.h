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
	DEVICE_MSG_NONE = 0,		 /**< No/invalid message. */
	DEVICE_MSG_IMU_QUATERNION,	 /**< ::imu_quaternion_msg_t payload. */
	DEVICE_MSG_IMU_ACCEL,		 /**< ::imu_accel_msg_t payload (linear acceleration). */
	DEVICE_MSG_IMU_GYRO,		 /**< ::imu_gyro_msg_t payload. */
	DEVICE_MSG_PPG,				 /**< ::ppg_msg_t payload. */
	DEVICE_MSG_TEMPERATURE,		 /**< ::temperature_msg_t payload. */
	DEVICE_MSG_TOUCH,			 /**< ::touch_msg_t payload. */
	DEVICE_MSG_BATTERY,			 /**< ::battery_msg_t payload (fuel gauge). */
	DEVICE_MSG_CHARGER,			 /**< ::charger_msg_t payload. */
	DEVICE_MSG_COUNT,			 /**< Number of valid message types. */
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
 * @brief Touch / gesture message.
 * @details Flattened decode of a touch controller sample. @c event carries the
 *          decoded touch/gesture identifier from the touch driver's event enum
 *          (negative for "invalid"); the raw position and status bytes are
 *          included for consumers that want the unmodified state.
 */
struct touch_msg_t {
	time_t timestamp;	  /**< Microseconds since the Unix epoch. */
	int32_t event;		  /**< Decoded touch/gesture event identifier. */
	bool touched;		  /**< True while a touch is present. */
	uint16_t x;			  /**< Touch X coordinate (valid when @c touched). */
	uint16_t y;			  /**< Touch Y coordinate (valid when @c touched). */
	uint8_t touch_state;  /**< Raw touch-state register byte. */
	uint8_t gesture_state; /**< Raw gesture-state register byte. */
};

/**
 * @brief Battery fuel-gauge message.
 * @details Decoded snapshot from the fuel gauge. Signed currents/powers are
 *          negative on discharge.
 */
struct battery_msg_t {
	time_t timestamp;					/**< Microseconds since the Unix epoch. */
	int32_t temperature_ddeg_c;			/**< Battery temperature in tenths of a degree Celsius. */
	int32_t voltage_mv;					/**< Cell voltage in millivolts. */
	int32_t average_current_ma;			/**< Average current in milliamperes (signed). */
	int32_t average_power_mw;			/**< Average power in milliwatts (signed). */
	int32_t state_of_charge_dpct;		/**< State of charge in tenths of a percent. */
	int32_t nominal_available_capacity_mah; /**< Nominal available capacity in mAh. */
	int32_t full_capacity_mah;			/**< Full available capacity in mAh. */
	int32_t remaining_capacity_mah;		/**< Remaining capacity in mAh. */
	bool learning_in_progress;			/**< True while the gauge is still qualifying capacity. */
};

/**
 * @brief Charger status message.
 * @details Normalized charger conditions decoded from the charger driver. @c
 *          flags carries the driver's complete packed status word for consumers
 *          that need the full bit set; the booleans expose the most commonly used
 *          conditions without requiring the charger driver header.
 */
struct charger_msg_t {
	time_t timestamp;	 /**< Microseconds since the Unix epoch. */
	time_t last_irq_time; /**< Timestamp of the last INT assertion, or -1 if none. */
	uint32_t flags;		 /**< Complete packed charger-state word from the driver. */
	bool power_good;	 /**< VIN is power-good. */
	bool charging;		 /**< Constant-current or constant-voltage charging. */
	bool charged;		 /**< Charge cycle is complete. */
	bool fault;			 /**< At least one charger fault is latched. */
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
		struct imu_quaternion_msg_t imu_quaternion; /**< Valid when @c type == ::DEVICE_MSG_IMU_QUATERNION. */
		struct imu_accel_msg_t imu_accel;			/**< Valid when @c type == ::DEVICE_MSG_IMU_ACCEL. */
		struct imu_gyro_msg_t imu_gyro;				/**< Valid when @c type == ::DEVICE_MSG_IMU_GYRO. */
		struct ppg_msg_t ppg;						/**< Valid when @c type == ::DEVICE_MSG_PPG. */
		struct temperature_msg_t temperature;		/**< Valid when @c type == ::DEVICE_MSG_TEMPERATURE. */
		struct touch_msg_t touch;					/**< Valid when @c type == ::DEVICE_MSG_TOUCH. */
		struct battery_msg_t battery;				/**< Valid when @c type == ::DEVICE_MSG_BATTERY. */
		struct charger_msg_t charger;				/**< Valid when @c type == ::DEVICE_MSG_CHARGER. */
	} payload;
};

/** @} */

#endif // SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_MESSAGES_H_
