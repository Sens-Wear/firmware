/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file imu_lbs.h
 * @brief SenseWear custom BLE IMU and IMU-configuration services.
 *
 * @defgroup sensewear_ble_imu SenseWear BLE IMU service
 * @ingroup io_interfaces
 * @{
 *
 * The IMU BLE service exposes BHI360 data that has already been normalized and
 * published by the device manager. The service does not call the IMU driver
 * directly and does not create a worker thread. After the application has
 * configured and started the device manager, call imu_lbs_register_streams() to
 * attach the service's zbus listener to the IMU streams. Listener callbacks run
 * in the device-manager thread context and update the GATT caches/notifications
 * directly.
 *
 * Two custom GATT services are defined:
 * - IMU data service: quaternion, linear acceleration, gyro, gesture, activity.
 * - IMU config service: physical-stream enable and FIFO drain period.
 *
 * @section sensewear_ble_imu_payloads Payload units
 *
 * All timestamps are Unix time in microseconds, copied from the device-manager
 * messages. Quaternion and vector values use the fixed-point units emitted by
 * the BHI360 driver and normalized by the device manager; this BLE layer only
 * transports the values.
 *
 * @section sensewear_ble_imu_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         imu_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     imu_lbs_clear_conn();
 * }
 *
 * // After device_manager_start() and after the BHI360 stream is ready:
 * if (imu_lbs_streams_ready()) {
 *     int ret = imu_lbs_register_streams();
 *     if (ret != 0) {
 *         LOG_WRN("IMU BLE stream registration failed: %d", ret);
 *     }
 * }
 * @endcode
 */

#ifndef IMU_LBS_H_
#define IMU_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

/** Custom IMU data service UUID value. */
#define BT_UUID_LBS_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x7d2b6c10, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Quaternion characteristic UUID value. */
#define BT_UUID_LBS_IMU_QUAT_VAL BT_UUID_128_ENCODE(0x7d2b6c11, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Linear-acceleration characteristic UUID value. */
#define BT_UUID_LBS_IMU_LACC_VAL BT_UUID_128_ENCODE(0x7d2b6c12, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Gyroscope characteristic UUID value. */
#define BT_UUID_LBS_IMU_GYRO_VAL BT_UUID_128_ENCODE(0x7d2b6c13, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Gesture characteristic UUID value. */
#define BT_UUID_LBS_IMU_GESTURE_VAL BT_UUID_128_ENCODE(0x7d2b6c14, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Activity characteristic UUID value. */
#define BT_UUID_LBS_IMU_ACTIVITY_VAL BT_UUID_128_ENCODE(0x7d2b6c15, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)

/** Custom IMU configuration service UUID value. */
#define BT_UUID_LBS_IMU_CONFIG_SERVICE_VAL BT_UUID_128_ENCODE(0x7d2b6c20, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** Physical-stream enable characteristic UUID value. */
#define BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE_VAL BT_UUID_128_ENCODE(0x7d2b6c21, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
/** FIFO drain-period characteristic UUID value. */
#define BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD_VAL BT_UUID_128_ENCODE(0x7d2b6c22, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)

#define BT_UUID_LBS_IMU_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_SERVICE_VAL)
#define BT_UUID_LBS_IMU_QUAT BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_QUAT_VAL)
#define BT_UUID_LBS_IMU_LACC BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_LACC_VAL)
#define BT_UUID_LBS_IMU_GYRO BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_GYRO_VAL)
#define BT_UUID_LBS_IMU_GESTURE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_GESTURE_VAL)
#define BT_UUID_LBS_IMU_ACTIVITY BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_ACTIVITY_VAL)

#define BT_UUID_LBS_IMU_CONFIG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_SERVICE_VAL)
#define BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE_VAL)
#define BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD_VAL)

/**
 * @brief Three-axis fixed-point IMU vector payload.
 * @details Used by the linear-acceleration and gyroscope characteristics.
 */
struct imu_lbs_vec3 {
	time_t timestamp; /**< Unix timestamp in microseconds. */
	int16_t x;		  /**< X-axis sample in the device-manager IMU units. */
	int16_t y;		  /**< Y-axis sample in the device-manager IMU units. */
	int16_t z;		  /**< Z-axis sample in the device-manager IMU units. */
} __packed;

/**
 * @brief Quaternion payload exposed by the IMU quaternion characteristic.
 */
struct imu_lbs_quat {
	time_t timestamp; /**< Unix timestamp in microseconds. */
	int16_t x;		  /**< Quaternion X component in the BHI360 fixed-point format. */
	int16_t y;		  /**< Quaternion Y component in the BHI360 fixed-point format. */
	int16_t z;		  /**< Quaternion Z component in the BHI360 fixed-point format. */
	int16_t w;		  /**< Quaternion W component in the BHI360 fixed-point format. */
	uint16_t accuracy; /**< BHI360-reported quaternion accuracy estimate. */
} __packed;

/**
 * @brief Discrete IMU gesture payload.
 */
struct imu_lbs_gesture {
	time_t timestamp; /**< Unix timestamp in microseconds. */
	uint8_t sensor_id; /**< BHI360 virtual sensor identifier that emitted the gesture. */
	uint8_t gesture;   /**< Device-manager gesture code. */
} __packed;

/**
 * @brief Discrete IMU activity payload.
 */
struct imu_lbs_activity {
	time_t timestamp; /**< Unix timestamp in microseconds. */
	uint8_t sensor_id;  /**< BHI360 virtual sensor identifier that emitted the activity. */
	uint8_t activity;   /**< Device-manager activity code. */
	uint8_t transition; /**< Device-manager activity-transition code. */
} __packed;

/**
 * @brief Attach the active BLE connection to the IMU service.
 * @details The connection is referenced by the service and used for GATT
 *          notifications. Passing NULL is ignored.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void imu_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the IMU service.
 * @details Drops the service's connection reference, if one is held.
 */
void imu_lbs_clear_conn(void);

/**
 * @brief Report whether all IMU device-manager streams are ready.
 *
 * @retval true Quaternion, acceleration, gyro, gesture, and activity streams are ready.
 * @retval false At least one required IMU stream is not ready.
 */
bool imu_lbs_streams_ready(void);

/**
 * @brief Register the IMU BLE listener on all IMU device-manager streams.
 * @details Idempotent. Must be called after the BHI360 device has been
 *          configured and the device manager has been started.
 *
 * @retval 0 All stream registrations are complete.
 * @retval -ENODEV A stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno propagated by device_manager_stream_register().
 */
int imu_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
