/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file touch_lbs.h
 * @brief SensWear custom BLE touch and touch-configuration services.
 *
 * @defgroup senswear_ble_touch SensWear BLE touch service
 * @ingroup io_interfaces
 * @{
 *
 * The touch BLE service exposes MTCH6102 position and gesture messages from the
 * device manager. Its synchronous zbus listeners update readable caches and
 * copy pending notifications into a bounded queue. The system workqueue sends
 * them without waiting for Bluetooth transmit buffers, so a congested link
 * cannot block device-event processing. Notifications are best-effort: a full
 * queue or unavailable transmit buffer drops the notification, while readable
 * caches retain the latest sample. No service thread is required. Runtime
 * acquisition control is exposed through a custom configuration service that
 * forwards writes to the device manager.
 *
 * @section senswear_ble_touch_payloads Payload units
 *
 * Touch timestamps are Unix time in microseconds. X is the host slider
 * coordinate (0..896, connector to tip, 64 units per 3 mm electrode pitch).
 * The legacy Y slot is reserved zero. Gesture identifiers retain their wire
 * encodings; @c gesture_state carries a host-generated MTCH6102 gesture code.
 * Raw touch_state remains a controller diagnostic; use touched for contact.
 * UUIDs, properties and packed layouts are unchanged.
 *
 * @section senswear_ble_touch_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         touch_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     touch_lbs_clear_conn();
 * }
 *
 * // After device_manager_start() and after MTCH6102 is configured:
 * if (touch_lbs_streams_ready()) {
 *     int ret = touch_lbs_register_streams();
 *     if (ret != 0) {
 *         LOG_WRN("Touch BLE stream registration failed: %d", ret);
 *     }
 * }
 * @endcode
 */

#ifndef TOUCH_LBS_H_
#define TOUCH_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

/** Custom touch data service UUID value. */
#define BT_UUID_LBS_TOUCH_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x33a5eb3f, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
/** Touch-state characteristic UUID value. */
#define BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL \
	BT_UUID_128_ENCODE(0x33a5eb42, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
/** Gesture-state characteristic UUID value. */
#define BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL \
	BT_UUID_128_ENCODE(0x33a5eb43, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
/** Raw touch data characteristic UUID value. */
#define BT_UUID_LBS_TOUCH_RAW_DATA_VAL \
	BT_UUID_128_ENCODE(0x33a5eb44, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

/** Custom touch configuration service UUID value. */
#define BT_UUID_LBS_TOUCH_CONFIG_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x33a5eb50, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
/** Sampling-enable characteristic UUID value. */
#define BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE_VAL \
	BT_UUID_128_ENCODE(0x33a5eb51, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

#define BT_UUID_LBS_TOUCH_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_SERVICE_VAL)
#define BT_UUID_LBS_TOUCH_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL)
#define BT_UUID_LBS_GESTURE_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL)
#define BT_UUID_LBS_RAW_DATA BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_RAW_DATA_VAL)

#define BT_UUID_LBS_TOUCH_CONFIG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_CONFIG_SERVICE_VAL)
#define BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE \
	BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE_VAL)

/**
 * @brief BLE touch-position payload.
 */
struct touch_lbs_touch_state {
	time_t timestamp; /**< Unix timestamp in microseconds. */
	bool touched;	  /**< True while a touch is present. */
	uint16_t x;		  /**< Slider X coordinate, 0..896. */
	uint16_t y;		  /**< Reserved zero for the linear strip. */
} __packed;

/**
 * @brief BLE touch-gesture payload.
 */
struct touch_lbs_gesture_state {
	time_t timestamp;	   /**< Unix timestamp in microseconds. */
	uint8_t gesture;	   /**< Normalized device-manager gesture code. */
	uint8_t gesture_state; /**< Host gesture in MTCH6102 byte encoding. */
} __packed;

/**
 * @brief Attach the active BLE connection to the touch service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void touch_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the touch service.
 */
void touch_lbs_clear_conn(void);

/**
 * @brief Report whether all touch device-manager streams are ready.
 *
 * @retval true Touch position and gesture streams are ready.
 * @retval false At least one required touch stream is not ready.
 */
bool touch_lbs_streams_ready(void);

/**
 * @brief Register the touch BLE listener on touch device-manager streams.
 * @details Idempotent. Must be called after MTCH6102 has been configured and
 *          the device manager has been started.
 *
 * @retval 0 All stream registrations are complete.
 * @retval -ENODEV A stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno propagated by device_manager_stream_register().
 */
int touch_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
