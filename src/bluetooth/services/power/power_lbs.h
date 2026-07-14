/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file power_lbs.h
 * @brief SenseWear BLE Battery Service bridge.
 *
 * @defgroup sensewear_ble_power SenseWear BLE power service
 * @ingroup io_interfaces
 * @{
 *
 * The power BLE service implements the Bluetooth SIG Battery Service (BAS). It
 * subscribes to the device-manager battery and charger streams, maintains a
 * Battery Level characteristic and a Battery Level Status characteristic, and
 * notifies connected clients when values change. The service does not call the
 * fuel-gauge or charger drivers directly.
 *
 * @section sensewear_ble_power_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         power_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     power_lbs_clear_conn();
 * }
 *
 * // After device_manager_start() and after charger/gauge streams are ready:
 * if (power_lbs_streams_ready()) {
 *     int ret = power_lbs_register_streams();
 *     if (ret != 0) {
 *         LOG_WRN("Power BLE stream registration failed: %d", ret);
 *     }
 * }
 * @endcode
 */

#ifndef POWER_LBS_H_
#define POWER_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

/** Standard Bluetooth SIG Battery Service UUID. */
#define BT_UUID_LBS_POWER_SERVICE BT_UUID_BAS
/** Standard BAS Battery Level characteristic UUID. */
#define BT_UUID_LBS_POWER_BATTERY_LEVEL BT_UUID_BAS_BATTERY_LEVEL
/** Standard BAS Battery Level Status characteristic UUID. */
#define BT_UUID_LBS_POWER_BATTERY_LEVEL_STATUS BT_UUID_BAS_BATTERY_LEVEL_STATUS

/**
 * @brief Attach the active BLE connection to the power service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void power_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the power service.
 */
void power_lbs_clear_conn(void);

/**
 * @brief Report whether both power-producing streams are ready.
 *
 * @retval true Battery and charger streams are ready.
 * @retval false At least one power stream is not ready.
 */
bool power_lbs_streams_ready(void);

/**
 * @brief Register the power BLE listener on battery and charger streams.
 * @details Idempotent. Battery and charger registrations are attempted
 *          independently so one stream can register even when the other is not
 *          ready yet.
 *
 * @retval 0 Required stream registrations completed.
 * @retval -ENODEV A stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno propagated by device_manager_stream_register().
 */
int power_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
