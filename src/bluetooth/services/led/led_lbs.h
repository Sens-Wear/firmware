/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file led_lbs.h
 * @brief SensWear custom BLE LED service.
 *
 * @defgroup senswear_ble_led SensWear BLE LED service
 * @ingroup io_interfaces
 * @{
 *
 * The LED BLE service exposes a custom read/write color characteristic. The
 * characteristic stores a 32-bit little-endian color value. The current service
 * only owns the GATT-facing state; actuator routing can be wired behind this
 * service without changing the client-facing contract.
 *
 * @section senswear_ble_led_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         led_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     led_lbs_clear_conn();
 * }
 *
 * // No stream registration is required. The service is a static GATT service.
 * @endcode
 */

#ifndef LED_LBS_H_
#define LED_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

/** Custom LED service UUID value. */
#define BT_UUID_LBS_LED_SERVICE_VAL BT_UUID_128_ENCODE(0x3c688942, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
/** LED color characteristic UUID value. */
#define BT_UUID_LBS_LED_COLOR_VAL BT_UUID_128_ENCODE(0x3c688943, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
/** Custom LED service UUID. */
#define BT_UUID_LBS_LED_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_LED_SERVICE_VAL)
/** LED color characteristic UUID. */
#define BT_UUID_LBS_LED_COLOR_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_LED_COLOR_VAL)

/**
 * @brief Attach the active BLE connection to the LED service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void led_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the LED service.
 */
void led_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
