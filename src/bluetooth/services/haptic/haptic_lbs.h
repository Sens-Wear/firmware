/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file haptic_lbs.h
 * @brief SenseWear custom BLE haptic service.
 *
 * @defgroup sensewear_ble_haptic SenseWear BLE haptic service
 * @ingroup io_interfaces
 * @{
 *
 * The haptic BLE service exposes a custom pattern-write characteristic. A client
 * writes a compact RTP pattern packet; the service validates the packet and
 * forwards it to the device manager haptic API when the haptic shield is built.
 *
 * Pattern packet format:
 * - byte 0: ::HAPTIC_LBS_PATTERN_VERSION
 * - byte 1: flags, currently 0
 * - bytes 2-3: frame count, little-endian
 * - repeated frames: uint16 duration_ms, uint8 amplitude
 *
 * @section sensewear_ble_haptic_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         haptic_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     haptic_lbs_clear_conn();
 * }
 *
 * // One 50 ms frame at amplitude 128:
 * const uint8_t pattern[] = {
 *     HAPTIC_LBS_PATTERN_VERSION, 0,
 *     1, 0,
 *     50, 0, 128,
 * };
 * // A BLE client writes pattern[] to BT_UUID_LBS_HAPTIC_PATTERN_CONF.
 * @endcode
 */

#ifndef HAPTIC_LBS_H_
#define HAPTIC_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#include "device_manager.h"

/** Custom haptic service UUID value. */
#define BT_UUID_LBS_HAPTIC_SERVICE_VAL BT_UUID_128_ENCODE(0xdaa05e91, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
/** Haptic pattern characteristic UUID value. */
#define BT_UUID_LBS_HAPTIC_PATTERN_VAL BT_UUID_128_ENCODE(0xdaa05e92, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
/** Custom haptic service UUID. */
#define BT_UUID_LBS_HAPTIC_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_SERVICE_VAL)
/** Haptic pattern characteristic UUID. */
#define BT_UUID_LBS_HAPTIC_PATTERN_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_PATTERN_VAL)

/** Current haptic pattern packet version accepted by the service. */
#define HAPTIC_LBS_PATTERN_VERSION 1U

#if defined(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
/** Maximum number of frames accepted by one haptic pattern write. */
#define HAPTIC_LBS_MAX_FRAMES DEVICE_MANAGER_HAPTIC_RTP_MAX
#else
/** Fallback maximum used when the haptic shield is not compiled. */
#define HAPTIC_LBS_MAX_FRAMES 32U
#endif

/**
 * @brief Attach the active BLE connection to the haptic service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void haptic_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the haptic service.
 */
void haptic_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
