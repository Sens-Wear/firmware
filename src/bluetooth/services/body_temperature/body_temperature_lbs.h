/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file body_temperature_lbs.h
 * @brief SensWear BLE Health Thermometer body-temperature service.
 *
 * @defgroup senswear_ble_body_temperature SensWear BLE body temperature service
 * @ingroup io_interfaces
 * @{
 *
 * The body-temperature BLE service implements the Bluetooth SIG Health
 * Thermometer Service (HTS). The Measurement characteristic is indicated from
 * the device-manager temperature stream. The Measurement Interval characteristic
 * is writable; writes are translated to the device-manager body-temperature
 * update period. The service is compiled only for the temperature shield.
 *
 * @section senswear_ble_body_temperature_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         body_temperature_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     body_temperature_lbs_clear_conn();
 * }
 *
 * // After device_manager_start() and after MAX30208 is configured:
 * if (body_temperature_lbs_stream_ready()) {
 *     int ret = body_temperature_lbs_register_stream();
 *     if (ret != 0) {
 *         LOG_WRN("Temperature BLE stream registration failed: %d", ret);
 *     }
 * }
 * @endcode
 */

#ifndef BODY_TEMPERATURE_LBS_H_
#define BODY_TEMPERATURE_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

/** Standard Bluetooth SIG Health Thermometer Service UUID. */
#define BT_UUID_LBS_BODY_TEMPERATURE_SERVICE BT_UUID_HTS
/** Standard HTS Temperature Measurement characteristic UUID. */
#define BT_UUID_LBS_BODY_TEMPERATURE_MEASUREMENT BT_UUID_HTS_MEASUREMENT
/** Standard HTS Temperature Type characteristic UUID. */
#define BT_UUID_LBS_BODY_TEMPERATURE_TYPE BT_UUID_HTS_TEMP_TYP
/** Standard HTS Measurement Interval characteristic UUID. */
#define BT_UUID_LBS_BODY_TEMPERATURE_MEASUREMENT_INTERVAL BT_UUID_HTS_INTERVAL

/**
 * @brief Attach the active BLE connection to the body-temperature service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void body_temperature_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the body-temperature service.
 * @details Also clears any in-flight indication state.
 */
void body_temperature_lbs_clear_conn(void);

/**
 * @brief Report whether the temperature device-manager stream is ready.
 *
 * @retval true The temperature stream can accept observer registration.
 * @retval false The MAX30208 producer is not ready.
 */
bool body_temperature_lbs_stream_ready(void);

/**
 * @brief Register the body-temperature BLE listener on the temperature stream.
 * @details Idempotent. Must be called after MAX30208 has been configured and
 *          the device manager has been started.
 *
 * @retval 0 Stream registration is complete.
 * @retval -ENODEV The temperature stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno propagated by device_manager_stream_register().
 */
int body_temperature_lbs_register_stream(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
