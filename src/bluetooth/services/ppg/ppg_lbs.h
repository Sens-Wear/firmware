/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file ppg_lbs.h
 * @brief SenseWear custom BLE PPG and PPG-configuration services.
 *
 * @defgroup sensewear_ble_ppg SenseWear BLE PPG service
 * @ingroup io_interfaces
 * @{
 *
 * The PPG BLE service publishes MAX30101 samples that arrive through the device
 * manager's PPG zbus stream. The service does not call the MAX30101 driver
 * directly for data and does not own a worker thread. Runtime acquisition
 * control is exposed through a separate configuration service; writes are
 * forwarded to the device manager.
 *
 * @section sensewear_ble_ppg_payloads Payload units
 *
 * Each PPG characteristic reports a single channel as a timestamp/value pair.
 * @c unix_ms is milliseconds since the Unix epoch. @c value is the raw
 * right-justified 18-bit ADC count from the MAX30101 channel.
 *
 * @section sensewear_ble_ppg_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         ppg_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     ppg_lbs_clear_conn();
 * }
 *
 * // After device_manager_start() and after MAX30101 is configured:
 * if (ppg_lbs_stream_ready()) {
 *     int ret = ppg_lbs_register_stream();
 *     if (ret != 0) {
 *         LOG_WRN("PPG BLE stream registration failed: %d", ret);
 *     }
 * }
 * @endcode
 */

#ifndef PPG_LBS_H_
#define PPG_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

/** Custom PPG data service UUID value. */
#define BT_UUID_LBS_PPG_SERVICE_VAL BT_UUID_128_ENCODE(0x029ca54e, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
/** Red-channel characteristic UUID value. */
#define BT_UUID_LBS_PPG_RED_VAL BT_UUID_128_ENCODE(0x029ca551, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
/** IR-channel characteristic UUID value. */
#define BT_UUID_LBS_PPG_IR_VAL BT_UUID_128_ENCODE(0x029ca552, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
/** Green-channel characteristic UUID value. */
#define BT_UUID_LBS_PPG_GREEN_VAL BT_UUID_128_ENCODE(0x029ca553, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)

/** Custom PPG configuration service UUID value. */
#define BT_UUID_LBS_PPG_CONFIG_SERVICE_VAL BT_UUID_128_ENCODE(0x029ca560, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
/** Sampling-enable characteristic UUID value. */
#define BT_UUID_LBS_PPG_CONFIG_SAMPLING_ENABLE_VAL BT_UUID_128_ENCODE(0x029ca561, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
/** Per-sample interrupt cadence characteristic UUID value. */
#define BT_UUID_LBS_PPG_CONFIG_PER_SAMPLE_IRQ_VAL BT_UUID_128_ENCODE(0x029ca562, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)

#define BT_UUID_LBS_PPG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_SERVICE_VAL)
#define BT_UUID_LBS_PPG_RED BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_RED_VAL)
#define BT_UUID_LBS_PPG_IR BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_IR_VAL)
#define BT_UUID_LBS_PPG_GREEN BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_GREEN_VAL)

#define BT_UUID_LBS_PPG_CONFIG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_CONFIG_SERVICE_VAL)
#define BT_UUID_LBS_PPG_CONFIG_SAMPLING_ENABLE BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_CONFIG_SAMPLING_ENABLE_VAL)
#define BT_UUID_LBS_PPG_CONFIG_PER_SAMPLE_IRQ BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_CONFIG_PER_SAMPLE_IRQ_VAL)

/**
 * @brief BLE payload for one PPG channel sample.
 */
struct ppg_sample_notification_t {
	uint64_t unix_ms; /**< Unix timestamp in milliseconds. */
	uint32_t value;   /**< Raw right-justified 18-bit ADC count. */
} __packed;

/**
 * @brief Attach the active BLE connection to the PPG service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void ppg_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the PPG service.
 */
void ppg_lbs_clear_conn(void);

/**
 * @brief Report whether the PPG device-manager stream is ready.
 *
 * @retval true The PPG stream can accept observer registration.
 * @retval false The MAX30101 producer is not ready.
 */
bool ppg_lbs_stream_ready(void);

/**
 * @brief Register the PPG BLE listener on the PPG device-manager stream.
 * @details Idempotent. Must be called after the MAX30101 has been configured
 *          and the device manager has been started.
 *
 * @retval 0 Stream registration is complete.
 * @retval -ENODEV The PPG stream's producing device is not ready.
 * @retval -ENOTSUP Runtime zbus observers are not enabled.
 * @return A negative errno propagated by device_manager_stream_register().
 */
int ppg_lbs_register_stream(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
