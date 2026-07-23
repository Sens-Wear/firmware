/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file time_lbs.h
 * @brief SensWear BLE Current Time Service.
 *
 * @defgroup senswear_ble_time SensWear BLE time service
 * @ingroup io_interfaces
 * @{
 *
 * The time BLE service implements the Bluetooth SIG Current Time Service (CTS).
 * A client may read or write Current Time. Writes are validated, converted to
 * Unix seconds, and forwarded to the device manager so the SensWear RTC is the
 * single owner of wall-clock state. Local Time Information and Reference Time
 * Information are exposed as read-only standard characteristics.
 *
 * @section senswear_ble_time_example Typical usage
 *
 * @code{.c}
 * static void connected(struct bt_conn *conn, uint8_t err)
 * {
 *     if (err == 0) {
 *         time_lbs_set_conn(conn);
 *     }
 * }
 *
 * static void disconnected(struct bt_conn *conn, uint8_t reason)
 * {
 *     ARG_UNUSED(conn);
 *     ARG_UNUSED(reason);
 *     time_lbs_clear_conn();
 * }
 *
 * // No stream registration is required. The GATT service is registered
 * // statically and writes are forwarded to device_manager_set_rtc_time().
 * @endcode
 */

#ifndef TIME_LBS_H_
#define TIME_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

/** Standard Bluetooth SIG Current Time Service UUID value. */
#define BT_UUID_LBS_CURRENT_TIME_SERVICE_VAL 0x1805
/** Standard Current Time characteristic UUID value. */
#define BT_UUID_LBS_CURRENT_TIME_VAL 0x2A2B
/** Standard Local Time Information characteristic UUID value. */
#define BT_UUID_LBS_LOCAL_TIME_INFORMATION_VAL 0x2A0F
/** Standard Reference Time Information characteristic UUID value. */
#define BT_UUID_LBS_REFERENCE_TIME_INFORMATION_VAL 0x2A14

#define BT_UUID_LBS_CURRENT_TIME_SERVICE BT_UUID_DECLARE_16(BT_UUID_LBS_CURRENT_TIME_SERVICE_VAL)
#define BT_UUID_LBS_CURRENT_TIME BT_UUID_DECLARE_16(BT_UUID_LBS_CURRENT_TIME_VAL)
#define BT_UUID_LBS_LOCAL_TIME_INFORMATION BT_UUID_DECLARE_16(BT_UUID_LBS_LOCAL_TIME_INFORMATION_VAL)
#define BT_UUID_LBS_REFERENCE_TIME_INFORMATION BT_UUID_DECLARE_16(BT_UUID_LBS_REFERENCE_TIME_INFORMATION_VAL)

/**
 * @brief Current Time Service Adjust Reason bit flags.
 */
enum time_lbs_adjust_reason {
	TIME_LBS_ADJUST_REASON_MANUAL_TIME_UPDATE = BIT(0), /**< Time changed by manual client update. */
	TIME_LBS_ADJUST_REASON_EXTERNAL_REFERENCE_TIME_UPDATE = BIT(1), /**< External reference changed time. */
	TIME_LBS_ADJUST_REASON_CHANGE_OF_TIME_ZONE = BIT(2), /**< Local time-zone offset changed. */
	TIME_LBS_ADJUST_REASON_CHANGE_OF_DST = BIT(3), /**< Daylight-saving offset changed. */
};

/**
 * @brief Bluetooth CTS Exact Time 256 payload.
 * @details Values are UTC. @c fractions256 is 1/256-second fractions.
 */
struct time_lbs_exact_time_256 {
	uint16_t year;		/**< Gregorian year, little-endian on the wire. */
	uint8_t month;		/**< Month, 1-12. */
	uint8_t day;		/**< Day of month, 1-31 depending on month. */
	uint8_t hours;		/**< Hour, 0-23. */
	uint8_t minutes;	/**< Minute, 0-59. */
	uint8_t seconds;	/**< Second, 0-59. */
	uint8_t day_of_week; /**< Day of week, 1=Monday through 7=Sunday. */
	uint8_t fractions256; /**< Fractions of a second in 1/256-second units. */
} __packed;

/**
 * @brief Bluetooth CTS Current Time characteristic payload.
 */
struct time_lbs_current_time {
	struct time_lbs_exact_time_256 exact_time_256; /**< UTC time fields. */
	uint8_t adjust_reason; /**< Bitmask of enum time_lbs_adjust_reason values. */
} __packed;

/**
 * @brief Bluetooth CTS Local Time Information characteristic payload.
 */
struct time_lbs_local_time_information {
	int8_t time_zone; /**< Offset from UTC in 15-minute increments, -128 if unknown. */
	uint8_t dst_offset; /**< Daylight-saving offset, 255 if unknown. */
} __packed;

/**
 * @brief Bluetooth CTS Reference Time Information characteristic payload.
 */
struct time_lbs_reference_time_information {
	uint8_t source; /**< Reference source identifier, 0 if unknown. */
	uint8_t accuracy; /**< Accuracy in 1/8-second units, 255 if unknown. */
	uint8_t days_since_update; /**< Days since last reference update, 255 if unknown. */
	uint8_t hours_since_update; /**< Hours since last reference update, 255 if unknown. */
} __packed;

/**
 * @brief Attach the active BLE connection to the time service.
 *
 * @param conn Active BLE connection from the application connection callback.
 */
void time_lbs_set_conn(struct bt_conn* conn);

/**
 * @brief Clear the active BLE connection from the time service.
 */
void time_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
