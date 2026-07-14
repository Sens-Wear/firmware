#include <string.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/timeutil.h>

#include "device_manager.h"
#include "time_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_TIME_BLUETOOTH_LOGGER);

#define TIME_LBS_REFERENCE_TIME_SOURCE_UNKNOWN 0
#define TIME_LBS_REFERENCE_TIME_ACCURACY_UNKNOWN 255
#define TIME_LBS_REFERENCE_TIME_AGE_UNKNOWN 255

static struct bt_conn* time_lbs_conn;
static bool notify_current_time_enabled;

static struct time_lbs_local_time_information local_time_cache = {
	.time_zone = -128,
	.dst_offset = 255,
};

static struct time_lbs_reference_time_information reference_time_cache = {
	.source = TIME_LBS_REFERENCE_TIME_SOURCE_UNKNOWN,
	.accuracy = TIME_LBS_REFERENCE_TIME_ACCURACY_UNKNOWN,
	.days_since_update = TIME_LBS_REFERENCE_TIME_AGE_UNKNOWN,
	.hours_since_update = TIME_LBS_REFERENCE_TIME_AGE_UNKNOWN,
};

static bool is_leap_year(uint16_t year) {
	return ((year % 4U) == 0U && (year % 100U) != 0U) || ((year % 400U) == 0U);
}

static uint8_t days_in_month(uint16_t year, uint8_t month) {
	static const uint8_t days_by_month[] = {
		31, 28, 31, 30, 31, 30,
		31, 31, 30, 31, 30, 31,
	};

	if (month < 1U || month > 12U) {
		return 0U;
	}

	if (month == 2U && is_leap_year(year)) {
		return 29U;
	}

	return days_by_month[month - 1U];
}

static bool current_time_from_unix(time_t unix_seconds, struct time_lbs_current_time* current_time) {
	struct tm tm_utc;

	if (current_time == NULL || unix_seconds == (time_t) -1 ||
	    gmtime_r(&unix_seconds, &tm_utc) == NULL) {
		return false;
	}

	current_time->exact_time_256.year = sys_cpu_to_le16((uint16_t) (tm_utc.tm_year + 1900));
	current_time->exact_time_256.month = (uint8_t) (tm_utc.tm_mon + 1);
	current_time->exact_time_256.day = (uint8_t) tm_utc.tm_mday;
	current_time->exact_time_256.hours = (uint8_t) tm_utc.tm_hour;
	current_time->exact_time_256.minutes = (uint8_t) tm_utc.tm_min;
	current_time->exact_time_256.seconds = (uint8_t) tm_utc.tm_sec;
	current_time->exact_time_256.day_of_week = (tm_utc.tm_wday == 0) ? 7 : (uint8_t) tm_utc.tm_wday;
	current_time->exact_time_256.fractions256 = 0;
	current_time->adjust_reason = 0;

	return true;
}

static bool current_time_to_unix(const struct time_lbs_current_time* current_time, time_t* unix_seconds) {
	if (current_time == NULL || unix_seconds == NULL) {
		return false;
	}

	uint16_t year = sys_le16_to_cpu(current_time->exact_time_256.year);
	uint8_t month = current_time->exact_time_256.month;
	uint8_t day = current_time->exact_time_256.day;
	uint8_t hours = current_time->exact_time_256.hours;
	uint8_t minutes = current_time->exact_time_256.minutes;
	uint8_t seconds = current_time->exact_time_256.seconds;

	if (year < 1582 || month < 1 || month > 12 || day < 1 ||
	    day > days_in_month(year, month) || hours > 23 || minutes > 59 || seconds > 59) {
		return false;
	}

	struct tm tm_utc = {
		.tm_year = (int) year - 1900,
		.tm_mon = (int) month - 1,
		.tm_mday = day,
		.tm_hour = hours,
		.tm_min = minutes,
		.tm_sec = seconds,
		.tm_isdst = 0,
	};

	time_t converted = timeutil_timegm(&tm_utc);

	if (converted == (time_t) -1) {
		return false;
	}

	*unix_seconds = converted;
	return true;
}

static void current_time_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_current_time_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Current time notifications %s", notify_current_time_enabled ? "enabled" : "disabled");
}

static ssize_t read_current_time(struct bt_conn* conn,
				 const struct bt_gatt_attr* attr,
				 void* buf,
				 uint16_t len,
				 uint16_t offset) {
	struct time_lbs_current_time current_time;
	time_t now = time(NULL);

	if (!current_time_from_unix(now, &current_time)) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_time, sizeof(current_time));
}

static ssize_t write_current_time(struct bt_conn* conn,
				  const struct bt_gatt_attr* attr,
				  const void* buf,
				  uint16_t len,
				  uint16_t offset,
				  uint8_t flags) {
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(struct time_lbs_current_time)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	struct time_lbs_current_time current_time;
	time_t unix_seconds;

	memcpy(&current_time, buf, sizeof(current_time));

	if (!current_time_to_unix(&current_time, &unix_seconds)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	int ret = device_manager_set_rtc_time(unix_seconds);

	if (ret != 0) {
		LOG_WRN("RTC time update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	LOG_INF("Current time written: %lld", (long long) unix_seconds);

	if (notify_current_time_enabled) {
		struct bt_conn* notify_conn = time_lbs_conn != NULL ? time_lbs_conn : conn;

		(void) bt_gatt_notify(notify_conn, attr, &current_time, sizeof(current_time));
	}

	return len;
}

static ssize_t read_local_time_information(struct bt_conn* conn,
					   const struct bt_gatt_attr* attr,
					   void* buf,
					   uint16_t len,
					   uint16_t offset) {
	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &local_time_cache,
				 sizeof(local_time_cache));
}

static ssize_t read_reference_time_information(struct bt_conn* conn,
					       const struct bt_gatt_attr* attr,
					       void* buf,
					       uint16_t len,
					       uint16_t offset) {
	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &reference_time_cache,
				 sizeof(reference_time_cache));
}

BT_GATT_SERVICE_DEFINE(
	time_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_CURRENT_TIME_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_CURRENT_TIME,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_current_time,
			       write_current_time,
			       NULL),
	BT_GATT_CCC(current_time_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LOCAL_TIME_INFORMATION,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_local_time_information,
			       NULL,
			       &local_time_cache),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_REFERENCE_TIME_INFORMATION,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_reference_time_information,
			       NULL,
			       &reference_time_cache));

void time_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (time_lbs_conn != NULL) {
		bt_conn_unref(time_lbs_conn);
	}

	time_lbs_conn = bt_conn_ref(conn);
	LOG_INF("Time BLE connection attached");
}

void time_lbs_clear_conn(void) {
	if (time_lbs_conn != NULL) {
		bt_conn_unref(time_lbs_conn);
		time_lbs_conn = NULL;
		LOG_INF("Time BLE connection cleared");
	}
}
