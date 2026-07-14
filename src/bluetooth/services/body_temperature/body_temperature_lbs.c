#include <stdint.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>

#include "body_temperature_lbs.h"
#include "device_manager.h"

LOG_MODULE_REGISTER(SENS_WEAR_BODY_TEMPERATURE_BLUETOOTH_LOGGER);

#define BODY_TEMPERATURE_MEASUREMENT_FLAG_TIMESTAMP_PRESENT BIT(1)
#define BODY_TEMPERATURE_MEASUREMENT_FLAG_TYPE_PRESENT BIT(2)
#define BODY_TEMPERATURE_TYPE_BODY 2U
#define BODY_TEMPERATURE_INTERVAL_DISABLED 0U
#define BODY_TEMPERATURE_SECONDS_PER_MINUTE 60U

struct body_temperature_lbs_date_time {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} __packed;

struct body_temperature_lbs_measurement {
	uint8_t flags;
	uint32_t temperature_celsius;
	struct body_temperature_lbs_date_time timestamp;
	uint8_t temperature_type;
} __packed;

static struct bt_conn* body_temperature_lbs_conn;
static bool indicate_temperature_enabled;
static bool indicate_interval_enabled;
static bool indication_in_flight;
static bool temperature_listener_registered;
static uint16_t measurement_interval_sec;
static const uint8_t temperature_type = BODY_TEMPERATURE_TYPE_BODY;
static struct body_temperature_lbs_measurement measurement_cache = {
	.flags = BODY_TEMPERATURE_MEASUREMENT_FLAG_TIMESTAMP_PRESENT |
		 BODY_TEMPERATURE_MEASUREMENT_FLAG_TYPE_PRESENT,
	.temperature_type = BODY_TEMPERATURE_TYPE_BODY,
};
static struct bt_gatt_indicate_params measurement_indicate_params;

static void body_temperature_interval_indicate(void);
static void body_temperature_indicate_measurement(void);
static void body_temperature_indication_complete(struct bt_conn* conn,
						 struct bt_gatt_indicate_params* params,
						 uint8_t err);

static uint32_t body_temperature_ieee11073_float_from_mdeg_c(int32_t temperature_mdeg_c) {
	int32_t mantissa = temperature_mdeg_c / 10;
	uint32_t exponent = 0xFEU;

	if (mantissa > 0x007FFFFF) {
		mantissa = 0x007FFFFF;
	} else if (mantissa < -0x00800000) {
		mantissa = -0x00800000;
	}

	return (exponent << 24) | ((uint32_t) mantissa & 0x00FFFFFFU);
}

static void body_temperature_date_time_from_unix(time_t unix_seconds,
						 struct body_temperature_lbs_date_time* date_time) {
	struct tm tm_utc;

	if (date_time == NULL || unix_seconds == (time_t) -1 ||
	    gmtime_r(&unix_seconds, &tm_utc) == NULL) {
		return;
	}

	date_time->year = sys_cpu_to_le16((uint16_t) (tm_utc.tm_year + 1900));
	date_time->month = (uint8_t) (tm_utc.tm_mon + 1);
	date_time->day = (uint8_t) tm_utc.tm_mday;
	date_time->hours = (uint8_t) tm_utc.tm_hour;
	date_time->minutes = (uint8_t) tm_utc.tm_min;
	date_time->seconds = (uint8_t) tm_utc.tm_sec;
}

static void body_temperature_measurement_ccc_changed(const struct bt_gatt_attr* attr,
						     uint16_t value) {
	ARG_UNUSED(attr);
	indicate_temperature_enabled = (value == BT_GATT_CCC_INDICATE);
	LOG_INF("Body temperature measurement indications %s",
		indicate_temperature_enabled ? "enabled" : "disabled");
}

static void body_temperature_interval_ccc_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	indicate_interval_enabled = (value == BT_GATT_CCC_INDICATE);
	LOG_INF("Body temperature interval indications %s",
		indicate_interval_enabled ? "enabled" : "disabled");
}

static ssize_t read_temperature_type(struct bt_conn* conn,
				     const struct bt_gatt_attr* attr,
				     void* buf,
				     uint16_t len,
				     uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature_type, sizeof(temperature_type));
}

static ssize_t read_measurement_interval(struct bt_conn* conn,
					 const struct bt_gatt_attr* attr,
					 void* buf,
					 uint16_t len,
					 uint16_t offset) {
	uint16_t interval = sys_cpu_to_le16(measurement_interval_sec);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &interval, sizeof(interval));
}

static ssize_t write_measurement_interval(struct bt_conn* conn,
					  const struct bt_gatt_attr* attr,
					  const void* buf,
					  uint16_t len,
					  uint16_t offset,
					  uint8_t flags) {
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t interval_sec = sys_get_le16(buf);
	uint16_t minutes = BODY_TEMPERATURE_INTERVAL_DISABLED;

	if (interval_sec != BODY_TEMPERATURE_INTERVAL_DISABLED) {
		minutes = (uint16_t) ((interval_sec + BODY_TEMPERATURE_SECONDS_PER_MINUTE - 1U) /
				      BODY_TEMPERATURE_SECONDS_PER_MINUTE);
	}

	int ret = device_manager_set_body_temperature_update_period(minutes);

	if (ret != 0) {
		LOG_WRN("Body temperature cadence update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	measurement_interval_sec = (minutes == BODY_TEMPERATURE_INTERVAL_DISABLED) ?
					   BODY_TEMPERATURE_INTERVAL_DISABLED :
					   (uint16_t) (minutes * BODY_TEMPERATURE_SECONDS_PER_MINUTE);
	LOG_INF("Body temperature measurement interval set to %u sec", measurement_interval_sec);
	body_temperature_interval_indicate();

	return len;
}

BT_GATT_SERVICE_DEFINE(
	body_temperature_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT,
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE,
			       NULL,
			       NULL,
			       NULL),
	BT_GATT_CCC(body_temperature_measurement_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_TEMP_TYP,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_temperature_type,
			       NULL,
			       (void*) &temperature_type),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_INTERVAL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_measurement_interval,
			       write_measurement_interval,
			       &measurement_interval_sec),
	BT_GATT_CCC(body_temperature_interval_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static void body_temperature_interval_indicate(void) {
	static struct bt_gatt_indicate_params interval_indicate_params;
	static uint16_t interval_le;

	if (!indicate_interval_enabled || body_temperature_lbs_conn == NULL) {
		return;
	}

	interval_le = sys_cpu_to_le16(measurement_interval_sec);
	interval_indicate_params.attr = &body_temperature_lbs_svc.attrs[7];
	interval_indicate_params.data = &interval_le;
	interval_indicate_params.len = sizeof(interval_le);

	(void) bt_gatt_indicate(body_temperature_lbs_conn, &interval_indicate_params);
}

static void body_temperature_indication_complete(struct bt_conn* conn,
						struct bt_gatt_indicate_params* params,
						uint8_t err) {
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	ARG_UNUSED(err);
	indication_in_flight = false;
}

static void body_temperature_indicate_measurement(void) {
	if (!indicate_temperature_enabled || body_temperature_lbs_conn == NULL || indication_in_flight) {
		return;
	}

	measurement_indicate_params.attr = &body_temperature_lbs_svc.attrs[2];
	measurement_indicate_params.data = &measurement_cache;
	measurement_indicate_params.len = sizeof(measurement_cache);
	measurement_indicate_params.func = body_temperature_indication_complete;

	if (bt_gatt_indicate(body_temperature_lbs_conn, &measurement_indicate_params) == 0) {
		indication_in_flight = true;
	}
}

void body_temperature_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (body_temperature_lbs_conn != NULL) {
		bt_conn_unref(body_temperature_lbs_conn);
	}

	body_temperature_lbs_conn = bt_conn_ref(conn);
	LOG_INF("Body temperature BLE connection attached");
}

void body_temperature_lbs_clear_conn(void) {
	if (body_temperature_lbs_conn != NULL) {
		bt_conn_unref(body_temperature_lbs_conn);
		body_temperature_lbs_conn = NULL;
		LOG_INF("Body temperature BLE connection cleared");
	}

	indication_in_flight = false;
}

static void body_temperature_handle_sample(const struct zbus_channel* chan) {
	const struct temperature_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	measurement_cache.temperature_celsius =
		sys_cpu_to_le32(body_temperature_ieee11073_float_from_mdeg_c(msg->temperature_mdeg_c));
	body_temperature_date_time_from_unix((time_t) (msg->timestamp / 1000000), &measurement_cache.timestamp);
	LOG_INF("Body temperature updated: %d mdeg C", msg->temperature_mdeg_c);
	body_temperature_indicate_measurement();
}

static void body_temperature_listener_cb(const struct zbus_channel* chan) {
	if (device_manager_stream_from_channel(chan) == device_manager_stream_Temperature) {
		body_temperature_handle_sample(chan);
	}
}

ZBUS_LISTENER_DEFINE(body_temperature_lbs_listener, body_temperature_listener_cb);

bool body_temperature_lbs_stream_ready(void) {
	return device_manager_stream_ready(device_manager_stream_Temperature);
}

int body_temperature_lbs_register_stream(void) {
	if (temperature_listener_registered) {
		LOG_INF("Body temperature stream already registered");
		return 0;
	}

	int ret = device_manager_stream_register(device_manager_stream_Temperature,
						 &body_temperature_lbs_listener,
						 K_MSEC(100));

	if (ret == 0) {
		temperature_listener_registered = true;
		LOG_INF("Body temperature stream registered");
	} else {
		LOG_INF("Body temperature stream registration failed: %d", ret);
	}

	return ret;
}
