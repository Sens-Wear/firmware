#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "temperature_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_TEMPERATURE_SENSOR_BLUETOOTH_LOGGER);

static struct bt_conn *temperature_lbs_conn;
static bool notify_temperature_enabled;
static temperature_lbs_notify_state_cb_t temperature_notify_cb;
static void *temperature_notify_user_data;
static void (*update_sampling_rate_callback)(uint16_t new_sampling_rate);
static void (*update_transfer_interval_callback)(uint16_t transfer_interval);
static struct temperature_lbs_sample temperature_state_cache;

static void temperature_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	notify_temperature_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (temperature_notify_cb) {
		temperature_notify_cb(notify_temperature_enabled, temperature_notify_user_data);
	}
}

static ssize_t update_sampling_rate(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    const void *buf, uint16_t len, uint16_t offset,
				    uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (update_sampling_rate_callback != NULL) {
		uint16_t value = sys_get_le16(buf);

		if (value == 0U) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		update_sampling_rate_callback(value);
	}

	return len;
}

static ssize_t update_transfer_interval(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					const void *buf, uint16_t len, uint16_t offset,
					uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (update_transfer_interval_callback != NULL) {
		uint16_t value = sys_get_le16(buf);

		if (value == 0U) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		update_transfer_interval_callback(value);
	}

	return len;
}

static ssize_t read_temperature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature_state_cache,
				 sizeof(temperature_state_cache));
}

BT_GATT_SERVICE_DEFINE(
	temperature_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_TEMPERATURE_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF,
			       BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
			       update_sampling_rate, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF,
			       BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
			       update_transfer_interval, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
			       read_temperature, NULL, &temperature_state_cache),
	BT_GATT_CCC(temperature_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

void temperature_lbs_register_notify_cb(temperature_lbs_notify_state_cb_t cb, void *user_data)
{
	temperature_notify_cb = cb;
	temperature_notify_user_data = user_data;
}

void temperature_lbs_register_sampling_rate_cb(void (*callback)(uint16_t))
{
	update_sampling_rate_callback = callback;
}

void temperature_lbs_register_transfer_interval_cb(void (*callback)(uint16_t))
{
	update_transfer_interval_callback = callback;
}

void temperature_lbs_set_conn(struct bt_conn *conn)
{
	if (conn == NULL) {
		return;
	}

	if (temperature_lbs_conn != NULL) {
		bt_conn_unref(temperature_lbs_conn);
	}

	temperature_lbs_conn = bt_conn_ref(conn);
}

void temperature_lbs_clear_conn(void)
{
	if (temperature_lbs_conn != NULL) {
		bt_conn_unref(temperature_lbs_conn);
		temperature_lbs_conn = NULL;
	}
}

int temperature_lbs_notify(const struct temperature_lbs_sample *sample)
{
	if (sample == NULL) {
		return -EINVAL;
	}

	temperature_state_cache = *sample;

	if (!notify_temperature_enabled) {
		return -EACCES;
	}

	if (temperature_lbs_conn == NULL) {
		return -ENOTCONN;
	}

	return bt_gatt_notify(temperature_lbs_conn, &temperature_lbs_svc.attrs[6], sample,
			      sizeof(*sample));
}
