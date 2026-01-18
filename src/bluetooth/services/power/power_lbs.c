#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "power_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_POWER_SENSOR_BLUETOOTH_LOGGER);

static struct power_lbs_charger_state charger_state_cache;
static struct power_lbs_gauge_state gauge_state_cache;
static const struct power_lbs_ops *power_ops;
static struct bt_conn *power_lbs_conn;

static bool notify_charger_enabled;
static bool notify_gauge_enabled;
static power_lbs_notify_state_cb_t charger_notify_cb;
static void *charger_notify_user_data;
static power_lbs_notify_state_cb_t gauge_notify_cb;
static void *gauge_notify_user_data;

static void charger_notify_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_charger_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (charger_notify_cb) {
		charger_notify_cb(notify_charger_enabled, charger_notify_user_data);
	}
}

static void gauge_notify_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_gauge_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (gauge_notify_cb) {
		gauge_notify_cb(notify_gauge_enabled, gauge_notify_user_data);
	}
}

static ssize_t read_charger_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	if (power_ops && power_ops->get_charger_state) {
		struct power_lbs_charger_state current;
		if (power_ops->get_charger_state(&current) == 0) {
			charger_state_cache = current;
		}
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &charger_state_cache,
				 sizeof(charger_state_cache));
}

static ssize_t read_gauge_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	if (power_ops && power_ops->get_gauge_state) {
		struct power_lbs_gauge_state current;
		if (power_ops->get_gauge_state(&current) == 0) {
			gauge_state_cache = current;
		}
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &gauge_state_cache,
				 sizeof(gauge_state_cache));
}

BT_GATT_SERVICE_DEFINE(
	power_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_POWER_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_POWER_CHARGER_STATE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_charger_state, NULL, NULL),
	BT_GATT_CCC(charger_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_POWER_GAUGE_STATE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_gauge_state, NULL, NULL),
	BT_GATT_CCC(gauge_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

void power_lbs_register_ops(const struct power_lbs_ops *ops)
{
	power_ops = ops;
}

void power_lbs_register_charger_notify_cb(power_lbs_notify_state_cb_t cb, void *user_data)
{
	charger_notify_cb = cb;
	charger_notify_user_data = user_data;
}

void power_lbs_register_gauge_notify_cb(power_lbs_notify_state_cb_t cb, void *user_data)
{
	gauge_notify_cb = cb;
	gauge_notify_user_data = user_data;
}

void power_lbs_set_conn(struct bt_conn *conn)
{
	if (conn == NULL) {
		return;
	}

	if (power_lbs_conn != NULL) {
		bt_conn_unref(power_lbs_conn);
	}

	power_lbs_conn = bt_conn_ref(conn);
}

void power_lbs_clear_conn(void)
{
	if (power_lbs_conn != NULL) {
		bt_conn_unref(power_lbs_conn);
		power_lbs_conn = NULL;
	}
}

int power_lbs_notify_charger_state(const struct power_lbs_charger_state *state)
{
	if (!notify_charger_enabled) {
		return -EACCES;
	}

	if (power_lbs_conn == NULL) {
		return -ENOTCONN;
	}

	return bt_gatt_notify(power_lbs_conn, &power_lbs_svc.attrs[2], state,
			      sizeof(*state));
}

int power_lbs_notify_gauge_state(const struct power_lbs_gauge_state *state)
{
	if (!notify_gauge_enabled) {
		return -EACCES;
	}

	if (power_lbs_conn == NULL) {
		return -ENOTCONN;
	}

	return bt_gatt_notify(power_lbs_conn, &power_lbs_svc.attrs[5], state,
			      sizeof(*state));
}
