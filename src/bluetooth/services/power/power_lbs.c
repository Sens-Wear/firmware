#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"
#include "power_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_POWER_SENSOR_BLUETOOTH_LOGGER);

#define POWER_LBS_BLS_FLAG_BATTERY_LEVEL_PRESENT BIT(1)

#define POWER_LBS_BLS_BATTERY_PRESENT_SHIFT 0
#define POWER_LBS_BLS_WIRED_POWER_SHIFT 1
#define POWER_LBS_BLS_WIRELESS_POWER_SHIFT 3
#define POWER_LBS_BLS_CHARGE_STATE_SHIFT 5
#define POWER_LBS_BLS_CHARGE_LEVEL_SHIFT 7
#define POWER_LBS_BLS_CHARGE_TYPE_SHIFT 9
#define POWER_LBS_BLS_CHARGING_FAULT_REASON_SHIFT 12

#define POWER_LBS_BLS_BATTERY_PRESENT_YES 1U
#define POWER_LBS_BLS_WIRED_POWER_NOT_CONNECTED 0U
#define POWER_LBS_BLS_WIRED_POWER_CONNECTED 1U
#define POWER_LBS_BLS_WIRED_POWER_UNKNOWN 2U
#define POWER_LBS_BLS_WIRELESS_POWER_NOT_CONNECTED 0U
#define POWER_LBS_BLS_CHARGE_STATE_UNKNOWN 0U
#define POWER_LBS_BLS_CHARGE_STATE_CHARGING 1U
#define POWER_LBS_BLS_CHARGE_STATE_DISCHARGING_ACTIVE 2U
#define POWER_LBS_BLS_CHARGE_STATE_DISCHARGING_INACTIVE 3U
#define POWER_LBS_BLS_CHARGE_LEVEL_UNKNOWN 0U
#define POWER_LBS_BLS_CHARGE_LEVEL_GOOD 1U
#define POWER_LBS_BLS_CHARGE_LEVEL_LOW 2U
#define POWER_LBS_BLS_CHARGE_LEVEL_CRITICAL 3U
#define POWER_LBS_BLS_CHARGE_TYPE_UNKNOWN 0U
#define POWER_LBS_BLS_CHARGING_FAULT_REASON_NONE 0U
#define POWER_LBS_BLS_CHARGING_FAULT_REASON_OTHER BIT(2)

struct power_lbs_battery_level_status {
	uint8_t flags;
	uint16_t power_state;
	uint8_t battery_level;
} __packed;

static struct bt_conn* power_lbs_conn;
static bool notify_battery_level_enabled;
static bool notify_battery_level_status_enabled;
static uint8_t battery_level_cache;
static bool battery_listener_registered;
static bool charger_listener_registered;
static struct power_lbs_battery_level_status battery_level_status_cache = {
	.flags = POWER_LBS_BLS_FLAG_BATTERY_LEVEL_PRESENT,
	.power_state = sys_cpu_to_le16((POWER_LBS_BLS_BATTERY_PRESENT_YES
					<< POWER_LBS_BLS_BATTERY_PRESENT_SHIFT) |
				       (POWER_LBS_BLS_WIRED_POWER_UNKNOWN
					<< POWER_LBS_BLS_WIRED_POWER_SHIFT) |
				       (POWER_LBS_BLS_WIRELESS_POWER_NOT_CONNECTED
					<< POWER_LBS_BLS_WIRELESS_POWER_SHIFT) |
				       (POWER_LBS_BLS_CHARGE_STATE_UNKNOWN
					<< POWER_LBS_BLS_CHARGE_STATE_SHIFT) |
				       (POWER_LBS_BLS_CHARGE_LEVEL_UNKNOWN
					<< POWER_LBS_BLS_CHARGE_LEVEL_SHIFT) |
				       (POWER_LBS_BLS_CHARGE_TYPE_UNKNOWN
					<< POWER_LBS_BLS_CHARGE_TYPE_SHIFT) |
				       (POWER_LBS_BLS_CHARGING_FAULT_REASON_NONE
					<< POWER_LBS_BLS_CHARGING_FAULT_REASON_SHIFT)),
	.battery_level = 0U,
};

static void battery_level_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_battery_level_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Battery level notifications %s", notify_battery_level_enabled ? "enabled" : "disabled");
}

static void battery_level_status_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_battery_level_status_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Battery level status notifications %s",
		notify_battery_level_status_enabled ? "enabled" : "disabled");
}

static ssize_t read_battery_level(struct bt_conn* conn,
				  const struct bt_gatt_attr* attr,
				  void* buf,
				  uint16_t len,
				  uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_level_cache, sizeof(battery_level_cache));
}

static ssize_t read_battery_level_status(struct bt_conn* conn,
					 const struct bt_gatt_attr* attr,
					 void* buf,
					 uint16_t len,
					 uint16_t offset) {
	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &battery_level_status_cache,
				 sizeof(battery_level_status_cache));
}

BT_GATT_SERVICE_DEFINE(
	power_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_battery_level,
			       NULL,
			       &battery_level_cache),
	BT_GATT_CCC(battery_level_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL_STATUS,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_battery_level_status,
			       NULL,
			       &battery_level_status_cache),
	BT_GATT_CCC(battery_level_status_notify_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

void power_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (power_lbs_conn != NULL) {
		bt_conn_unref(power_lbs_conn);
	}

	power_lbs_conn = bt_conn_ref(conn);
	LOG_INF("Power BLE connection attached");
}

void power_lbs_clear_conn(void) {
	if (power_lbs_conn != NULL) {
		bt_conn_unref(power_lbs_conn);
		power_lbs_conn = NULL;
		LOG_INF("Power BLE connection cleared");
	}
}

static uint8_t power_lbs_soc_to_battery_level(int32_t state_of_charge_dpct) {
	if (state_of_charge_dpct <= 0) {
		return 0U;
	}
	if (state_of_charge_dpct >= 1000) {
		return 100U;
	}

	return (uint8_t) ((state_of_charge_dpct + 5) / 10);
}

static uint8_t power_lbs_charge_level_from_battery_level(uint8_t battery_level) {
	if (battery_level <= 5U) {
		return POWER_LBS_BLS_CHARGE_LEVEL_CRITICAL;
	}
	if (battery_level <= 20U) {
		return POWER_LBS_BLS_CHARGE_LEVEL_LOW;
	}

	return POWER_LBS_BLS_CHARGE_LEVEL_GOOD;
}

static void power_lbs_set_level_status_power_state(uint8_t wired_power,
						   uint8_t charge_state,
						   uint8_t charge_level,
						   uint8_t charge_type,
						   uint8_t fault_reason) {
	uint16_t power_state =
		(POWER_LBS_BLS_BATTERY_PRESENT_YES << POWER_LBS_BLS_BATTERY_PRESENT_SHIFT) |
		(wired_power << POWER_LBS_BLS_WIRED_POWER_SHIFT) |
		(POWER_LBS_BLS_WIRELESS_POWER_NOT_CONNECTED << POWER_LBS_BLS_WIRELESS_POWER_SHIFT) |
		(charge_state << POWER_LBS_BLS_CHARGE_STATE_SHIFT) |
		(charge_level << POWER_LBS_BLS_CHARGE_LEVEL_SHIFT) |
		(charge_type << POWER_LBS_BLS_CHARGE_TYPE_SHIFT) |
		(fault_reason << POWER_LBS_BLS_CHARGING_FAULT_REASON_SHIFT);

	battery_level_status_cache.power_state = sys_cpu_to_le16(power_state);
}

static void power_lbs_notify_battery_level(void) {
	if (notify_battery_level_enabled && power_lbs_conn != NULL) {
		(void) bt_gatt_notify(power_lbs_conn,
				      &power_lbs_svc.attrs[2],
				      &battery_level_cache,
				      sizeof(battery_level_cache));
	}
}

static void power_lbs_notify_battery_level_status(void) {
	if (notify_battery_level_status_enabled && power_lbs_conn != NULL) {
		(void) bt_gatt_notify(power_lbs_conn,
				      &power_lbs_svc.attrs[5],
				      &battery_level_status_cache,
				      sizeof(battery_level_status_cache));
	}
}

static void power_lbs_handle_battery(const struct zbus_channel* chan) {
	const struct battery_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	battery_level_cache = power_lbs_soc_to_battery_level(msg->state_of_charge_dpct);
	battery_level_status_cache.battery_level = battery_level_cache;

	uint16_t power_state = sys_le16_to_cpu(battery_level_status_cache.power_state);

	power_state &= ~(BIT_MASK(2) << POWER_LBS_BLS_CHARGE_LEVEL_SHIFT);
	power_state |= power_lbs_charge_level_from_battery_level(battery_level_cache)
		       << POWER_LBS_BLS_CHARGE_LEVEL_SHIFT;
	battery_level_status_cache.power_state = sys_cpu_to_le16(power_state);

	LOG_INF("Battery level updated: %u%%", battery_level_cache);
	power_lbs_notify_battery_level();
	power_lbs_notify_battery_level_status();
}

static void power_lbs_handle_charger(const struct zbus_channel* chan) {
	const struct charger_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	uint8_t wired_power = msg->power_good ? POWER_LBS_BLS_WIRED_POWER_CONNECTED :
						POWER_LBS_BLS_WIRED_POWER_NOT_CONNECTED;
	uint8_t charge_state = POWER_LBS_BLS_CHARGE_STATE_DISCHARGING_ACTIVE;

	if (msg->charging) {
		charge_state = POWER_LBS_BLS_CHARGE_STATE_CHARGING;
	} else if (msg->charged) {
		charge_state = POWER_LBS_BLS_CHARGE_STATE_DISCHARGING_INACTIVE;
	}

	power_lbs_set_level_status_power_state(
		wired_power,
		charge_state,
		power_lbs_charge_level_from_battery_level(battery_level_cache),
		POWER_LBS_BLS_CHARGE_TYPE_UNKNOWN,
		msg->fault ? POWER_LBS_BLS_CHARGING_FAULT_REASON_OTHER :
			     POWER_LBS_BLS_CHARGING_FAULT_REASON_NONE);
	LOG_INF("Charger status updated: power_good=%d charging=%d charged=%d fault=%d",
		msg->power_good,
		msg->charging,
		msg->charged,
		msg->fault);
	power_lbs_notify_battery_level_status();
}

static void power_lbs_listener_cb(const struct zbus_channel* chan) {
	switch (device_manager_stream_from_channel(chan)) {
	case device_manager_stream_Battery:
		power_lbs_handle_battery(chan);
		break;
	case device_manager_stream_Charger:
		power_lbs_handle_charger(chan);
		break;
	default:
		break;
	}
}

ZBUS_LISTENER_DEFINE(power_lbs_listener, power_lbs_listener_cb);

bool power_lbs_streams_ready(void) {
	return device_manager_stream_ready(device_manager_stream_Battery) &&
	       device_manager_stream_ready(device_manager_stream_Charger);
}

int power_lbs_register_streams(void) {
	int ret = 0;

	if (!battery_listener_registered) {
		ret = device_manager_stream_register(device_manager_stream_Battery,
						     &power_lbs_listener,
						     K_MSEC(100));
		if (ret == 0) {
			battery_listener_registered = true;
			LOG_INF("Battery stream registered");
		} else {
			LOG_INF("Battery stream registration failed: %d", ret);
		}
	} else {
		LOG_INF("Battery stream already registered");
	}
	if (!charger_listener_registered) {
		int charger_ret = device_manager_stream_register(device_manager_stream_Charger,
								 &power_lbs_listener,
								 K_MSEC(100));
		if (charger_ret == 0) {
			charger_listener_registered = true;
			LOG_INF("Charger stream registered");
		} else if (ret == 0) {
			ret = charger_ret;
			LOG_INF("Charger stream registration failed: %d", charger_ret);
		} else {
			LOG_INF("Charger stream registration failed: %d", charger_ret);
		}
	} else {
		LOG_INF("Charger stream already registered");
	}

	return ret;
}
