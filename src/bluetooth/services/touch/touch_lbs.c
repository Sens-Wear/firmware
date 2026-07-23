#include <stdint.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"
#include "touch_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_TOUCH_SENSOR_BLUETOOTH_LOGGER);

static struct bt_conn* touch_lbs_conn;

static bool notify_touch_state_enabled;
static bool notify_gesture_state_enabled;
static bool notify_raw_data_enabled;
static bool touch_listener_registered;
static bool gesture_listener_registered;
static bool touch_sampling_enabled;

static struct touch_lbs_touch_state touch_state_cache;
static struct touch_lbs_gesture_state gesture_state_cache;
static struct touch_msg_t raw_touch_cache;

static void touch_state_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_touch_state_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Touch state notifications %s", notify_touch_state_enabled ? "enabled" : "disabled");
}

static void gesture_state_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_gesture_state_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Touch gesture notifications %s", notify_gesture_state_enabled ? "enabled" : "disabled");
}

static void raw_data_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_raw_data_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Touch raw-data notifications %s", notify_raw_data_enabled ? "enabled" : "disabled");
}

static ssize_t read_touch_state(struct bt_conn* conn,
				const struct bt_gatt_attr* attr,
				void* buf,
				uint16_t len,
				uint16_t offset) {
	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &touch_state_cache,
				 sizeof(touch_state_cache));
}

static ssize_t read_gesture_state(struct bt_conn* conn,
				  const struct bt_gatt_attr* attr,
				  void* buf,
				  uint16_t len,
				  uint16_t offset) {
	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &gesture_state_cache,
				 sizeof(gesture_state_cache));
}

static ssize_t read_raw_data(struct bt_conn* conn,
			     const struct bt_gatt_attr* attr,
			     void* buf,
			     uint16_t len,
			     uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &raw_touch_cache, sizeof(raw_touch_cache));
}

static ssize_t read_sampling_enable(struct bt_conn* conn,
				    const struct bt_gatt_attr* attr,
				    void* buf,
				    uint16_t len,
				    uint16_t offset) {
	uint8_t enabled = touch_sampling_enabled ? 1U : 0U;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &enabled, sizeof(enabled));
}

static ssize_t write_sampling_enable(struct bt_conn* conn,
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
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t enabled = *(const uint8_t*) buf;

	if (enabled > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	bool requested_enabled = (enabled != 0U);

	if (requested_enabled == touch_sampling_enabled) {
		LOG_INF("Touch sampling already %s", touch_sampling_enabled ? "enabled" : "disabled");
		return len;
	}

	int ret = device_manager_set_touch_sampling_enabled(requested_enabled);

	if (ret != 0) {
		LOG_WRN("Touch sampling update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	touch_sampling_enabled = requested_enabled;
	LOG_INF("Touch sampling %s", touch_sampling_enabled ? "enabled" : "disabled");
	return len;
}

BT_GATT_SERVICE_DEFINE(
	touch_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_TOUCH_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TOUCH_STATE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_touch_state,
			       NULL,
			       &touch_state_cache),
	BT_GATT_CUD("Touch State", BT_GATT_PERM_READ),
	BT_GATT_CCC(touch_state_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_GESTURE_STATE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gesture_state,
			       NULL,
			       &gesture_state_cache),
	BT_GATT_CUD("Touch Gesture", BT_GATT_PERM_READ),
	BT_GATT_CCC(gesture_state_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_RAW_DATA,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_raw_data,
			       NULL,
			       &raw_touch_cache),
	BT_GATT_CUD("Touch Raw Data", BT_GATT_PERM_READ),
	BT_GATT_CCC(raw_data_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

BT_GATT_SERVICE_DEFINE(
	touch_lbs_config_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_TOUCH_CONFIG_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_sampling_enable,
			       write_sampling_enable,
			       NULL),
	BT_GATT_CUD("Touch Sampling Enable", BT_GATT_PERM_READ));

void touch_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (touch_lbs_conn != NULL) {
		bt_conn_unref(touch_lbs_conn);
	}

	touch_lbs_conn = bt_conn_ref(conn);
	LOG_INF("Touch BLE connection attached");
}

void touch_lbs_clear_conn(void) {
	if (touch_lbs_conn != NULL) {
		bt_conn_unref(touch_lbs_conn);
		touch_lbs_conn = NULL;
		LOG_INF("Touch BLE connection cleared");
	}
}

static void touch_lbs_handle_touch(const struct zbus_channel* chan) {
	const struct touch_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	raw_touch_cache = *msg;
	touch_state_cache.timestamp = msg->timestamp;
	touch_state_cache.touched = msg->touched;
	touch_state_cache.x = msg->x;
	touch_state_cache.y = msg->y;

	if (touch_lbs_conn == NULL) {
		return;
	}

	if (notify_touch_state_enabled) {
		(void) bt_gatt_notify(touch_lbs_conn,
				      &touch_lbs_svc.attrs[2],
				      &touch_state_cache,
				      sizeof(touch_state_cache));
	}
	if (notify_raw_data_enabled) {
		(void) bt_gatt_notify(touch_lbs_conn,
				      &touch_lbs_svc.attrs[10],
				      &raw_touch_cache,
				      sizeof(raw_touch_cache));
	}
}

static void touch_lbs_handle_gesture(const struct zbus_channel* chan) {
	const struct touch_gesture_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	gesture_state_cache.timestamp = msg->timestamp;
	gesture_state_cache.gesture = (uint8_t) msg->gesture;
	gesture_state_cache.gesture_state = msg->gesture_state;

	if (notify_gesture_state_enabled && touch_lbs_conn != NULL) {
		(void) bt_gatt_notify(touch_lbs_conn,
				      &touch_lbs_svc.attrs[6],
				      &gesture_state_cache,
				      sizeof(gesture_state_cache));
	}
}

static void touch_lbs_listener_cb(const struct zbus_channel* chan) {
	switch (device_manager_stream_from_channel(chan)) {
	case device_manager_stream_Touch:
		touch_lbs_handle_touch(chan);
		break;
	case device_manager_stream_TouchGesture:
		touch_lbs_handle_gesture(chan);
		break;
	default:
		break;
	}
}

ZBUS_LISTENER_DEFINE(touch_lbs_listener, touch_lbs_listener_cb);

bool touch_lbs_streams_ready(void) {
	return device_manager_stream_ready(device_manager_stream_Touch) &&
	       device_manager_stream_ready(device_manager_stream_TouchGesture);
}

static int touch_lbs_register_stream(enum device_manager_stream_type stream, bool* registered) {
	if (*registered) {
		LOG_INF("Touch stream %d already registered", stream);
		return 0;
	}

	int ret = device_manager_stream_register(stream, &touch_lbs_listener, K_MSEC(100));

	if (ret == 0) {
		*registered = true;
		LOG_INF("Touch stream %d registered", stream);
	} else {
		LOG_INF("Touch stream %d registration failed: %d", stream, ret);
	}

	return ret;
}

int touch_lbs_register_streams(void) {
	int ret = touch_lbs_register_stream(device_manager_stream_Touch, &touch_listener_registered);

	if (ret != 0) {
		return ret;
	}

	return touch_lbs_register_stream(device_manager_stream_TouchGesture, &gesture_listener_registered);
}
