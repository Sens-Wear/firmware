#include <stdint.h>
#include <errno.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"
#include "touch_lbs.h"

/* Preserve the existing little-endian BLE contracts, including raw padding. */
BUILD_ASSERT(sizeof(struct touch_lbs_touch_state) == 13);
BUILD_ASSERT(sizeof(struct touch_lbs_gesture_state) == 10);
BUILD_ASSERT(sizeof(struct touch_msg_t) == 16);
BUILD_ASSERT(offsetof(struct touch_msg_t, x) == 10);
BUILD_ASSERT(offsetof(struct touch_msg_t, touch_state) == 14);

LOG_MODULE_REGISTER(SENS_WEAR_TOUCH_SENSOR_BLUETOOTH_LOGGER);

static struct bt_conn* touch_lbs_conn;
static struct k_spinlock touch_lbs_lock;
static uint32_t touch_conn_generation;

#define TOUCH_NOTIFY_STATE BIT(0)
#define TOUCH_NOTIFY_RAW BIT(1)
#define TOUCH_NOTIFY_GESTURE BIT(2)
#define TOUCH_NOTIFY_QUEUE_DEPTH 16
#define TOUCH_NOTIFY_WORK_BUDGET 4

struct touch_notification {
	uint32_t connection_generation;
	uint8_t streams;
	union {
		struct touch_msg_t touch;
		struct touch_lbs_gesture_state gesture;
	} data;
};

K_MSGQ_DEFINE(touch_notify_queue, sizeof(struct touch_notification), TOUCH_NOTIFY_QUEUE_DEPTH, 4);

static void touch_lbs_notify_work_handler(struct k_work* work);
K_WORK_DEFINE(touch_notify_work, touch_lbs_notify_work_handler);

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
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	notify_touch_state_enabled = enabled;
	k_spin_unlock(&touch_lbs_lock, key);
	LOG_INF("Touch state notifications %s", enabled ? "enabled" : "disabled");
}

static void gesture_state_notification_cfg_changed(const struct bt_gatt_attr* attr,
												   uint16_t value) {
	ARG_UNUSED(attr);
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	notify_gesture_state_enabled = enabled;
	k_spin_unlock(&touch_lbs_lock, key);
	LOG_INF("Touch gesture notifications %s", enabled ? "enabled" : "disabled");
}

static void raw_data_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	notify_raw_data_enabled = enabled;
	k_spin_unlock(&touch_lbs_lock, key);
	LOG_INF("Touch raw-data notifications %s", enabled ? "enabled" : "disabled");
}

static ssize_t read_touch_state(struct bt_conn* conn,
								const struct bt_gatt_attr* attr,
								void* buf,
								uint16_t len,
								uint16_t offset) {
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	struct touch_lbs_touch_state snapshot = touch_state_cache;
	k_spin_unlock(&touch_lbs_lock, key);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &snapshot, sizeof(snapshot));
}

static ssize_t read_gesture_state(struct bt_conn* conn,
								  const struct bt_gatt_attr* attr,
								  void* buf,
								  uint16_t len,
								  uint16_t offset) {
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	struct touch_lbs_gesture_state snapshot = gesture_state_cache;
	k_spin_unlock(&touch_lbs_lock, key);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &snapshot, sizeof(snapshot));
}

static ssize_t read_raw_data(struct bt_conn* conn,
							 const struct bt_gatt_attr* attr,
							 void* buf,
							 uint16_t len,
							 uint16_t offset) {
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	struct touch_msg_t snapshot = raw_touch_cache;
	k_spin_unlock(&touch_lbs_lock, key);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &snapshot, sizeof(snapshot));
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

BT_GATT_SERVICE_DEFINE(touch_lbs_config_svc,
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

	struct bt_conn* retained = bt_conn_ref(conn);
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	struct bt_conn* previous = touch_lbs_conn;
	touch_lbs_conn = retained;
	++touch_conn_generation;
	k_spin_unlock(&touch_lbs_lock, key);
	if (previous != NULL) {
		bt_conn_unref(previous);
	}
	LOG_INF("Touch BLE connection attached");
}

void touch_lbs_clear_conn(void) {
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	struct bt_conn* previous = touch_lbs_conn;
	touch_lbs_conn = NULL;
	++touch_conn_generation;
	k_spin_unlock(&touch_lbs_lock, key);
	if (previous != NULL) {
		bt_conn_unref(previous);
		LOG_INF("Touch BLE connection cleared");
	}
}

/** Called with touch_lbs_lock held. */
static uint8_t touch_lbs_enabled_streams(void) {
	return (notify_touch_state_enabled ? TOUCH_NOTIFY_STATE : 0U) |
		   (notify_raw_data_enabled ? TOUCH_NOTIFY_RAW : 0U) |
		   (notify_gesture_state_enabled ? TOUCH_NOTIFY_GESTURE : 0U);
}

static void touch_lbs_submit_notify_work(void) {
	int ret = k_work_submit(&touch_notify_work);

	if (ret < 0) {
		LOG_WRN_RATELIMIT("Touch notification work submission failed: %d", ret);
	}
}

static void touch_lbs_queue_notification(const struct touch_notification* notification) {
	if (notification->streams == 0U) {
		return;
	}

	int ret = k_msgq_put(&touch_notify_queue, notification, K_NO_WAIT);

	if (ret != 0) {
		LOG_WRN_RATELIMIT("Touch notification queue full; sample dropped");
	}
	touch_lbs_submit_notify_work();
}

static void touch_lbs_notify(struct bt_conn* conn,
							 const struct bt_gatt_attr* attr,
							 const void* data,
							 uint16_t length) {
	int ret = bt_gatt_notify(conn, attr, data, length);

	if (ret != 0 && ret != -ENOTCONN) {
		LOG_WRN_RATELIMIT("Touch notification dropped: %d", ret);
	}
}

static void touch_lbs_notify_work_handler(struct k_work* work) {
	ARG_UNUSED(work);
	struct touch_notification notification;

	/* Zephyr uses K_NO_WAIT for ATT buffer allocation on the system workqueue.
	 * A synchronous zbus listener runs on the device-manager thread, where
	 * bt_gatt_notify() may instead block forever waiting for those buffers.
	 * Bound each invocation so other system work can run during sustained input.
	 */
	for (unsigned int i = 0U; i < TOUCH_NOTIFY_WORK_BUDGET; ++i) {
		if (k_msgq_get(&touch_notify_queue, &notification, K_NO_WAIT) != 0) {
			return;
		}

		struct bt_conn* conn = NULL;
		k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
		uint8_t streams = notification.streams & touch_lbs_enabled_streams();

		if (streams != 0U && touch_lbs_conn != NULL &&
			notification.connection_generation == touch_conn_generation) {
			conn = bt_conn_ref(touch_lbs_conn);
		}
		k_spin_unlock(&touch_lbs_lock, key);
		if (conn == NULL) {
			continue;
		}

		if ((streams & TOUCH_NOTIFY_STATE) != 0U) {
			const struct touch_msg_t* touch = &notification.data.touch;
			struct touch_lbs_touch_state state = {
				.timestamp = touch->timestamp,
				.touched = touch->touched,
				.x = touch->x,
				.y = touch->y,
			};
			touch_lbs_notify(conn, &touch_lbs_svc.attrs[2], &state, sizeof(state));
		}
		if ((streams & TOUCH_NOTIFY_RAW) != 0U) {
			touch_lbs_notify(conn,
							 &touch_lbs_svc.attrs[10],
							 &notification.data.touch,
							 sizeof(notification.data.touch));
		}
		if ((streams & TOUCH_NOTIFY_GESTURE) != 0U) {
			touch_lbs_notify(conn,
							 &touch_lbs_svc.attrs[6],
							 &notification.data.gesture,
							 sizeof(notification.data.gesture));
		}
		bt_conn_unref(conn);
	}

	if (k_msgq_num_used_get(&touch_notify_queue) != 0U) {
		touch_lbs_submit_notify_work();
	}
}

static void touch_lbs_handle_touch(const struct zbus_channel* chan) {
	const struct touch_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	struct touch_notification notification = {
		.data.touch = *msg,
	};
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	raw_touch_cache = *msg;
	touch_state_cache.timestamp = msg->timestamp;
	touch_state_cache.touched = msg->touched;
	touch_state_cache.x = msg->x;
	touch_state_cache.y = msg->y;

	notification.connection_generation = touch_conn_generation;
	if (touch_lbs_conn != NULL) {
		notification.streams = touch_lbs_enabled_streams() &
							   (TOUCH_NOTIFY_STATE | TOUCH_NOTIFY_RAW);
	}
	k_spin_unlock(&touch_lbs_lock, key);
	touch_lbs_queue_notification(&notification);
}

static void touch_lbs_handle_gesture(const struct zbus_channel* chan) {
	const struct touch_gesture_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	struct touch_notification notification = {0};
	k_spinlock_key_t key = k_spin_lock(&touch_lbs_lock);
	gesture_state_cache.timestamp = msg->timestamp;
	gesture_state_cache.gesture = (uint8_t) msg->gesture;
	gesture_state_cache.gesture_state = msg->gesture_state;

	notification.data.gesture = gesture_state_cache;
	notification.connection_generation = touch_conn_generation;
	if (touch_lbs_conn != NULL) {
		notification.streams = touch_lbs_enabled_streams() & TOUCH_NOTIFY_GESTURE;
	}
	k_spin_unlock(&touch_lbs_lock, key);
	touch_lbs_queue_notification(&notification);
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

	return touch_lbs_register_stream(device_manager_stream_TouchGesture,
									 &gesture_listener_registered);
}
