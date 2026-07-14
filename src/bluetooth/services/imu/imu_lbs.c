#include <stdint.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"
#include "imu_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_IMU_BLUETOOTH_LOGGER);

#define IMU_LBS_DRAIN_PERIOD_DEFAULT_MS 100U

static struct bt_conn* imu_lbs_conn;

static bool notify_quat_enabled;
static bool notify_lacc_enabled;
static bool notify_gyro_enabled;
static bool notify_gesture_enabled;
static bool notify_activity_enabled;

static bool quat_listener_registered;
static bool lacc_listener_registered;
static bool gyro_listener_registered;
static bool gesture_listener_registered;
static bool activity_listener_registered;

static bool phy_streams_enabled;
static uint32_t drain_period_ms = IMU_LBS_DRAIN_PERIOD_DEFAULT_MS;

static struct imu_lbs_quat quat_cache;
static struct imu_lbs_vec3 lacc_cache;
static struct imu_lbs_vec3 gyro_cache;
static struct imu_lbs_gesture gesture_cache;
static struct imu_lbs_activity activity_cache;

static void quat_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_quat_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void lacc_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_lacc_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void gyro_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_gyro_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void gesture_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_gesture_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void activity_notify_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_activity_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_quat(struct bt_conn* conn,
			 const struct bt_gatt_attr* attr,
			 void* buf,
			 uint16_t len,
			 uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &quat_cache, sizeof(quat_cache));
}

static ssize_t read_lacc(struct bt_conn* conn,
			 const struct bt_gatt_attr* attr,
			 void* buf,
			 uint16_t len,
			 uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &lacc_cache, sizeof(lacc_cache));
}

static ssize_t read_gyro(struct bt_conn* conn,
			 const struct bt_gatt_attr* attr,
			 void* buf,
			 uint16_t len,
			 uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &gyro_cache, sizeof(gyro_cache));
}

static ssize_t read_gesture(struct bt_conn* conn,
			    const struct bt_gatt_attr* attr,
			    void* buf,
			    uint16_t len,
			    uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &gesture_cache, sizeof(gesture_cache));
}

static ssize_t read_activity(struct bt_conn* conn,
			     const struct bt_gatt_attr* attr,
			     void* buf,
			     uint16_t len,
			     uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &activity_cache, sizeof(activity_cache));
}

static ssize_t read_phy_enable(struct bt_conn* conn,
			       const struct bt_gatt_attr* attr,
			       void* buf,
			       uint16_t len,
			       uint16_t offset) {
	uint8_t enabled = phy_streams_enabled ? 1U : 0U;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &enabled, sizeof(enabled));
}

static ssize_t write_phy_enable(struct bt_conn* conn,
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

	int ret = device_manager_set_imu_phy_streams_enabled(enabled != 0U, drain_period_ms);

	if (ret != 0) {
		LOG_WRN("IMU physical stream update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	phy_streams_enabled = (enabled != 0U);
	return len;
}

static ssize_t read_drain_period(struct bt_conn* conn,
				 const struct bt_gatt_attr* attr,
				 void* buf,
				 uint16_t len,
				 uint16_t offset) {
	uint32_t period = sys_cpu_to_le32(drain_period_ms);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &period, sizeof(period));
}

static ssize_t write_drain_period(struct bt_conn* conn,
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
	if (len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t period = sys_get_le32(buf);

	if (period == 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (phy_streams_enabled) {
		int ret = device_manager_set_imu_phy_streams_enabled(true, period);

		if (ret != 0) {
			LOG_WRN("IMU drain-period update failed: %d", ret);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
	}

	drain_period_ms = period;
	return len;
}

BT_GATT_SERVICE_DEFINE(
	imu_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_IMU_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_QUAT,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_quat,
			       NULL,
			       &quat_cache),
	BT_GATT_CCC(quat_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_LACC,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_lacc,
			       NULL,
			       &lacc_cache),
	BT_GATT_CCC(lacc_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_GYRO,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gyro,
			       NULL,
			       &gyro_cache),
	BT_GATT_CCC(gyro_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_GESTURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gesture,
			       NULL,
			       &gesture_cache),
	BT_GATT_CCC(gesture_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_ACTIVITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_activity,
			       NULL,
			       &activity_cache),
	BT_GATT_CCC(activity_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

BT_GATT_SERVICE_DEFINE(
	imu_lbs_config_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_IMU_CONFIG_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_phy_enable,
			       write_phy_enable,
			       NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_drain_period,
			       write_drain_period,
			       NULL));

void imu_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (imu_lbs_conn != NULL) {
		bt_conn_unref(imu_lbs_conn);
	}

	imu_lbs_conn = bt_conn_ref(conn);
}

void imu_lbs_clear_conn(void) {
	if (imu_lbs_conn != NULL) {
		bt_conn_unref(imu_lbs_conn);
		imu_lbs_conn = NULL;
	}
}

static void imu_lbs_handle_quat(const struct zbus_channel* chan) {
	const struct imu_quaternion_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	quat_cache.timestamp = msg->timestamp;
	quat_cache.x = msg->x;
	quat_cache.y = msg->y;
	quat_cache.z = msg->z;
	quat_cache.w = msg->w;
	quat_cache.accuracy = msg->accuracy;

	if (notify_quat_enabled && imu_lbs_conn != NULL) {
		(void) bt_gatt_notify(imu_lbs_conn, &imu_lbs_svc.attrs[2], &quat_cache, sizeof(quat_cache));
	}
}

static void imu_lbs_handle_lacc(const struct zbus_channel* chan) {
	const struct imu_accel_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	lacc_cache.timestamp = msg->timestamp;
	lacc_cache.x = msg->x;
	lacc_cache.y = msg->y;
	lacc_cache.z = msg->z;

	if (notify_lacc_enabled && imu_lbs_conn != NULL) {
		(void) bt_gatt_notify(imu_lbs_conn, &imu_lbs_svc.attrs[5], &lacc_cache, sizeof(lacc_cache));
	}
}

static void imu_lbs_handle_gyro(const struct zbus_channel* chan) {
	const struct imu_gyro_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	gyro_cache.timestamp = msg->timestamp;
	gyro_cache.x = msg->x;
	gyro_cache.y = msg->y;
	gyro_cache.z = msg->z;

	if (notify_gyro_enabled && imu_lbs_conn != NULL) {
		(void) bt_gatt_notify(imu_lbs_conn, &imu_lbs_svc.attrs[8], &gyro_cache, sizeof(gyro_cache));
	}
}

static void imu_lbs_handle_gesture(const struct zbus_channel* chan) {
	const struct imu_gesture_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	gesture_cache.timestamp = msg->timestamp;
	gesture_cache.sensor_id = msg->sensor_id;
	gesture_cache.gesture = (uint8_t) msg->gesture;

	if (notify_gesture_enabled && imu_lbs_conn != NULL) {
		(void) bt_gatt_notify(imu_lbs_conn,
				      &imu_lbs_svc.attrs[11],
				      &gesture_cache,
				      sizeof(gesture_cache));
	}
}

static void imu_lbs_handle_activity(const struct zbus_channel* chan) {
	const struct imu_activity_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	activity_cache.timestamp = msg->timestamp;
	activity_cache.sensor_id = msg->sensor_id;
	activity_cache.activity = (uint8_t) msg->activity;
	activity_cache.transition = (uint8_t) msg->transition;

	if (notify_activity_enabled && imu_lbs_conn != NULL) {
		(void) bt_gatt_notify(imu_lbs_conn,
				      &imu_lbs_svc.attrs[14],
				      &activity_cache,
				      sizeof(activity_cache));
	}
}

static void imu_lbs_listener_cb(const struct zbus_channel* chan) {
	switch (device_manager_stream_from_channel(chan)) {
	case device_manager_stream_ImuQuaternion:
		imu_lbs_handle_quat(chan);
		break;
	case device_manager_stream_ImuAccel:
		imu_lbs_handle_lacc(chan);
		break;
	case device_manager_stream_ImuGyro:
		imu_lbs_handle_gyro(chan);
		break;
	case device_manager_stream_ImuGesture:
		imu_lbs_handle_gesture(chan);
		break;
	case device_manager_stream_ImuActivity:
		imu_lbs_handle_activity(chan);
		break;
	default:
		break;
	}
}

ZBUS_LISTENER_DEFINE(imu_lbs_listener, imu_lbs_listener_cb);

bool imu_lbs_streams_ready(void) {
	return device_manager_stream_ready(device_manager_stream_ImuQuaternion) &&
	       device_manager_stream_ready(device_manager_stream_ImuAccel) &&
	       device_manager_stream_ready(device_manager_stream_ImuGyro) &&
	       device_manager_stream_ready(device_manager_stream_ImuGesture) &&
	       device_manager_stream_ready(device_manager_stream_ImuActivity);
}

static int imu_lbs_register_stream(enum device_manager_stream_type stream, bool* registered) {
	if (*registered) {
		return 0;
	}

	int ret = device_manager_stream_register(stream, &imu_lbs_listener, K_MSEC(100));

	if (ret == 0) {
		*registered = true;
	}

	return ret;
}

int imu_lbs_register_streams(void) {
	int ret = 0;

	ret = imu_lbs_register_stream(device_manager_stream_ImuQuaternion, &quat_listener_registered);
	if (ret != 0) {
		return ret;
	}
	ret = imu_lbs_register_stream(device_manager_stream_ImuAccel, &lacc_listener_registered);
	if (ret != 0) {
		return ret;
	}
	ret = imu_lbs_register_stream(device_manager_stream_ImuGyro, &gyro_listener_registered);
	if (ret != 0) {
		return ret;
	}
	ret = imu_lbs_register_stream(device_manager_stream_ImuGesture, &gesture_listener_registered);
	if (ret != 0) {
		return ret;
	}
	return imu_lbs_register_stream(device_manager_stream_ImuActivity, &activity_listener_registered);
}
