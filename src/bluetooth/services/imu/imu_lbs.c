#include <errno.h>
#include <stddef.h>

#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/gatt.h>

#include "imu_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_IMU_BLUETOOTH_LOGGER);

static bool notify_quat_enabled;
static bool notify_lacc_enabled;

static imu_lbs_notify_state_cb_t quat_notify_cb;
static void *quat_notify_user_data;
static imu_lbs_notify_state_cb_t lacc_notify_cb;
static void *lacc_notify_user_data;

static void quat_notify_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_quat_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (quat_notify_cb) {
		quat_notify_cb(notify_quat_enabled, quat_notify_user_data);
	}
}

static void lacc_notify_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_lacc_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (lacc_notify_cb) {
		lacc_notify_cb(notify_lacc_enabled, lacc_notify_user_data);
	}
}

BT_GATT_SERVICE_DEFINE(
	imu_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_IMU_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_QUAT, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(quat_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_IMU_LACC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(lacc_notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

void imu_lbs_register_quat_notify_cb(imu_lbs_notify_state_cb_t cb, void *user_data)
{
	quat_notify_cb = cb;
	quat_notify_user_data = user_data;
}

void imu_lbs_register_lacc_notify_cb(imu_lbs_notify_state_cb_t cb, void *user_data)
{
	lacc_notify_cb = cb;
	lacc_notify_user_data = user_data;
}

int imu_lbs_notify_quat(const struct imu_lbs_quat *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	if (!notify_quat_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &imu_lbs_svc.attrs[2], data, sizeof(*data));
}

int imu_lbs_notify_lacc(const struct imu_lbs_lacc *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	if (!notify_lacc_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &imu_lbs_svc.attrs[5], data, sizeof(*data));
}
