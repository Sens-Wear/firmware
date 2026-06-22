#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/imu_ble_bridge.h"
#include "bluetooth/services/imu/imu_lbs.h"
#include "bhi360.h"

static bool imu_bridge_started;
static atomic_t imu_quat_notify_enabled;
static atomic_t imu_lacc_notify_enabled;
static atomic_t imu_streaming_enabled;

static void imu_update_streaming_state(void)
{
	bool enabled = atomic_get(&imu_quat_notify_enabled) ||
		       atomic_get(&imu_lacc_notify_enabled);

	if (enabled) {
		atomic_set(&imu_streaming_enabled, 1);
		imu_set_streaming_enabled(true);
		(void)imu_start();
	} else {
		atomic_set(&imu_streaming_enabled, 0);
		imu_set_streaming_enabled(false);
		imu_stop();
	}
}

static void imu_quat_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&imu_quat_notify_enabled, enabled ? 1 : 0);
	imu_update_streaming_state();
}

static void imu_lacc_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&imu_lacc_notify_enabled, enabled ? 1 : 0);
	imu_update_streaming_state();
}

static void imu_quat_cb(const struct imu_quat_data *data, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!atomic_get(&imu_streaming_enabled)) {
		return;
	}

	struct imu_lbs_quat ble_data = {
		.x = data->x,
		.y = data->y,
		.z = data->z,
		.w = data->w,
		.accuracy = data->accuracy,
	};

	(void)imu_lbs_notify_quat(&ble_data);
}

static void imu_lacc_cb(const struct imu_lacc_data *data, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!atomic_get(&imu_streaming_enabled)) {
		return;
	}

	struct imu_lbs_lacc ble_data = {
		.x = data->x,
		.y = data->y,
		.z = data->z,
	};

	(void)imu_lbs_notify_lacc(&ble_data);
}

int imu_ble_bridge_init(void)
{
	if (imu_bridge_started) {
		return 0;
	}

	imu_lbs_register_quat_notify_cb(imu_quat_notify_state_cb, NULL);
	imu_lbs_register_lacc_notify_cb(imu_lacc_notify_state_cb, NULL);
	imu_register_quaternion_callback(imu_quat_cb, NULL);
	imu_register_linear_accel_callback(imu_lacc_cb, NULL);
	atomic_set(&imu_streaming_enabled, 0);
	imu_set_streaming_enabled(false);
	return 0;
}
