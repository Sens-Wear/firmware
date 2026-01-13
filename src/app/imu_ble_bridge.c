#include <errno.h>
#include <zephyr/kernel.h>

#include "app/imu_ble_bridge.h"
#include "bluetooth/services/imu/imu_lbs.h"
#include "drivers/sensors/imu/bhi360.h"

#define IMU_BLE_BRIDGE_STACK_SIZE 4096
#define IMU_BLE_BRIDGE_PRIORITY 5

static K_THREAD_STACK_DEFINE(imu_bridge_stack, IMU_BLE_BRIDGE_STACK_SIZE);
static struct k_thread imu_bridge_thread;
static bool imu_bridge_started;

static void imu_bridge_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	imu_sensor_init();
}

static void imu_quat_cb(const struct imu_quat_data *data, void *user_data)
{
	ARG_UNUSED(user_data);

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

	imu_register_quaternion_callback(imu_quat_cb, NULL);
	imu_register_linear_accel_callback(imu_lacc_cb, NULL);

	k_thread_create(&imu_bridge_thread,
			imu_bridge_stack,
			K_THREAD_STACK_SIZEOF(imu_bridge_stack),
			imu_bridge_thread_fn,
			NULL, NULL, NULL,
			IMU_BLE_BRIDGE_PRIORITY,
			0,
			K_NO_WAIT);

	imu_bridge_started = true;
	return 0;
}
