#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/imu_ble_bridge.h"
#include "bluetooth/services/imu/imu_lbs.h"
#include "bhi360.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define IMU_EVENT_THREAD_PRIO 7
#define IMU_EVENT_STACK_SIZE 2048

static bool imu_bridge_started;
static atomic_t imu_quat_notify_enabled;
static atomic_t imu_lacc_notify_enabled;
static atomic_t imu_streaming_enabled;
static bool imu_event_thread_started;
K_THREAD_STACK_DEFINE(imu_event_stack, IMU_EVENT_STACK_SIZE);
static struct k_thread imu_event_thread;

static void imu_event_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != BHI360_DEVICE_DTS_ID) {
			continue;
		}

		if (event.event_id == bhi360_event_Irq) {
			(void)bhi360_process_irq();
			continue;
		}

		if ((event.event_id == bhi360_event_QuaternionBatch) &&
		    atomic_get(&imu_streaming_enabled) &&
		    (event.p_param != 0U) &&
		    (event.v_param > 0U)) {
			const struct bhi360_quat_data *samples =
				(const struct bhi360_quat_data *)(uintptr_t)event.p_param;
			const struct bhi360_quat_data *data = &samples[event.v_param - 1U];
			struct imu_lbs_quat ble_data = {
				.x = data->x,
				.y = data->y,
				.z = data->z,
				.w = data->w,
				.accuracy = data->accuracy,
			};
			(void)imu_lbs_notify_quat(&ble_data);
		}

		if ((event.event_id == bhi360_event_LinearAccelerationBatch) &&
		    atomic_get(&imu_streaming_enabled) &&
		    (event.p_param != 0U) &&
		    (event.v_param > 0U)) {
			const struct bhi360_lacc_data *samples =
				(const struct bhi360_lacc_data *)(uintptr_t)event.p_param;
			const struct bhi360_lacc_data *data = &samples[event.v_param - 1U];
			struct imu_lbs_lacc ble_data = {
				.x = data->x,
				.y = data->y,
				.z = data->z,
			};
			(void)imu_lbs_notify_lacc(&ble_data);
		}
	}
}

static int imu_start_event_thread(void)
{
	if (imu_event_thread_started) {
		return 0;
	}

	if (device_driver_event_get_queue() == NULL) {
		return -ENODEV;
	}

	k_tid_t tid = k_thread_create(&imu_event_thread, imu_event_stack,
				      K_THREAD_STACK_SIZEOF(imu_event_stack),
				      imu_event_thread_fn, NULL, NULL, NULL,
				      IMU_EVENT_THREAD_PRIO, 0, K_NO_WAIT);
	if (tid == NULL) {
		return -EIO;
	}

	k_thread_name_set(tid, "imu_evt");
	imu_event_thread_started = true;
	return 0;
}

static void imu_update_streaming_state(void)
{
	bool enabled = atomic_get(&imu_quat_notify_enabled) ||
		       atomic_get(&imu_lacc_notify_enabled);

	if (enabled) {
		atomic_set(&imu_streaming_enabled, 1);
		(void)bhi360_configure();
	} else {
		atomic_set(&imu_streaming_enabled, 0);
		bhi360_stop();
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

int imu_ble_bridge_init(void)
{
	if (imu_bridge_started) {
		return 0;
	}

	imu_lbs_register_quat_notify_cb(imu_quat_notify_state_cb, NULL);
	imu_lbs_register_lacc_notify_cb(imu_lacc_notify_state_cb, NULL);
	atomic_set(&imu_streaming_enabled, 0);
	int ret = imu_start_event_thread();
	if (ret != 0) {
		return ret;
	}

	if (!bhi360_init()) {
		return -EIO;
	}

	imu_bridge_started = true;
	return 0;
}
