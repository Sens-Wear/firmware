/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BHI360 IMU driver.
 *
 * Starts the sensor and consumes BHI360 device events. Sample events carry
 * driver-owned payload pointers in p_param; packed metadata rides in v_param.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bhi360.h"
#include "device_driver_events.h"

#define BHI360_DEVICE_ID DT_DEP_ORD(DT_NODELABEL(bhi360))

#define TEST_POLL_INTERVAL K_SECONDS(2)
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

static struct bhi360_quat_data last_quat;
static struct bhi360_lacc_data last_lacc;
static struct bhi360_gyro_data last_gyro;
static struct bhi360_pedometer_data last_pedometer;
static struct bhi360_gesture_data last_gesture;
static struct bhi360_activity_data last_activity;
static uint32_t last_meta_event;
static atomic_t have_quat;
static atomic_t have_lacc;
static atomic_t have_gyro;
static atomic_t have_pedometer;
static atomic_t have_gesture;
static atomic_t have_activity;
static atomic_t have_meta;

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static const char* bhi360_event_to_string(uint32_t event_id) {
	switch ((enum bhi360_event_type) event_id) {
	case bhi360_event_Irq:
		return "Irq";
	case bhi360_event_Quaternion:
		return "Quaternion";
	case bhi360_event_LinearAcceleration:
		return "LinearAcceleration";
	case bhi360_event_Gyro:
		return "Gyro";
	case bhi360_event_Pedometer:
		return "Pedometer";
	case bhi360_event_Gesture:
		return "Gesture";
	case bhi360_event_Activity:
		return "Activity";
	case bhi360_event_MetaEvent:
		return "MetaEvent";
	default:
		return "Unknown";
	}
}

static void bhi360_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != BHI360_DEVICE_ID) {
			continue;
		}

		printk("event[bhi360]: %s (%u), v=0x%08x p=%p\n",
			   bhi360_event_to_string(event.event_id),
			   event.event_id,
			   event.v_param,
			   (void*) event.p_param);

		if (event.event_id == bhi360_event_Irq) {
			int ret = bhi360_process_irq();
			if (ret != 0) {
				printk("bhi360_process_irq() failed: %d\n", ret);
			}
			continue;
		}

		if ((event.event_id == bhi360_event_Quaternion) && (event.p_param != 0U)) {
			last_quat = *(const struct bhi360_quat_data*) (uintptr_t) event.p_param;
			atomic_set(&have_quat, 1);
			printk("  quaternion sensor=%u\n", event.v_param);
		} else if ((event.event_id == bhi360_event_LinearAcceleration) && (event.p_param != 0U)) {
			last_lacc = *(const struct bhi360_lacc_data*) (uintptr_t) event.p_param;
			atomic_set(&have_lacc, 1);
			printk("  linear-accel sensor=%u\n", event.v_param);
		} else if ((event.event_id == bhi360_event_Gyro) && (event.p_param != 0U)) {
			last_gyro = *(const struct bhi360_gyro_data*) (uintptr_t) event.p_param;
			atomic_set(&have_gyro, 1);
			printk("  gyro sensor=%u\n", event.v_param);
		} else if ((event.event_id == bhi360_event_Pedometer) && (event.p_param != 0U)) {
			last_pedometer = *(const struct bhi360_pedometer_data*) (uintptr_t) event.p_param;
			atomic_set(&have_pedometer, 1);
			printk("  pedometer sensor=%u\n", event.v_param);
		} else if ((event.event_id == bhi360_event_Gesture) && (event.p_param != 0U)) {
			last_gesture = *(const struct bhi360_gesture_data*) (uintptr_t) event.p_param;
			atomic_set(&have_gesture, 1);
			printk("  gesture sensor=%u value=0x%02x\n",
				   last_gesture.sensor_id,
				   last_gesture.value);
		} else if ((event.event_id == bhi360_event_Activity) && (event.p_param != 0U)) {
			last_activity = *(const struct bhi360_activity_data*) (uintptr_t) event.p_param;
			atomic_set(&have_activity, 1);
			printk("  activity sensor=%u activity=0x%04x\n",
				   last_activity.sensor_id,
				   last_activity.activity);
		} else if (event.event_id == bhi360_event_MetaEvent) {
			last_meta_event = event.v_param;
			atomic_set(&have_meta, 1);
			printk("  meta type=0x%02x byte1=0x%02x byte2=0x%02x\n",
				   (uint8_t) (event.v_param >> 16),
				   (uint8_t) (event.v_param >> 8),
				   (uint8_t) event.v_param);
		}
	}
}

static bool start_bhi360_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  bhi360_event_consumer_thread,
								  NULL,
								  NULL,
								  NULL,
								  TEST_EVENT_THREAD_PRIO,
								  0,
								  K_NO_WAIT);

	if (tid == NULL) {
		printk("failed to start event consumer thread\n");
		return false;
	}

	k_thread_name_set(tid, "bhi360_evt");
	return true;
}

int main(void) {
	printk("\n=== BHI360 IMU driver test ===\n");

	if (!start_bhi360_event_consumer()) {
		printk("failed to start BHI360 event consumer\n");
		return 0;
	}

	if (!bhi360_init()) {
		printk("bhi360_init() failed\n");
		return 0;
	}

	if (!bhi360_configure()) {
		printk("bhi360_configure() failed\n");
		return 0;
	}

	printk("streaming; printing latest sample every 2 s and consuming events...\n");

	while (1) {
		if (atomic_get(&have_quat)) {
			printk("quat:  x=%6d y=%6d z=%6d w=%6d acc=%u\n",
				   last_quat.x,
				   last_quat.y,
				   last_quat.z,
				   last_quat.w,
				   last_quat.accuracy);
		} else {
			printk("quat:  <no sample yet>\n");
		}

		if (atomic_get(&have_lacc)) {
			printk("lacc:  x=%6d y=%6d z=%6d\n", last_lacc.x, last_lacc.y, last_lacc.z);
		} else {
			printk("lacc:  <no sample yet>\n");
		}

		if (atomic_get(&have_gyro)) {
			printk("gyro:  x=%6d y=%6d z=%6d\n", last_gyro.x, last_gyro.y, last_gyro.z);
		} else {
			printk("gyro:  <no sample yet>\n");
		}

		if (atomic_get(&have_pedometer)) {
			printk("pedometer: sensor=%u count=%u detected=%u\n",
				   last_pedometer.sensor_id,
				   last_pedometer.step_count,
				   last_pedometer.step_detected ? 1 : 0);
		} else {
			printk("pedometer: <no event yet>\n");
		}

		if (atomic_get(&have_gesture)) {
			printk("gesture: sensor=%u value=0x%02x\n", last_gesture.sensor_id, last_gesture.value);
		} else {
			printk("gesture: <no event yet>\n");
		}

		if (atomic_get(&have_activity)) {
			printk("activity: sensor=%u activity=0x%04x\n",
				   last_activity.sensor_id,
				   last_activity.activity);
		} else {
			printk("activity: <no event yet>\n");
		}

		if (atomic_get(&have_meta)) {
			printk("meta: v=0x%08x\n", last_meta_event);
		} else {
			printk("meta: <no event yet>\n");
		}

		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
