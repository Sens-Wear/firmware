/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BHI360 IMU driver.
 *
 * Starts the sensor and consumes BHI360 device events. Sample events carry
 * driver-owned payload pointers in p_param; packed metadata rides in v_param.
 * Sample payloads also include timestamps in microseconds since BHI360 firmware
 * boot.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include "bhi360.h"
#include "device_driver_dts_ids.h"
#include "device_driver_events.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(bhi360_test, CONFIG_LOG_DEFAULT_LEVEL);

#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static void bhi360_event_consumer_thread(void* a, void* b, void* c) {
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

		const char* event_name = bhi360_event_name(event.event_id, event.v_param);

		if (event.event_id == bhi360_event_Irq) {
			int ret = bhi360_process_irq();
			if ((ret != 0) && (ret != -ENODEV)) {
				printk("bhi360_process_irq() failed: %d\n", ret);
			}
			continue;
		}

		if ((event.event_id == bhi360_event_QuaternionBatch) && (event.p_param != 0U)) {
			/* v_param is the number of samples in this batch; p_param points at the
			 * first of that many. Print the count and the most recent sample only,
			 * so the consumer stays fast and the FIFO does not back up. */
			uint32_t count = event.v_param;
			const struct bhi360_quat_data_t* quat =
				(const struct bhi360_quat_data_t*) (uintptr_t) event.p_param;
			const struct bhi360_quat_data_t* last = &quat[count - 1U];
			printk("  %s n=%u last: t=%lld us x=%6d y=%6d z=%6d w=%6d acc=%u\n",
				   event_name,
				   count,
				   (long long) last->timestamp,
				   last->x,
				   last->y,
				   last->z,
				   last->w,
				   last->accuracy);
		} else if ((event.event_id == bhi360_event_LinearAccelerationBatch) &&
				   (event.p_param != 0U)) {
			uint32_t count = event.v_param;
			const struct bhi360_lacc_data_t* lacc =
				(const struct bhi360_lacc_data_t*) (uintptr_t) event.p_param;
			const struct bhi360_lacc_data_t* last = &lacc[count - 1U];
			printk("  %s n=%u last: t=%lld us x=%6d y=%6d z=%6d\n",
				   event_name,
				   count,
				   (long long) last->timestamp,
				   last->x,
				   last->y,
				   last->z);
		} else if ((event.event_id == bhi360_event_GyroBatch) && (event.p_param != 0U)) {
			uint32_t count = event.v_param;
			const struct bhi360_gyro_data_t* gyro =
				(const struct bhi360_gyro_data_t*) (uintptr_t) event.p_param;
			const struct bhi360_gyro_data_t* last = &gyro[count - 1U];
			printk("  %s n=%u last: t=%lld us x=%6d y=%6d z=%6d\n",
				   event_name,
				   count,
				   (long long) last->timestamp,
				   last->x,
				   last->y,
				   last->z);
		} else if ((event.event_id == bhi360_event_Pedometer) && (event.p_param != 0U)) {
			const struct bhi360_pedometer_data_t* pedometer =
				(const struct bhi360_pedometer_data_t*) (uintptr_t) event.p_param;
			printk("  %s sensor=%u t=%lld us count=%u detected=%u\n",
				   event_name,
				   event.v_param,
				   (long long) pedometer->timestamp,
				   pedometer->step_count,
				   pedometer->step_detected ? 1 : 0);
		} else if ((event.event_id == bhi360_event_Gesture) && (event.p_param != 0U)) {
			const struct bhi360_gesture_data_t* gesture =
				(const struct bhi360_gesture_data_t*) (uintptr_t) event.p_param;
			printk("  %s sensor=%u t=%lld us value=0x%02x\n",
				   event_name,
				   gesture->sensor_id,
				   (long long) gesture->timestamp,
				   gesture->value);
		} else if ((event.event_id == bhi360_event_Activity) && (event.p_param != 0U)) {
			const struct bhi360_activity_data_t* activity =
				(const struct bhi360_activity_data_t*) (uintptr_t) event.p_param;
			printk("  %s sensor=%u t=%lld us activity=0x%04x\n",
				   event_name,
				   activity->sensor_id,
				   (long long) activity->timestamp,
				   activity->activity);
		} else if (event.event_id == bhi360_event_MetaEvent) {
			printk("  %s type=0x%02x byte1=0x%02x byte2=0x%02x\n",
				   event_name,
				   (uint8_t) (event.v_param >> 16),
				   (uint8_t) (event.v_param >> 8),
				   (uint8_t) event.v_param);
		} else {
			printk("event[bhi360]: %s (%u), v=0x%08x p=%p\n",
				   event_name,
				   event.event_id,
				   event.v_param,
				   (void*) event.p_param);
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

	if (!bhi360_init()) {
		printk("bhi360_init() failed\n");
		return 0;
	}

	if (!bhi360_configure(false, 0)) {
		printk("bhi360_configure() failed\n");
		return 0;
	}

	if (!start_bhi360_event_consumer()) {
		printk("failed to start BHI360 event consumer\n");
		return 0;
	}

	printk("streaming; consuming and printing events from consumer thread...\n");

	uint32_t startTime = k_uptime_get_32();
	bool streamsStarted = false;
	bool streamsStopped = false;
	while (1) {
		uint32_t currentTime = k_uptime_get_32();
		uint32_t elapsedTime = currentTime - startTime;
		if ((elapsedTime >= 10000) && !streamsStarted) {
			int ret = bhi360_start_phy_sensor_streams(250);
			if (ret == 0) {
				LOG_INF("Started physical sensor streams after 10 seconds");
				streamsStarted = true;
			} else {
				LOG_ERR("Failed to start physical sensor streams: %d", ret);
			}
		}

		if ((elapsedTime >= 30000) && streamsStarted && !streamsStopped) {
			int ret = bhi360_stop_phy_sensor_streams();
			if (ret == 0) {
				LOG_INF("Stopped physical sensor streams after 30 seconds");
				streamsStopped = true;
				ret = bhi360_start_periodic_timer(1000);
			} else {
				LOG_ERR("Failed to stop physical sensor streams: %d", ret);
			}
		}

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
