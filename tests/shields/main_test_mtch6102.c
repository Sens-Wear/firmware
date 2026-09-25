/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SensWear MTCH6102 touch driver on the senswear_touch
 * shield.
 *
 * init() powers and probes the controller, config() selects acquisition,
 * and start() enables SYNC frame-complete events plus recovery polling.
 * The consumer handles coalesced mtch6102_Irq requests and prints host-decoded
 * slider position/gestures from immutable, owned sample events. Every sample
 * is released after printing. See boards/shields/senswear_touch/README.md for
 * physical mapping and the complete hardware validation procedure.
 *
 * Build with the test_mtch6102 preset; this is an on-target test.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "mtch6102.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"
#include "sys_i2c.h"
#include "tpsm83102_registers.h"

#define TEST_EVENT_WAIT_MS 1000
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

/* Only run after a failed probe in this isolated bring-up image. These reads
 * do not change rail
 * settings or device addresses. The scan reads one byte
 * at each non-reserved address; it may
 * consume pending peripheral status.
 */
static void print_probe_diagnostics(void) {
	static struct sys_i2c_dt_spec bus = SYS_I2C_DT_SPEC_GET(DT_NODELABEL(tpsm83102));
	const uint8_t registers[] = {
		tpsm83102_register_CONTROL1,
		tpsm83102_register_VOUT,
		tpsm83102_register_CONTROL2,
	};
	int ret = sys_i2c_lock(&bus, K_MSEC(100));
	if (ret != 0) {
		printk("diagnostic bus lock failed: %d\n", ret);
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(registers); ++i) {
		uint8_t value = 0;
		ret = sys_i2c_write_read(&bus, &registers[i], 1, &value, 1);
		printk("regulator register 0x%02x: ret=%d value=0x%02x\n", registers[i], ret, value);
	}
	ret = sys_i2c_release(&bus);
	if (ret != 0) {
		printk("diagnostic bus release failed: %d\n", ret);
		return;
	}
	for (uint16_t address = 0x08; address <= 0x77; ++address) {
		bus.config.addr = address;
		ret = sys_i2c_lock(&bus, K_MSEC(100));
		if (ret != 0) {
			printk("scan bus lock failed: %d\n", ret);
			break;
		}
		uint8_t value;
		int read_ret = sys_i2c_read(&bus, &value, 1);
		ret = sys_i2c_release(&bus);
		if (read_ret == 0) {
			printk("I2C read ACK at 0x%02x\n", address);
		}
		if (ret != 0) {
			printk("scan bus release failed: %d\n", ret);
			break;
		}
	}
	printk("Probe diagnostics complete; regulator readback is not a voltage measurement.\n");
}

/* The queued event owns this immutable sample until the consumer releases it. */
static void print_sample(const struct device_driver_event_t* event) {
	const struct touch_sensor_sample_t* sample =
		(const struct touch_sensor_sample_t*) event->p_param;

	if (sample == NULL) {
		return;
	}

	printk("  ts=%lld us  touched=%d  x=%u  y=%u  touch_state=0x%02x  gesture=0x%02x\n",
		   (long long) sample->timestamp,
		   sample->position.touched,
		   sample->position.x,
		   sample->position.y,
		   sample->position.touch_state,
		   sample->gesture_state);
}

static void mtch6102_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (device_driver_event_wait(K_MSEC(TEST_EVENT_WAIT_MS), &event)) {
			if (event.device_id != MTCH6102_DEVICE_DTS_ID) {
				continue;
			}

			if (event.event_id == mtch6102_Irq) {
				/* Read the completed acquisition frame and publish its host sample. */
				mtch6102_irq_handler();
				continue;
			}

			/* Classified touch/gesture event: name it and print the sample. */
			printk("event: %s (touch_state=0x%02x)\n",
				   mtch6102_event_name((enum mtch6102_event_type) event.event_id),
				   event.v_param);
			print_sample(&event);
			mtch6102_release_sample((const struct touch_sensor_sample_t*) event.p_param);
		}
	}
}

static bool start_mtch6102_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  mtch6102_event_consumer_thread,
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

	k_thread_name_set(tid, "mtch6102_evt");
	return true;
}

int main(void) {
	printk("\n=== MTCH6102 touch test (senswear_touch shield) ===\n");

	int ret = mtch6102_init();
	if (ret != 0) {
		printk("mtch6102_init() failed: %d\n", ret);
		print_probe_diagnostics();
		return 0;
	}

	if (mtch6102_config(NULL) != 0) {
		printk("mtch6102_config() failed\n");
		return 0;
	}

	if (!mtch6102_is_ready()) {
		printk("MTCH6102 not ready after configuration\n");
		return 0;
	}

	if (mtch6102_start() != 0) {
		printk("mtch6102_start() failed\n");
		return 0;
	}

	if (!start_mtch6102_event_consumer()) {
		printk("failed to start MTCH6102 event consumer\n");
		return 0;
	}

	printk(
		"Acquisition started; touch the pad to see decoded events from the consumer thread...\n");

	return 0;
}
