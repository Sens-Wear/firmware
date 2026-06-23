/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BQ27427 fuel-gauge driver.
 *
 * Initialises the gauge, applies the default configuration with a known
 * battery capacity, then polls the battery state every 2 seconds. Each poll
 * triggers a StateUpdated driver event; a dedicated consumer thread drains the
 * device event manager and prints the latest battery state, mirroring the
 * BQ25180 test consumer.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "bq27427.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define TEST_BATTERY_CAPACITY 450 /* mAh */
#define TEST_POLL_INTERVAL K_SECONDS(2)
#define TEST_EVENT_THREAD_PRIO 7
/* Immediate-mode logging (CONFIG_LOG_MODE_IMMEDIATE) formats and outputs on the
 * calling thread's stack, so the consumer needs room for bq27427_print_state()'s
 * large LOG_INF. Sized to match the main/system-workqueue stacks. */
#define TEST_EVENT_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static void bq27427_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != BQ27427_DEVICE_DTS_ID) {
			printk("event[other]: dev=%u id=%u v=%u p=%p\n",
				   event.device_id,
				   event.event_id,
				   event.v_param,
				   event.p_param);
			continue;
		}

		printk("event[bq27427]: %s (%u), v=%u p=%p\n",
			   bq27427_event_name(event.event_id),
			   event.event_id,
			   event.v_param,
			   event.p_param);
		bq27427_print_state();
	}
}

static bool start_bq27427_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  bq27427_event_consumer_thread,
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

	k_thread_name_set(tid, "bq27427_evt");
	return true;
}

int main(void) {
	printk("\n=== BQ27427 fuel-gauge driver test ===\n");

	if (!start_bq27427_event_consumer()) {
		printk("failed to start BQ27427 event consumer\n");
		return 0;
	}

	if (!bq27427_init()) {
		printk("bq27427_init() failed\n");
		return 0;
	}

	if (!bq27427_is_ready()) {
		printk("bq27427_is_ready() == false\n");
		return 0;
	}

	struct bq27427_config_t config;

	bq27427_get_default_config(&config);
	config.battery_capacity = TEST_BATTERY_CAPACITY;

	if (!bq27427_config(&config)) {
		printk("bq27427_config() failed\n");
		return 0;
	}

	printk("configured; polling state every 2 s and consuming driver events...\n");

	while (1) {
		if (!bq27427_update_state(NULL)) {
			printk("bq27427_update_state() failed\n");
		}

		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
