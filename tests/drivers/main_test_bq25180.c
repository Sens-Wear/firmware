/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BQ25180 charger driver.
 *
 * Initialises the charger, applies the default Li-Po / USB charger
 * configuration, then consumes driver events and refreshes the decoded charger
 * state when an interrupt is reported. The cached update and IRQ timestamps are
 * printed in Unix-epoch microseconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "bq25180.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define TEST_IDLE_INTERVAL K_SECONDS(2)
#define TEST_EVENT_THREAD_PRIO 7
/* Immediate-mode logging (CONFIG_LOG_MODE_IMMEDIATE) formats and outputs on the
 * calling thread's stack, so the consumer needs room for bq25180_print_state()'s
 * large LOG_INF. Sized to match the main/system-workqueue stacks. */
#define TEST_EVENT_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static void bq25180_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != BQ25180_DEVICE_DTS_ID) {
			printk("event[other]: dev=%u id=%u v=%u p=%lu\n",
				   event.device_id,
				   event.event_id,
				   event.v_param,
				   event.p_param);
			continue;
		}

		printk("event[bq25180]: %s (%u), v=%u p=%lu\n",
			   bq25180_event_name(event.event_id),
			   event.event_id,
			   event.v_param,
			   event.p_param);

		if (event.event_id == bq25180_event_Irq) {
			struct bq25180_charger_state_t state;

			if (bq25180_update_state(&state) == 0) {
				bq25180_print_state(&state.state);
				printk("state refreshed at %lld us, irq at %lld us\n",
					   (long long) state.last_update_time,
					   (long long) bq25180_get_last_irq_timestamp_ms());
				printk("============================================\n");
			} else {
				printk("bq25180_update_state() failed\n");
			}
			continue;
		}

		bq25180_print_state(NULL);
	}
}

static bool start_bq25180_event_consumer(void) {
	if (device_driver_event_get_queue() == NULL) {
#ifdef CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS_MAX
		int ret = device_driver_event_init(CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS_MAX);
		if (ret != 0 && ret != -EALREADY) {
			printk("device_driver_event_init() failed: %d\n", ret);
			return false;
		}
#else
		printk("device driver event config symbols are unavailable\n");
		return false;
#endif
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  bq25180_event_consumer_thread,
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

	k_thread_name_set(tid, "bq25180_evt");
	return true;
}

int main(void) {
	printk("\n=== BQ25180 charger driver test ===\n");

	if (!start_bq25180_event_consumer()) {
		printk("failed to start BQ25180 event consumer\n");
		return 0;
	}

	if (!bq25180_init()) {
		printk("bq25180_init() failed\n");
		return 0;
	}

	if (!bq25180_is_ready()) {
		printk("bq25180_is_ready() == false\n");
		return 0;
	}

	struct bq25180_config_t config;

	bq25180_get_default_lipo_usb_charger_config(&config);
	config.battery_uvlo = bq25180_battery_UVLO_threshold_2V8;

	if (!bq25180_config(&config)) {
		printk("bq25180_config() failed\n");
		return 0;
	}

	printk("configured; consuming driver events and refreshing state on IRQs...\n");

	while (1) {
		k_sleep(TEST_IDLE_INTERVAL);
	}

	return 0;
}
