/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SenseWear MAX30208 temperature driver on the
 * sensewear_temperature shield.
 *
 * Unlike the PPG daughter board, the temperature daughter board exposes neither
 * a regulator nor a daughter-connector interrupt line to this driver: the rail
 * is owned by the daughter-board manager and there is no hardware INT pin. The
 * driver therefore substitutes a periodic software timer for the missing
 * interrupt line. The test brings the sensor up through the driver lifecycle:
 *
 *   max30208_init()  - verify the shared bus, probe PART_ID, program the FIFO
 *   max30208_start() - flush the buffer, start the sampling timer, and publish
 *                      max30208_event_SamplingStarted
 *
 * Acquisition is timer driven. The MAX30208 sampling timer posts
 * ::max30208_TimerIrq from timer context; a dedicated consumer thread drains the
 * shared device-event queue and, on that event, calls max30208_get_samples(),
 * which performs the conversion, appends it to the internal buffer, and
 * publishes ::max30208_event_SampleReady, carrying the sample count in `v_param`
 * and a pointer to the decoded ::temperature_sample array in `p_param`, which the
 * consumer prints. This mirrors the consumer-thread structure used by the
 * tests/shields MAX30101 bring-up test.
 *
 * Build with the `test_max30208` preset (which also enables the shield); see
 * tests/shields/README.md.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "max30208.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define TEST_EVENT_WAIT_MS 2000
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

/* Print the decoded samples carried by a max30208_event_SampleReady event.
 * The driver delivers the sample array by pointer in p_param and the sample
 * count in v_param; the array lives in the driver's static context and is valid
 * until the next conversion, so it is safe to read here in the consumer thread. */
static void print_samples(const struct device_driver_event_t* event) {
	const struct temperature_sample* samples = (const struct temperature_sample*) event->p_param;

	if (samples == NULL) {
		return;
	}

	for (uint32_t i = 0; i < event->v_param; i++) {
		int32_t mdeg = samples[i].temperature_mdeg_c;

		printk("  T=%d.%03d C\n", mdeg / 1000, (mdeg < 0 ? -mdeg : mdeg) % 1000);
	}
}

static void max30208_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (device_driver_event_wait(K_MSEC(TEST_EVENT_WAIT_MS), &event)) {
			if (event.device_id != MAX30208_DEVICE_DTS_ID) {
				continue;
			}

			switch (event.event_id) {
			case max30208_TimerIrq:
				/* Sampling tick: perform a conversion and drain the buffer. The
				 * driver republishes max30208_event_SampleReady below. */
				(void) max30208_get_samples(NULL, 0);
				break;
			case max30208_event_SampleReady:
				printk("SampleReady (%u sample(s))\n", event.v_param);
				print_samples(&event);
				break;
			case max30208_event_SamplingStarted:
				printk("event: %s\n", max30208_event_name(event.event_id));
				break;
			case max30208_event_SamplingStopped:
				printk("event: %s\n", max30208_event_name(event.event_id));
				break;
			default:
				printk("event: %s (%u)\n", max30208_event_name(event.event_id), event.event_id);
				break;
			}
		} else {
			/* No timer tick arrived; poll a conversion so the test still makes
			 * progress. get_samples() republishes max30208_event_SampleReady,
			 * which the next wait will deliver. */
			(void) max30208_get_samples(NULL, 0);
		}
	}
}

static bool start_max30208_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  max30208_event_consumer_thread,
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

	k_thread_name_set(tid, "max30208_evt");
	return true;
}

int main(void) {
	printk("\n=== MAX30208 temperature test (sensewear_temperature shield) ===\n");

	if (max30208_init() != 0) {
		printk("max30208_init() failed\n");
		return 0;
	}

	if (!max30208_is_ready()) {
		printk("MAX30208 not ready\n");
		return 0;
	}

	if (!start_max30208_event_consumer()) {
		printk("failed to start MAX30208 event consumer\n");
		return 0;
	}

	if (max30208_start() != 0) {
		printk("max30208_start() failed\n");
		return 0;
	}

	printk("Sampling started; consuming and printing temperature samples from consumer thread...\n");

	return 0;
}
