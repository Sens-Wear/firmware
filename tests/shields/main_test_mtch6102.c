/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SenseWear MTCH6102 touch driver on the sensewear_touch
 * shield.
 *
 * The MTCH6102 sits on the daughter-board connector, so it owns the TPSM83102
 * (VDD_DAUGHTER) rail through its devicetree vin-supply phandle; the driver
 * powers that rail itself when acquisition starts. The test brings the
 * controller up through the driver lifecycle:
 *
 *   mtch6102_init()   - verify the shared bus, validate the rail, probe the
 *                       firmware ID, claim the daughter_if INT/SYNC lines, and
 *                       arm the INT interrupt
 *   mtch6102_config() - apply the SenseWear default configuration (NULL =
 *                       defaults); the I2C address register is never written
 *   mtch6102_start()  - power the rail; the INT line is already armed, so the
 *                       device begins asserting INT on touch
 *
 * Acquisition is interrupt driven. The MTCH6102 INT line posts ::mtch6102_Irq
 * from ISR context; a dedicated consumer thread drains the shared device-event
 * queue and, on that event, calls mtch6102_irq_handler() to read the touch and
 * gesture state. The handler classifies the sample and publishes a decoded
 * `mtch6102_event_*` identifier, carrying the raw TOUCH_STATE byte in `v_param`
 * and a pointer to the decoded ::touch_sensor_sample in `p_param`, which the
 * consumer prints. This mirrors the consumer-thread structure used by the
 * tests/shields MAX30101 bring-up test.
 *
 * As a bench fallback (in case the INT line is not wired on the rig) the handler
 * is also polled whenever the event wait times out; it republishes a decoded
 * event so the test still makes progress.
 *
 * Build with the `test_mtch6102` preset (which also enables the shield); see
 * tests/shields/README.md.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "mtch6102.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"
#include "zephyr/drivers/regulator.h"

#define TEST_EVENT_WAIT_MS 1000
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

/* Print the decoded touch sample carried by a classified MTCH6102 event. The
 * driver delivers the sample by pointer in p_param; it lives in the driver's
 * static context and is valid until the next interrupt read, so it is safe to
 * read here in the consumer thread. */
static void print_sample(const struct device_driver_event_t* event) {
	const struct touch_sensor_sample* sample = (const struct touch_sensor_sample*) event->p_param;

	if (sample == NULL) {
		return;
	}

	printk("  touched=%d  x=%u  y=%u  touch_state=0x%02x  gesture=0x%02x\n",
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
				/* INT asserted: read the touch/gesture state and publish a
				 * decoded event, delivered to this loop on the next wait. */
				mtch6102_irq_handler();
				continue;
			}

			/* Classified touch/gesture event: name it and print the sample. */
			printk("event: %s (touch_state=0x%02x)\n",
				   mtch6102_event_name((enum mtch6102_event_type) event.event_id),
				   event.v_param);
			print_sample(&event);
		} else {
			/* No interrupt arrived; poll the handler so the test still makes
			 * progress on rigs where INT is not wired. The handler publishes a
			 * decoded event, which the next wait will deliver. */
			// printk("Polling mtch6102_irq_handler()...\n");
			// mtch6102_irq_handler();
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
	printk("\n=== MTCH6102 touch test (sensewear_touch shield) ===\n");

	/* The driver owns the VDD_DAUGHTER rail and validates its state during
	 * init; start from a known-off rail so init powers it cleanly. */
	const struct device* const regulator = DEVICE_DT_GET(DT_NODELABEL(tpsm83102));
	regulator_disable(regulator);

	if (mtch6102_init() != 0) {
		printk("mtch6102_init() failed\n");
		return 0;
	}

	if (!mtch6102_is_ready()) {
		printk("MTCH6102 not ready\n");
		return 0;
	}

	if (mtch6102_config(NULL) != 0) {
		printk("mtch6102_config() failed\n");
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
