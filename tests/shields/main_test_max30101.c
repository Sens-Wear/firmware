/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SenseWear MAX30101 PPG driver on the sensewear_ppg
 * shield.
 *
 * The MAX30101 sits on the daughter-board connector, so the sensor's TPSM83102
 * (VDD_DAUGHTER) regulator is passed to max30101_init(), which powers the PPG
 * board's rail before probing. The test then brings the sensor up through the
 * driver lifecycle:
 *
 *   max30101_init()    - power the rail, probe PART_ID, claim the daughter_if IRQ
 *   max30101_config()  - apply the default acquisition configuration
 *   max30101_enable_wrist_hr_sampling(per_sample_irq) - start multi-LED
 *       (IR/Red/Green) sampling; the argument picks the FIFO IRQ cadence
 *       (see TEST_PER_SAMPLE_IRQ): false = per almost-full batch, true = per sample
 *
 * Acquisition is interrupt driven. The MAX30101 INT line posts ::max30101_Irq
 * from ISR context; a dedicated consumer thread drains the shared device-event
 * queue and, on that event, calls max30101_irq_handler() to decode the status
 * and fill the internal sample buffer. The handler then publishes
 * ::max30101_event_FifoDataReady, carrying the sample count in `v_param` and a
 * pointer to the decoded ::max30101_ppg_sample_t array in `p_param`, which the
 * consumer prints. This mirrors the consumer-thread structure used by the
 * tests/drivers bring-up tests.
 *
 * As a bench fallback (in case the INT line is not wired on the rig) the handler
 * is also polled whenever the event wait times out; it republishes
 * ::max30101_event_FifoDataReady so samples still flow on the next iteration.
 *
 * Build with the `test_max30101` preset (which also enables the shield); see
 * tests/shields/README.md.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "max30101.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"
#include "zephyr/drivers/regulator.h"

#define TEST_EVENT_WAIT_MS 1000
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

/* FIFO notification cadence under test: false = one IRQ per FIFO almost-full
 * batch (FifoDataReady carries several samples), true = one IRQ per sample. */
#define TEST_PER_SAMPLE_IRQ true

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

/* Print the decoded samples carried by a max30101_event_FifoDataReady event.
 * The driver delivers the sample array by pointer in p_param and the sample
 * count in v_param; the array lives in the driver's static context and is valid
 * until the next FIFO drain, so it is safe to read here in the consumer thread. */
static void print_samples(const struct device_driver_event_t* event) {
	const struct max30101_ppg_sample_t* samples =
		(const struct max30101_ppg_sample_t*) event->p_param;

	if (samples == NULL) {
		return;
	}

	for (uint32_t i = 0; i < event->v_param; i++) {
		printk("  t=%llu us  IR=%u  Red=%u  Green=%u\n",
			   (unsigned long long) samples[i].timestamp,
			   samples[i].ir,
			   samples[i].red,
			   samples[i].green);
	}
}

static void max30101_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;
	bool sampling_started = false;

	while (1) {
		if (device_driver_event_wait(K_MSEC(TEST_EVENT_WAIT_MS), &event)) {
			if (event.device_id != MAX30101_DEVICE_DTS_ID) {
				continue;
			}

			switch (event.event_id) {
			case max30101_Irq:
				/* INT asserted: decode status and fill the sample buffer. The
				 * handler republishes the decoded interrupt source(s) below. */
				max30101_irq_handler();
				break;
			case max30101_event_PowerReady:
				/* PWR_RDY: device powered up / returned from brown-out. */
				printk("event: %s (PWR_RDY)\n", max30101_event_name(event.event_id));
				break;
			case max30101_event_Proximity:
				/* PROX_INT: proximity threshold crossed. */
				printk("event: %s (PROX_INT)\n", max30101_event_name(event.event_id));
				break;
			case max30101_event_AmbientLightCancelOverflow:
				/* ALC_OVF: ambient-light canceller saturated; readings suspect. */
				printk("event: %s (ALC_OVF)\n", max30101_event_name(event.event_id));
				break;
			case max30101_event_DieTemperatureReady:
				/* DIE_TEMP_RDY: temperature conversion complete. */
				printk("event: %s (DIE_TEMP_RDY)\n", max30101_event_name(event.event_id));
				break;
			case max30101_event_FifoDataReady:
				printk("FifoDataReady (%u samples)\n", event.v_param);
				print_samples(&event);
				break;
			default:
				printk("event: %s (%u)\n", max30101_event_name(event.event_id), event.event_id);
				break;
			}
		} else {
			/* No interrupt arrived; poll the handler so the test still makes
			 * progress on rigs where INT is not wired. The handler republishes
			 * max30101_event_FifoDataReady, which the next wait will deliver. */
			if (!sampling_started) {
				printk("Starting sampling (per_sample_irq=%d)...\n", TEST_PER_SAMPLE_IRQ);
				max30101_enable_wrist_hr_sampling(TEST_PER_SAMPLE_IRQ);
				sampling_started = true;
			} else {
				printk("Polling max30101_irq_handler()...\n");
				max30101_irq_handler();
			}
		}
	}
}

static bool start_max30101_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  max30101_event_consumer_thread,
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

	k_thread_name_set(tid, "max30101_evt");
	return true;
}

int main(void) {
	printk("\n=== MAX30101 PPG test (sensewear_ppg shield) ===\n");

	const struct device* const regulator = DEVICE_DT_GET(DT_NODELABEL(tpsm83102));
	regulator_disable(regulator);

	if (max30101_init() != 0) {
		printk("max30101_init() failed\n");
		return 0;
	}

	if (!max30101_is_ready()) {
		printk("MAX30101 not ready\n");
		return 0;
	}

	if (max30101_config(NULL) != 0) {
		printk("max30101_config() failed\n");
		return 0;
	}

	printk("MAX30101 configured: %d LED channel(s), ~%d Hz/channel\n",
		   max30101_get_led_count(),
		   (int) max30101_get_sampling_rate());

	if (!start_max30101_event_consumer()) {
		printk("failed to start MAX30101 event consumer\n");
		return 0;
	}

	printk(
		"Sampling started; consuming and printing IR/Red/Green samples from consumer thread...\n");

	return 0;
}
