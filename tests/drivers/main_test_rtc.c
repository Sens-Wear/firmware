/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the SenseWear software RTC.
 *
 * Sets a known wall-clock time, reads it back through both the rtc API and the
 * POSIX time()/gmtime_r() path to confirm they agree, then arms the recurring
 * minute alarm and a one-shot alarm a few seconds out and prints each event as
 * the device-event consumer thread receives it.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include <time.h>

#include "rtc.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE  2048
#define TEST_ONESHOT_DELAY_SEC 5

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static void rtc_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != RTC0_DEVICE_DTS_ID) {
			continue;
		}

		struct tm now;
		(void) rtc_get_time(&now);

		printk("event[rtc]: %s (%u) at %04d-%02d-%02d %02d:%02d:%02d UTC (unix=%u)\n",
			   rtc_event_name(event.event_id),
			   event.event_id,
			   now.tm_year + 1900,
			   now.tm_mon + 1,
			   now.tm_mday,
			   now.tm_hour,
			   now.tm_min,
			   now.tm_sec,
			   event.v_param);
	}
}

int main(void) {
	if (!rtc_init()) {
		printk("rtc_init failed\n");
		return 0;
	}

	/* 2026-06-29 12:34:50 UTC. */
	struct tm set = {
		.tm_year = 2026 - 1900,
		.tm_mon = 6 - 1,
		.tm_mday = 29,
		.tm_hour = 12,
		.tm_min = 34,
		.tm_sec = 50,
	};

	if (!rtc_set_time(&set)) {
		printk("rtc_set_time failed\n");
		return 0;
	}

	/* Read back through the rtc API and through the POSIX wall clock. */
	struct tm via_rtc;
	(void) rtc_get_time(&via_rtc);

	time_t posix_now = time(NULL);
	struct tm via_posix;
	gmtime_r(&posix_now, &via_posix);

	printk("rtc  : %04d-%02d-%02d %02d:%02d:%02d UTC\n",
		   via_rtc.tm_year + 1900, via_rtc.tm_mon + 1, via_rtc.tm_mday,
		   via_rtc.tm_hour, via_rtc.tm_min, via_rtc.tm_sec);
	printk("posix: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
		   via_posix.tm_year + 1900, via_posix.tm_mon + 1, via_posix.tm_mday,
		   via_posix.tm_hour, via_posix.tm_min, via_posix.tm_sec);
	printk("timestamp_ms=%lld timestamp_us=%lld\n",
		   (long long) rtc_get_timestamp_ms(),
		   (long long) rtc_get_timestamp_us());

	k_thread_create(&test_event_thread, test_event_stack, TEST_EVENT_STACK_SIZE,
					rtc_event_consumer_thread, NULL, NULL, NULL,
					TEST_EVENT_THREAD_PRIO, 0, K_NO_WAIT);

	(void) rtc_enable_minute_alarm(true);
	(void) rtc_set_time_alarm_unix(rtc_get_unix() + TEST_ONESHOT_DELAY_SEC);

	printk("Armed minute alarm and one-shot alarm (+%d s). Waiting for events...\n",
		   TEST_ONESHOT_DELAY_SEC);

	return 0;
}
