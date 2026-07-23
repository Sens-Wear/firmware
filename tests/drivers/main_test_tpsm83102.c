/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the TPSM83102 regulator driver.
 *
 * The driver is a standard Zephyr regulator device (managed through devicetree
 * and the regulator framework), so this test drives it through the public
 * regulator API rather than a driver-private interface:
 *
 *   init (device ready) -> set minimum voltage -> read it back -> enable ->
 *   ramp to maximum -> ramp back to minimum.
 *
 * Every 25-mV ramp step is written first and then read back through the public
 * regulator API.
 *
 * The driver also publishes lifecycle events (Enabled / VOUT_UPDATED /
 * Disabled) into the SensWear device-event manager, tagged with its own
 * generated id (TPSM83102_DEVICE_DTS_ID) -- the caller does not supply one. A
 * dedicated consumer thread drains the manager and prints those events,
 * mirroring the bq25180 / bq27427 tests.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>

#include "tpsm83102.h"
#include "tpsm83102_registers.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define TEST_RAMP_INTERVAL K_MSEC(100)
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 4096

#define TEST_MIN_UV ((int32_t) TPSM83102_VOUT_MIN_UV)
#define TEST_MAX_UV ((int32_t) TPSM83102_VOUT_MAX_UV)
#define TEST_STEP_UV ((int32_t) TPSM83102_VOUT_STEP_UV)

static const struct device* const reg = DEVICE_DT_GET(DT_NODELABEL(tpsm83102));

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

static const char* tpsm83102_event_to_string(uint32_t event_id) {
	switch ((enum tpsm83102_event_type) event_id) {
	case tpsm83102_event_Enabled:
		return "Enabled";
	case tpsm83102_event_VOUT_UPDATED:
		return "VOUT_UPDATED";
	case tpsm83102_event_Disabled:
		return "Disabled";
	default:
		return "Unknown";
	}
}

static void tpsm83102_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (!device_driver_event_wait(K_FOREVER, &event)) {
			continue;
		}

		if (event.device_id != TPSM83102_DEVICE_DTS_ID) {
			printk("event[other]: dev=%u id=%u v=%u p=%lu\n",
				   event.device_id,
				   event.event_id,
				   event.v_param,
				   event.p_param);
			continue;
		}

		printk("event[tpsm83102]: %s (%u), VOUT=%u uV\n",
			   tpsm83102_event_to_string(event.event_id),
			   event.event_id,
			   event.v_param);
	}
}

static bool start_tpsm83102_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  tpsm83102_event_consumer_thread,
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

	k_thread_name_set(tid, "tpsm83102_evt");
	return true;
}

static int set_and_read_voltage(int32_t target_uv) {
	int32_t readback_uv = 0;
	int ret = regulator_set_voltage(reg, target_uv, target_uv);

	if (ret != 0) {
		printk("regulator_set_voltage(%d uV) failed: %d\n", target_uv, ret);
		return ret;
	}

	ret = regulator_get_voltage(reg, &readback_uv);
	if (ret != 0) {
		printk("regulator_get_voltage() after setting %d uV failed: %d\n", target_uv, ret);
		return ret;
	}

	// printk("tpsm83102: set=%d uV readback=%d uV (%d.%03d V)\n",
	//        target_uv, readback_uv,
	//        readback_uv / 1000000, (readback_uv / 1000) % 1000);

	if (readback_uv != target_uv) {
		printk("voltage readback mismatch: expected %d uV, got %d uV\n", target_uv, readback_uv);
		return -EIO;
	}

	return 0;
}

int main(void) {
	int ret;

	printk("\n=== TPSM83102 regulator driver test ===\n");

	if (!start_tpsm83102_event_consumer()) {
		printk("failed to start TPSM83102 event consumer\n");
		return 0;
	}

	if (!device_is_ready(reg)) {
		printk("regulator device %s not ready\n", reg->name);
		return 0;
	}
	printk("regulator %s ready\n", reg->name);

	ret = set_and_read_voltage(TEST_MIN_UV);
	if (ret != 0) {
		return 0;
	}

	ret = regulator_enable(reg);
	if (ret != 0) {
		printk("regulator_enable() failed: %d\n", ret);
		return 0;
	}

	/* Exercise the disabled set-voltage path used by the MAX30101 when BLE
	 * sampling is stopped and then started again. */
	ret = regulator_disable(reg);
	if (ret != 0) {
		printk("regulator_disable() failed: %d\n", ret);
		return 0;
	}
	ret = set_and_read_voltage(TEST_MAX_UV);
	if (ret != 0) {
		return 0;
	}
	ret = regulator_enable(reg);
	if (ret != 0) {
		printk("regulator re-enable failed: %d\n", ret);
		return 0;
	}
	printk("regulator enabled; ramping from %d uV to %d uV and back...\n",
		   TEST_MIN_UV,
		   TEST_MAX_UV);

	while (1) {
		for (int32_t uv = TEST_MIN_UV + TEST_STEP_UV; uv <= TEST_MAX_UV; uv += TEST_STEP_UV) {
			ret = set_and_read_voltage(uv);
			if (ret != 0) {
				return 0;
			}
			k_sleep(TEST_RAMP_INTERVAL);
		}

		for (int32_t uv = TEST_MAX_UV - TEST_STEP_UV; uv >= TEST_MIN_UV; uv -= TEST_STEP_UV) {
			ret = set_and_read_voltage(uv);
			if (ret != 0) {
				return 0;
			}
			k_sleep(TEST_RAMP_INTERVAL);
		}
	}

	return 0;
}
