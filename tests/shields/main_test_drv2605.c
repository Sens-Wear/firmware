/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SensWear-patched DRV2605 haptic driver on the
 * senswear_haptic shield.
 *
 * The driver auto-initialises at POST_KERNEL (claims the daughter_if GPIO0
 * enable line, resets and configures the device), so this test first checks
 * that the haptics device is ready through the Zephyr haptics API and then
 * exercises the two playback paths:
 *
 *   RTP - a real-time amplitude ramp streamed frame-by-frame from host buffers.
 *   ROM - a single waveform from the on-chip LRA library, triggered with the
 *         internal GO bit and played autonomously by the device.
 *
 * The test runs both patterns in sequence with a 2 s pause after each one.
 * Build with the `test_drv2605` preset (which also enables the shield); see
 * tests/shields/README.md.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <errno.h>

#include "drv2605.h"
#include "device_driver_dts_ids.h"
#include "device_driver_events.h"

#define TEST_CYCLE_PERIOD K_SECONDS(2)
#define TEST_PLAY_TIMEOUT_MS 2000
#define TEST_PLAY_POLL_MS 10

static const struct device* const haptic_dev = DEVICE_DT_GET(DT_ALIAS(senswear_haptic));

/*
 * RTP "buzz": ramp the amplitude up then drop to off. RTP playback runs
 * asynchronously and streams directly from these buffers, so they are static and
 * stay valid for the whole stream.
 */
static uint32_t rtp_hold_us[] = {150000, 150000, 150000, 150000};
static uint8_t rtp_input[] = {80, 160, 255, 0};
static struct drv2605_rtp_data rtp_buzz = {
	.size = ARRAY_SIZE(rtp_input),
	.rtp_hold_us = rtp_hold_us,
	.rtp_input = rtp_input,
};

/* ROM: a single waveform (effect 1) from the LRA library, terminated by 0. */
static struct drv2605_rom_data rom_click = {
	.trigger = DRV2605_MODE_INTERNAL_TRIGGER,
	.library = DRV2605_LIBRARY_LRA,
	.seq_regs = {1, 0},
};

static int wait_until_idle(void) {
	int elapsed_ms;

	for (elapsed_ms = 0; elapsed_ms < TEST_PLAY_TIMEOUT_MS; elapsed_ms += TEST_PLAY_POLL_MS) {
		int active = drv2605_is_active(haptic_dev);

		if (active < 0) {
			return active;
		}
		if (active == 0) {
			return 0;
		}
		k_msleep(TEST_PLAY_POLL_MS);
	}

	return -ETIMEDOUT;
}

static int drain_haptic_events(void) {
	struct device_driver_event_t event;
	int playback_error = 0;

	while (device_driver_event_wait(K_NO_WAIT, &event)) {
		if (event.device_id == DRV2605_DEVICE_DTS_ID) {
			printk("  event: %s (%u)\n",
				   drv2605_event_name((enum drv2605_event_type) event.event_id),
				   event.v_param);
			if ((event.event_id == drv2605_event_Error) && (playback_error == 0)) {
				playback_error = event.v_param == 0U ? -EIO : -(int) event.v_param;
			}
		}
	}

	return playback_error;
}

static int play_rtp(void) {
	const union drv2605_config_data cfg = {.rtp_data = &rtp_buzz};
	int ret;

	ret = drv2605_haptic_config(haptic_dev, DRV2605_HAPTICS_SOURCE_RTP, &cfg);
	if (ret < 0) {
		printk("  RTP config failed: %d\n", ret);
		return ret;
	}

	ret = haptics_start_output(haptic_dev);
	if (ret < 0) {
		printk("  RTP start failed: %d\n", ret);
		return ret;
	}

	ret = wait_until_idle();
	if (ret < 0) {
		printk("  RTP playback failed or timed out: %d\n", ret);
		(void) haptics_stop_output(haptic_dev);
		return ret;
	}

	ret = drain_haptic_events();
	if (ret < 0) {
		printk("  RTP worker failed: %d\n", ret);
		return ret;
	}
	printk("  RTP done\n");
	return 0;
}

static int play_rom(void) {
	const union drv2605_config_data cfg = {.rom_data = &rom_click};
	int ret;

	ret = drv2605_haptic_config(haptic_dev, DRV2605_HAPTICS_SOURCE_ROM, &cfg);
	if (ret < 0) {
		printk("  ROM config failed: %d\n", ret);
		return ret;
	}

	ret = haptics_start_output(haptic_dev);
	if (ret < 0) {
		printk("  ROM start failed: %d\n", ret);
		return ret;
	}

	ret = wait_until_idle();
	if (ret < 0) {
		printk("  ROM playback failed or timed out: %d\n", ret);
		(void) haptics_stop_output(haptic_dev);
		return ret;
	}

	/* Clear the already-low GO bit and publish the driver's Stopped event. */
	ret = haptics_stop_output(haptic_dev);
	if (ret < 0) {
		printk("  ROM stop failed: %d\n", ret);
		return ret;
	}

	ret = drain_haptic_events();
	if (ret < 0) {
		printk("  ROM playback reported an error: %d\n", ret);
		return ret;
	}
	printk("  ROM done\n");
	return 0;
}

int main(void) {
	printk("\n=== DRV2605 haptics test (senswear_haptic shield) ===\n");

	if (!device_is_ready(haptic_dev)) {
		printk("DRV2605 device not ready\n");
		return 0;
	}

	printk("DRV2605 ready; configuring and exercising RTP buzz and ROM click\n");

	while (1) {
		printk("RTP buzz (ramp 80 -> 160 -> 255 -> off)\n");
		if (play_rtp() != 0) {
			printk("DRV2605 test aborted after RTP failure\n");
			return 0;
		}
		k_sleep(TEST_CYCLE_PERIOD);

		printk("ROM click (LRA library, effect 1)\n");
		if (play_rom() != 0) {
			printk("DRV2605 test aborted after ROM failure\n");
			return 0;
		}
		k_sleep(TEST_CYCLE_PERIOD);
	}

	return 0;
}
