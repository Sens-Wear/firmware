/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SenseWear-patched DRV2605 haptic driver on the
 * sensewear_haptic shield.
 *
 * The driver auto-initialises at POST_KERNEL (claims the daughter_if GPIO0
 * enable line, resets and configures the device), so this test only checks that
 * the device is ready and then exercises the two playback paths through the
 * Zephyr haptics API:
 *
 *   RTP - a real-time amplitude ramp streamed frame-by-frame from host buffers
 *         (asynchronous; the test waits for completion via
 *         drv2605_rtp_is_active()).
 *   ROM - a single waveform from the on-chip LRA library, triggered with the
 *         internal GO bit and played autonomously by the device.
 *
 * The two patterns alternate every 2 s. Build with the `test_drv2605` preset
 * (which also enables the shield); see tests/shields/README.md.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "drv2605.h"

#define TEST_CYCLE_PERIOD        K_SECONDS(2)
#define TEST_RTP_WAIT_TIMEOUT_MS 5000
#define TEST_RTP_POLL_MS         20
#define TEST_ROM_PLAY_MS         500

static const struct device *const haptic_dev = DEVICE_DT_GET(DT_ALIAS(sensewear_haptic));

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

static int play_rtp(void)
{
	const union drv2605_config_data cfg = {.rtp_data = &rtp_buzz};
	int waited = 0;
	int ret;

	if (drv2605_rtp_is_active(haptic_dev)) {
		printk("  RTP still active, skipping\n");
		return -EBUSY;
	}

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

	while (drv2605_rtp_is_active(haptic_dev) && waited < TEST_RTP_WAIT_TIMEOUT_MS) {
		k_msleep(TEST_RTP_POLL_MS);
		waited += TEST_RTP_POLL_MS;
	}

	printk("  RTP done (%d ms)\n", waited);
	return 0;
}

static int play_rom(void)
{
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

	/* ROM effects play autonomously on the device; allow time to finish. */
	k_msleep(TEST_ROM_PLAY_MS);
	printk("  ROM done\n");
	return 0;
}

int main(void)
{
	bool rtp_turn = true;

	printk("\n=== DRV2605 haptics test (sensewear_haptic shield) ===\n");

	if (!device_is_ready(haptic_dev)) {
		printk("DRV2605 device not ready\n");
		return 0;
	}

	printk("DRV2605 ready; alternating RTP buzz and ROM click every 2 s\n");

	while (1) {
		if (rtp_turn) {
			printk("RTP buzz (ramp 80 -> 160 -> 255 -> off)\n");
			(void)play_rtp();
		} else {
			printk("ROM click (LRA library, effect 1)\n");
			(void)play_rom();
		}

		rtp_turn = !rtp_turn;
		k_sleep(TEST_CYCLE_PERIOD);
	}

	return 0;
}
