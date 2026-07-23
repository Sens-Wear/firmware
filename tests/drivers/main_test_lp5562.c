/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the LP5562 LED controller driver.
 *
 * Drives the bare LP5562 driver directly (no led_controller / LED-manager
 * layer) and demonstrates all three ways a channel can be driven:
 *
 *   RED   - direct I2C PWM control, toggled on/off from this thread.
 *   BLUE  - autonomous "breathing" ramp running on program engine 1.
 *   GREEN - autonomous blink running on program engine 2.
 *
 * The blue/green waveforms are built from the engine instruction builders
 * (ramp / wait / set-PWM / branch) and loaded into the engines; they then run
 * on the device with no host involvement while the main loop only toggles red.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "lp5562.h"

/* Per-channel max current in 0.1 mA steps (~6.4 mA). */
#define TEST_LED_CURRENT   0x40u
#define TEST_TOGGLE_PERIOD K_SECONDS(2)

/* Convenience: build a command and store it byte-swapped into engine memory. */
#define STORE(buf, idx, cmd) ((buf)[(idx)++] = lp5562_command_reorder_bytes(cmd))

/*
 * Build a smooth breathing waveform (ramp up to full, ramp down to off, loop).
 * The 7-bit ramp step count maxes at 128, so each direction uses two ramps.
 */
static uint8_t build_breathe_program(uint16_t *cmd)
{
	uint8_t n = 0;

	STORE(cmd, n, lp5562_set_pwm_command(0));
	STORE(cmd, n, lp5562_ramp_command(127, true, 1, true));  /* ramp up */
	STORE(cmd, n, lp5562_ramp_command(127, true, 1, true));
	STORE(cmd, n, lp5562_ramp_command(127, false, 1, true)); /* ramp down */
	STORE(cmd, n, lp5562_ramp_command(127, false, 1, true));
	STORE(cmd, n, lp5562_branch_command(0, 0));              /* loop forever */
	return n;
}

/*
 * Build a hard on/off blink waveform (full on, wait, off, wait, loop).
 * wait(63, prescale) holds for 63 * 15.625 ms ~= 1 s per command.
 */
static uint8_t build_blink_program(uint16_t *cmd)
{
	uint8_t n = 0;

	STORE(cmd, n, lp5562_set_pwm_command(255));
	STORE(cmd, n, lp5562_wait_command(63, true));
	STORE(cmd, n, lp5562_wait_command(63, true));
	STORE(cmd, n, lp5562_set_pwm_command(0));
	STORE(cmd, n, lp5562_wait_command(63, true));
	STORE(cmd, n, lp5562_wait_command(63, true));
	STORE(cmd, n, lp5562_branch_command(0, 0));              /* loop forever */
	return n;
}

/* Route a LED to an engine, load that engine's program, and start it. */
static bool start_engine_led(enum lp5562_led_type led,
			     enum lp5562_engine_type engine,
			     uint8_t (*build)(uint16_t *cmd))
{
	if (!lp5562_led_configure(led, TEST_LED_CURRENT, 0, engine)) {
		return false;
	}

	uint16_t *cmd = lp5562_engine_get_commands(engine);

	if (cmd == NULL) {
		return false;
	}

	uint8_t count = build(cmd);

	return lp5562_engine_configure_commands(engine, count, true);
}

int main(void)
{
	printk("\n=== LP5562 LED controller test (bare driver) ===\n");

	lp5562_initialize();

	if (!lp5562_reset()) {
		printk("lp5562_reset() failed\n");
		return 0;
	}

	if (!lp5562_configure_clock(lp5562_clk_Automatic)) {
		printk("lp5562_configure_clock() failed\n");
		return 0;
	}

	(void)lp5562_power_save(true);
	(void)lp5562_pwm_hf(true);

	/* The chip must be enabled before the engines will run. */
	if (!lp5562_chip_enable()) {
		printk("lp5562_chip_enable() failed\n");
		return 0;
	}

	/* RED: direct I2C PWM control, starting fully off. */
	if (!lp5562_led_configure(lp5562_led_Red, TEST_LED_CURRENT, 0,
				  lp5562_engine_i2c_pwm)) {
		printk("lp5562_led_configure(Red) failed\n");
		return 0;
	}

	/* BLUE: breathing waveform on engine 1. */
	if (!start_engine_led(lp5562_led_Blue, lp5562_engine_1,
			      build_breathe_program)) {
		printk("failed to start BLUE breathing engine\n");
	}

	/* GREEN: blink waveform on engine 2. */
	if (!start_engine_led(lp5562_led_Green, lp5562_engine_2,
			      build_blink_program)) {
		printk("failed to start GREEN blink engine\n");
	}

	bool on = false;

	printk("RED toggles every 2 s; BLUE breathes (eng1); GREEN blinks (eng2)\n");

	while (1) {
		on = !on;

		if (!lp5562_led_set_i2c_dim(lp5562_led_Red, on ? 255u : 0u)) {
			printk("lp5562_led_set_i2c_dim(Red) failed\n");
		}

		printk("red LED %s\n", on ? "on" : "off");

		k_sleep(TEST_TOGGLE_PERIOD);
	}

	return 0;
}
