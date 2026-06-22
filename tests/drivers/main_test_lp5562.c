/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the LP5562 LED controller driver.
 *
 * Drives the chip through the higher-level led_controller façade (the same
 * path the application uses): initialise, configure, then toggle the first LED
 * red on/off every 2 seconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "led_controller.h"

#define TEST_LED_ID        0u
#define TEST_TOGGLE_PERIOD K_SECONDS(2)

int main(void)
{
	printk("\n=== LP5562 LED controller test ===\n");

	if (!led_controller_init()) {
		printk("led_controller_init() failed\n");
		return 0;
	}

	if (!led_controller_configure()) {
		printk("led_controller_configure() failed\n");
		return 0;
	}

	const union led_color_t red = {
		.leds = {.red = 255, .green = 0, .blue = 0, .white = 0},
	};

	bool on = false;

	printk("toggling LED %u every 2 s...\n", TEST_LED_ID);

	while (1) {
		on = !on;

		if (on) {
			led_controller_turn_on_leds(TEST_LED_ID, red);
		} else {
			led_controller_turn_off_leds(TEST_LED_ID);
		}

		printk("LED %u %s\n", TEST_LED_ID, on ? "on (red)" : "off");

		k_sleep(TEST_TOGGLE_PERIOD);
	}

	return 0;
}
