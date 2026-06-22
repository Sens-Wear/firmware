/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BQ27427 fuel-gauge driver.
 *
 * Initialises the gauge, applies the default configuration with a known
 * battery capacity, then polls and prints the battery state every 2 seconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "bq27427.h"

#define TEST_GAUGE_DEVICE_ID  2u
#define TEST_BATTERY_CAPACITY 450 /* mAh */
#define TEST_POLL_INTERVAL    K_SECONDS(2)

int main(void)
{
	printk("\n=== BQ27427 fuel-gauge driver test ===\n");

	if (!bq27427_init(TEST_GAUGE_DEVICE_ID)) {
		printk("bq27427_init() failed\n");
		return 0;
	}

	if (!bq27427_is_ready()) {
		printk("bq27427_is_ready() == false\n");
		return 0;
	}

	struct bq27427_config_t config;

	bq27427_get_default_config(&config);
	config.battery_capacity = TEST_BATTERY_CAPACITY;

	if (!bq27427_config(&config)) {
		printk("bq27427_config() failed\n");
		return 0;
	}

	printk("configured; polling state every 2 s...\n");

	while (1) {
		struct bq27427_battery_state_t state;

		if (bq27427_update_state(&state)) {
			bq27427_print_state();
		} else {
			printk("bq27427_update_state() failed\n");
		}

		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
