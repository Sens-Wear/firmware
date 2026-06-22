/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BQ25180 charger driver.
 *
 * Initialises the charger, applies the default Li-Po / USB charger
 * configuration, then polls and prints the charger state every 2 seconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>

#include "bq25180.h"

/* Arbitrary device id for the device-event manager (no dispatch wired here). */
#define TEST_CHARGER_DEVICE_ID 1u
#define TEST_POLL_INTERVAL     K_SECONDS(2)

int main(void)
{
	printk("\n=== BQ25180 charger driver test ===\n");

	if (!bq25180_init(TEST_CHARGER_DEVICE_ID)) {
		printk("bq25180_init() failed\n");
		return 0;
	}

	if (!bq25180_is_ready()) {
		printk("bq25180_is_ready() == false\n");
		return 0;
	}

	struct bq25180_config_t config;

	bq25180_get_default_lipo_usb_charger_config(&config);
	config.battery_uvlo = bq25180_battery_UVLO_threshold_2V8;

	if (!bq25180_config(&config)) {
		printk("bq25180_config() failed\n");
		return 0;
	}

	printk("configured; polling state every 2 s...\n");

	while (1) {
		union bq25180_charger_state_t state;

		if (bq25180_update_state(&state)) {
			bq25180_print_state();
		} else {
			printk("bq25180_update_state() failed\n");
		}

		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
