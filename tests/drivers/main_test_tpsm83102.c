/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the TPSM83102 regulator driver.
 *
 * NOTE: tpsm83102.h currently exposes no public API (the regulator is managed
 * through devicetree / the Zephyr regulator framework and the standalone
 * driver has no init/config/update entry points yet). This test is a
 * placeholder that boots and idles so the per-driver test set stays complete;
 * fill in the body once the driver grows a public interface.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash.
 */

#include <zephyr/kernel.h>

#include "tpsm83102.h"

#define TEST_POLL_INTERVAL K_SECONDS(2)

int main(void)
{
	printk("\n=== TPSM83102 regulator driver test ===\n");
	printk("driver exposes no public API yet; nothing to exercise (TODO)\n");

	while (1) {
		printk("tpsm83102: idle\n");
		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
