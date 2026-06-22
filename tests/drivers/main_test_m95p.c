/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the M95P EEPROM driver.
 *
 * Initialises the device, prints the JEDEC id and geometry, performs a
 * single page write/read-back round-trip, then prints the busy/ready status
 * every 2 seconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <string.h>

#include <zephyr/kernel.h>

#include "m95p.h"

#define TEST_PAGE          0u
#define TEST_POLL_INTERVAL K_SECONDS(2)

int main(void)
{
	printk("\n=== M95P EEPROM driver test ===\n");

	if (!m95p_init(NULL)) {
		printk("m95p_init() failed\n");
		return 0;
	}

	if (!m95p_is_ready()) {
		printk("m95p_is_ready() == false\n");
		return 0;
	}

	const union m95p_jedec_id_t id = m95p_get_jedec_id();

	printk("JEDEC id: manufacturer=0x%02x type=0x%02x capacity=0x%02x\n",
	       id.fields.manufacturer_id, id.fields.memory_type, id.fields.capacity);
	printk("geometry: size=%zu page=%zu sector=%zu pages=%zu sectors=%zu\n",
	       m95p_get_size(), m95p_get_page_size(), m95p_get_sector_size(),
	       m95p_get_page_count(), m95p_get_sector_count());

	/* Write/read-back round-trip on a single page. */
	const size_t page_size = m95p_get_page_size();
	uint8_t wbuf[256];
	uint8_t rbuf[256];

	if (page_size > sizeof(wbuf)) {
		printk("page (%zu) larger than test buffer; skipping round-trip\n", page_size);
	} else {
		for (size_t i = 0; i < page_size; i++) {
			wbuf[i] = (uint8_t)(i ^ 0xA5);
		}

		if (!m95p_program_page(TEST_PAGE, wbuf, page_size)) {
			printk("m95p_program_page() failed\n");
		} else if (!m95p_read((uint32_t)TEST_PAGE * page_size, rbuf, page_size)) {
			printk("m95p_read() failed\n");
		} else {
			printk("round-trip: %s\n",
			       (memcmp(wbuf, rbuf, page_size) == 0) ? "OK" : "MISMATCH");
		}
	}

	printk("polling status every 2 s...\n");

	while (1) {
		printk("ready=%d busy=%d\n", m95p_is_ready(), m95p_is_busy());
		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
