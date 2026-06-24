/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file daughter_if.c
 * @brief SenseWear daughter-board connector GPIO ownership arbiter.
 * @details Aggregates the connector's four GPIO lines and grants exclusive
 *          control to the first driver that claims each line. Ownership is
 *          tracked in a per-line table guarded by a mutex (runtime arbitration).
 */

#include "daughter_if.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(daughter_if, CONFIG_GPIO_LOG_LEVEL);

#define DAUGHTER_IF_NODE DT_NODELABEL(daughter_board_if)

BUILD_ASSERT(DT_PROP_LEN(DAUGHTER_IF_NODE, sensor_gpios) == daughter_if_gpio_count,
			 "daughter_if: sensor-gpios count must match daughter_if_gpio_count");

/** Devicetree GPIO spec for each connector line, in line order. */
static const struct gpio_dt_spec daughter_if_pins[daughter_if_gpio_count] = {
	GPIO_DT_SPEC_GET_BY_IDX(DAUGHTER_IF_NODE, sensor_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(DAUGHTER_IF_NODE, sensor_gpios, 1),
	GPIO_DT_SPEC_GET_BY_IDX(DAUGHTER_IF_NODE, sensor_gpios, 2),
	GPIO_DT_SPEC_GET_BY_IDX(DAUGHTER_IF_NODE, sensor_gpios, 3),
};

/** Devicetree name of each connector line, in line order. */
static const char *const daughter_if_names[daughter_if_gpio_count] = {
	DT_PROP_BY_IDX(DAUGHTER_IF_NODE, gpio_line_names, 0),
	DT_PROP_BY_IDX(DAUGHTER_IF_NODE, gpio_line_names, 1),
	DT_PROP_BY_IDX(DAUGHTER_IF_NODE, gpio_line_names, 2),
	DT_PROP_BY_IDX(DAUGHTER_IF_NODE, gpio_line_names, 3),
};

/** Per-line claimed flag; false means the line is free. Guarded by the mutex. */
static bool daughter_if_claimed[daughter_if_gpio_count];
static K_MUTEX_DEFINE(daughter_if_lock);

const struct gpio_dt_spec *daughter_if_gpio_claim(enum daughter_if_gpio line)
{
	if (line >= daughter_if_gpio_count) {
		return NULL;
	}
	if (!gpio_is_ready_dt(&daughter_if_pins[line])) {
		LOG_ERR("%s GPIO controller not ready", daughter_if_names[line]);
		return NULL;
	}

	bool granted;

	k_mutex_lock(&daughter_if_lock, K_FOREVER);
	granted = !daughter_if_claimed[line];
	if (granted) {
		daughter_if_claimed[line] = true;
	}
	k_mutex_unlock(&daughter_if_lock);

	if (!granted) {
		LOG_WRN("%s already owned; claim denied", daughter_if_names[line]);
		return NULL;
	}
	return &daughter_if_pins[line];
}

int daughter_if_gpio_release(enum daughter_if_gpio line)
{
	if (line >= daughter_if_gpio_count) {
		return -EINVAL;
	}

	k_mutex_lock(&daughter_if_lock, K_FOREVER);
	daughter_if_claimed[line] = false;
	k_mutex_unlock(&daughter_if_lock);

	return 0;
}

bool daughter_if_gpio_is_claimed(enum daughter_if_gpio line)
{
	if (line >= daughter_if_gpio_count) {
		return false;
	}

	bool claimed;

	k_mutex_lock(&daughter_if_lock, K_FOREVER);
	claimed = daughter_if_claimed[line];
	k_mutex_unlock(&daughter_if_lock);

	return claimed;
}

const char *daughter_if_gpio_name(enum daughter_if_gpio line)
{
	if (line >= daughter_if_gpio_count) {
		return NULL;
	}
	return daughter_if_names[line];
}
