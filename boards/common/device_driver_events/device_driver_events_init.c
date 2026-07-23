/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_driver_events_init.c
 * @brief Automatic system initialization for the device driver event manager.
 *
 * This module provides the SYS_INIT hook that automatically initializes the
 * device driver event manager at system startup if
 * CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_AUTO_INIT is enabled.
 */

#include "device_driver_events.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_AUTO_INIT

LOG_MODULE_REGISTER(device_driver_events_init, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief System initialization hook for the device driver event manager.
 *
 * Called at POST_KERNEL initialization priority if auto-init is enabled.
 * Initializes the event manager with CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_MAX
 * queued events.
 *
 * @retval 0 Initialization succeeded.
 * @return Negative errno if initialization failed.
 */
static int device_driver_events_init_hook(void) {
	int ret = device_driver_event_init(CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_MAX);

	if (ret != 0) {
		LOG_ERR("Device driver event manager initialization failed: %d", ret);
		return ret;
	}

	LOG_INF("Device driver event manager initialized with %d events",
			CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_MAX);
	return 0;
}

SYS_INIT(device_driver_events_init_hook, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /* CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS_AUTO_INIT */
