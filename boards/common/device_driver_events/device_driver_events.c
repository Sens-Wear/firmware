/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_driver_events.c
 * @brief SensWear centralized device event manager implementation.
 *
 * The manager owns a single Zephyr message queue. Producers copy events into it
 * (non-blocking, ISR-safe); the lone consumer copies them out via
 * device_driver_event_wait(). The queue storage is allocated once from the system heap
 * at device_driver_event_init() time, sized for @c max_events events.
 */

#include "device_driver_events.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(device_driver_events, CONFIG_LOG_DEFAULT_LEVEL);

static struct device_manager_t {
	/** Backing queue for all device events. */
	struct k_msgq msgq;

	/** True once device_driver_event_init() has created the queue. */
	bool ready;
} device_driver_event_manager = {0}; /* zero-initialised by static storage duration */
/**
 * Common enqueue path shared by the thread and ISR producers. The caller chooses
 * the timeout: K_NO_WAIT for the ISR path, a (possibly blocking) timeout for the
 * thread path.
 */
static int device_driver_event_enqueue(uint32_t device_id,
									   uint32_t event_id,
									   uint32_t v_param,
									   uintptr_t p_param,
									   k_timeout_t timeout) {
	if (!device_driver_event_manager.ready) {
		return -ENODEV;
	}

	if (event_id == DEVICE_DRIVER_EVENT_ID_INVALID) {
		return -EINVAL;
	}

	struct device_driver_event_t event = {
		.device_id = device_id,
		.event_id = event_id,
		.v_param = v_param,
		.p_param = p_param,
	};

	return k_msgq_put(&device_driver_event_manager.msgq, &event, timeout);
}

int device_driver_event_init(int max_events) {
	if (max_events <= 0) {
		return -EINVAL;
	}

	if (device_driver_event_manager.ready) {
		return (device_driver_event_manager.msgq.max_msgs == (uint32_t) max_events) ? 0 : -EALREADY;
	}

	int ret = k_msgq_alloc_init(&device_driver_event_manager.msgq,
								sizeof(struct device_driver_event_t),
								(uint32_t) max_events);
	if (ret != 0) {
		LOG_ERR("event queue allocation failed (%d)", ret);
		return ret;
	}

	device_driver_event_manager.ready = true;
	return 0;
}

struct k_msgq* device_driver_event_get_queue(void) {
	return device_driver_event_manager.ready ? &device_driver_event_manager.msgq : NULL;
}

int device_driver_event_post(uint32_t device_id,
							 uint32_t event_id,
							 uint32_t v_param,
							 uintptr_t p_param,
							 k_timeout_t timeout) {
	return device_driver_event_enqueue(device_id, event_id, v_param, p_param, timeout);
}

int device_driver_event_post_isr(uint32_t device_id,
								 uint32_t event_id,
								 uint32_t v_param,
								 uintptr_t p_param) {
	return device_driver_event_enqueue(device_id, event_id, v_param, p_param, K_NO_WAIT);
}

bool device_driver_event_wait(k_timeout_t timeout_ms, struct device_driver_event_t* event) {
	if (!device_driver_event_manager.ready || event == NULL) {
		return false;
	}

	return k_msgq_get(&device_driver_event_manager.msgq, event, timeout_ms) == 0;
}

int device_driver_event_get_count(void) {
	if (!device_driver_event_manager.ready) {
		return 0;
	}

	return (int) k_msgq_num_used_get(&device_driver_event_manager.msgq);
}
