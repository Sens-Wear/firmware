/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_driver_events.h
 * @brief SenseWear centralized device event manager.
 *
 * @defgroup sensewear_device_driver_events SenseWear device driver event manager
 * @ingroup io_interfaces
 * @{
 *
 * Many SenseWear devices (charger, fuel gauge, IMU, daughter board, ...) signal
 * asynchronous activity from interrupt handlers. Rather than giving each device
 * its own thread, they all publish into one shared event queue that a single
 * consumer thread drains and dispatches. This keeps device-event handling in one
 * place, in thread context, and on one stack.
 *
 * @code{.text}
 * device A IRQ ─┐
 * device B IRQ ─┤ device_driver_event_post[_isr]()   (producers)
 * device C IRQ ─┘            │
 *                            v
 *                   [ shared k_msgq queue ]
 *                            │
 *                   device_driver_event_wait()        (single consumer)
 *                            │
 *                   dispatch by device_id / event_id
 * @endcode
 *
 * @section sensewear_device_driver_events_model Ownership model
 *
 * Events are delivered @b by @b value: device_driver_event_wait() copies the next event
 * into a caller-provided ::device_driver_event_t. There is no allocation to release and
 * no internal pointer is exposed. The copy is valid for as long as the caller's
 * buffer lives, so a synchronous consumer can simply keep it on its stack.
 *
 * The @ref device_driver_event_t.p_param pointer is copied verbatim; the memory it
 * points at is @b not copied. A producer must therefore point @ref
 * device_driver_event_t.p_param at storage that remains valid until the consumer has
 * processed the event (for example a device's static context). Transient stack
 * buffers must not be passed through @ref device_driver_event_t.p_param. Small payloads
 * should be carried inline in @ref device_driver_event_t.v_param instead.
 *
 * @section sensewear_device_driver_events_context Execution context
 *
 * Producers may run in either thread or interrupt context:
 * - device_driver_event_post() is for thread context.
 * - device_driver_event_post_isr() is for interrupt context and never blocks.
 *
 * device_driver_event_wait() blocks and must only be called from a thread. The design
 * assumes exactly one consumer thread; the queue is otherwise multi-producer and
 * thread-/ISR-safe.
 *
 * @section sensewear_device_driver_events_lifecycle Lifecycle
 *
 * device_driver_event_init() must be called once during start-up to create the backing
 * queue before any event is posted or awaited. device_driver_event_get_queue() exposes
 * the underlying Zephyr message queue for advanced integration (for example
 * adding it to a @c k_poll set).
 *
 * @section sensewear_device_driver_events_example Typical usage
 *
 * Initialization, once at start-up (automatic if CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS_AUTO_INIT is
 * enabled):
 *
 * @code{.c}
 * device_driver_event_init(CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS_MAX);
 * @endcode
 *
 * Producer side, from a device's deferred interrupt handler:
 *
 * @code{.c}
 * #define DEV_CHARGER 1u
 * #define CHG_EVT_IRQ 1u
 *
 * // From a work handler (thread context):
 * device_driver_event_post(DEV_CHARGER, CHG_EVT_IRQ, raw_status, NULL);
 *
 * // Or directly from an ISR:
 * device_driver_event_post_isr(DEV_CHARGER, CHG_EVT_IRQ, 0u, NULL);
 * @endcode
 *
 * Consumer side, the single device-manager thread:
 *
 * @code{.c}
 * static void device_manager_thread(void *a, void *b, void *c)
 * {
 *     struct device_driver_event_t ev;          // lives on this thread's stack
 *
 *     while (device_driver_event_wait(K_FOREVER, &ev)) {
 *         switch (ev.device_id) {
 *         case DEV_CHARGER:
 *             charger_handle_event(&ev);
 *             break;
 *         default:
 *             break;
 *         }
 *     }
 * }
 * @endcode
 */

#ifndef SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_EVENTS_H_
#define SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_EVENTS_H_

#include "zephyr/kernel.h"
#include "zephyr/sys/clock.h"
#include <stdint.h>

/**
 * @brief Reserved sentinel for an unused or invalid event identifier.
 * @details Producers must not post this value as @ref device_driver_event_t.event_id.
 *          It is available to callers as a "no event" marker.
 */
#define DEVICE_DRIVER_EVENT_ID_INVALID (UINT32_MAX)

/**
 * @brief A single device event passed from a producer to the consumer.
 * @details The structure is copied by value through the queue. Scalar payloads
 *          should travel in @ref v_param; @ref p_param is an optional pointer
 *          whose pointee lifetime is the producer's responsibility.
 */
struct device_driver_event_t {
	uint32_t device_id; /**< Identifier of the device that produced the event. */
	uint32_t event_id;	/**< Device-specific event code (not ::DEVICE_DRIVER_EVENT_ID_INVALID). */
	uint32_t v_param;	/**< Inline scalar payload (event code, value, flags, ...). */
	void* p_param;		/**< Optional pointer payload; pointee must outlive delivery. */
};

/**
 * @brief Initialize the event manager and its backing queue.
 *
 * Must be called once, before any other operation, to create the shared queue.
 * This is typically called automatically at system startup if
 * CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS_AUTO_INIT is enabled.
 *
 * @param max_events Maximum number of events the queue can hold at once.
 * @retval 0 The manager was initialized.
 * @retval -EINVAL @p max_events is not positive.
 * @retval -EALREADY The manager was already initialized.
 * @return A negative errno if the backing queue could not be allocated.
 */
int device_driver_event_init(int max_events);

/**
 * @brief Access the underlying Zephyr message queue.
 *
 * Intended for advanced integration, such as adding the queue to a @c k_poll
 * set. Most callers should use the post/wait helpers instead.
 *
 * @return Pointer to the manager's backing message queue.
 */
struct k_msgq* device_driver_event_get_queue(void);

/**
 * @brief Post an event from thread context, blocking for queue space if needed.
 *
 * @param device_id Identifier of the producing device.
 * @param event_id Device-specific event code.
 * @param v_param Inline scalar payload.
 * @param p_param Optional pointer payload, or NULL. The pointee must remain valid
 *        until the consumer has processed the event.
 * @param timeout Maximum time to wait for room when the queue is full, as a
 *        Zephyr timeout (K_NO_WAIT, K_MSEC(...), or K_FOREVER). Must not be
 *        called from an ISR with a blocking timeout.
 * @retval 0 The event was queued.
 * @retval -EAGAIN The queue stayed full until @p timeout elapsed.
 * @return A negative errno if the event could not be queued.
 */
int device_driver_event_post(uint32_t device_id,
							 uint32_t event_id,
							 uint32_t v_param,
							 void* p_param,
							 k_timeout_t timeout);

/**
 * @brief Post an event from interrupt context.
 *
 * Like device_driver_event_post() but guaranteed never to block (K_NO_WAIT); intended
 * for use directly inside an ISR.
 *
 * @param device_id Identifier of the producing device.
 * @param event_id Device-specific event code.
 * @param v_param Inline scalar payload.
 * @param p_param Optional pointer payload, or NULL. The pointee must remain valid
 *        until the consumer has processed the event.
 * @retval 0 The event was queued.
 * @return A negative errno if the event could not be queued (for example the
 *         queue is full).
 */
int device_driver_event_post_isr(uint32_t device_id,
								 uint32_t event_id,
								 uint32_t v_param,
								 void* p_param);

/**
 * @brief Wait for and copy the next queued event (single consumer).
 *
 * Blocks until an event is available or @p timeout_ms elapses, then copies it
 * into @p event. Intended to be called from one consumer thread only.
 *
 * @param timeout_ms Maximum time to wait, as a Zephyr timeout (for example
 *        K_MSEC(100), K_NO_WAIT, or K_FOREVER).
 * @param event Destination buffer that receives the next event. Must not be NULL.
 * @retval true An event was dequeued and copied into @p event.
 * @retval false The timeout elapsed with no event available; @p event is
 *         unchanged.
 */
bool device_driver_event_wait(k_timeout_t timeout_ms, struct device_driver_event_t* event);

/**
 * @brief Return the number of events currently queued.
 *
 * @return Count of pending events not yet consumed. The value is advisory and
 *         may change immediately in the presence of concurrent producers.
 */
int device_driver_event_get_count(void);

/** @} */

#endif // SENSWEAR_DRIVERS_COMMON_DEVICE_DRIVER_EVENTS_H_
