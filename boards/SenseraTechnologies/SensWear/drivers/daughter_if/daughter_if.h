/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file daughter_if.h
 * @brief SenseWear daughter-board connector GPIO ownership arbiter.
 *
 * @defgroup sensewear_daughter_if SenseWear daughter-board interface
 * @ingroup io_interfaces
 * @{
 *
 * The daughter-board connector exposes four shared GPIO lines (SENSOR_GPIO_0..3)
 * that different daughter-board drivers (haptic, PPG, temperature, touch, ...)
 * may want to use for their own signals — interrupts, resets, mode pins, etc.
 * Because only one daughter board is fitted at a time and each board assigns the
 * lines differently, this module @b aggregates the connector lines in one place
 * and arbitrates which driver controls each line.
 *
 * @section sensewear_daughter_if_model Ownership model
 *
 * Each line has a single owner slot. A driver calls daughter_if_gpio_claim() for
 * the line it wants and receives the line's @ref gpio_dt_spec to configure and
 * drive — but only when the line is free. Possession of that returned non-NULL
 * pointer @e is the ownership: a second driver that claims the same line gets
 * NULL, so two drivers can never hold the same pin. daughter_if_gpio_release()
 * returns the line to the free pool.
 *
 * This is a @b runtime arbiter: ownership is resolved while the program runs, so
 * it naturally supports hot daughter-board changes and probe-order independence.
 * The check is serialized with a mutex and is safe to call from thread context.
 *
 * @section sensewear_daughter_if_usage Typical usage
 *
 * @code{.c}
 * const struct gpio_dt_spec *irq = daughter_if_gpio_claim(daughter_if_gpio_0);
 *
 * if (irq != NULL) {
 *     gpio_pin_configure_dt(irq, GPIO_INPUT);
 *     // ... use the line ...
 *     daughter_if_gpio_release(daughter_if_gpio_0);
 * }
 * @endcode
 */
#ifndef SENSWEAR_DRIVERS_DAUGHTER_IF_H_
#define SENSWEAR_DRIVERS_DAUGHTER_IF_H_

#include <zephyr/drivers/gpio.h>

/**
 * @brief Daughter-board connector GPIO line identifiers.
 * @details Indices match the connector's @c sensor-gpios / @c gpio-line-names
 *          order. ::daughter_if_gpio_count is the number of lines and is not a
 *          valid line.
 */
enum daughter_if_gpio {
	daughter_if_gpio_0 = 0, /**< SENSOR_GPIO_0 (P1.14). */
	daughter_if_gpio_1,		/**< SENSOR_GPIO_1 (P1.13). */
	daughter_if_gpio_2,		/**< SENSOR_GPIO_2 (P1.12). */
	daughter_if_gpio_3,		/**< SENSOR_GPIO_3 (P1.11). */
	daughter_if_gpio_count,
};

/**
 * @brief Claim exclusive control of a connector GPIO line.
 * @param line The connector line to claim.
 * @return The line's devicetree GPIO spec (non-NULL) when the claim is granted —
 *         the line was free and its controller is ready. The caller owns the
 *         line until it releases it. Returns NULL when the line is already owned
 *         by another driver, @p line is out of range, or the underlying GPIO
 *         controller is not ready.
 */
const struct gpio_dt_spec *daughter_if_gpio_claim(enum daughter_if_gpio line);

/**
 * @brief Release a connector GPIO line, returning it to the free pool.
 * @param line The connector line to release.
 * @retval 0 The line was released, or was already free.
 * @retval -EINVAL @p line is out of range.
 */
int daughter_if_gpio_release(enum daughter_if_gpio line);

/**
 * @brief Report whether a connector GPIO line is currently owned.
 * @param line The connector line to query.
 * @retval true The line is claimed by a driver.
 * @retval false The line is free, or @p line is invalid.
 */
bool daughter_if_gpio_is_claimed(enum daughter_if_gpio line);

/**
 * @brief Return the devicetree line name of a connector GPIO.
 * @param line The connector line to query.
 * @return The @c gpio-line-names entry, or NULL if @p line is invalid.
 */
const char *daughter_if_gpio_name(enum daughter_if_gpio line);

/** @} */

#endif /* SENSWEAR_DRIVERS_DAUGHTER_IF_H_ */
