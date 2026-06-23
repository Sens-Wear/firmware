/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file lp5562.h
 * @brief SenseWear LP5562 four-channel LED-controller API.
 *
 * @defgroup sensewear_lp5562 SenseWear LP5562 LED controller
 * @ingroup io_interfaces
 * @{
 *
 * The LP5562 drives the board's RGBW indicator LED. Each channel can be driven
 * either directly by an I2C PWM/current register or by one of three programmable
 * execution engines that run small instruction sequences (ramp, wait, set-PWM,
 * branch, end, trigger) for autonomous animations.
 *
 * The driver talks to the device directly through Zephyr's I2C API using an
 * I2C_DT_SPEC_GET() specification, and toggles the controller's enable line
 * through a GPIO_DT_SPEC. The device is resolved through DT_NODELABEL(lp5562),
 * so the node must use that label.
 *
 * @section sensewear_lp5562_organization Source organization
 *
 * The driver is split into three headers, mirroring the bq25180 / bq27427
 * layout:
 * - @ref lp5562_registers.h — I2C register map, bit masks, and register unions.
 * - @ref lp5562_led_engines.h — LED/engine model and the program-engine
 *   instruction types and builders (the LED PWM programming model).
 * - @ref lp5562.h (this file) — the public API.
 *
 * @section sensewear_lp5562_model Singleton driver
 *
 * The board has exactly one LP5562, so the driver is a file-scope singleton
 * inside @c lp5562.c that owns the devicetree I2C/GPIO specs and the cached
 * register/LED/engine state. The public API therefore takes no driver handle:
 * call lp5562_initialize() once to seed the singleton, then drive the device
 * through the operations below. Each hardware operation returns @c true on a
 * successful I2C transfer.
 */
#ifndef _LP5562_H_
#define _LP5562_H_

#include <stdbool.h>
#include <stdint.h>

#include "lp5562_led_engines.h"
#include "lp5562_registers.h"

/** Shared-I2C / I2C transfer timeout in milliseconds. */
#ifndef LP5562_I2C_TIMEOUT
#define LP5562_I2C_TIMEOUT 100
#endif

/**
 * \brief Initializes the singleton LP5562 driver with its default state.
 * \details Binds the devicetree I2C/GPIO specs and resets the cached state.
 *          Safe to call more than once; subsequent calls are no-ops.
 */
void lp5562_initialize(void);
/**
 * \brief Enables LOGarithmic increase of PWMs
 * @param enable TRUE to enable, FALSE to disable the LOGarithmic PWM control
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_log_pwm_enable(bool enable);
/**
 * \brief Configures the clock settings of the controller.
 * @param clock The clock settings
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_configure_clock(enum lp5562_clock_type clock);
/**
 * \brief Enables or disables power save mode of LP5562.
 * @param enable TRUE to enable, FALSE to disable the power save mode.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_power_save(bool enable);
/**
 * \brief Enables or disables high frequency PWM mode of LP5562.
 * @param enable TRUE to enable, FALSE to disable the high frequency PWM mode.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_pwm_hf(bool enable);
/**
 * \brief Enables the controller by setting the Chip Enable bit.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_chip_enable(void);
/** \brief Disables the controller and drops the enable line. */
void lp5562_chip_disable(void);

/**
 * \brief Configures a LED of the LP5562.
 * @param led The LED to configure.
 * @param current Max current limit of the driver.
 * @param i2c_pwm I2C pwm value.
 * @param engine The current command execution engine of the LED.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_led_configure(enum lp5562_led_type led,
						  uint8_t current,
						  uint8_t i2c_pwm,
						  enum lp5562_engine_type engine);
/**
 * \brief Returns the cached max-current setting of a LED.
 * @param led The LED to query.
 * @return The cached current value in 0.1 mA steps.
 */
uint8_t lp5562_led_get_current(enum lp5562_led_type led);
/**
 * \brief Returns the start address of the local cache of the execution engine commands.
 * @param engine The engine.
 * @return NULL if the specified engine does not contain a command cache, otherwise the start
 * address.
 */
uint16_t* lp5562_engine_get_commands(enum lp5562_engine_type engine);
/**
 * \brief Configures the SRAM memory of an execution engine.
 * @param engine The engine
 * @param commandCount The number of commands in the local cache.
 * @param run TRUE to start execution of the engine.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_engine_configure_commands(enum lp5562_engine_type engine,
									  uint8_t commandCount,
									  bool run);
/**
 * \brief Changes the mode of the specified execution engine.
 * @param engine The engine
 * @param mode The new mode.
 */
bool lp5562_engine_set_mode(enum lp5562_engine_type engine, enum lp5562_engine_mode_type mode);
/**
 * \brief Sets the engine state of the specified execution engine.
 * @param engine The execution engine.
 * @param state The new state
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_engine_set_state(enum lp5562_engine_type engine, enum lp5562_engine_state_type state);
/** \brief Sets the same engine state on all three execution engines. */
bool lp5562_engine_all_set_state(enum lp5562_engine_state_type state);
/**
 * \brief Sets the MAX current value of a LED, keeping its PWM and engine.
 */
bool lp5562_led_set_current(enum lp5562_led_type led, uint8_t current);
/**
 * \brief Sets the I2C PWM value of a LED, keeping its current and engine.
 */
bool lp5562_led_set_i2c_dim(enum lp5562_led_type led, uint8_t dim);
/**
 * \brief Sets the execution engine of a LED, keeping its current and PWM.
 */
bool lp5562_led_set_engine(enum lp5562_led_type led, enum lp5562_engine_type engine);
/**
 * \brief Sends RESET command to reset the device registers to their default values.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_reset(void);

/** @} */

#endif // _LP5562_H_
