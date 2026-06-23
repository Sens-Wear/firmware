/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file lp5562_registers.h
 * @brief Register map and bit definitions for the TI LP5562 LED driver.
 *
 * @defgroup sensewear_lp5562 SenseWear LP5562 LED controller
 * @ingroup io_interfaces
 * @{
 *
 * This header is the hardware-facing layer of the LP5562 driver: it declares
 * the I2C register addresses (::lp5562_register_type), byte-level @c union views
 * of the bit-mapped control registers, and the power-on default values. It
 * contains no driver state or behavior; the LED-programming data model lives in
 * @ref lp5562_led_engines.h and the public driver API in @ref lp5562.h.
 */
#ifndef LP5562_REGISTERS_H_
#define LP5562_REGISTERS_H_

#include <stdint.h>

/**
 * @brief LP5562 I2C register addresses.
 * @details Each enumerator is the device register sub-address used in an I2C
 *          transfer. The PWM/CURRENT registers hold a plain 8-bit value; the
 *          bit-mapped control registers (ENABLE, OP_MODE, CONFIG, LED_MAP) have
 *          a matching @c union view below.
 */
enum lp5562_register_type {
	lp5562_register_ENABLE = 0x00,        /**< Master enable, per-engine EXEC, PWM law. */
	lp5562_register_OP_MODE = 0x01,       /**< Per-engine operating mode. */
	lp5562_register_B_PWM = 0x02,         /**< Blue channel PWM duty cycle. */
	lp5562_register_G_PWM = 0x03,         /**< Green channel PWM duty cycle. */
	lp5562_register_R_PWM = 0x04,         /**< Red channel PWM duty cycle. */
	lp5562_register_B_CURRENT = 0x05,     /**< Blue channel current (0.1 mA steps). */
	lp5562_register_G_CURRENT = 0x06,     /**< Green channel current (0.1 mA steps). */
	lp5562_register_R_CURRENT = 0x07,     /**< Red channel current (0.1 mA steps). */
	lp5562_register_CONFIG = 0x08,        /**< Clock source, power-save, PWM frequency. */
	lp5562_register_RESET = 0x0D,         /**< Write ::LP5562_RESET to reset all registers. */
	lp5562_register_W_PWM = 0x0E,         /**< White channel PWM duty cycle. */
	lp5562_register_W_CURRENT = 0x0F,     /**< White channel current (0.1 mA steps). */
	lp5562_register_PROG_MEM_ENG1 = 0x10, /**< Engine 1 program-memory base. */
	lp5562_register_PROG_MEM_ENG2 = 0x30, /**< Engine 2 program-memory base. */
	lp5562_register_PROG_MEM_ENG3 = 0x50, /**< Engine 3 program-memory base. */
	lp5562_register_ENG_SEL = 0x70,       /**< LED_MAP: per-channel source select. */
};

/**
 * @brief ENABLE register (0x00) byte view.
 * @details Maps the master-enable, log-PWM, and per-engine EXEC fields. Each
 *          2-bit EXEC field takes an @ref lp5562_engine_state_type code.
 */
union lp5562_enable_register_t {
	uint8_t value;
	struct lp5562_enable_register_fields {
		uint8_t eng3_exec : 2;
		uint8_t eng2_exec : 2;
		uint8_t eng1_exec : 2;
		uint8_t chip_en : 1;
		uint8_t log_en : 1;
	} bits;
};

/**
 * @brief OP_MODE register (0x01) byte view.
 * @details Maps the per-engine operating-mode fields. Each 2-bit mode field
 *          takes an @ref lp5562_engine_mode_type code.
 */
union lp5562_op_mode_register_t {
	uint8_t value;
	struct lp5562_op_mode_register_bits {
		uint8_t eng3_mode : 2;
		uint8_t eng2_mode : 2;
		uint8_t eng1_mode : 2;
		uint8_t reserved : 2;
	} bits;
};

/**
 * @brief CONFIG register (0x08) byte view.
 * @details Maps clock-source, power-save, and PWM-frequency fields. The 2-bit
 *          clock field takes an @ref lp5562_clock_type code.
 */
union lp5562_config_register_t {
	uint8_t value;
	struct lp5562_config_register_bits {
		uint8_t clk_config : 2;
		uint8_t reserved_1 : 3;
		uint8_t powersave_en : 1;
		uint8_t pwm_hf : 1;
		uint8_t reserved_2 : 1;
	} bits;
};

/**
 * @brief LED_MAP register (0x70) byte view.
 * @details Maps the two-bit engine-select field for each LED channel. A field
 *          value of 0 selects direct I2C PWM; 1/2/3 select engine 1/2/3.
 */
union lp5562_led_map_register_t {
	uint8_t value;
	struct lp5562_led_map_register_bits {
		uint8_t b_eng_sel : 2;
		uint8_t g_eng_sel : 2;
		uint8_t r_eng_sel : 2;
		uint8_t w_eng_sel : 2;
	} bits;
};

/**
 * @name Reset and power-on default register values
 * @{
 */
#define LP5562_RESET 0xFF           /**< RESET register payload: reset all registers. */
#define LP5562_CONFIG_DEFAULT 0x00  /**< Default CONFIG register value. */
#define LP5562_CURRENT_DEFAULT 0xA7 /**< Default per-channel current register value. */
#define LP5562_PWM_DEFAULT 0x00     /**< Default per-channel PWM register value. */
#define LP5562_LED_MAP_DEFAULT 0x39 /**< Default LED_MAP register value. */
/** @} */

/** @brief LP5562 clock configuration options. */
enum lp5562_clock_type {
	lp5562_clk_External = 0,   //!< lp5562_clk_External
	lp5562_clk_Internal = 1,   //!< lp5562_clk_Internal
	lp5562_clk_Automatic = 2,  //!< lp5562_clk_Automatic
	lp5562_clk_Internal_2 = 3, //!< lp5562_clk_Internal_2
};

/** @} */

#endif /* LP5562_REGISTERS_H_ */
