/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file lp5562_led_engines.h
 * @brief LP5562 LED engine model and program-instruction builders.
 *
 * @defgroup sensewear_lp5562 SenseWear LP5562 LED controller
 * @ingroup io_interfaces
 * @{
 *
 * This header holds the functional model of the LP5562 program engines: the LED
 * and engine identifiers, the per-engine state/mode model, and the bit-accurate
 * representation of the LP5562 program-engine instruction set (ramp, wait,
 * set-PWM, branch, end, trigger).
 *
 * The instruction builders are pure: they perform @b no I2C and instead return
 * the corresponding register object (the command @c union) by value. Callers
 * hand those objects to the @ref lp5562.h driver functions, which serialize them
 * into engine program memory over I2C. This separates @e what to run (these
 * builders) from @e how it reaches the device (the driver).
 *
 * The implementations live in @c lp5562_led_engine.c; the raw register map is in
 * @ref lp5562_registers.h.
 */
#ifndef LP5562_LED_ENGINES_H_
#define LP5562_LED_ENGINES_H_

#include <stdbool.h>
#include <stdint.h>

/** Number of LED output channels (R, G, B, W). */
#define LP5562_LED_COUNT 4
/** Number of independent program-execution engines. */
#define LP5562_ENGINE_COUNT 3
/** Maximum number of instruction words held per engine program. */
#define LP5562_ENGINE_COMMAND_COUNT 16

/** @brief LED output-channel identifiers. */
enum lp5562_led_type {
	lp5562_led_Red = 0,
	lp5562_led_Green = 1,
	lp5562_led_Blue = 2,
	lp5562_led_White = 3,
};

/**
 * @brief Execution-engine identifiers.
 * @details Encoded as a bit per engine so callers can OR them together (for
 *          example in trigger commands); @c lp5562_engine_i2c_pwm denotes direct
 *          I2C PWM control rather than a program engine.
 */
enum lp5562_engine_type {
	lp5562_engine_1 = 1,
	lp5562_engine_2 = 2,
	lp5562_engine_3 = 4,
	lp5562_engine_i2c_pwm = 8,
};

/** @brief Per-LED configuration: channel, driving engine, current, and dim. */
struct lp5562_led_t {
	enum lp5562_led_type type;		//!< The LED identifier.
	enum lp5562_engine_type engine; //!< the execution engine of the LED.
	uint8_t current;				//!< The current flow through the LED in 0.1mA per step.
	uint8_t
		i2c_dim; //!< The pwm duty cycle control from i2c registers in %. 0 is 0% and 255 is 100%.
};

/** @brief Engine execution state requested through the ENABLE register. */
enum lp5562_engine_state_type {
	lp5562_engine_state_Hold = 0,
	lp5562_engine_state_Step = 1,
	lp5562_engine_state_Run = 2,
	lp5562_engine_state_Execute = 3,
};

/** @brief Engine operating mode requested through the OP_MODE register. */
enum lp5562_engine_mode_type {
	lp5562_engine_mode_Disabled = 0,
	lp5562_engine_mode_Load = 1,
	lp5562_engine_mode_Run = 2,
	lp5562_engine_mode_I2CControl = 3,
};

/** @brief One engine and its locally cached instruction program. */
struct lp5562_engine_t {
	enum lp5562_engine_type id;
	enum lp5562_engine_state_type state;
	enum lp5562_engine_mode_type mode;
	uint16_t commands[LP5562_ENGINE_COMMAND_COUNT];
	uint8_t command_count;
};

//-----------------------------------------------------------------
// The command are big endian
/** @brief RAMP instruction: step the PWM up or down over time. */
union lp5562_ramp_command_t {
	uint16_t command;
	struct lp5562_ramp_command_fields {
		uint16_t increment : 7;
		uint16_t sign : 1;
		uint16_t step_time : 6;
		uint16_t prescale : 1;
		uint16_t const_zero : 1; //!< write 0
	} fields;
};

/** @brief WAIT instruction: hold the current PWM for a time interval. */
union lp5562_wait_command_t {
	uint16_t command;
	struct lp5562_wait_command_fields {
		uint16_t const_zero1 : 7; //!< write 0
		uint16_t sign : 1;
		uint16_t step_time : 6;
		uint16_t prescale : 1;
		uint16_t const_zero2 : 1; //!< write 0
	} fields;
};

/** @brief SET PWM instruction: set the PWM output to an absolute value. */
union lp5562_set_pwm_command_t {
	uint16_t command;
	struct lp5562_set_pwm_command_fields {
		uint16_t pwm : 8;
		uint16_t const_pwm_set : 8; //!< write 0x40
	} fields;
};

/** @brief BRANCH instruction: jump to a step, optionally looping. */
union lp5562_branch_command_t {
	uint16_t command;
	struct lp5562_branch_command_fields {
		uint16_t step_number : 4;  //!< The step number in the command execution command
		uint16_t not_care : 3;	   //!< DoNot care bits
		uint16_t loop_count : 6;   //!< Number of loops to be done.
		uint16_t const_branch : 3; //!< The constant bits of the command: Write 5
	} fields;
};

/** @brief END instruction: stop the program, optionally resetting PWM/IRQ. */
union lp5562_end_command_t {
	uint16_t command;
	struct lp5562_end_command_fields {
		uint16_t not_care : 11;
		uint16_t reset : 1;		//!< Write 1 to reset PWM to 0 value after executing the command.
		uint16_t irq : 1;		//!< INT bit.
		uint16_t const_end : 3; //!< The constant bits of the command: Write 6
	} fields;
};

/** @brief TRIGGER instruction: send and/or wait for inter-engine triggers. */
union lp5562_trigger_command_t {
	uint16_t command;
	struct lp5562_trigger_command_fields {
		uint16_t not_care_1 : 1;
		uint16_t send_trigger_for_eng_1 : 1;
		uint16_t send_trigger_for_eng_2 : 1;
		uint16_t send_trigger_for_eng_3 : 1;
		uint16_t not_care_2 : 3;
		uint16_t wait_trigger_for_eng_1 : 1;
		uint16_t wait_trigger_for_eng_2 : 1;
		uint16_t wait_trigger_for_eng_3 : 1;
		uint16_t not_care_3 : 3;
		uint16_t const_trigger : 3; //!< The constant bits of the command: Write 7
	} fields;
};

/**
 * \brief Creates a RAMP command register object.
 * @param numberOfSteps The number of steps in the ramp/
 * @param bIncrement TRUE if incrementing, FALSE if decrementing.
 * @param stepTime The time of a step. It depends on the prescale value.
 * @param bPrescale TRUE if the clock is prescaled, FALSE if not.
 * @return The created command register object.
 */
union lp5562_ramp_command_t lp5562_ramp_command(uint8_t numberOfSteps,
												bool bIncrement,
												uint8_t stepTime,
												bool bPrescale);
/**
 * \brief Create a WAIT command register object.
 * @param stepTime The time of a step. It depends on the prescale value.
 * @param bPrescale TRUE if the clock is prescaled, FALSE if not.
 * @return The created command register object (a zero-increment ramp).
 */
#define lp5562_wait_command(stepTime, bPrescale) lp5562_ramp_command(0, false, stepTime, bPrescale)
/**
 * \brief Creates a SET PWM command register object.
 * @param pwm The PWM duty cycle value in %. 0 corresponds to 0% and 255 corresponds to 100%.
 * @return The created command register object.
 */
union lp5562_set_pwm_command_t lp5562_set_pwm_command(uint8_t pwm);
/**
 * \brief Go to start command
 */
#define lp5562_goto_start_command() 0x0000
/**
 * \brief Create a BRANCH command register object.
 * @param stepNumber The step number to be loaded to program counter
 * @param loopCount The number loops to be done. 0 means infinite loop.
 * @return The created command register object.
 */
union lp5562_branch_command_t lp5562_branch_command(uint8_t stepNumber, uint8_t loopCount);
/**
 * \brief Creates an END command register object.
 * @param bInterrupt TRUE if command creates and interrupt, FALSE if not.
 * @param bResetPWM TRUE to reset the PWM value to 0, FALSE to keep the current value/
 * @return The created command register object.
 */
union lp5562_end_command_t lp5562_end_command(bool bInterrupt, bool bResetPWM);
/**
 * \brief Creates a TRIGGER command register object. OR more than one engine identifiers.
 * @param sendTriggers a logical of engine identifiers for sending triggers.
 * @param waitTriggers a logical of engine identifiers for waiting triggers.
 * @return The created command register object.
 */
union lp5562_trigger_command_t lp5562_trigger_command(
	uint8_t sendTriggers,
	uint8_t waitTriggers); // OR more than one engine identifiers.

/**
 * \brief Converts a command word from LS-byte-first to MS-byte-first ordering.
 * @param command The command word.
 * @return The byte-swapped command word.
 */
static inline uint16_t lp5562_reorder_command_word(uint16_t command) {
	uint16_t retVal = (command >> 8) & 0x00FF;
	retVal += (command << 8) & 0xFF00;
	return retVal;
}

/**
 * \brief Byte-swaps a command register object into engine-memory order.
 * @details Accepts any command register object (every command @c union exposes a
 *          @c command word) and returns the byte-swapped word ready to load into
 *          an engine's program memory.
 * @param cmd A command register object returned by one of the builders.
 * @return The byte-swapped command word.
 */
#define lp5562_command_reorder_bytes(cmd) lp5562_reorder_command_word((cmd).command)

/** @} */

#endif /* LP5562_LED_ENGINES_H_ */
