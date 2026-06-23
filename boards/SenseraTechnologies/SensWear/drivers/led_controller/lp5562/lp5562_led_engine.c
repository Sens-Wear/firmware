/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file lp5562_led_engine.c
 * @brief LP5562 program-engine instruction builders.
 *
 * Pure construction of LP5562 engine instruction words. These helpers perform no
 * I2C: each returns the command register object (a command @c union) by value,
 * which the @ref lp5562.h driver functions later serialize into engine program
 * memory. See @ref lp5562_led_engines.h for the instruction model.
 */

#include "lp5562_led_engines.h"

union lp5562_ramp_command_t lp5562_ramp_command(uint8_t numberOfSteps,
												bool bIncrement,
												uint8_t stepTime,
												bool bPrescale) {
	union lp5562_ramp_command_t rampCommand = {0};
	rampCommand.fields.increment = (numberOfSteps > 0) ? (numberOfSteps - 1) : 0;
	rampCommand.fields.sign = (bIncrement == false) ? 1 : 0;
	rampCommand.fields.step_time = stepTime;
	rampCommand.fields.prescale = (bPrescale == false) ? 0 : 1;
	return rampCommand;
}

union lp5562_set_pwm_command_t lp5562_set_pwm_command(uint8_t pwm) {
	union lp5562_set_pwm_command_t pwmCommand = {0};
	pwmCommand.fields.pwm = pwm;
	pwmCommand.fields.const_pwm_set = 0x40;
	return pwmCommand;
}

union lp5562_branch_command_t lp5562_branch_command(uint8_t stepNumber, uint8_t loopCount) {
	union lp5562_branch_command_t branchCommand = {0};
	branchCommand.fields.step_number = stepNumber;
	branchCommand.fields.loop_count = loopCount;
	branchCommand.fields.const_branch = 5;
	return branchCommand;
}

union lp5562_end_command_t lp5562_end_command(bool bInterrupt, bool bResetPWM) {
	union lp5562_end_command_t endCommand = {0};
	endCommand.fields.const_end = 6;
	endCommand.fields.reset = (bResetPWM == false) ? 0 : 1;
	endCommand.fields.irq = (bInterrupt == false) ? 0 : 1;
	return endCommand;
}

union lp5562_trigger_command_t lp5562_trigger_command(uint8_t sendTriggers, uint8_t waitTriggers) {
	union lp5562_trigger_command_t triggerCommand = {0};
	triggerCommand.fields.send_trigger_for_eng_1 = ((sendTriggers & lp5562_engine_1) == 0) ? 0 : 1;
	triggerCommand.fields.send_trigger_for_eng_2 = ((sendTriggers & lp5562_engine_2) == 0) ? 0 : 1;
	triggerCommand.fields.send_trigger_for_eng_3 = ((sendTriggers & lp5562_engine_3) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_1 = ((waitTriggers & lp5562_engine_1) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_2 = ((waitTriggers & lp5562_engine_2) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_3 = ((waitTriggers & lp5562_engine_3) == 0) ? 0 : 1;
	triggerCommand.fields.const_trigger = 7;
	return triggerCommand;
}
