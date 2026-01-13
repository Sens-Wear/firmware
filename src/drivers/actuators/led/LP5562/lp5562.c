/*
 * lp5562.c
 *
 *  Created on: Jul 18, 2017
 *      Author: Huseyin Yigitler
 */

#include "lp5562.h"
#include "string.h"
#include "assert.h"

static const struct i2c_dt_spec lp5562_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(lp5562));
static const struct gpio_dt_spec lp5562_en = GPIO_DT_SPEC_GET(DT_NODELABEL(lp5562), enable_gpios);


static void lp5562_enable_pin_set(struct lp5562_t* driver) {
    gpio_pin_set_dt(driver->enable_gpio, 1);
}

static void lp5562_enable_pin_reset(struct lp5562_t* driver) {
    gpio_pin_set_dt(driver->enable_gpio, 0);
}
/**
 * \brief Resets the device field to their reset values.
 * @param driver The LP5562 object.
 */
static void lp5562_reset_object(struct lp5562_t* driver) {
	int ind;
	// keep the address
	// ... and keep the configuration
	union lp5562_config_t config = {.config = driver->config.config};
	//-----------------------------------------------------------------
	// keep only the initialized flag to indicate that the address and enable pin are valid --
	driver->config.config = LP5562_CONFIG_DEFAULT;
	driver->config.bits.bInitialized = config.bits.bInitialized;
	//-----------------------------------------------------------------
	for(ind = 0; ind < LP5562_ENGINE_COUNT; ind++) {
		driver->engines[ind].id = (enum lp5562_engine_type) (1 << ind);
		driver->engines[ind].command_count = 0;
		memset(driver->engines[ind].commands,
			   0,
			   sizeof(uint16_t) * LP5562_ENGINE_COMMAND_COUNT);
		driver->engines[ind].state = lp5562_engine_state_Hold;
		driver->engines[ind].mode = lp5562_engine_mode_Disabled;
	}
	//-----------------------------------------------------------------
	// set the default LED configuration
	for(ind = 0; ind < LP5562_LED_COUNT; ind++) {
		driver->leds[ind].type = (enum lp5562_led_type) ind;
		driver->leds[ind].current = LP5562_CURRENT_DEFAULT;
		driver->leds[ind].i2c_dim = LP5562_PWM_DEFAULT;
	}
	driver->leds[(uint8_t) lp5562_led_Red].engine = lp5562_engine_3;
	driver->leds[(uint8_t) lp5562_led_Green].engine = lp5562_engine_2;
	driver->leds[(uint8_t) lp5562_led_Blue].engine = lp5562_engine_1;
	driver->leds[(uint8_t) lp5562_led_White].engine = lp5562_engine_i2c_pwm;
	//-----------------------------------------------------------------
	driver->led_engines.value = LP5562_LED_MAP_DEFAULT;
}
/*
 * \brief Creates a RAMP command.
 * @param numberOfSteps The number of steps in the ramp/
 * @param bIncrement true if incrementing, false if decrementing.
 * @param stepTime The time of a step. It depends on the prescale value.
 * @param bPrescale true if the clock is prescaled, false if not.
 * @return The created command.
 */
uint16_t lp5562_ramp_command(uint8_t numberOfSteps,
							 bool bIncrement,
							 uint8_t stepTime,
							 bool bPrescale) {
	union lp5562_ramp_command_t rampCommand = {0};
	rampCommand.fields.increment = (numberOfSteps > 0) ? (numberOfSteps - 1)
													   : 0;
	rampCommand.fields.sign = (bIncrement == false) ? 1 : 0;
	rampCommand.fields.step_time = stepTime;
	rampCommand.fields.prescale = (bPrescale == false) ? 0 : 1;
	return rampCommand.command;
}
/*
 * \brief Creates a SET PWM command.
 * @param pwm The PWM duty cycle value in %. 0 corresponds to 0% and 255 corresponds to 100%.
 * @return The created command.
 */
uint16_t lp5562_set_pwm_command(uint8_t pwm) {
	union lp5562_set_pwm_command_t pwmCommand = {0};
	pwmCommand.fields.pwm = pwm;
	pwmCommand.fields.const_pwm_set = 0x40;
	return pwmCommand.command;
}

/*
 * \brief Create a BRANCH command
 * @param stepNumber The step number to be loaded to program counter
 * @param loopCount The number loops to be done. 0 means infinite loop.
 * @return The created command.
 */
uint16_t lp5562_branch_command(uint8_t stepNumber, uint8_t loopCount) {
	union lp5562_branch_command_t branchCommand = {0};
	branchCommand.fields.step_number = stepNumber;
	branchCommand.fields.loop_count = loopCount;
	branchCommand.fields.const_branch = 5;
	return branchCommand.command;
}

/*
 * \brief Creates an END command
 * @param bInterrupt true if command creates and interrupt, false if not.
 * @param bResetPWM true to reset the PWM value to 0, false to keep the current value/
 * @return The created command
 */
uint16_t lp5562_end_command(bool bInterrupt, bool bResetPWM) {
	union lp5562_end_command_t endCommand = {0};
	endCommand.fields.const_end = 6;
	endCommand.fields.reset = (bResetPWM == false) ? 0 : 1;
	endCommand.fields.irq = (bInterrupt == false) ? 0 : 1;

	return endCommand.command;
}

/*
 * \brief Creates a TRIGGER command. OR more than one engine identifiers.
 * @param sendTriggers a logical of engine identifiers for sending triggers.
 * @param waitTriggers a logical of engine identifiers for waiting triggers.
 * @return The created command.
 */
uint16_t lp5562_trigger_command(uint8_t sendTriggers, uint8_t waitTriggers) {
	union lp5562_trigger_command_t triggerCommand = {0};
	triggerCommand.fields.send_trigger_for_eng_1 =
		((sendTriggers & lp5562_engine_1) == 0) ? 0 : 1;
	triggerCommand.fields.send_trigger_for_eng_2 =
		((sendTriggers & lp5562_engine_2) == 0) ? 0 : 1;
	triggerCommand.fields.send_trigger_for_eng_3 =
		((sendTriggers & lp5562_engine_3) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_1 =
		((waitTriggers & lp5562_engine_1) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_2 =
		((waitTriggers & lp5562_engine_2) == 0) ? 0 : 1;
	triggerCommand.fields.wait_trigger_for_eng_3 =
		((waitTriggers & lp5562_engine_3) == 0) ? 0 : 1;
	triggerCommand.fields.const_trigger = 7;
	return triggerCommand.command;
}

/*
 * \brief initializes the LP5562 object with its default values.
 * @param driver The driver object.
 * @param address I2C slave address of the controller.
 */
void lp5562_initialize(struct lp5562_t* driver) {
	assert(driver != NULL);
	if(driver->config.bits.bInitialized == 0) {
		lp5562_reset_object(driver);
		driver->device = &lp5562_i2c;
		driver->enable_gpio = &lp5562_en;
		driver->config.bits.bInitialized = 1;
	}
}

/*
 * \brief Enables LOGarithmic increase of PWMs
 * @param driver The LP5562 object
 * @param enable true to enable, false to disable the LOGarithmic PWM control
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_log_pwm_enable(struct lp5562_t* driver, bool enable) {
	bool bRet;
	union lp5562_enable_register_t enableRegister;
	assert(driver->config.bits.bInitialized != 0);
	enableRegister.value = 0;
	enableRegister.bits.eng3_exec = driver->engines[2].state;
	enableRegister.bits.eng2_exec = driver->engines[1].state;
	enableRegister.bits.eng1_exec = driver->engines[0].state;
	enableRegister.bits.chip_en = driver->config.bits.bChipEnable;
	enableRegister.bits.log_en = (uint8_t) enable;
	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_ENABLE,
									 enableRegister.value);
	if(bRet != false) {
		driver->config.bits.bLogEnable = (enable == false) ? 0 : 1;
		return true;
	}
	return false;
}

/*
 * \brief Configures the clock settings of the specified controller.
 * @param driver The LP5562 object.
 * @param clock The clock settings
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_configure_clock(struct lp5562_t* driver,
							enum lp5562_clock_type clock) {
	bool bRet;
	union lp5562_config_register_t configRegister;
	assert(driver->config.bits.bInitialized != 0);
	configRegister.value = 0;
	configRegister.bits.clk_config = (uint8_t) clock;
	configRegister.bits.reserved_1 = 0;
	configRegister.bits.powersave_en = driver->config.bits.bPowerSave;
	configRegister.bits.pwm_hf = driver->config.bits.bPWM_hf;

	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_CONFIG,
									 configRegister.value);
	if(bRet != false) {
		driver->config.bits.ClkConfig = (uint8_t) clock;
		return true;
	}
	return false;
}

/*
 * \brief Enables or disables power save mode of LP5562.
 * @param driver The LP5562 object
 * @param enable true to enable, false to disable the power save mode.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_power_save(struct lp5562_t* driver, bool enable) {
	bool bRet;
	union lp5562_config_register_t configRegister;
	assert(driver->config.bits.bInitialized != 0);
	configRegister.value = 0;
	configRegister.bits.clk_config = driver->config.bits.ClkConfig;
	configRegister.bits.powersave_en = (enable == false) ? 0 : 1;
	configRegister.bits.pwm_hf = driver->config.bits.bPWM_hf;
	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_CONFIG,
									 configRegister.value);
	if(bRet != false) {
		driver->config.bits.bPowerSave = (enable == false) ? 0 : 1;
		return true;
	}
	return false;
}

/*
 * \brief Enables or disables high frequency PWM mode of LP5562.
 * @param driver The LP5562 object
 * @param enable true to enable, false to disable the high frequency PWM mode.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_pwm_hf(struct lp5562_t* driver, bool enable) {
	bool bRet;
	union lp5562_config_register_t configRegister;
	assert(driver->config.bits.bInitialized != 0);
	configRegister.value = 0;
	configRegister.bits.clk_config = driver->config.bits.ClkConfig;
	configRegister.bits.powersave_en = driver->config.bits.bPowerSave;
	configRegister.bits.pwm_hf = (enable == false) ? 0 : 1;
	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_CONFIG,
									 configRegister.value);
	if(bRet != false) {
		driver->config.bits.bPWM_hf = (enable == false) ? 0 : 1;
		return true;
	}
	return false;
}

/*
 * \brief Enables/disables the controller by enabling the Chip Enable bit.
 * @param driver The LP5562 object.
 * @param enable true to enable, false to disable the  controller.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_chip_enable(struct lp5562_t* driver) {
	union lp5562_enable_register_t enableRegister;
	bool bRet;

	lp5562_enable_pin_set(driver);

	if(driver->config.bits.bChipEnable == 0) {
		union lp5562_config_register_t configRegister;
		//		bRet = lp5562_i2c_write_register(driver, LP5562_REG_RESET, 0xFF);
		//		if(bRet == false) {
		//			return false;
		//		}
		//		driver->engines[0].command_count = 0;
		//		driver->engines[1].command_count = 0;
		//		driver->engines[2].command_count = 0;

		configRegister.value = 0;
		configRegister.bits.clk_config = driver->config.bits.ClkConfig;
		configRegister.bits.powersave_en =
			(driver->config.bits.bPowerSave == 0) ? 0 : 1;
		configRegister.bits.pwm_hf = driver->config.bits.bPWM_hf;
		bRet = lp5562_i2c_write_register(driver,
										 LP5562_REG_CONFIG,
										 configRegister.value);
		//-----------------------------------------------------------------

		enableRegister.value = 0;
		enableRegister.bits.eng3_exec = driver->engines[2].state;
		enableRegister.bits.eng2_exec = driver->engines[1].state;
		enableRegister.bits.eng1_exec = driver->engines[0].state;
		enableRegister.bits.chip_en = 1;
		enableRegister.bits.log_en = driver->config.bits.bLogEnable;

		bRet = lp5562_i2c_write_register(driver,
										 LP5562_REG_ENABLE,
										 enableRegister.value);
		if(bRet != false) {
			k_msleep(1);
			driver->config.bits.bChipEnable = 1;
			return true;
		}
		return false;
	}
	return true;
}

void lp5562_chip_disable(struct lp5562_t* driver) {
	union lp5562_enable_register_t enableRegister;
	bool bRet;
	if(driver->config.bits.bChipEnable != 0) {
		enableRegister.value = 0;
		enableRegister.bits.eng3_exec = lp5562_engine_state_Hold;
		enableRegister.bits.eng2_exec = lp5562_engine_state_Hold;
		enableRegister.bits.eng1_exec = lp5562_engine_state_Hold;
		enableRegister.bits.chip_en = 0;
		enableRegister.bits.log_en = driver->config.bits.bLogEnable;
		bRet = lp5562_i2c_write_register(driver,
										 LP5562_REG_ENABLE,
										 enableRegister.value);
		if(bRet != false) {
			driver->config.bits.bChipEnable = 0;
			driver->engines[0].state = lp5562_engine_state_Hold;
			driver->engines[1].state = lp5562_engine_state_Hold;
			driver->engines[2].state = lp5562_engine_state_Hold;
		}
	}
	lp5562_enable_pin_reset(driver);
}

bool lp5562_i2c_enable(struct lp5562_t* driver) {
	bool bRet = true;
	assert(driver->config.bits.bInitialized != 0);
	// bRet = LP5562_I2C_LOCK(driver, driver->i2c_config, LP5562_I2C_TIMEOUT);
	// if(bRet != false) {
		lp5562_enable_pin_set(driver);
	// }
	return bRet;
}
void lp5562_i2c_disable(struct lp5562_t* driver) {
	// LP5562_I2C_UNLOCK(driver);
}
/*
 * \brief Configures a LED of the specified LP5562.
 * @param driver The LP5562 object.
 * @param led The LED to configured.
 * @param current Max current limit of the driver.
 * @param i2c_pwm I2C pwm value.
 * @param engine The current command execution engine of the LED.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_led_configure(struct lp5562_t* driver,
						  enum lp5562_led_type led,
						  uint8_t current,
						  uint8_t i2c_pwm,
						  enum lp5562_engine_type engine) {
	uint8_t currentRegister, pwmRegister;
	union lp5562_led_map_register_t ledRegister;
	struct lp5562_led_t* pLED;
	bool bRet;
	assert(driver->config.bits.bInitialized != 0);
	pLED = &(driver->leds[(uint8_t) led]);
	ledRegister.value = driver->led_engines.value;
	switch(pLED->type) {
	case lp5562_led_Red:
		currentRegister = LP5562_REG_R_CURRENT;
		pwmRegister = LP5562_REG_R_PWM;
		switch(engine) {
		case lp5562_engine_1:
			ledRegister.bits.r_eng_sel = 1;
			break;
		case lp5562_engine_2:
			ledRegister.bits.r_eng_sel = 2;
			break;
		case lp5562_engine_3:
			ledRegister.bits.r_eng_sel = 3;
			break;
		case lp5562_engine_i2c_pwm:
			ledRegister.bits.r_eng_sel = 0;
			break;
		default:
			return false;
		}
		break;
	case lp5562_led_Green:
		currentRegister = LP5562_REG_G_CURRENT;
		pwmRegister = LP5562_REG_G_PWM;
		switch(engine) {
		case lp5562_engine_1:
			ledRegister.bits.g_eng_sel = 1;
			break;
		case lp5562_engine_2:
			ledRegister.bits.g_eng_sel = 2;
			break;
		case lp5562_engine_3:
			ledRegister.bits.g_eng_sel = 3;
			break;
		case lp5562_engine_i2c_pwm:
			ledRegister.bits.g_eng_sel = 0;
			break;
		default:
			return false;
		}
		break;
	case lp5562_led_Blue:
		currentRegister = LP5562_REG_B_CURRENT;
		pwmRegister = LP5562_REG_B_PWM;
		switch(engine) {
		case lp5562_engine_1:
			ledRegister.bits.b_eng_sel = 1;
			break;
		case lp5562_engine_2:
			ledRegister.bits.b_eng_sel = 2;
			break;
		case lp5562_engine_3:
			ledRegister.bits.b_eng_sel = 3;
			break;
		case lp5562_engine_i2c_pwm:
			ledRegister.bits.b_eng_sel = 0;
			break;
		default:
			return false;
		}
		break;
	case lp5562_led_White:
		currentRegister = LP5562_REG_W_CURRENT;
		pwmRegister = LP5562_REG_W_PWM;
		switch(engine) {
		case lp5562_engine_1:
			ledRegister.bits.w_eng_sel = 1;
			break;
		case lp5562_engine_2:
			ledRegister.bits.w_eng_sel = 2;
			break;
		case lp5562_engine_3:
			ledRegister.bits.w_eng_sel = 3;
			break;
		case lp5562_engine_i2c_pwm:
			ledRegister.bits.w_eng_sel = 0;
			break;
		default:
			return false;
		}
		break;
	default:
		return false;
	}

	bRet = lp5562_i2c_write_register(driver, currentRegister, current);
	if(bRet == false) {
		return false;
	}
	bRet = lp5562_i2c_write_register(driver, pwmRegister, i2c_pwm);
	if(bRet == false) {
		return false;
	}

	pLED->current = current;
	pLED->i2c_dim = i2c_pwm;

	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_ENG_SEL,
									 ledRegister.value);
	if(bRet != false) {
		pLED->engine = engine;
		driver->led_engines.value = ledRegister.value;
		return true;
	}
	return false;
}

/*
 * \brief Returns the start address of the local cache of the execution engine commands.
 * @param driver The LP5562 object.
 * @param engine The engine.
 * @return NULL if the specified engine does not contain a command cache, otherwise the start address.
 */
uint16_t* lp5562_engine_get_commands(struct lp5562_t* driver,
									 enum lp5562_engine_type engine) {
	assert(driver->config.bits.bInitialized != 0);
	switch(engine) {
	case lp5562_engine_1:
		return driver->engines[0].commands;
	case lp5562_engine_2:
		return driver->engines[1].commands;
	case lp5562_engine_3:
		return driver->engines[2].commands;
	default:
		return NULL;
	}
}

/*
 * \brief Configures the SRAM memory of an execution engine of the specified LP5562 controller.
 * @param driver The LP5562 object.
 * @param engine The engine
 * @param commandCount The number of commands in the local cache.
 * @param run true to start execution of the engine.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_engine_configure_commands(struct lp5562_t* driver,
									  enum lp5562_engine_type engine,
									  uint8_t commandCount,
									  bool run) {
	bool bRet, retVal;
	assert(driver->config.bits.bInitialized != 0);
	uint8_t regAddr;
	uint16_t* commands;
	uint8_t* commandCountPtr;
	bRet = lp5562_engine_set_mode(driver, engine, lp5562_engine_mode_Load);
	if(bRet == false) {
		return false;
	}
	switch(engine) {
	case lp5562_engine_1:
		regAddr = LP5562_REG_PROG_MEM_ENG1;
		commands = driver->engines[0].commands;
		commandCountPtr = &(driver->engines[0].command_count);
		break;
	case lp5562_engine_2:
		regAddr = LP5562_REG_PROG_MEM_ENG2;
		commands = driver->engines[1].commands;
		commandCountPtr = &(driver->engines[1].command_count);
		break;
	case lp5562_engine_3:
		regAddr = LP5562_REG_PROG_MEM_ENG3;
		commands = driver->engines[2].commands;
		commandCountPtr = &(driver->engines[2].command_count);
		break;
	case lp5562_engine_i2c_pwm:
	default:
		return false;
	}

	bRet = lp5562_i2c_write_memory(driver,
								   regAddr,
								   (uint8_t*) commands,
								   commandCount * sizeof(uint16_t));
	retVal = bRet;
	if(bRet != false) {
		*commandCountPtr = commandCount;
	}
	bRet = lp5562_engine_set_mode(driver, engine, lp5562_engine_mode_Run);
	if(bRet == false) {
		return false;
	} else {
		retVal &= bRet;
		if((retVal != false) && (run != false)) {
			return lp5562_engine_set_state(driver,
										   engine,
										   lp5562_engine_state_Run);
		} else {
			return false;
		}
	}
}

/*
 * \brief Changes the mode of the specified execution engine.
 * @param driver The LP5562 object
 * @param engine The engine
 * @param mode The new mode.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_engine_set_mode(struct lp5562_t* driver,
							enum lp5562_engine_type engine,
							enum lp5562_engine_mode_type mode) {
	bool bRet;
	union lp5562_op_mode_register_t opModeRegister;
	assert(driver->config.bits.bInitialized != 0);
	opModeRegister.value = 0;
	opModeRegister.bits.eng3_mode = driver->engines[2].mode;
	opModeRegister.bits.eng2_mode = driver->engines[1].mode;
	opModeRegister.bits.eng1_mode = driver->engines[0].mode;
	switch(engine) {
	case lp5562_engine_1:
		opModeRegister.bits.eng1_mode = mode;
		break;
	case lp5562_engine_2:
		opModeRegister.bits.eng2_mode = mode;
		break;
	case lp5562_engine_3:
		opModeRegister.bits.eng3_mode = mode;
		break;
	default:
		return false;
	}
	if(mode == lp5562_engine_mode_Load) {
		bRet =
			lp5562_engine_set_state(driver, engine, lp5562_engine_state_Hold);
		if(bRet == false) {
			return false;
		}
	}
	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_OP_MODE,
									 opModeRegister.value);
	if(bRet != false) {
		switch(engine) {
		case lp5562_engine_1:
			driver->engines[0].mode = mode;
			break;
		case lp5562_engine_2:
			driver->engines[1].mode = mode;
			break;
		case lp5562_engine_3:
			driver->engines[2].mode = mode;
			break;
		default:
			return false;
		}
		return true;
	}
	return false;
}

/*
 * \brief Sets the engine state of the specified execution engine.
 * @param driver The LP5562 object.
 * @param engine The execution engine.
 * @param state The new state
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_engine_set_state(struct lp5562_t* driver,
							 enum lp5562_engine_type engine,
							 enum lp5562_engine_state_type state) {
	union lp5562_enable_register_t enableRegister = {0};
	bool bRet;
	assert(driver->config.bits.bInitialized != 0);

	enableRegister.bits.eng3_exec = driver->engines[2].state;
	enableRegister.bits.eng2_exec = driver->engines[1].state;
	enableRegister.bits.eng1_exec = driver->engines[0].state;
	enableRegister.bits.chip_en = driver->config.bits.bChipEnable;
	enableRegister.bits.log_en = driver->config.bits.bLogEnable;

	switch(engine) {
	case lp5562_engine_1:
		enableRegister.bits.eng1_exec = state;
		break;
	case lp5562_engine_2:
		enableRegister.bits.eng2_exec = state;
		break;
	case lp5562_engine_3:
		enableRegister.bits.eng3_exec = state;
		break;
	default:
		return false;
	}
	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_ENABLE,
									 enableRegister.value);
	if(bRet != false) {
		switch(engine) {
		case lp5562_engine_1:
			driver->engines[0].state = state;
			break;
		case lp5562_engine_2:
			driver->engines[1].state = state;
			break;
		case lp5562_engine_3:
			driver->engines[2].state = state;
			break;
		default:
			return false;
		}
		return true;
	}
	return false;
}

bool lp5562_engine_all_set_state(struct lp5562_t* driver,
								 enum lp5562_engine_state_type state) {
	union lp5562_enable_register_t enableRegister = {0};
	bool bRet;
	assert(driver->config.bits.bInitialized != 0);

	enableRegister.bits.eng3_exec = state;
	enableRegister.bits.eng2_exec = state;
	enableRegister.bits.eng1_exec = state;
	enableRegister.bits.chip_en = driver->config.bits.bChipEnable;
	enableRegister.bits.log_en = driver->config.bits.bLogEnable;

	bRet = lp5562_i2c_write_register(driver,
									 LP5562_REG_ENABLE,
									 enableRegister.value);
	if(bRet != false) {
		driver->engines[0].state = state;
		driver->engines[1].state = state;
		driver->engines[2].state = state;
		return true;
	}
	return false;
}
/*
 * \brief Sends RESET command to reset the device registers to their default values.
 * @param driver The LP5562 object.
 * @return true if the I2C communication ends successfully, false otherwise.
 */
bool lp5562_reset(struct lp5562_t* driver) {
	bool bRet;
	assert(driver->config.bits.bInitialized != 0);
	bRet = lp5562_i2c_write_register(driver, LP5562_REG_RESET, 0xFF);
	if(bRet != false) {
		lp5562_reset_object(driver);
		return true;
	}
	return false;
}
