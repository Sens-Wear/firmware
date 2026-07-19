/*
 * lp5562.c
 *
 *  Created on: Jul 18, 2017
 *      Author: Huseyin Yigitler
 */

#include "lp5562.h"
#include "sys_i2c.h"

#include <assert.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * @brief Software configuration and lifecycle state of the LP5562 singleton.
 * @details Mirrors the CONFIG register control bits and adds a software-only
 *          @c bInitialized flag. This is a driver-side aggregate, not a raw
 *          register image.
 */
union lp5562_config_t {
	uint8_t config;
	struct lp5562_config_bits {
		uint8_t bChipEnable : 1;
		uint8_t bLogEnable : 1;
		uint8_t bPWM_hf : 1;
		uint8_t bPowerSave : 1;
		uint8_t ClkConfig : 2;
		uint8_t reserved : 1;
		uint8_t bInitialized : 1; //!< Indicates the singleton has been initialized.
	} bits;
};

/** @brief Cached state of the single LP5562 controller instance. */
struct lp5562_t {
	union lp5562_config_t config;						 //!< The configuration.
	struct lp5562_engine_t engines[LP5562_ENGINE_COUNT]; //!< Command execution engines.
	struct lp5562_led_t leds[LP5562_LED_COUNT];			 //!< The LEDs.
	union lp5562_led_map_register_t led_engines;		 //!< The operation engines of the LEDs.
};

/* The board has exactly one LP5562 and it lives on the shared SensWear I2C bus,
 * so the driver is a file-scope singleton that talks through the sys_i2c
 * ownership wrapper. Every high-level operation acquires the shared bus once and
 * releases it on exit; the private register helpers assume the bus is owned. */
static const struct sys_i2c_dt_spec lp5562_i2c = SYS_I2C_DT_SPEC_GET(DT_NODELABEL(lp5562));
static const struct gpio_dt_spec lp5562_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(lp5562), enable_gpios);
static struct lp5562_t lp5562;

/** Acquire shared-I2C ownership for one high-level operation. */
static int lp5562_bus_lock(void) {
	return sys_i2c_lock(&lp5562_i2c, K_MSEC(LP5562_I2C_TIMEOUT));
}

/** Release one level of shared-I2C ownership. */
static void lp5562_bus_unlock(void) {
	(void) sys_i2c_unlock(&lp5562_i2c);
}

/** Write one register. The caller must own the shared bus. */
static bool lp5562_write_register(uint8_t regAddress, uint8_t value) {
	uint8_t tx[2] = {regAddress, value};
	return sys_i2c_write(&lp5562_i2c, tx, sizeof(tx)) == 0;
}

/** Burst-write engine program memory. The caller must own the shared bus. */
static bool lp5562_write_memory(uint8_t regAddress, const uint8_t* pData, size_t dataLength) {
	uint8_t tx[1 + LP5562_ENGINE_COMMAND_COUNT * sizeof(uint16_t)];

	if(pData == NULL || dataLength == 0U || dataLength > (sizeof(tx) - 1U)) {
		return false;
	}
	tx[0] = regAddress;
	memcpy(&tx[1], pData, dataLength);
	return sys_i2c_write(&lp5562_i2c, tx, dataLength + 1U) == 0;
}

static void lp5562_enable_pin_set(void) {
	gpio_pin_set_dt(&lp5562_en, 1);
}

static void lp5562_enable_pin_reset(void) {
	gpio_pin_set_dt(&lp5562_en, 0);
}

/**
 * \brief Resets the cached device fields to their reset values.
 */
static void lp5562_reset_object(void) {
	int ind;
	// keep only the initialized flag to indicate that the singleton is valid --
	union lp5562_config_t config = {.config = lp5562.config.config};
	lp5562.config.config = LP5562_CONFIG_DEFAULT;
	lp5562.config.bits.bInitialized = config.bits.bInitialized;
	//-----------------------------------------------------------------
	for(ind = 0; ind < LP5562_ENGINE_COUNT; ind++) {
		lp5562.engines[ind].id = (enum lp5562_engine_type) (1 << ind);
		lp5562.engines[ind].command_count = 0;
		memset(lp5562.engines[ind].commands,
			   0,
			   sizeof(uint16_t) * LP5562_ENGINE_COMMAND_COUNT);
		lp5562.engines[ind].state = lp5562_engine_state_Hold;
		lp5562.engines[ind].mode = lp5562_engine_mode_Disabled;
	}
	//-----------------------------------------------------------------
	// set the default LED configuration
	for(ind = 0; ind < LP5562_LED_COUNT; ind++) {
		lp5562.leds[ind].type = (enum lp5562_led_type) ind;
		lp5562.leds[ind].current = LP5562_CURRENT_DEFAULT;
		lp5562.leds[ind].i2c_dim = LP5562_PWM_DEFAULT;
	}
	lp5562.leds[(uint8_t) lp5562_led_Red].engine = lp5562_engine_3;
	lp5562.leds[(uint8_t) lp5562_led_Green].engine = lp5562_engine_2;
	lp5562.leds[(uint8_t) lp5562_led_Blue].engine = lp5562_engine_1;
	lp5562.leds[(uint8_t) lp5562_led_White].engine = lp5562_engine_i2c_pwm;
	//-----------------------------------------------------------------
	lp5562.led_engines.value = LP5562_LED_MAP_DEFAULT;
}

void lp5562_initialize(void) {
	if(lp5562.config.bits.bInitialized != 0) {
		return;
	}
	lp5562_reset_object();
	// Drive the controller's enable line so the chip powers up and accepts I2C.
	if(gpio_is_ready_dt(&lp5562_en)) {
		(void) gpio_pin_configure_dt(&lp5562_en, GPIO_OUTPUT_ACTIVE);
		k_msleep(1);
	}
	lp5562.config.bits.bInitialized = 1;
}

bool lp5562_log_pwm_enable(bool enable) {
	union lp5562_enable_register_t enableRegister;
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	enableRegister.value = 0;
	enableRegister.bits.eng3_exec = lp5562.engines[2].state;
	enableRegister.bits.eng2_exec = lp5562.engines[1].state;
	enableRegister.bits.eng1_exec = lp5562.engines[0].state;
	enableRegister.bits.chip_en = lp5562.config.bits.bChipEnable;
	enableRegister.bits.log_en = (uint8_t) enable;
	bRet = lp5562_write_register(lp5562_register_ENABLE, enableRegister.value);
	if(bRet) {
		lp5562.config.bits.bLogEnable = (enable == false) ? 0 : 1;
	}
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_configure_clock(enum lp5562_clock_type clock) {
	union lp5562_config_register_t configRegister;
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	configRegister.value = 0;
	configRegister.bits.clk_config = (uint8_t) clock;
	configRegister.bits.reserved_1 = 0;
	configRegister.bits.powersave_en = lp5562.config.bits.bPowerSave;
	configRegister.bits.pwm_hf = lp5562.config.bits.bPWM_hf;
	bRet = lp5562_write_register(lp5562_register_CONFIG, configRegister.value);
	if(bRet) {
		lp5562.config.bits.ClkConfig = (uint8_t) clock;
	}
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_power_save(bool enable) {
	union lp5562_config_register_t configRegister;
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	configRegister.value = 0;
	configRegister.bits.clk_config = lp5562.config.bits.ClkConfig;
	configRegister.bits.powersave_en = (enable == false) ? 0 : 1;
	configRegister.bits.pwm_hf = lp5562.config.bits.bPWM_hf;
	bRet = lp5562_write_register(lp5562_register_CONFIG, configRegister.value);
	if(bRet) {
		lp5562.config.bits.bPowerSave = (enable == false) ? 0 : 1;
	}
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_pwm_hf(bool enable) {
	union lp5562_config_register_t configRegister;
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	configRegister.value = 0;
	configRegister.bits.clk_config = lp5562.config.bits.ClkConfig;
	configRegister.bits.powersave_en = lp5562.config.bits.bPowerSave;
	configRegister.bits.pwm_hf = (enable == false) ? 0 : 1;
	bRet = lp5562_write_register(lp5562_register_CONFIG, configRegister.value);
	if(bRet) {
		lp5562.config.bits.bPWM_hf = (enable == false) ? 0 : 1;
	}
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_chip_enable(void) {
	union lp5562_enable_register_t enableRegister;
	union lp5562_config_register_t configRegister;
	bool bRet;

	lp5562_enable_pin_set();

	if(lp5562.config.bits.bChipEnable != 0) {
		return true;
	}
	if(lp5562_bus_lock() != 0) {
		return false;
	}

	configRegister.value = 0;
	configRegister.bits.clk_config = lp5562.config.bits.ClkConfig;
	configRegister.bits.powersave_en = (lp5562.config.bits.bPowerSave == 0) ? 0 : 1;
	configRegister.bits.pwm_hf = lp5562.config.bits.bPWM_hf;
	(void) lp5562_write_register(lp5562_register_CONFIG, configRegister.value);
	//-----------------------------------------------------------------
	enableRegister.value = 0;
	enableRegister.bits.eng3_exec = lp5562.engines[2].state;
	enableRegister.bits.eng2_exec = lp5562.engines[1].state;
	enableRegister.bits.eng1_exec = lp5562.engines[0].state;
	enableRegister.bits.chip_en = 1;
	enableRegister.bits.log_en = lp5562.config.bits.bLogEnable;
	bRet = lp5562_write_register(lp5562_register_ENABLE, enableRegister.value);
	if(bRet) {
		k_msleep(1);
		lp5562.config.bits.bChipEnable = 1;
	}
	lp5562_bus_unlock();
	return bRet;
}

void lp5562_chip_disable(void) {
	union lp5562_enable_register_t enableRegister;

	if((lp5562.config.bits.bChipEnable != 0) && (lp5562_bus_lock() == 0)) {
		enableRegister.value = 0;
		enableRegister.bits.eng3_exec = lp5562_engine_state_Hold;
		enableRegister.bits.eng2_exec = lp5562_engine_state_Hold;
		enableRegister.bits.eng1_exec = lp5562_engine_state_Hold;
		enableRegister.bits.chip_en = 0;
		enableRegister.bits.log_en = lp5562.config.bits.bLogEnable;
		if(lp5562_write_register(lp5562_register_ENABLE, enableRegister.value)) {
			lp5562.config.bits.bChipEnable = 0;
			lp5562.engines[0].state = lp5562_engine_state_Hold;
			lp5562.engines[1].state = lp5562_engine_state_Hold;
			lp5562.engines[2].state = lp5562_engine_state_Hold;
		}
		lp5562_bus_unlock();
	}
	lp5562_enable_pin_reset();
}

bool lp5562_led_configure(enum lp5562_led_type led,
						  uint8_t current,
						  uint8_t i2c_pwm,
						  enum lp5562_engine_type engine) {
	uint8_t currentRegister, pwmRegister;
	union lp5562_led_map_register_t ledRegister;
	struct lp5562_led_t* pLED;
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	pLED = &(lp5562.leds[(uint8_t) led]);
	ledRegister.value = lp5562.led_engines.value;
	switch(pLED->type) {
	case lp5562_led_Red:
		currentRegister = lp5562_register_R_CURRENT;
		pwmRegister = lp5562_register_R_PWM;
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
		currentRegister = lp5562_register_G_CURRENT;
		pwmRegister = lp5562_register_G_PWM;
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
		currentRegister = lp5562_register_B_CURRENT;
		pwmRegister = lp5562_register_B_PWM;
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
		currentRegister = lp5562_register_W_CURRENT;
		pwmRegister = lp5562_register_W_PWM;
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

	if(lp5562_bus_lock() != 0) {
		return false;
	}
	bRet = lp5562_write_register(currentRegister, current) &&
		   lp5562_write_register(pwmRegister, i2c_pwm);
	if(bRet) {
		pLED->current = current;
		pLED->i2c_dim = i2c_pwm;
		bRet = lp5562_write_register(lp5562_register_ENG_SEL, ledRegister.value);
		if(bRet) {
			pLED->engine = engine;
			lp5562.led_engines.value = ledRegister.value;
		}
	}
	lp5562_bus_unlock();
	return bRet;
}

uint8_t lp5562_led_get_current(enum lp5562_led_type led) {
	return lp5562.leds[(uint8_t) led].current;
}

uint16_t* lp5562_engine_get_commands(enum lp5562_engine_type engine) {
	assert(lp5562.config.bits.bInitialized != 0);
	switch(engine) {
	case lp5562_engine_1:
		return lp5562.engines[0].commands;
	case lp5562_engine_2:
		return lp5562.engines[1].commands;
	case lp5562_engine_3:
		return lp5562.engines[2].commands;
	default:
		return NULL;
	}
}

bool lp5562_engine_configure_commands(enum lp5562_engine_type engine,
									  uint8_t commandCount,
									  bool run) {
	uint8_t regAddr;
	uint16_t* commands;
	uint8_t* commandCountPtr;
	bool bRet = false;
	assert(lp5562.config.bits.bInitialized != 0);

	switch(engine) {
	case lp5562_engine_1:
		regAddr = lp5562_register_PROG_MEM_ENG1;
		commands = lp5562.engines[0].commands;
		commandCountPtr = &(lp5562.engines[0].command_count);
		break;
	case lp5562_engine_2:
		regAddr = lp5562_register_PROG_MEM_ENG2;
		commands = lp5562.engines[1].commands;
		commandCountPtr = &(lp5562.engines[1].command_count);
		break;
	case lp5562_engine_3:
		regAddr = lp5562_register_PROG_MEM_ENG3;
		commands = lp5562.engines[2].commands;
		commandCountPtr = &(lp5562.engines[2].command_count);
		break;
	case lp5562_engine_i2c_pwm:
	default:
		return false;
	}

	/* Hold the shared bus for the whole load sequence; the nested mode/state
	 * calls re-enter the recursive sys_i2c lock and release their own level. */
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	if(!lp5562_engine_set_mode(engine, lp5562_engine_mode_Load)) {
		goto out;
	}
	if(!lp5562_write_memory(regAddr,
							(const uint8_t*) commands,
							commandCount * sizeof(uint16_t))) {
		goto out;
	}
	*commandCountPtr = commandCount;
	if(!lp5562_engine_set_mode(engine, lp5562_engine_mode_Run)) {
		goto out;
	}
	if(run != false) {
		bRet = lp5562_engine_set_state(engine, lp5562_engine_state_Run);
	}
out:
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_engine_set_mode(enum lp5562_engine_type engine,
							enum lp5562_engine_mode_type mode) {
	union lp5562_op_mode_register_t opModeRegister;
	bool bRet = false;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	opModeRegister.value = 0;
	opModeRegister.bits.eng3_mode = lp5562.engines[2].mode;
	opModeRegister.bits.eng2_mode = lp5562.engines[1].mode;
	opModeRegister.bits.eng1_mode = lp5562.engines[0].mode;
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
		goto out;
	}
	if(mode == lp5562_engine_mode_Load) {
		if(!lp5562_engine_set_state(engine, lp5562_engine_state_Hold)) {
			goto out;
		}
	}
	if(lp5562_write_register(lp5562_register_OP_MODE, opModeRegister.value)) {
		switch(engine) {
		case lp5562_engine_1:
			lp5562.engines[0].mode = mode;
			break;
		case lp5562_engine_2:
			lp5562.engines[1].mode = mode;
			break;
		case lp5562_engine_3:
			lp5562.engines[2].mode = mode;
			break;
		default:
			goto out;
		}
		bRet = true;
	}
out:
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_engine_set_state(enum lp5562_engine_type engine,
							 enum lp5562_engine_state_type state) {
	union lp5562_enable_register_t enableRegister = {0};
	bool bRet = false;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	enableRegister.bits.eng3_exec = lp5562.engines[2].state;
	enableRegister.bits.eng2_exec = lp5562.engines[1].state;
	enableRegister.bits.eng1_exec = lp5562.engines[0].state;
	enableRegister.bits.chip_en = lp5562.config.bits.bChipEnable;
	enableRegister.bits.log_en = lp5562.config.bits.bLogEnable;
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
		goto out;
	}
	if(lp5562_write_register(lp5562_register_ENABLE, enableRegister.value)) {
		switch(engine) {
		case lp5562_engine_1:
			lp5562.engines[0].state = state;
			break;
		case lp5562_engine_2:
			lp5562.engines[1].state = state;
			break;
		case lp5562_engine_3:
			lp5562.engines[2].state = state;
			break;
		default:
			goto out;
		}
		bRet = true;
	}
out:
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_engine_all_set_state(enum lp5562_engine_state_type state) {
	union lp5562_enable_register_t enableRegister = {0};
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	enableRegister.bits.eng3_exec = state;
	enableRegister.bits.eng2_exec = state;
	enableRegister.bits.eng1_exec = state;
	enableRegister.bits.chip_en = lp5562.config.bits.bChipEnable;
	enableRegister.bits.log_en = lp5562.config.bits.bLogEnable;
	bRet = lp5562_write_register(lp5562_register_ENABLE, enableRegister.value);
	if(bRet) {
		lp5562.engines[0].state = state;
		lp5562.engines[1].state = state;
		lp5562.engines[2].state = state;
	}
	lp5562_bus_unlock();
	return bRet;
}

bool lp5562_led_set_current(enum lp5562_led_type led, uint8_t current) {
	return lp5562_led_configure(led,
								current,
								lp5562.leds[(uint8_t) led].i2c_dim,
								lp5562.leds[(uint8_t) led].engine);
}

bool lp5562_led_set_i2c_dim(enum lp5562_led_type led, uint8_t dim) {
	return lp5562_led_configure(led,
								lp5562.leds[(uint8_t) led].current,
								dim,
								lp5562.leds[(uint8_t) led].engine);
}

bool lp5562_led_set_engine(enum lp5562_led_type led, enum lp5562_engine_type engine) {
	return lp5562_led_configure(led,
								lp5562.leds[(uint8_t) led].current,
								lp5562.leds[(uint8_t) led].i2c_dim,
								engine);
}

bool lp5562_reset(void) {
	bool bRet;
	assert(lp5562.config.bits.bInitialized != 0);
	if(lp5562_bus_lock() != 0) {
		return false;
	}
	bRet = lp5562_write_register(lp5562_register_RESET, LP5562_RESET);
	if(bRet) {
		lp5562_reset_object();
	}
	lp5562_bus_unlock();
	return bRet;
}
