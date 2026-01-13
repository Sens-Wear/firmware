/*
 * pingbit_led_controller.c
 *
 *  Created on: Jul 19, 2017
 *      Author: yusei
 */

#include "led_controller.h"
#include <string.h>
#include <assert.h>

//extern I2C_HandleTypeDef LP5562_I2C_DRIVER;
#define LP5562_I2C_ADDRESS(__id) ((LP5562_I2C_ADDRESS_BASE) + (__id << 1))

#define RUNNING_STATE 3
#define LP5562_MASTER_CLOCK_FREQ 2048
//64

static struct led_controller_t m_pb_led_controller = {.status.value = 0};
struct led_controller_t* pLedController = &m_pb_led_controller;

/**
 * \brief Sends data to the specified LED controller.
 *
 *
 * @param deviceId The device identifier.
 * @param pData The data
 * @param dataLength The number of bytes forming the data array.
 *
 * @return true if I2C transfer completes successfully; false otherwise.
 */
bool lp5562_i2c_write(const struct lp5562_t* pDriver,
					  const uint8_t* pData,
					  const size_t data_count);

/*
 * \brief Initializes led controller wrapper library.
 */
bool led_controller_init(void) {
	if(pLedController->status.bits.bInitialized != 0) {
		return true;
	}
	// pLedController->device = DEVICE_DT_GET(DT_NODELABEL(lp5562));
	//-----------------------------------------------------------------
	// initialize the enable pins
	{
		static const struct gpio_dt_spec enable_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(lp5562), enable_gpios);
		int ret;
		if (!gpio_is_ready_dt(&enable_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&enable_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}
		// gpio_pin_set_dt(&enable_gpio, GPIO_OUTPUT_INACTIVE);
	}
	// initialize the lp5562 object
	lp5562_initialize(&(m_pb_led_controller.led_driver));
	pLedController->status.bits.bInitialized = 1;
	return true;
}

/*
 * \brief Configures the LED controllers of the system.
 * @return
 */
bool led_controller_configure(void) {
	if(pLedController->status.bits.bInitialized == 0) {
		return false;
	} else if(pLedController->status.bits.bConfigured != 0) {
		return true;
	} else {
		enum lp5562_clock_type clock =
			(enum lp5562_clock_type) ((PINGBIT_LED_FORCE_INTERNAL_CLOCK == 0)
										  ? lp5562_clk_Automatic
										  : lp5562_clk_Internal);
		bool bPowerSave = (PINGBIT_LED_ENABLE_POWERSAVE == 0) ? false : true;
		bool bPWMhf = (PINGBIT_LED_ENABLE_PWM_HF == 0) ? false : true;
		bool bLogPWM = (PINGBIT_LED_USE_LOG_PWM == 0) ? false : true;

		bool bRet = lp5562_i2c_enable(&(pLedController->led_driver));
		if(bRet == false) {
			return false;
		}
		//-----------------------------------------------------------------
		bRet = lp5562_reset(&(pLedController->led_driver));
		if(bRet == false) {
			return false;
		}
		//-----------------------------------------------------------------
		bRet = lp5562_configure_clock(&(pLedController->led_driver), clock);
		if(bRet == false) {
			return false;
		}
		//-----------------------------------------------------------------
		bRet = lp5562_power_save(&(pLedController->led_driver), bPowerSave);
		if(bRet == false) {
			return false;
		}

		//-----------------------------------------------------------------
		bRet = lp5562_pwm_hf(&(pLedController->led_driver), bPWMhf);
		if(bRet == false) {
			return false;
		}

		//-----------------------------------------------------------------
		bRet = lp5562_log_pwm_enable(&(pLedController->led_driver), bLogPWM);
		if(bRet == false) {
			return false;
		}

		//-----------------------------------------------------------------
		bRet = lp5562_led_configure(&(pLedController->led_driver),
									lp5562_led_Red,
									RED_LED_MAX_CURRENT *
										PINGBIT_LED_DRIVER_CURRENT_SCALE,
									0,
									lp5562_engine_i2c_pwm);
		if(bRet == false) {
			return false;
		}
		bRet = lp5562_led_configure(&(pLedController->led_driver),
									lp5562_led_Green,
									GREEN_LED_MAX_CURRENT *
										PINGBIT_LED_DRIVER_CURRENT_SCALE,
									0,
									lp5562_engine_i2c_pwm);
		if(bRet == false) {
			return false;
		}
		bRet = lp5562_led_configure(&(pLedController->led_driver),
									lp5562_led_Blue,
									BLUE_LED_MAX_CURRENT *
										PINGBIT_LED_DRIVER_CURRENT_SCALE,
									0,
									lp5562_engine_i2c_pwm);
		if(bRet == false) {
			return false;
		}
		bRet = lp5562_led_configure(&(pLedController->led_driver),
									lp5562_led_White,
									WHITE_LED_MAX_CURRENT *
										PINGBIT_LED_DRIVER_CURRENT_SCALE,
									0,
									lp5562_engine_i2c_pwm);
		if(bRet == false) {
			return false;
		}
		bRet = lp5562_chip_enable(&(pLedController->led_driver));
		if(bRet == false) {
			return false;
		}
		pLedController->status.bits.bConfigured = 1;
		return true;
	}
}

bool lp5562_i2c_write_register(const struct lp5562_t* pDriver,
							   uint8_t regAddress,
							   uint8_t value) {
	int ret;

    if (pDriver == NULL) {
        return false;
    }

    if (!device_is_ready(pDriver->device->bus)) {
        return false;
    }

    /* Optional: lock if your driver can be called concurrently */
    /* k_mutex_lock((struct k_mutex *)&pDriver->lock, K_FOREVER); */

	// ret = i2c_configure(pDriver->device.bus, pDriver->i2c_config.i2c_cfg);
	// if (ret < 0) {
	// 	/* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */
	// 	return false;
	// }

    /* Blocking write to the target address from DTS (pDriver->i2c.addr) */
    ret = i2c_reg_write_byte_dt(pDriver->device, regAddress, value);
	assert(ret == 0);
    /* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */

    return (ret == 0);
}

bool lp5562_i2c_write_memory(const struct lp5562_t *pDriver,
                             uint8_t regAddress,
                             uint8_t *pData,
                             size_t dataLength) {
	int ret;

    if (pDriver == NULL || pData == NULL || dataLength == 0U) {
        return false;
    }

    if (!device_is_ready(pDriver->device->bus)) {
        return false;
    }

    /* Optional: lock if your driver can be called concurrently */
    /* k_mutex_lock((struct k_mutex *)&pDriver->lock, K_FOREVER); */

	// ret = i2c_configure(pDriver->device.bus, pDriver->i2c_config.i2c_cfg);
	// if (ret < 0) {
	// 	/* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */
	// 	return false;
	// }

    /* Blocking write to the target address from DTS (pDriver->i2c.addr) */
    ret = i2c_burst_write_dt(pDriver->device, regAddress, pData, dataLength);

    /* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */

    return (ret == 0);
}

/*
 * \brief Sends data to the specified LED controller.
 *
 *
 * @param deviceId The device identifier.
 * @param pData The data
 * @param dataLength The number of bytes forming the data array.
 * @param timeOut The timeout duration in OS ticks.
 *
 * @return true if I2C transfer completes successfully; false otherwise.
 */
bool lp5562_i2c_write(const struct lp5562_t* pDriver,
					  const uint8_t* pData,
					  const size_t dataLength) {
	int ret;

    if (pDriver == NULL || pData == NULL || dataLength == 0U) {
        return false;
    }

    if (!device_is_ready(pDriver->device->bus)) {
        return false;
    }

    /* Optional: lock if your driver can be called concurrently */
    /* k_mutex_lock((struct k_mutex *)&pDriver->lock, K_FOREVER); */

	// ret = i2c_configure(pDriver->device.bus, pDriver->i2c_config.i2c_cfg);
	// if (ret < 0) {
	// 	/* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */
	// 	return false;
	// }

    /* Blocking write to the target address from DTS (pDriver->i2c.addr) */
    ret = i2c_write_dt(pDriver->device, pData, dataLength);

    /* k_mutex_unlock((struct k_mutex *)&pDriver->lock); */

    return (ret == 0);
}

/*
 * \brief turns off all the leds.
 */
void led_controller_shutdown(void) {
	led_controller_turn_off_leds(0);
	led_controller_turn_off_leds(1);
}
/*
 * \brief Turns on the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @param color The color of the LED when turn on.
 * @return true if the LED is successfully turned on, false otherwise.
 */
bool led_controller_turn_on_leds(uint8_t led, union led_color_t color) {
	if(pLedController->status.value != RUNNING_STATE) {
		return false;
	} else {
		bool bRet;
		switch(led) {
		case 0:
			lp5562_chip_enable(&(pLedController->led_driver));
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Red,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Red]
											.current,
										color.leds.red,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Green,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Green]
											.current,
										color.leds.green,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Blue,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Blue]
											.current,
										color.leds.blue,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_White,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_White]
											.current,
										color.leds.white,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			break;
		default:
			return false;
		}
		return true;
	}
}
/*
 * \brief Turns off the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @return true if the LED is successfully turned off, false otherwise.
 */
bool led_controller_turn_off_leds(uint8_t led) {
	if(pLedController->status.value != RUNNING_STATE) {
		return false;
	} else {
		bool bRet;
		union led_color_t color = {0};
		switch(led) {
		case 0:
			lp5562_chip_enable(&(pLedController->led_driver));
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Red,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Red]
											.current,
										color.leds.red,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Green,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Green]
											.current,
										color.leds.green,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_Blue,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_Blue]
											.current,
										color.leds.blue,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_White,
										pLedController->led_driver
											.leds[(uint8_t) lp5562_led_White]
											.current,
										color.leds.white,
										lp5562_engine_i2c_pwm);
			if(bRet == false) {
				return false;
			}
			lp5562_chip_enable(&(pLedController->led_driver));
			break;
		default:
			return false;
		}
		return true;
	}
}

static int lp5562_ramp_command_for_duration(uint16_t* pCommands,
											enum led_pwm_duration_type duration,
											bool bIncrement,
											uint8_t intensity) {
	float fadeTime = 0.0;
	int retVal = 0;

	switch(duration) {
	case led_pwm_duration_0ms:
		fadeTime = 0;
		break;
	case led_pwm_duration_64ms:
		fadeTime = 0.064;
		break;
	case led_pwm_duration_128ms:
		fadeTime = 0.128;
		break;
	case led_pwm_duration_256ms:
		fadeTime = 0.256;
		break;
	case led_pwm_duration_512ms:
		fadeTime = 0.512;
		break;
	case led_pwm_duration_1024ms:
		fadeTime = 1.024;
		break;
	case led_pwm_duration_2048ms:
		fadeTime = 2.048;
		break;
	case led_pwm_duration_3072ms:
		fadeTime = 3.072;
		break;
	case led_pwm_duration_4096ms:
		fadeTime = 4.096;
		break;
	case led_pwm_duration_16320ms:
		fadeTime = 16.320;
		break;
	}

	fadeTime *= LP5562_MASTER_CLOCK_FREQ;
	if(intensity < 16) {
		// this is a set pwm command
		pCommands[0] = lp5562_command_reorder_bytes(lp5562_set_pwm_command(0));
		retVal = 1;
	} else {
		uint16_t stepTime;
		fadeTime = (fadeTime) / ((float) intensity);
		stepTime = ((uint16_t) (fadeTime)) + 1;
		if(stepTime < 64) {
			if(intensity > 128) {
				if(stepTime > 1) {
					//stepTime --;
				}
				pCommands[0] = lp5562_command_reorder_bytes(
					lp5562_ramp_command(127, bIncrement, stepTime, false));
				pCommands[1] = lp5562_command_reorder_bytes(
					lp5562_ramp_command(intensity - 128,
										bIncrement,
										stepTime,
										false));
				retVal = 2;
			} else {
				pCommands[0] =
					lp5562_command_reorder_bytes(lp5562_ramp_command(intensity,
																	 bIncrement,
																	 stepTime,
																	 false));
				retVal = 1;
			}
		} else {
			retVal = 0;
		}
	}
	return retVal;
}

static int lp5562_wait_command_for_duration(uint16_t* pCommands,
											enum led_pwm_duration_type duration,
											int commandIndex) {
	int retVal = 0;
	switch(duration) {
	case led_pwm_duration_0ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(1, true));
		retVal = 1;
		break;
	case led_pwm_duration_64ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(3, true));
		retVal = 1;
		break;
	case led_pwm_duration_128ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(7, true));
		retVal = 1;
		break;
	case led_pwm_duration_256ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(15, true));
		retVal = 1;
		break;
	case led_pwm_duration_512ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(31, true));
		retVal = 1;
		break;
	case led_pwm_duration_1024ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(63, true));
		retVal = 1;
		break;
	case led_pwm_duration_2048ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(63, true));
		pCommands[1] = lp5562_command_reorder_bytes(
			lp5562_branch_command((uint8_t) (commandIndex & 0x0000000F), 2));
		retVal = 2;
		break;
	case led_pwm_duration_3072ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(63, true));
		pCommands[1] = lp5562_command_reorder_bytes(
			lp5562_branch_command((uint8_t) (commandIndex & 0x0000000F), 3));
		retVal = 2;
		break;
	case led_pwm_duration_4096ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(63, true));
		pCommands[1] = lp5562_command_reorder_bytes(
			lp5562_branch_command((uint8_t) (commandIndex & 0x0000000F), 4));
		retVal = 2;
		break;
		;
	case led_pwm_duration_16320ms:
		pCommands[0] =
			lp5562_command_reorder_bytes(lp5562_wait_command(63, true));
		pCommands[1] = lp5562_command_reorder_bytes(
			lp5562_branch_command((uint8_t) (commandIndex & 0x0000000F), 16));
		retVal = 2;
		break;
	}
	return retVal;
}

static int lp5562_pwm_to_command(uint16_t* pCommands,
								 struct led_pwm_t* pwm,
								 uint8_t intensity,
								 int index) {
	//-----------------------------------------------------------------
	// start from from dim level 0
	pCommands[index++] =
		lp5562_command_reorder_bytes(lp5562_set_pwm_command(0));

	//-----------------------------------------------------------------
	// ramp up
	index += lp5562_ramp_command_for_duration(&(pCommands[index]),
											  pwm->state_durations[(
												  int) led_pwm_FadeOn],
											  true,
											  intensity);
	//-----------------------------------------------------------------
	// hold ON state
	index += lp5562_wait_command_for_duration(&(pCommands[index]),
											  pwm->state_durations[(
												  int) led_pwm_FullyOn],
											  index);
	//-----------------------------------------------------------------
	// ramp down
	index += lp5562_ramp_command_for_duration(&(pCommands[index]),
											  pwm->state_durations[(
												  int) led_pwm_FadeOff],
											  false,
											  intensity);
	//-----------------------------------------------------------------
	// hold OFF state
	index += lp5562_wait_command_for_duration(&(pCommands[index]),
											  pwm->state_durations[(
												  int) led_pwm_FullyOff],
											  index);
	//-----------------------------------------------------------------
	// branch to start
	//	pCommands[index ++] = lp5562_command_reorder_bytes(lp5562_branch_command(0, 0));
	//
	//	pCommands[index ++] = lp5562_command_reorder_bytes(lp5562_end_command(false, false));
	return index;
}

static int lp5562_pwm_end_command(uint16_t* pCommands, int index) {
	//-----------------------------------------------------------------
	// branch to start
	pCommands[index++] =
		lp5562_command_reorder_bytes(lp5562_branch_command(0, 0));

	pCommands[index++] =
		lp5562_command_reorder_bytes(lp5562_end_command(false, false));
	return index;
}
/*
 * \brief Sets a PWM operation for the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @param pwm A pointer to the PWM object.
 * @return true if the LED is successfully turned off, false otherwise.
 */
bool led_controller_set_led_pwm(uint8_t led, struct led_pwm_t* pwm) {
	if(pLedController->status.value != RUNNING_STATE) {
		return false;
	} else if(pwm->id != 0) {
		return false;
	} else {
		uint16_t* pCommands;
		int commandCount = 0;
		bool bRet;
		switch(led) {
		case 0:
			lp5562_chip_enable(&(pLedController->led_driver));
			k_msleep(1);
			/*-------------------------------------------------------------------------*/
			// 1st controller
			//-----------------------------------------------------------------
			// white LED
			bRet = lp5562_led_configure(&(pLedController->led_driver),
										lp5562_led_White,
										pLedController->led_driver
											.leds[(int) lp5562_led_White]
											.current,
										pwm->color.leds.white,
										lp5562_engine_i2c_pwm);
			//-----------------------------------------------------------------
			// red LED
			pCommands =
				lp5562_engine_get_commands(&(pLedController->led_driver),
										   lp5562_engine_1);
			commandCount =
				lp5562_pwm_to_command(pCommands, pwm, pwm->color.leds.red, 0);
			pCommands[commandCount] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(lp5562_engine_1 | lp5562_engine_2 |
										   lp5562_engine_3,
									   0));
			pCommands[commandCount + 1] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(0, lp5562_engine_2 | lp5562_engine_3));
			commandCount += 2;
			commandCount = lp5562_pwm_end_command(pCommands, commandCount);

			bRet = lp5562_engine_configure_commands(&(pLedController
														  ->led_driver),
													lp5562_engine_1,
													commandCount,
													true);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_set_engine((&pLedController->led_driver),
										 lp5562_led_Red,
										 lp5562_engine_1);
			if(bRet == false) {
				return false;
			}
			//-----------------------------------------------------------------
			// green LED
			pCommands =
				lp5562_engine_get_commands(&(pLedController->led_driver),
										   lp5562_engine_2);
			commandCount =
				lp5562_pwm_to_command(pCommands, pwm, pwm->color.leds.green, 0);
			pCommands[commandCount] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(lp5562_engine_1 | lp5562_engine_2 |
										   lp5562_engine_3,
									   0));
			pCommands[commandCount + 1] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(0, lp5562_engine_1 | lp5562_engine_3));
			commandCount += 2;
			commandCount = lp5562_pwm_end_command(pCommands, commandCount);

			bRet = lp5562_engine_configure_commands(&(pLedController
														  ->led_driver),
													lp5562_engine_2,
													commandCount,
													true);
			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_set_engine((&pLedController->led_driver),
										 lp5562_led_Green,
										 lp5562_engine_2);
			if(bRet == false) {
				return false;
			}
			//-----------------------------------------------------------------
			// blue LED
			pCommands =
				lp5562_engine_get_commands(&(pLedController->led_driver),
										   lp5562_engine_3);
			commandCount =
				lp5562_pwm_to_command(pCommands, pwm, pwm->color.leds.blue, 0);
			pCommands[commandCount] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(lp5562_engine_1 | lp5562_engine_2 |
										   lp5562_engine_3,
									   0));
			pCommands[commandCount + 1] = lp5562_command_reorder_bytes(
				lp5562_trigger_command(0, lp5562_engine_1 | lp5562_engine_2));
			commandCount += 2;
			commandCount = lp5562_pwm_end_command(pCommands, commandCount);

			bRet = lp5562_engine_configure_commands(&(pLedController
														  ->led_driver),
													lp5562_engine_3,
													commandCount,
													true);

			if(bRet == false) {
				return false;
			}
			bRet = lp5562_led_set_engine((&pLedController->led_driver),
										 lp5562_led_Blue,
										 lp5562_engine_3);
			if(bRet == false) {
				return false;
			}
			break;
		default:
			return false;
			break;
		}
		return true;
	}
}
