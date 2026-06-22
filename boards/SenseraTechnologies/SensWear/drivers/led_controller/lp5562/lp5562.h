/*
 * lp5562.h
 *
 *  Created on: Jul 18, 2017
 *      Author: Huseyin Yigitler
 */

#ifndef _LP5562_H_
#define _LP5562_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#ifndef LP5562_I2C_TIMEOUT
#define LP5562_I2C_TIMEOUT 100
#endif

#define LP5562_LED_COUNT 4
#define LP5562_ENGINE_COUNT 3
#define LP5562_ENGINE_COMMAND_COUNT 16

/* ENABLE Register 00h */
#define LP5562_REG_ENABLE 0x00
#define LP5562_EXEC_ENG1_M 0x30
#define LP5562_EXEC_ENG2_M 0x0C
#define LP5562_EXEC_ENG3_M 0x03
#define LP5562_EXEC_M 0x3F
#define LP5562_MASTER_ENABLE 0x40	/* Chip master enable */
#define LP5562_LOGARITHMIC_PWM 0x80 /* Logarithmic PWM adjustment */
#define LP5562_EXEC_RUN 0x2A
#define LP5562_ENABLE_DEFAULT (LP5562_MASTER_ENABLE | LP5562_LOGARITHMIC_PWM)
#define LP5562_ENABLE_RUN_PROGRAM (LP5562_ENABLE_DEFAULT | LP5562_EXEC_RUN)

/* OPMODE Register 01h */
#define LP5562_REG_OP_MODE 0x01
#define LP5562_MODE_ENG1_M 0x30
#define LP5562_MODE_ENG2_M 0x0C
#define LP5562_MODE_ENG3_M 0x03
#define LP5562_LOAD_ENG1 0x10
#define LP5562_LOAD_ENG2 0x04
#define LP5562_LOAD_ENG3 0x01
#define LP5562_RUN_ENG1 0x20
#define LP5562_RUN_ENG2 0x08
#define LP5562_RUN_ENG3 0x02
#define LP5562_ENG1_IS_LOADING(mode) \
	((mode & LP5562_MODE_ENG1_M) == LP5562_LOAD_ENG1)
#define LP5562_ENG2_IS_LOADING(mode) \
	((mode & LP5562_MODE_ENG2_M) == LP5562_LOAD_ENG2)
#define LP5562_ENG3_IS_LOADING(mode) \
	((mode & LP5562_MODE_ENG3_M) == LP5562_LOAD_ENG3)

/* BRIGHTNESS Registers */
#define LP5562_REG_R_PWM 0x04
#define LP5562_REG_G_PWM 0x03
#define LP5562_REG_B_PWM 0x02
#define LP5562_REG_W_PWM 0x0E

/* CURRENT Registers */
#define LP5562_REG_R_CURRENT 0x07
#define LP5562_REG_G_CURRENT 0x06
#define LP5562_REG_B_CURRENT 0x05
#define LP5562_REG_W_CURRENT 0x0F

/* CONFIG Register 08h */
#define LP5562_REG_CONFIG 0x08
#define LP5562_PWM_HF 0x40
#define LP5562_PWRSAVE_EN 0x20
#define LP5562_CLK_INT 0x01 /* Internal clock */
#define LP5562_DEFAULT_CFG (LP5562_PWM_HF | LP5562_PWRSAVE_EN)

/* RESET Register 0Dh */
#define LP5562_REG_RESET 0x0D
#define LP5562_RESET 0xFF

/* PROGRAM ENGINE Registers */
#define LP5562_REG_PROG_MEM_ENG1 0x10
#define LP5562_REG_PROG_MEM_ENG2 0x30
#define LP5562_REG_PROG_MEM_ENG3 0x50

/* LEDMAP Register 70h */
#define LP5562_REG_ENG_SEL 0x70
#define LP5562_ENG_SEL_PWM 0
#define LP5562_ENG_FOR_RGB_M 0x3F
#define LP5562_ENG_SEL_RGB 0x1B /* R:ENG1, G:ENG2, B:ENG3 */
#define LP5562_ENG_FOR_W_M 0xC0
#define LP5562_ENG1_FOR_W 0x40 /* W:ENG1 */
#define LP5562_ENG2_FOR_W 0x80 /* W:ENG2 */
#define LP5562_ENG3_FOR_W 0xC0 /* W:ENG3 */

/* Program Commands */
#define LP5562_CMD_DISABLE 0x00
#define LP5562_CMD_LOAD 0x15
#define LP5562_CMD_RUN 0x2A
#define LP5562_CMD_DIRECT 0x3F
#define LP5562_PATTERN_OFF 0

#define LP5562_CONFIG_DEFAULT 0x00
#define LP5562_CURRENT_DEFAULT 0xA7
#define LP5562_PWM_DEFAULT 0x00
#define LP5562_LED_MAP_DEFAULT 0x39

enum lp5562_led_type {
	lp5562_led_Red = 0,
	lp5562_led_Green = 1,
	lp5562_led_Blue = 2,
	lp5562_led_White = 3,
};

enum lp5562_engine_type {
	lp5562_engine_1 = 1,
	lp5562_engine_2 = 2,
	lp5562_engine_3 = 4,
	lp5562_engine_i2c_pwm = 8,
};

struct lp5562_led_t {
	enum lp5562_led_type type;		//!< The LED identifier.
	enum lp5562_engine_type engine; //!< the execution engine of the LED.
	uint8_t current; //!< The current flow through the LED in 0.1mA per step.
	uint8_t
		i2c_dim; //!< The pwm duty cycle control from i2c registers in %. 0 is 0% and 255 is 100%.
};

enum lp5562_engine_state_type {
	lp5562_engine_state_Hold = 0,
	lp5562_engine_state_Step = 1,
	lp5562_engine_state_Run = 2,
	lp5562_engine_state_Execute = 3,
};

enum lp5562_engine_mode_type {
	lp5562_engine_mode_Disabled = 0,
	lp5562_engine_mode_Load = 1,
	lp5562_engine_mode_Run = 2,
	lp5562_engine_mode_I2CControl = 3,
};

struct lp5562_engine_t {
	enum lp5562_engine_type id;
	enum lp5562_engine_state_type state;
	enum lp5562_engine_mode_type mode;
	uint16_t commands[LP5562_ENGINE_COMMAND_COUNT];
	uint8_t command_count;
};

//-----------------------------------------------------------------
// The command are big endian
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

union lp5562_set_pwm_command_t {
	uint16_t command;
	struct lp5562_set_pwm_command_fields {
		uint16_t pwm : 8;
		uint16_t const_pwm_set : 8; //!< write 0x40
	} fields;
};

union lp5562_branch_command_t {
	uint16_t command;
	struct lp5562_branch_command_fields {
		uint16_t
			step_number : 4; //!< The step number in the command execution command
		uint16_t not_care : 3;	 //!< DoNot care bits
		uint16_t loop_count : 6; //!< Number of loops to be done.
		uint16_t
			const_branch : 3; //!< The constant bits of the command: Write 5
	} fields;
};

union lp5562_end_command_t {
	uint16_t command;
	struct lp5562_end_command_fields {
		uint16_t not_care : 11;
		uint16_t
			reset : 1; //!< Write 1 to reset PWM to 0 value after executing the command.
		uint16_t irq : 1;		//!< INT bit.
		uint16_t const_end : 3; //!< The constant bits of the command: Write 6
	} fields;
};

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
		uint16_t
			const_trigger : 3; //!< The constant bits of the command: Write 7
	} fields;
};
/**
 * \brief Creates a RAMP command.
 * @param numberOfSteps The number of steps in the ramp/
 * @param bIncrement TRUE if incrementing, FALSE if decrementing.
 * @param stepTime The time of a step. It depends on the prescale value.
 * @param bPrescale TRUE if the clock is prescaled, FALSE if not.
 * @return The created command.
 */
uint16_t lp5562_ramp_command(uint8_t numberOfSteps,
							 bool bIncrement,
							 uint8_t stepTime,
							 bool bPrescale);
/**
 * \brief Create a WAIT command
 * @param stepTime The time of a step. It depends on the prescale value.
 * @param bPrescale TRUE if the clock is prescaled, FALSE if not.
 * @return The created command.
 */
#define lp5562_wait_command(stepTime, bPrescale) \
	lp5562_ramp_command(0, false, stepTime, bPrescale)
/**
 * \brief Creates a SET PWM command.
 * @param pwm The PWM duty cycle value in %. 0 corresponds to 0% and 255 corresponds to 100%.
 * @return The created command.
 */
uint16_t lp5562_set_pwm_command(uint8_t pwm);
/**
 * \brief Go to start command
 */
#define lp5562_goto_start_command() 0x0000
/**
 * \brief Create a BRANCH command
 * @param stepNumber The step number to be loaded to program counter
 * @param loopCount The number loops to be done. 0 means infinite loop.
 * @return The created command.
 */
uint16_t lp5562_branch_command(uint8_t stepNumber, uint8_t loopCount);
/**
 * \brief Creates an END command
 * @param bInterrupt TRUE if command creates and interrupt, FALSE if not.
 * @param bResetPWM TRUE to reset the PWM value to 0, FALSE to keep the current value/
 * @return The created command
 */
uint16_t lp5562_end_command(bool bInterrupt, bool bResetPWM);
/**
 * \brief Creates a TRIGGER command. OR more than one engine identifiers.
 * @param sendTriggers a logical of engine identifiers for sending triggers.
 * @param waitTriggers a logical of engine identifiers for waiting triggers.
 * @return The created command.
 */
uint16_t lp5562_trigger_command(
	uint8_t sendTriggers,
	uint8_t waitTriggers); // OR more than one engine identifiers.

/**
 * \brief Converts from LS Byte first ordering to MS byte first ordering.
 * @param __command The command as a WORD.
 * @return The re-order command as a WORD.
 */
static inline uint16_t lp5562_command_reorder_bytes(uint16_t __command) {
	uint16_t retVal = (__command >> 8) & 0x00FF;
	retVal += (__command << 8) & 0xFF00;
	return retVal;
}

/**
 * \brief LP5562 Clock configuration options.
 */
enum lp5562_clock_type {
	lp5562_clk_External = 0,   //!< lp5562_clk_External
	lp5562_clk_Internal = 1,   //!< lp5562_clk_Internal
	lp5562_clk_Automatic = 2,  //!< lp5562_clk_Automatic
	lp5562_clk_Internal_2 = 3, //!< lp5562_clk_Internal_2
};

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

union lp5562_op_mode_register_t {
	uint8_t value;
	struct lp5562_op_mode_register_bits {
		uint8_t eng3_mode : 2;
		uint8_t eng2_mode : 2;
		uint8_t eng1_mode : 2;
		uint8_t reserved : 2;
	} bits;
};

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
 * \brief LP5562 configuration
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
		uint8_t
			bInitialized : 1; //!< Indicates that LP5562 object has been initialized.
	} bits;
};

struct lp5562_t {
	const struct i2c_dt_spec* device;	  //!< The I2C driver of the LED controller.
	const struct gpio_dt_spec* enable_gpio;
	union lp5562_config_t config; //!< The configuration.
	struct lp5562_engine_t engines
		[LP5562_ENGINE_COUNT]; //!< The number of command execution engines.
	struct lp5562_led_t leds[LP5562_LED_COUNT]; //!< The number of LEDs
	union lp5562_led_map_register_t
		led_engines;				  //!< The operation engines of the LEDs.
};

/**
 * \brief The I2C write function that used by the LP5562 driver to modify its configuration.
 */
extern bool lp5562_i2c_write_memory(const struct lp5562_t* pDriver,
									uint8_t regAddress,
									uint8_t* pData,
									size_t dataLength);
/**
 * \brief Writes the specified value to the specified register of LP5562 
 * using I2C
 * 
 * \param pDriver The driver
 * \param regAddress The register address
 * \param value The value to be written
 * \return True if successfully written, Otherwise False.
 */
extern bool lp5562_i2c_write_register(const struct lp5562_t* pDriver,
									  uint8_t regAddress,
									  uint8_t value);
/**
 * \brief initializes the LP5562 object with its default values.
 * @param pDriver The driver object.
 */
void lp5562_initialize(struct lp5562_t* pDrivers);
/**
 * \brief Enables LOGarithmic increase of PWMs
 * @param pDriver The LP5562 object
 * @param enable TRUE to enable, FALSE to disable the LOGarithmic PWM control
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_log_pwm_enable(struct lp5562_t* pDriver, bool enable);
/**
 * \brief Configures the clock settings of the specified controller.
 * @param pDriver The LP5562 object.
 * @param clock The clock settings
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_configure_clock(struct lp5562_t* pDriver,
							enum lp5562_clock_type clock);
/**
 * \brief Enables or disables power save mode of LP5562.
 * @param pDriver The LP5562 object
 * @param enable TRUE to enable, FALSE to disable the power save mode.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_power_save(struct lp5562_t* pDriver, bool enable);
/**
 * \brief Enables or disables high frequency PWM mode of LP5562.
 * @param pDriver The LP5562 object
 * @param enable TRUE to enable, FALSE to disable the high frequency PWM mode.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_pwm_hf(struct lp5562_t* pDriver, bool enable);
/**
 * \brief Enables/disables the controller by enabling Chip Enable bit.
 * @param pDriver The LP5562 object.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
*/
bool lp5562_chip_enable(struct lp5562_t* pDriver);
void lp5562_chip_disable(struct lp5562_t* pDriver);

bool lp5562_i2c_enable(struct lp5562_t* pDriver);
void lp5562_i2c_disable(struct lp5562_t* pDriver);

/**
 * \brief Configures a LED of the specified LP5562.
 * @param pDriver The LP5562 object.
 * @param led The LED to configured.
 * @param current Max current limit of the driver.
 * @param i2c_pwm I2C pwm value.
 * @param engine The current command execution engine of the LED.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_led_configure(struct lp5562_t* pDriver,
						  enum lp5562_led_type led,
						  uint8_t current,
						  uint8_t i2c_pwm,
						  enum lp5562_engine_type engine);
/**
 * \brief Returns the start address of the local cache of the execution engine commands.
 * @param pDriver The LP5562 object.
 * @param engine The engine.
 * @return NULL if the specified engine does not contain a command cache, otherwise the start address.
 */
uint16_t* lp5562_engine_get_commands(struct lp5562_t* pDriver,
									 enum lp5562_engine_type engine);
/**
 * \brief Configures the SRAM memory of an execution engine of the specified LP5562 controller.
 * @param pDriver The LP5562 object.
 * @param engine The engine
 * @param commandCount The number of commands in the local cache.
 * @param run TRUE to start execution of the engine.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_engine_configure_commands(struct lp5562_t* pDriver,
									  enum lp5562_engine_type engine,
									  uint8_t commandCount,
									  bool run);
/**
 * \brief Changes the mode of the specified execution engine.
 * @param pDriver The LP5562 object
 * @param engine The engine
 * @param mode The new mode.
 */
bool lp5562_engine_set_mode(struct lp5562_t* pDriver,
							enum lp5562_engine_type engine,
							enum lp5562_engine_mode_type mode);
/**
 * \brief Sets the engine state of the specified execution engine.
 * @param pDriver The LP5562 object.
 * @param engine The execution engine.
 * @param state The new state
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_engine_set_state(struct lp5562_t* pDriver,
							 enum lp5562_engine_type engine,
							 enum lp5562_engine_state_type state);
bool lp5562_engine_all_set_state(struct lp5562_t* pDriver,
								 enum lp5562_engine_state_type state);
/**
 * \brief Sets the MAX current value of a LED.
 */
#define lp5562_led_set_current(pDriver, led, current)          \
	lp5562_led_configure(pDriver,                              \
						 led,                                  \
						 current,                              \
						 pDriver->leds[(uint8_t) led].i2c_dim, \
						 pDriver->leds[(uint8_t) led].engine)
/**
 * \brief Sets the I2C PWM value of the specified LED.
 */
#define lp5562_led_set_i2c_dim(pDriver, led, dim)              \
	lp5562_led_configure(pDriver,                              \
						 led,                                  \
						 pDriver->leds[(uint8_t) led].current, \
						 dim,                                  \
						 pDriver->leds[(uint8_t) led].engine)
/**
 * \brief Sets the execution engine of the specified LED.
 */
#define lp5562_led_set_engine(pDriver, led, engine)            \
	lp5562_led_configure(pDriver,                              \
						 led,                                  \
						 pDriver->leds[(uint8_t) led].current, \
						 pDriver->leds[(uint8_t) led].i2c_dim, \
						 engine)
/**
 * \brief Sends RESET command to reset the device registers to their default values.
 * @param pDriver The LP5562 object.
 * @return TRUE if the I2C communication ends successfully, FALSE otherwise.
 */
bool lp5562_reset(struct lp5562_t* pDriver);
#endif // _LP5562_H_
