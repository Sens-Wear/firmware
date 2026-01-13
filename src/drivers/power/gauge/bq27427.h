#ifndef BQ27427_H_
#define BQ27427_H_

#include <stdint.h>
#include <stdbool.h>


#define BQ27427_I2C_ADDRESS (0xAA)
#ifndef BQ27427_I2C_TIMEOUT
#define BQ27427_I2C_TIMEOUT (100)
#endif

enum bq27427_chemistry_type {
	bq27427_chemistry_Lipo4V35 = 0x3230, // Chemistry A
	bq27427_chemistry_LiPo4V20 = 0x1202, // Chemistry B
	bq27427_chemistry_LiPo4V4 = 0x3142	 // Chemistry C
};

enum bq27427_command_type {
	bq27427_command_Control = 0x00,
	bq27427_command_Temperature = 0x02,
	bq27427_command_Voltage = 0x04,
	bq27427_command_Flags = 0x06,
	bq27427_command_NominalAvailableCapacity = 0x08,
	bq27427_command_FullAvailableCapacity = 0x0A,
	bq27427_command_RemainingCapacity = 0x0C,
	bq27427_command_FullChargeCapacity = 0x0E,
	bq27427_command_AverageCurrent = 0x10,
	bq27427_command_AveragePower = 0x18,
	bq27427_command_StateOfCharge = 0x1C,
	bq27427_command_InternalTemperature = 0x1E,
	bq27427_command_RemainingCapacityUnfiltered = 0x28,
	bq27427_command_RemainingCapacityFiltered = 0x2A,
	bq27427_command_FullChargeCapacityUnfiltered = 0x2C,
	bq27427_command_FullChargeCapacityFiltered = 0x2E,
	bq27427_command_StateOfChargeUnfiltered = 0x30,
};

enum bq27427_extended_command_t {
	bq27427_extended_command_DataClass = 0x3E,
	bq27427_extended_command_DataBlock = 0x3F,
	bq27427_extended_command_BlockDataStart =
		0x40, //-- First byte in 32-byte block is accessed with 0x40 -- 1 byte offset -- 0x41 etc.
	bq27427_extended_command_BlockDataEnd = 0x5F,
	bq27427_extended_command_BlockDataChecksum = 0x60,
	bq27427_extended_command_BlockDataControl = 0x61
};

enum bq27427_control_subcommand_type {
	bq27427_control_subcommand_ControlStatus = 0x0000,
	bq27427_control_subcommand_DeviceType = 0x0001,
	bq27427_control_subcommand_FWVersion = 0x0002,
	bq27427_control_subcommand_DataMemoryCode = 0x0004,
	bq27427_control_subcommand_PreviousMACCommandCode = 0x0007,
	bq27427_control_subcommand_ChemistryIdentifier = 0x0008,
	bq27427_control_subcommand_BatteryInsert = 0x000C,
	bq27427_control_subcommand_BatteryRemove = 0x000D,
	bq27427_control_subcommand_SetConfigurationUpdate = 0x0013,
	bq27427_control_subcommand_SmoothSynchronize = 0x0019,
	bq27427_control_subcommand_ShutdownEnable = 0x001B,
	bq27427_control_subcommand_Shutdown = 0x001C,
	bq27427_control_subcommand_Sealed = 0x0020,
	bq27427_control_subcommand_PulseGPIOPin = 0x023,
	bq27427_control_subcommand_Chemistry_A = 0x0030,
	bq27427_control_subcommand_Chemistry_B = 0x0031,
	bq27427_control_subcommand_Chemistry_C = 0x0032,
	bq27427_control_subcommand_Reset = 0x0041,
	bq27427_control_subcommand_SoftReset = 0x0042
};

enum bq27427_flash_class_type {
	bq27427_flash_class_ConfigurationDischarge = 49,
	bq27427_flash_class_ConfigurationRegisters = 64,
	bq27427_flash_class_CurrentThreshold = 81,
	bq27427_flash_class_GasGaugeState = 82,
	bq27427_flash_class_RaTables = 89,
	bq27427_flash_class_Calibration = 105,
	bq27427_flash_class_ChemistryData = 109,
};

union bq27427_control_status_register_t {
	uint16_t value;
	struct bq27427_control_status_register_bits {
		unsigned int bChemistryChanged : 1;
		unsigned int bCellVoltageOK : 1;
		unsigned int bRaTableUpdateDisable : 1;
		unsigned int bConstantPowerModel : 1;
		unsigned int bSleepMode : 1;
		unsigned int : 2;
		unsigned int bInitComplete : 1;
		unsigned int bResistanceUpdated : 1;
		unsigned int bQMaxUpdated : 1;
		unsigned int bCalibrationActive : 1;
		unsigned int bCoulombCounterCalibrationActive : 1;
		unsigned int bCalibrationMode : 1;
		unsigned int bSealedMode : 1;
		unsigned int bWatchdogReset : 1;
		unsigned int bShutdownEnableReceived : 1;
	} bits;
};

union bq27427_flags_register_t {
	uint16_t value;
	struct bq27427_flags_bits {
		unsigned int bDischargeDetected : 1;
		unsigned int bStateOfChargeFull : 1;
		unsigned int bStateOfCharge1 : 1;
		unsigned int bBatteryDetected : 1;
		unsigned int bConfigUpdateMode : 1;
		unsigned int bPOROrReset : 1;
		unsigned int bDOCCorrection : 1;
		unsigned int bOCVTaken : 1;
		unsigned int bFastChargingAllowed : 1;
		unsigned int bFullCharge : 1;
		unsigned int : 4;
		unsigned int bUnderTemperature : 1;
		unsigned int bOverTemperature : 1;
	} bits;
};

union bq27427_op_config_register_t {
	uint16_t value;
	struct bq27427_op_config_register_bits {
		unsigned int bTempSource : 2;
		unsigned int bBatteryLowEnable : 1;
		unsigned int bEnableFastSoC : 1;
		unsigned int bRMUpdateFullCharge : 1;
		unsigned int bSleepEnable : 1;
		unsigned int bEnableRaStep : 1;
		unsigned int : 4;
		unsigned int bGPIOPolarity : 1;
		unsigned int : 1;
		unsigned int bBatteryInsertionEnable : 1;
	} bits;
};

#define BQ27427_BATTERY_CAPACITY_CLASS_OFFSET (0)

#define BQ27427_DEVICE_TYPE (0x0427)
#define BQ27427_FW_VERSION (0x0202)
#define BQ27427_UNSEAL_KEY (0x8000)
#define BQ27427_LOAD_MODE_MEMORY_OFFSET (0x05u)
#define BQ27427_DESIGN_CAPACITY_MEMORY_OFFSET (0x06u)
#define BQ27427_DESIGN_ENERGY_MEMORY_OFFSET (0x08u)
#define BQ27427_TERMINATION_VOLTAGE_MEMORY_OFFSET (0x0Au)
#define BQ27427_TAPER_RATE_MEMORY_OFFSET (0x15u)
#define BQ27427_SLEEP_CURRENT_MEMORY_OFFSET (0x17u)
#define BQ27427_OP_CONFIG_MEMORY_OFFSET (0x00u)
#define BQ27427_CC_GAIN_MEMORY_OFFSET (0x04u)
#define BQ27427_DISCHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET (0x00u)
#define BQ27427_CHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET (0x00u)
#define BQ27427_QUIT_CURRENT_THRESHOLD_MEMORY_OFFSET (0x04u)
#define BQ27427_V_AT_CHARGE_TERM_MEMORY_OFFSET (0x06u)
#define BQ27427_TAPER_VOLTAGE_MEMORY_OFFSET (0x08u)

#define BQ27427_DEFAULT_BATTERY_TYPE (bq27427_chemistry_LiPo4V20)
#define BQ27427_DEFAULT_BATTERY_CAPACITY (450u)
#define BQ27427_DEFAULT_BATTERY_ENERGY (1665u)
#define BQ27427_DEFAULT_BATTERY_TERMINATION_VOLTAGE (3200u)
#define BQ27427_DEFAULT_TAPER_RATE (110u)
#define BQ27427_DEFAULT_GAUGE_SLEEP_CURRENT (1u)

#define BQ27427_DEFAULT_DISCHARGE_CURRENT_THRESHOLD (990u)
#define BQ27427_DEFAULT_CHARGE_CURRENT_THRESHOLD (200u)
#define BQ27427_DEFAULT_QUIT_CURRENT_THRESHOLD (1000u)
#define BQ27427_DEFAULT_V_AT_CHARGE_TERM (4190u)
#define BQ27427_DEFAULT_TAPER_VOLTAGE (4170u)

#define BQ27427_DEFAULT_RA_VALUES_ {97, 107, 121, 136, 103, 102, 101, 100, 101, 104, 108, 122, 161, 501, 122}
#define BQ27427_RAM_TABLE_SIZE_ (15)

struct bq27427_config_t {
	enum bq27427_chemistry_type battery_type;
	// --------------------------------------------
	// Gas Gauging class -- State subclass -- 82
	uint16_t battery_capacity; // offset 6
	uint16_t battery_energy; // offset 8
	uint16_t battery_termination_voltage; // offset 10
	uint16_t taper_rate; // offset 21
	uint16_t gauge_sleep_current; // offset 23
	// -----------------------------------------------------------------------------
	// Gas gauging class -- Current thresholds subclass -- 81
	uint16_t discharge_current_threshold; // offset 0
	uint16_t charge_current_threshold; // offset 2
	uint16_t quit_current_threshold; // offset 4
	// -----------------------------------------------------------------------------
	// Chemistry Info class -- Chem Data subclass -- 109
	uint16_t voltage_at_charge_termination; // offset 6
	uint16_t taper_voltage; // offset 8
	// -----------------------------------------------------------------------------
	// Ra Tables class -- Ra0 RAM -- 89
	// uint16_t ra_values[15];
};

union bq27427_state_t {
	unsigned int value;
	struct bq27427_state_bits {
		unsigned int bInitialized : 1;
		unsigned int bConfigured : 1;
		unsigned int bDischarging : 1;
	} bits;
};

struct bq27427_battery_state_t {
	int temperature;
	int voltage;
	int average_current;
	int average_power;
	int state_of_charge;
	int nominal_available_capacity;
	int full_battery_capacity;
	int remaining_capacity;
	bool learning_in_progress;
};

struct bq27427_t {
	I2C_HandleTypeDef* device;
	I2C_InitTypeDef i2c_config;

	struct bq27427_config_t config;
	union bq27427_state_t state;
	struct bq27427_battery_state_t battery_state;
};

/**
 * \brief Initializes BQ27427 driver
 *
 * \param i2c I2C device to be used by the driver
 */
bool bq27427_init(void);

/**
 * \brief Configures BQ27427 is with the specified configuration.
 * \details This function also enables some interrupts.
 *
 * \param config The configuration to be loaded.
 */
bool bq27427_config(struct bq27427_config_t* config);

/**
 * \brief Resets BQ27427 to default state and reapplies given configuration
 *
 * \details This function performs both soft and hard resets of the device by:
 * 1. Sending reset commands
 * 2. Waiting for reset to complete
 * 3. Reapplying the provided configuration
 *
 * \param config The configuration to apply after reset
 * \return true if reset and reconfiguration succeeded, false otherwise
 */
bool bq27427_reset(struct bq27427_config_t *config);
/**
 * \brief Returns the default configuration of BQ27427. This configuration
 * is composed of values loaded to the registers.
 *
 * \param config The destination configuration
 */
void bq27427_get_default_config(struct bq27427_config_t* config);

/**
 * \brief Updates the state of BQ27427 driver from the device
 *
 * \return The detected event as a result of handled IRQ
 */
bool bq27427_update_state(struct bq27427_battery_state_t* state);

/**
 * \brief Prints the latest battery state 
 */
void bq27427_print_state(void);
#endif //! BQ27427_H_
