/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bq27427_registers.h
 * @brief Register, command, and data-flash declarations for the BQ27427 fuel
 *        gauge.
 *
 * @defgroup sensewear_bq27427 SenseWear BQ27427 fuel gauge
 * @ingroup io_interfaces
 * @{
 *
 * This header contains the standard-command map, Control() subcommands,
 * data-flash class identifiers, register bit-field overlays, and SenseWear
 * battery defaults for the BQ27427 register map. It is intended for low-level
 * register access helpers used by the fuel-gauge driver.
 */
#ifndef BQ27427_REGISTERS_H
#define BQ27427_REGISTERS_H

#include <stdint.h>

/**
 * @brief BQ27427 battery chemistry identifiers.
 * @details Each enumerator is the chemistry ID reported by the
 *          ChemistryIdentifier subcommand and selected through one of the
 *          Chemistry_A/B/C Control() subcommands.
 */
enum bq27427_chemistry_type {
	bq27427_chemistry_Lipo4V35 = 0x3230, /**< Chemistry A: LiPo, 4.35 V max. */
	bq27427_chemistry_LiPo4V20 = 0x1202, /**< Chemistry B: LiPo, 4.20 V max. */
	bq27427_chemistry_LiPo4V4 = 0x3142	 /**< Chemistry C: LiPo, 4.40 V max. */
};

/**
 * @brief BQ27427 standard-command register map.
 * @details Each enumerator is the command-register address placed at the start
 *          of an I2C transaction. Capacities are reported in mAh, voltage in
 *          mV, current in mA, power in mW, temperature in 0.1 K, and state of
 *          charge in percent.
 */
enum bq27427_command_type {
	bq27427_command_Control = 0x00,						 /**< Control() command/status access. */
	bq27427_command_Temperature = 0x02,					 /**< Reported battery temperature. */
	bq27427_command_Voltage = 0x04,						 /**< Measured cell voltage. */
	bq27427_command_Flags = 0x06,						 /**< Status and alert flags. */
	bq27427_command_NominalAvailableCapacity = 0x08,	 /**< Uncompensated remaining capacity. */
	bq27427_command_FullAvailableCapacity = 0x0A,		 /**< Uncompensated full capacity. */
	bq27427_command_RemainingCapacity = 0x0C,			 /**< Compensated remaining capacity. */
	bq27427_command_FullChargeCapacity = 0x0E,			 /**< Compensated full-charge capacity. */
	bq27427_command_AverageCurrent = 0x10,				 /**< Average current (signed). */
	bq27427_command_AveragePower = 0x18,				 /**< Average power (signed). */
	bq27427_command_StateOfCharge = 0x1C,				 /**< Compensated state of charge. */
	bq27427_command_InternalTemperature = 0x1E,			 /**< Internal die temperature. */
	bq27427_command_RemainingCapacityUnfiltered = 0x28,	 /**< Remaining capacity, unfiltered. */
	bq27427_command_RemainingCapacityFiltered = 0x2A,	 /**< Remaining capacity, filtered. */
	bq27427_command_FullChargeCapacityUnfiltered = 0x2C, /**< Full-charge capacity, unfiltered. */
	bq27427_command_FullChargeCapacityFiltered = 0x2E,	 /**< Full-charge capacity, filtered. */
	bq27427_command_StateOfChargeUnfiltered = 0x30,		 /**< State of charge, unfiltered. */
};

/**
 * @brief BQ27427 extended-command (data-flash) register map.
 * @details These commands provide block access to data flash. A class is
 *          selected with DataClass, a 32-byte block within the class with
 *          DataBlock, and the block payload is read or written through the
 *          BlockData window.
 */
enum bq27427_extended_command_t {
	bq27427_extended_command_DataClass = 0x3E, /**< Selects the data-flash subclass. */
	bq27427_extended_command_DataBlock = 0x3F, /**< Selects the 32-byte block index. */
	bq27427_extended_command_BlockDataStart =
		0x40, /**< First byte of the 32-byte block window (0x40 + offset). */
	bq27427_extended_command_BlockDataEnd = 0x5F,	   /**< Last byte of the block window. */
	bq27427_extended_command_BlockDataChecksum = 0x60, /**< Block checksum for write commit. */
	bq27427_extended_command_BlockDataControl = 0x61   /**< Enables raw data-flash block access. */
};

/**
 * @brief BQ27427 Control() subcommands.
 * @details Each value is written as the 16-bit argument to the Control()
 *          command (0x00) to read device status or trigger a device action.
 */
enum bq27427_control_subcommand_type {
	bq27427_control_subcommand_ControlStatus = 0x0000,	/**< Read CONTROL_STATUS register. */
	bq27427_control_subcommand_DeviceType = 0x0001,		/**< Read device type (0x0427). */
	bq27427_control_subcommand_FWVersion = 0x0002,		/**< Read firmware version. */
	bq27427_control_subcommand_DataMemoryCode = 0x0004, /**< Read data-memory version code. */
	bq27427_control_subcommand_PreviousMACCommandCode = 0x0007, /**< Read prior MAC command code. */
	bq27427_control_subcommand_ChemistryIdentifier = 0x0008,	/**< Read active chemistry ID. */
	bq27427_control_subcommand_BatteryInsert = 0x000C, /**< Force battery-insert detection. */
	bq27427_control_subcommand_BatteryRemove = 0x000D, /**< Force battery-remove detection. */
	bq27427_control_subcommand_SetConfigurationUpdate =
		0x0013,											   /**< Enter configuration-update mode. */
	bq27427_control_subcommand_SmoothSynchronize = 0x0019, /**< Synchronize the smoothing engine. */
	bq27427_control_subcommand_ShutdownEnable = 0x001B,	   /**< Arm the shutdown sequence. */
	bq27427_control_subcommand_Shutdown = 0x001C,		   /**< Enter SHUTDOWN low-power state. */
	bq27427_control_subcommand_Sealed = 0x0020,			   /**< Re-seal the device. */
	bq27427_control_subcommand_PulseGPIOPin = 0x023,	   /**< Pulse the GPOUT pin. */
	bq27427_control_subcommand_Chemistry_A = 0x0030,	   /**< Select chemistry A (4.35 V). */
	bq27427_control_subcommand_Chemistry_B = 0x0031,	   /**< Select chemistry B (4.20 V). */
	bq27427_control_subcommand_Chemistry_C = 0x0032,	   /**< Select chemistry C (4.40 V). */
	bq27427_control_subcommand_Reset = 0x0041,			   /**< Full device reset. */
	bq27427_control_subcommand_SoftReset = 0x0042		   /**< Exit configuration-update mode. */
};

/**
 * @brief BQ27427 data-flash subclass identifiers.
 * @details Each value is written to the DataClass extended command to select a
 *          data-flash subclass for block read/write access.
 */
enum bq27427_flash_class_type {
	bq27427_flash_class_ConfigurationDischarge = 49, /**< Discharge configuration. */
	bq27427_flash_class_ConfigurationRegisters = 64, /**< Operation-config registers. */
	bq27427_flash_class_CurrentThreshold = 81,		 /**< Charge/discharge current thresholds. */
	bq27427_flash_class_GasGaugeState = 82,			 /**< Gas-gauge state (capacity, energy). */
	bq27427_flash_class_RaTables = 89,				 /**< Cell impedance (Ra) tables. */
	bq27427_flash_class_Calibration = 105,			 /**< Coulomb-counter calibration. */
	bq27427_flash_class_ChemistryData = 109,		 /**< Chemistry data block. */
};

/**
 * @brief BQ27427 CONTROL_STATUS register representation.
 * @details Returned by the ControlStatus subcommand. The bit fields mirror the
 *          device register; @ref value provides access to the complete 16-bit
 *          value read over I2C.
 */
union bq27427_control_status_register_t {
	uint16_t value; /**< Complete raw register value. */
	/**
	 * @brief CONTROL_STATUS bit-field mapping.
	 * @details Maps fields from bit 0 through bit 15.
	 */
	struct bq27427_control_status_register_bits {
		unsigned int bChemistryChanged : 1;				   /**< Chemistry ID has changed. */
		unsigned int bCellVoltageOK : 1;				   /**< Cell voltage valid for OCV. */
		unsigned int bRaTableUpdateDisable : 1;			   /**< Ra-table updates are disabled. */
		unsigned int bConstantPowerModel : 1;			   /**< Constant-power model selected. */
		unsigned int bSleepMode : 1;					   /**< Device is in SLEEP. */
		unsigned int : 2;								   /**< Reserved. */
		unsigned int bInitComplete : 1;					   /**< Initialization complete. */
		unsigned int bResistanceUpdated : 1;			   /**< Resistance has been updated. */
		unsigned int bQMaxUpdated : 1;					   /**< Qmax has been updated. */
		unsigned int bCalibrationActive : 1;			   /**< Calibration is in progress. */
		unsigned int bCoulombCounterCalibrationActive : 1; /**< CC calibration in progress. */
		unsigned int bCalibrationMode : 1;				   /**< Calibration mode entered. */
		unsigned int bSealedMode : 1;					   /**< Device is sealed. */
		unsigned int bWatchdogReset : 1;				   /**< Watchdog reset occurred. */
		unsigned int bShutdownEnableReceived : 1;		   /**< ShutdownEnable was received. */
	} bits;
};

/**
 * @brief BQ27427 Flags() register representation.
 * @details Returned by the Flags command (0x06). The bit fields mirror the
 *          device register; @ref value provides access to the complete 16-bit
 *          value read over I2C.
 */
union bq27427_flags_register_t {
	uint16_t value; /**< Complete raw register value. */
	/**
	 * @brief Flags() bit-field mapping.
	 * @details Maps fields from bit 0 through bit 15.
	 */
	struct bq27427_flags_bits {
		unsigned int bDischargeDetected : 1;   /**< Discharge has been detected. */
		unsigned int bStateOfChargeFull : 1;   /**< Full-charge state reached. */
		unsigned int bStateOfCharge1 : 1;	   /**< SOC at or below SOC1 threshold. */
		unsigned int bBatteryDetected : 1;	   /**< Battery is present. */
		unsigned int bConfigUpdateMode : 1;	   /**< Configuration-update mode active. */
		unsigned int bPOROrReset : 1;		   /**< Power-on or reset occurred. */
		unsigned int bDOCCorrection : 1;	   /**< Discharge-overcurrent correction. */
		unsigned int bOCVTaken : 1;			   /**< Open-circuit voltage measurement taken. */
		unsigned int bFastChargingAllowed : 1; /**< Fast charging is permitted. */
		unsigned int bFullCharge : 1;		   /**< Full-charge condition detected. */
		unsigned int : 4;					   /**< Reserved. */
		unsigned int bUnderTemperature : 1;	   /**< Under-temperature condition. */
		unsigned int bOverTemperature : 1;	   /**< Over-temperature condition. */
	} bits;
};

/**
 * @brief BQ27427 OpConfig register representation.
 * @details Maps the Operation-Config register stored in the configuration-
 *          registers subclass. @ref value provides access to the complete
 *          16-bit data-flash value.
 */
union bq27427_op_config_register_t {
	uint16_t value; /**< Complete raw register value. */
	/**
	 * @brief OpConfig bit-field mapping.
	 * @details Maps fields from bit 0 through bit 15.
	 */
	struct bq27427_op_config_register_bits {
		unsigned int bTempSource : 2;			  /**< Temperature measurement source select. */
		unsigned int bBatteryLowEnable : 1;		  /**< Enable battery-low GPOUT signaling. */
		unsigned int bEnableFastSoC : 1;		  /**< Enable fast state-of-charge convergence. */
		unsigned int bRMUpdateFullCharge : 1;	  /**< Update RemainingCapacity on full charge. */
		unsigned int bSleepEnable : 1;			  /**< Enable automatic SLEEP entry. */
		unsigned int bEnableRaStep : 1;			  /**< Enable Ra-table stepping. */
		unsigned int : 4;						  /**< Reserved. */
		unsigned int bGPIOPolarity : 1;			  /**< GPOUT pin active polarity. */
		unsigned int : 1;						  /**< Reserved. */
		unsigned int bBatteryInsertionEnable : 1; /**< Enable battery-insertion detection. */
	} bits;
};

/** @brief Block-data offset of the battery-capacity class within its block. */
#define BQ27427_BATTERY_CAPACITY_CLASS_OFFSET (0)

/** @brief Expected DeviceType value returned by the DeviceType subcommand. */
#define BQ27427_DEVICE_TYPE (0x0427)
/** @brief Expected firmware version returned by the FWVersion subcommand. */
#define BQ27427_FW_VERSION (0x0202)
/** @brief Key written twice to Control() to unseal the device. */
#define BQ27427_UNSEAL_KEY (0x8000)
/** @brief Block-data offset of LoadMode in the gas-gauge state subclass. */
#define BQ27427_LOAD_MODE_MEMORY_OFFSET (0x05u)
/** @brief Block-data offset of DesignCapacity in the gas-gauge state subclass. */
#define BQ27427_DESIGN_CAPACITY_MEMORY_OFFSET (0x06u)
/** @brief Block-data offset of DesignEnergy in the gas-gauge state subclass. */
#define BQ27427_DESIGN_ENERGY_MEMORY_OFFSET (0x08u)
/** @brief Block-data offset of TerminateVoltage in the gas-gauge state subclass. */
#define BQ27427_TERMINATION_VOLTAGE_MEMORY_OFFSET (0x0Au)
/** @brief Block-data offset of TaperRate in the gas-gauge state subclass. */
#define BQ27427_TAPER_RATE_MEMORY_OFFSET (0x15u)
/** @brief Block-data offset of SleepCurrent in the gas-gauge state subclass. */
#define BQ27427_SLEEP_CURRENT_MEMORY_OFFSET (0x17u)
/** @brief Block-data offset of OpConfig in the configuration-registers subclass. */
#define BQ27427_OP_CONFIG_MEMORY_OFFSET (0x00u)
/** @brief Block-data offset of CC Gain in the calibration subclass. */
#define BQ27427_CC_GAIN_MEMORY_OFFSET (0x04u)
/** @brief Block-data offset of Dsg Current Threshold in the current-threshold subclass. */
#define BQ27427_DISCHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET (0x00u)
/** @brief Block-data offset of Chg Current Threshold in the current-threshold subclass. */
#define BQ27427_CHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET (0x00u)
/** @brief Block-data offset of Quit Current in the current-threshold subclass. */
#define BQ27427_QUIT_CURRENT_THRESHOLD_MEMORY_OFFSET (0x04u)
/** @brief Block-data offset of Cell BL Set Volt Threshold in the current-threshold subclass. */
#define BQ27427_V_AT_CHARGE_TERM_MEMORY_OFFSET (0x06u)
/** @brief Block-data offset of TaperVoltage in the current-threshold subclass. */
#define BQ27427_TAPER_VOLTAGE_MEMORY_OFFSET (0x08u)

/** @brief SenseWear default battery chemistry (LiPo, 4.20 V max). */
#define BQ27427_DEFAULT_BATTERY_TYPE (bq27427_chemistry_LiPo4V20)
/** @brief SenseWear default DesignCapacity in mAh. */
#define BQ27427_DEFAULT_BATTERY_CAPACITY (23u)
/** @brief SenseWear default DesignEnergy in mWh. */
#define BQ27427_DEFAULT_BATTERY_ENERGY (851u)
/** @brief SenseWear default TerminateVoltage in mV. */
#define BQ27427_DEFAULT_BATTERY_TERMINATION_VOLTAGE (3400u)
/** @brief SenseWear default TaperRate (charge-current divisor). */
#define BQ27427_DEFAULT_TAPER_RATE (110u)
/** @brief SenseWear default SleepCurrent threshold in mA. */
#define BQ27427_DEFAULT_GAUGE_SLEEP_CURRENT (1u)

/** @brief SenseWear default discharge current threshold in mA. */
#define BQ27427_DEFAULT_DISCHARGE_CURRENT_THRESHOLD (990u)
/** @brief SenseWear default charge current threshold in mA. */
#define BQ27427_DEFAULT_CHARGE_CURRENT_THRESHOLD (200u)
/** @brief SenseWear default quit current threshold in mA. */
#define BQ27427_DEFAULT_QUIT_CURRENT_THRESHOLD (1000u)
/** @brief SenseWear default cell voltage at charge termination in mV. */
#define BQ27427_DEFAULT_V_AT_CHARGE_TERM (4190u)
/** @brief SenseWear default taper voltage in mV. */
#define BQ27427_DEFAULT_TAPER_VOLTAGE (4170u)

/** @brief SenseWear default Ra-table seed values. */
#define BQ27427_DEFAULT_RA_VALUES_ \
	{97, 107, 121, 136, 103, 102, 101, 100, 101, 104, 108, 122, 161, 501, 122}
/** @brief Number of entries in the Ra table. */
#define BQ27427_RAM_TABLE_SIZE_ (15)

/** @} */

#endif // BQ27427_REGISTERS_H
