/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bq25180_registers.h
 * @brief Register declarations for the BQ25180 register map.
 *
 * @defgroup senswear_bq25180 SensWear BQ25180 charger
 * @ingroup io_interfaces
 * @{
 *
 * This header contains the register declarations for the BQ25180 register map.
 * It is intended for low-level register access helpers used by the charger
 * driver.
 */
#ifndef BQ25180_REGISTERS_H
#define BQ25180_REGISTERS_H

#include <stdint.h>

/**
 * @brief SensWear LiPo battery regulation-voltage default.
 * @details Maps to the VBAT_CTRL.VBATREG field after conversion from 4200 mV.
 */
#define BQ25180_DEFAULT_BATTERY_VOLTAGE (4200u)

/**
 * @brief SensWear LiPo fast-charge-current default.
 * @details Maps to the ICHG_CTRL.ICHG field after conversion from 100 mA.
 */
#define BQ25180_DEFAULT_BATTERY_CHARGE_CURRENT (100u)

/**
 * @brief SensWear charge-termination-current default.
 * @details Maps to CHARGECTRL0.ITERM as 20 percent of fast-charge current.
 */
#define BQ25180_DEFAULT_TERMINATION_CURRENT (bq25180_termination_current_20Percent)

/**
 * @brief SensWear precharge-current default.
 * @details Maps to CHARGECTRL0.IPRECHARGE and selects the termination current.
 */
#define BQ25180_DEFAULT_PRECHARGE_CURRENT (bq25180_precharge_current_Termination)

/**
 * @brief SensWear recharge-voltage threshold default.
 * @details Maps to IC_CTRL.VRECHG and restarts charging 200 mV below VBATREG.
 */
#define BQ25180_DEFAULT_RECHARGE_VOLTAGE_THRESHOLD (bq25180_recharge_voltage_threshold_200mV)

/**
 * @brief SensWear precharge-voltage threshold default.
 * @details Maps to IC_CTRL.VLOWV and selects a 3.0 V threshold.
 */
#define BQ25180_DEFAULT_PRECHARGE_VOLTAGE_THRESHOLD (bq25180_precharge_voltage_threshold_3V0)

/**
 * @brief SensWear battery-undervoltage threshold default.
 * @details Maps to CHARGECTRL1.BUVLO and selects a 2.8 V threshold.
 */
#define BQ25180_DEFAULT_UVLO_THRESHOLD (bq25180_battery_UVLO_threshold_2V8)

/**
 * @brief SensWear battery discharge-current-limit default.
 * @details Maps to CHARGECTRL1.BATOCP and disables battery OCP limiting.
 */
#define BQ25180_DEFAULT_BATTERY_DISCHARGE_CURRENT_LIMIT \
	(bq25180_battery_discharge_current_limit_Disabled)

/**
 * @brief SensWear input-current-limit default.
 * @details Maps to TMR_ILIM.ILIM and selects 1100 mA.
 */
#define BQ25180_DEFAULT_INPUT_CURRENT_LIMIT (bq25180_input_current_limit_1100mA)

/**
 * @brief SensWear input-voltage DPM threshold default.
 * @details Maps to CHARGECTRL0.VINDPM and selects 4.7 V.
 */
#define BQ25180_DEFAULT_VINDPM_LEVEL (bq25180_VINDPM_level_4V7)

/**
 * @brief BQ25180 register-address map.
 * @details Each enumerator maps directly to the 8-bit register address placed
 *          at the start of a BQ25180 I2C register transaction.
 */
enum bq25180_register_type {
	bq25180_register_STAT0 = 0x0,		/**< Charger and input status. */
	bq25180_register_STAT1 = 0x1,		/**< Wake, timer, TS, and fault status. */
	bq25180_register_FLAG0 = 0x2,		/**< Latched charger fault flags. */
	bq25180_register_VBAT_CTRL = 0x3,	/**< Battery regulation voltage. */
	bq25180_register_ICHG_CTRL = 0x4,	/**< Charge current and charge enable. */
	bq25180_register_CHARGECTRL0 = 0x5, /**< Charge-profile controls. */
	bq25180_register_CHARGECTRL1 = 0x6, /**< Protection and interrupt controls. */
	bq25180_register_IC_CTRL = 0x7,		/**< Watchdog, timers, and TS controls. */
	bq25180_register_TMR_ILIM = 0x8,	/**< Input limit and button timers. */
	bq25180_register_SHIP_RST = 0x9,	/**< Ship, shutdown, and reset controls. */
	bq25180_register_SYS_REG = 0xA,		/**< System rail behavior. */
	bq25180_register_TS_CONTROL = 0xB,	/**< Battery temperature thresholds. */
	bq25180_register_MASK_ID = 0xC,		/**< Device ID and interrupt masks. */
};

/**
 * @brief BQ25180 charging-state encoding.
 * @details Maps directly to STAT0.CHG_STAT, register 0x00 bits 6:5.
 */
enum bq25180_charging_status_type {
	bq25180_charging_status_NotCharging = 0,			 /**< Charging is idle. */
	bq25180_charging_status_ChargingConstantCurrent = 1, /**< Constant-current phase. */
	bq25180_charging_status_ChargingConstantVoltage = 2, /**< Constant-voltage phase. */
	bq25180_charging_status_ChargingDone = 3,			 /**< Charge cycle complete. */
};

/**
 * @brief BQ25180 STAT0 register representation.
 * @details Maps to register 0x00. The bit fields mirror the device register;
 *          @ref value provides access to the complete value read over I2C.
 */
union bq25180_STAT0_register_t {
	unsigned int value; /**< Complete raw register value. */
	/**
	 * @brief STAT0 bit-field mapping.
	 * @details Maps fields in register 0x00 from bit 0 through bit 7.
	 */
	struct bq25180_STAT0_register_bits {
		unsigned int bVinPgoodStat : 1;				   /**< VIN power-good indication. */
		unsigned int bThermalRegulationActiveStat : 1; /**< Thermal regulation active. */
		unsigned int bVINDPPMActiveStat : 1;		   /**< Input-voltage DPM active. */
		unsigned int bVDPPMActiveStat : 1;			   /**< Battery/system DPPM active. */
		unsigned int bILIMActiveStat : 1;			   /**< Input current limit active. */
		unsigned int bChgStat : 2;					   /**< A bq25180_charging_status_type value. */
		unsigned int bTSOpenStat : 1;				   /**< Battery TS input is open. */
	} bits;
};

/**
 * @brief BQ25180 battery-temperature status encoding.
 * @details Maps directly to STAT1.TS_STAT, register 0x01 bits 4:3.
 */
enum bq25180_ts_status_type {
	bq25180_ts_status_Normal = 0,	 /**< Temperature is in the normal region. */
	bq25180_ts_status_ColdOrHot = 1, /**< Temperature is outside charge limits. */
	bq25180_ts_status_Cool = 2,		 /**< Temperature is in the cool region. */
	bq25180_ts_status_Warm = 3,		 /**< Temperature is in the warm region. */
};

/**
 * @brief BQ25180 STAT1 register representation.
 * @details Maps to register 0x01 and exposes wake, timer, temperature, battery
 *          UVLO, and VIN overvoltage status.
 */
union bq25180_STAT1_register_t {
	unsigned int value; /**< Complete raw register value. */
	/**
	 * @brief STAT1 bit-field mapping.
	 * @details Maps fields in register 0x01 from bit 0 through bit 7.
	 */
	struct bq25180_STAT1_register_bits {
		unsigned int bWake1Flag : 1;			/**< WAKE1 timer event occurred (STAT1 bit 0). */
		unsigned int bWake2Flag : 1;			/**< WAKE2 timer event occurred (STAT1 bit 1). */
		unsigned int bSafetyTimerFaultFlag : 1; /**< Charge safety timer expired. */
		unsigned int bTSStatus : 2;				/**< A bq25180_ts_status_type value. */
		unsigned int _reserved : 1;				/**< Reserved; ignore when reading. */
		unsigned int bBattUVLOStatus : 1;		/**< Battery is below UVLO threshold. */
		unsigned int bVinOVPFault : 1;			/**< VIN overvoltage fault is active. */
	} bits;
};

/**
 * @brief BQ25180 FLAG0 register representation.
 * @details Maps to register 0x02 and exposes the device's latched fault and
 *          operating-condition flags.
 */
union bq25180_FLAG0_register_t {
	unsigned int value; /**< Complete raw register value. */
	/**
	 * @brief FLAG0 bit-field mapping.
	 * @details Maps fields in register 0x02 from bit 0 through bit 7.
	 */
	struct bq25180_FLAG0_register_bits {
		unsigned int bBattOCPFault : 1;			 /**< Battery overcurrent fault. */
		unsigned int bBattUVLOFault : 1;		 /**< Battery UVLO fault. */
		unsigned int bVinOVPFault : 1;			 /**< VIN overvoltage fault. */
		unsigned int bThermalRegulationFlag : 1; /**< Thermal regulation occurred. */
		unsigned int bVinDPMFlag : 1;			 /**< Input-voltage DPM occurred. */
		unsigned int bVDPPMFlag : 1;			 /**< Battery/system DPPM occurred. */
		unsigned int bILIMFlag : 1;				 /**< Input current limiting occurred. */
		unsigned int bTSFault : 1;				 /**< Battery temperature fault. */
	} bits;
};

/**
 * @brief Convert a VBAT_CTRL.VBATREG code to millivolts.
 * @details Implements the BQ25180 3500 mV base plus 10 mV per register step.
 * @param __val Seven-bit VBATREG field value.
 * @return Programmed battery regulation voltage in millivolts.
 */
#define BQ25180_VBAT_REG_VAL_TO_OUT(__val) (uint16_t) (3500 + ((__val) * 10))

/**
 * @brief Convert a battery regulation voltage to a VBATREG code.
 * @details Accepts voltages above 3500 mV through 4650 mV in 10 mV steps.
 *          Values outside that range map to code zero, which represents
 *          3500 mV. Non-aligned values are rounded down to the nearest step.
 * @param __val Requested battery regulation voltage in millivolts.
 * @return Seven-bit value for VBAT_CTRL.VBATREG.
 */
#define BQ25180_VBAT_REG_VAL_FROM_OUT(__val) \
	(uint8_t) (((__val) > 3500 && (__val) <= 4650) ? ((((__val) - 3500) / 10) & 0x7F) : 0)

/**
 * @brief BQ25180 reset code for VBAT_CTRL.VBATREG.
 * @details Register code 70 maps to 4200 mV.
 */
#define BQ25180_VBAT_REG_DEFAULT (70)

/**
 * @brief BQ25180 reset battery regulation voltage in millivolts.
 * @details Derived from BQ25180_VBAT_REG_DEFAULT and maps to 4200 mV.
 */
#define BQ25180_VBAT_OUT_DEFAULT BQ25180_VBAT_REG_VAL_TO_OUT(BQ25180_VBAT_REG_DEFAULT)

/**
 * @brief Complete VBAT_CTRL register reset value.
 * @details Maps to register 0x03 and contains VBATREG code 70.
 */
#define BQ25180_VBAT_CTRL_DEFAULT (70)

/**
 * @brief BQ25180 VBAT_CTRL register representation.
 * @details Maps to register 0x03. VBATREG occupies bits 6:0 and controls the
 *          battery regulation voltage.
 */
union bq25180_VBAT_CTRL_register_t {
	unsigned int value; /**< Complete raw register value. */
	/**
	 * @brief VBAT_CTRL bit-field mapping.
	 * @details Maps the VBATREG field and reserved bit in register 0x03.
	 */
	struct bq25180_VBAT_CTRL_register_bits {
		unsigned int bVBattReg : 7; /**< 10 mV steps above 3500 mV. */
		unsigned int : 1;			/**< Reserved. */
	} bits;
};

/**
 * @brief Convert an ICHG_CTRL.ICHG code to milliamperes.
 * @details Codes 0 through 35 map to 5 through 40 mA in 1 mA steps; larger
 *          codes map to the higher-current range in 10 mA steps.
 * @param __val Seven-bit ICHG field value.
 * @return Fast-charge current in milliamperes.
 */
#define BQ25180_ICHG_VAL_TO_OUT(__val) \
	(((__val) <= 35) ? ((__val) + 5) : ((((__val) - 31) * 10) + 40))

/**
 * @brief Convert fast-charge current to an ICHG register code.
 * @details Currents from 5 through 40 mA use 1 mA steps. Inputs from 41 through
 *          89 mA map to code 36, while values from 90 mA use 10 mA steps.
 *          Callers must validate the device-supported current range.
 * @param __val Requested fast-charge current in milliamperes.
 * @return Seven-bit value for ICHG_CTRL.ICHG.
 */
#define BQ25180_ICHG_OUT_TO_VAL(__val) \
	(((__val) < 90u) ? (((__val) <= 40u) ? ((__val) - 5u) : 36u) : ((((__val) - 40u) / 10u) + 31u))

/**
 * @brief BQ25180 reset code for ICHG_CTRL.ICHG.
 * @details Register code 5 maps to a 10 mA fast-charge current.
 */
#define BQ25180_ICHG_VAL_DEFAULT (5)

/**
 * @brief BQ25180 reset fast-charge current in milliamperes.
 * @details Derived from BQ25180_ICHG_VAL_DEFAULT and maps to 10 mA.
 */
#define BQ25180_ICHG_OUT_DEFAULT BQ25180_ICHG_VAL_TO_OUT(BQ25180_ICHG_VAL_DEFAULT)

/**
 * @brief Complete ICHG_CTRL register reset value.
 * @details Maps to register 0x04 with charging enabled and ICHG code 5.
 */
#define BQ25180_ICHG_CTRL_DEFAULT (5)

/**
 * @brief BQ25180 ICHG_CTRL register representation.
 * @details Maps to register 0x04. Bits 6:0 encode fast-charge current and bit 7
 *          disables charging when set.
 */
union bq25180_ICHG_CTRL_register_t {
	unsigned int value; /**< Complete raw register value. */
	/**
	 * @brief ICHG_CTRL bit-field mapping.
	 * @details Maps the ICHG and CHG_DIS fields in register 0x04.
	 */
	struct bq25180_ICHG_CTRL_register_bits {
		unsigned int bICHG : 7;			 /**< Encoded fast-charge current. */
		unsigned int bChargeDisable : 1; /**< Set to disable battery charging. */
	} bits;
};

/**
 * @brief BQ25180 die thermal-regulation encoding.
 * @details Maps to CHARGECTRL0.THERM_REG, register 0x05 bits 1:0.
 */
enum bq25180_thermal_regulation_threshold_type {
	bq25180_thermal_regulation_threshold_100C = 0,	   /**< Regulate at 100 degrees C. */
	bq25180_thermal_regulation_threshold_Disabled = 3, /**< Disable thermal regulation. */
};

/**
 * @brief BQ25180 input-voltage DPM encoding.
 * @details Maps to CHARGECTRL0.VINDPM, register 0x05 bits 3:2.
 */
enum bq25180_VINDPM_level_type {
	bq25180_VINDPM_level_4V2 = 0,	   /**< 4.2 V threshold. */
	bq25180_VINDPM_level_4V5 = 1,	   /**< 4.5 V threshold. */
	bq25180_VINDPM_level_4V7 = 2,	   /**< 4.7 V threshold. */
	bq25180_VINDPM_level_Disabled = 3, /**< Disable input-voltage DPM. */
};

/**
 * @brief BQ25180 charge-termination-current encoding.
 * @details Maps to CHARGECTRL0.ITERM, register 0x05 bits 5:4, as a percentage
 *          of the programmed fast-charge current.
 */
enum bq25180_termination_current_type {
	bq25180_termination_current_Disabled = 0,  /**< Disable termination. */
	bq25180_termination_current_5Percent = 1,  /**< Terminate at 5 percent. */
	bq25180_termination_current_10Percent = 2, /**< Terminate at 10 percent. */
	bq25180_termination_current_20Percent = 3, /**< Terminate at 20 percent. */
};

/**
 * @brief BQ25180 precharge-current encoding.
 * @details Maps to CHARGECTRL0.IPRECHARGE, register 0x05 bits 7:6, relative to
 *          the selected termination current.
 */
enum bq25180_precharge_current_type {
	bq25180_precharge_current_2xTermination = 0, /**< Twice termination current. */
	bq25180_precharge_current_Termination = 1	 /**< Equal to termination current. */
};

/**
 * @brief Complete CHARGECTRL0 register reset value.
 * @details Maps to register 0x05 and provides the initial field values used by
 *          the driver before applying a requested configuration.
 */
#define BQ25180_CHARGECTRL0_DEFAULT (0x2C)

/**
 * @brief BQ25180 CHARGECTRL0 register representation.
 * @details Maps to register 0x05 and combines thermal regulation, input-voltage
 *          DPM, termination-current, and precharge-current settings.
 */
union bq25180_CHARGECTRL0_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief CHARGECTRL0 bit-field mapping.
	 * @details Maps all configuration fields in register 0x05.
	 */
	struct bq25180_CHARGECTRL0_register_bits {
		unsigned int bThermalRegulationThresold : 2; /**< Thermal threshold encoding. */
		unsigned int bVINDPMLvel : 2;				 /**< VINDPM threshold encoding. */
		unsigned int bTerminationCurrent : 2;		 /**< Termination-current encoding. */
		unsigned int bPrechargeCurrent : 2;			 /**< Precharge-current encoding. */
		unsigned int : 1;							 /**< Reserved. */
	} bits;
};

/**
 * @brief BQ25180 battery-undervoltage lockout encoding.
 * @details Maps to CHARGECTRL1.BUVLO, register 0x06 bits 5:3.
 */
enum bq25180_battery_UVLO_threshold_type {
	bq25180_battery_UVLO_threshold_3V0 = 0,	  /**< 3.0 V threshold, encoding 0. */
	bq25180_battery_UVLO_threshold_3V0_1 = 1, /**< 3.0 V threshold, encoding 1. */
	bq25180_battery_UVLO_threshold_3V0_2 = 2, /**< 3.0 V threshold, encoding 2. */
	bq25180_battery_UVLO_threshold_2V8 = 3,	  /**< 2.8 V threshold. */
	bq25180_battery_UVLO_threshold_2V6 = 4,	  /**< 2.6 V threshold. */
	bq25180_battery_UVLO_threshold_2V4 = 5,	  /**< 2.4 V threshold. */
	bq25180_battery_UVLO_threshold_2V2 = 6,	  /**< 2.2 V threshold. */
	bq25180_battery_UVLO_threshold_2V0 = 7,	  /**< 2.0 V threshold. */
};

/**
 * @brief BQ25180 battery discharge overcurrent-limit encoding.
 * @details Maps to CHARGECTRL1.BATOCP, register 0x06 bits 7:6.
 */
enum bq25180_battery_discharge_current_limit_type {
	bq25180_battery_discharge_current_limit_500mA = 0,	  /**< 500 mA limit. */
	bq25180_battery_discharge_current_limit_1000mA = 1,	  /**< 1000 mA limit. */
	bq25180_battery_discharge_current_limit_1500mA = 2,	  /**< 1500 mA limit. */
	bq25180_battery_discharge_current_limit_Disabled = 3, /**< Disable protection. */
};

/**
 * @brief Complete CHARGECTRL1 register reset value.
 * @details Maps to register 0x06 and supplies the initial interrupt-mask,
 *          battery UVLO, and battery OCP fields used during configuration.
 */
#define BQ25180_CHARGECTRL1_DEFAULT (0x56)

/**
 * @brief BQ25180 CHARGECTRL1 register representation.
 * @details Maps to register 0x06 and combines interrupt masks with battery
 *          undervoltage and discharge-current protection settings.
 */
union bq25180_CHARGECTRL1_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief CHARGECTRL1 bit-field mapping.
	 * @details Maps all interrupt-mask and battery-protection fields in
	 *          register 0x06.
	 */
	struct bq25180_CHARGECTRL1_register_bits {
		unsigned int bMaskVINDPMInterrupt : 1;			/**< Mask VINDPM interrupt. */
		unsigned int bMaskILIMInterrupt : 1;			/**< Mask input-limit interrupt. */
		unsigned int bMaskChargingStatusInterrupt : 1;	/**< Mask charge-status interrupt. */
		unsigned int bBatteryUVLOThreshold : 3;			/**< Battery UVLO encoding. */
		unsigned int bBatteryDischargeCurrentLimit : 2; /**< Discharge-limit encoding. */
	} bits;
};

/**
 * @brief BQ25180 I2C-watchdog encoding.
 * @details Maps to IC_CTRL.WATCHDOG_SEL, register 0x07 bits 1:0.
 */
enum bq25180_watchdog_selection_type {
	bq25180_watchdog_selection_160sDefaultRegisterValues = 0, /**< Restore defaults after 160 s. */
	bq25180_watchdog_selection_160sHardwareReset = 1,		  /**< Hardware reset after 160 s. */
	bq25180_watchdog_selection_40sHardwareReset = 2,		  /**< Hardware reset after 40 s. */
	bq25180_watchdog_selection_Disable = 3,					  /**< Disable the watchdog. */
};

/**
 * @brief BQ25180 fast-charge safety-timer encoding.
 * @details Maps to IC_CTRL.SAFETY_TIMER, register 0x07 bits 3:2.
 */
enum bq25180_fast_charge_time_type {
	bq25180_fast_charge_time_3h = 0,	  /**< Three-hour limit. */
	bq25180_fast_charge_time_6h = 1,	  /**< Six-hour limit. */
	bq25180_fast_charge_time_12h = 2,	  /**< Twelve-hour limit. */
	bq25180_fast_charge_time_Disable = 3, /**< Disable the safety timer. */
};

/**
 * @brief BQ25180 recharge-voltage encoding.
 * @details Maps to IC_CTRL.VRECHG, register 0x07 bit 5, and specifies the
 *          voltage drop below VBATREG that restarts charging.
 */
enum bq25180_recharge_voltage_threshold_type {
	bq25180_recharge_voltage_threshold_100mV = 0, /**< Restart 100 mV below regulation. */
	bq25180_recharge_voltage_threshold_200mV = 1, /**< Restart 200 mV below regulation. */
};

/**
 * @brief BQ25180 precharge-voltage encoding.
 * @details Maps to IC_CTRL.VLOWV, register 0x07 bit 6, and selects the battery
 *          threshold for transition from precharge to fast charge.
 */
enum bq25180_precharge_voltage_threshold_type {
	bq25180_precharge_voltage_threshold_2V8 = 0, /**< 2.8 V threshold. */
	bq25180_precharge_voltage_threshold_3V0 = 1, /**< 3.0 V threshold. */
};

/**
 * @brief Complete IC_CTRL register reset value.
 * @details Maps to register 0x07 and supplies the initial watchdog, timer,
 *          recharge, precharge, and TS automatic-control fields.
 */
#define BQ25180_IC_CTRL_DEFAULT (0x84)

/**
 * @brief BQ25180 IC_CTRL register representation.
 * @details Maps to register 0x07 and combines watchdog, charge timer, recharge,
 *          precharge, and battery-temperature automatic-control settings.
 */
union bq25180_IC_CTRL_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief IC_CTRL bit-field mapping.
	 * @details Maps all control fields in register 0x07.
	 */
	struct bq25180_IC_CTRL_register_bits {
		unsigned int bWatchdogSelection : 2;		 /**< Watchdog mode encoding. */
		unsigned int bSafetyFastChargeTimer : 2;	 /**< Safety-timer encoding. */
		unsigned int b2XTimerEnable : 1;			 /**< Double timer during thermal regulation. */
		unsigned int bRechargeVoltage : 1;			 /**< Recharge threshold encoding. */
		unsigned int bPrechargeVoltageThreshold : 1; /**< Precharge threshold encoding. */
		unsigned int bTSAutoFunctionEnable : 1;		 /**< Enable automatic TS charge control. */
	} bits;
};

/**
 * @brief BQ25180 input-current-limit encoding.
 * @details Maps to TMR_ILIM.ILIM, register 0x08 bits 2:0.
 */
enum bq25180_input_current_limit_type {
	bq25180_input_current_limit_50mA = 0,	/**< 50 mA. */
	bq25180_input_current_limit_100mA = 1,	/**< 100 mA. */
	bq25180_input_current_limit_200mA = 2,	/**< 200 mA. */
	bq25180_input_current_limit_300mA = 3,	/**< 300 mA. */
	bq25180_input_current_limit_400mA = 4,	/**< 400 mA. */
	bq25180_input_current_limit_500mA = 5,	/**< 500 mA. */
	bq25180_input_current_limit_700mA = 6,	/**< 700 mA. */
	bq25180_input_current_limit_1100mA = 7, /**< 1100 mA. */
};

/**
 * @brief BQ25180 automatic wake-up restart-timer encoding.
 * @details Maps to TMR_ILIM.MRRESET_VIN, register 0x08 bits 4:3.
 */
enum bq25180_auto_wakeup_timer_restart_type {
	bq25180_auto_wakeup_timer_restart_0p5s = 0, /**< 0.5 seconds. */
	bq25180_auto_wakeup_timer_restart_1s = 1,	/**< 1 second. */
	bq25180_auto_wakeup_timer_restart_2s = 2,	/**< 2 seconds. */
	bq25180_auto_wakeup_timer_restart_4s = 3,	/**< 4 seconds. */
};

/**
 * @brief BQ25180 push-button long-press duration encoding.
 * @details Maps to TMR_ILIM.MR_LPRESS, register 0x08 bits 7:6.
 */
enum bq25180_pb_long_press_duration_type {
	bq25180_pb_long_press_duration_5s = 0,	/**< 5 seconds. */
	bq25180_pb_long_press_duration_10s = 1, /**< 10 seconds. */
	bq25180_pb_long_press_duration_15s = 2, /**< 15 seconds. */
	bq25180_pb_long_press_duration_20s = 3, /**< 20 seconds. */
};

/**
 * @brief Complete TMR_ILIM register reset value.
 * @details Maps to register 0x08 and supplies the initial input-current limit
 *          and push-button timing fields.
 */
#define BQ25180_TMR_ILIM_DEFAULT (0x4D)

/**
 * @brief BQ25180 TMR_ILIM register representation.
 * @details Maps to register 0x08 and combines input-current limiting,
 *          automatic wake-up timing, reset conditions, and long-press timing.
 */
union bq25180_TMR_ILIM_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief TMR_ILIM bit-field mapping.
	 * @details Maps all current-limit and timing fields in register 0x08.
	 */
	struct bq25180_TMR_ILIM_register_bits {
		unsigned int bInputCurrentLimit : 3;	  /**< Input-current limit encoding. */
		unsigned int bAutoWakeupTimer : 2;		  /**< Automatic wake-up interval. */
		unsigned int bHardwareResetCondition : 1; /**< Hardware-reset condition selection. */
		unsigned int bLongPressDuration : 2;	  /**< Long-press duration encoding. */
	} bits;
};

/**
 * @brief BQ25180 WAKE2 timer encoding.
 * @details Maps to SHIP_RST.WAKE2_TIMER, register 0x09 bit 1.
 */
enum bq25180_wake2_timer_set_type {
	bq25180_wake2_timer_set_2s = 0, /**< 2 seconds. */
	bq25180_wake2_timer_set_3s = 1, /**< 3 seconds. */
};

/**
 * @brief BQ25180 WAKE1 timer encoding.
 * @details Maps to SHIP_RST.WAKE1_TIMER, register 0x09 bit 2.
 */
enum bq25180_wake1_timer_set_type {
	bq25180_wake1_timer_set_300ms = 0, /**< 300 milliseconds. */
	bq25180_wake1_timer_set_1s = 1,	   /**< 1 second. */
};

/**
 * @brief BQ25180 push-button long-press action encoding.
 * @details Maps to SHIP_RST.MR_LPRESS_ACTION, register 0x09 bits 4:3.
 */
enum bq25180_long_press_action_type {
	bq25180_long_press_action_DoNothing = 0,	 /**< Take no power action. */
	bq25180_long_press_action_HardwareReset = 1, /**< Perform a hardware reset. */
	bq25180_long_press_action_ShipMode = 2,		 /**< Enter ship mode. */
	bq25180_long_press_action_Shutdown = 3,		 /**< Enter shutdown mode. */
};

/**
 * @brief BQ25180 ship, shutdown, and reset command encoding.
 * @details Maps to SHIP_RST.EN_SHIP_MODE, register 0x09 bits 6:5.
 */
enum bq25180_reset_shipment_mode_type {
	bq25180_reset_shipment_mode_DoNothing = 0, /**< Do not request a mode change. */
	bq25180_reset_shipment_mode_Shutdown = 1,  /**< Request shutdown mode. */
	bq25180_reset_shipment_mode_ShipMode = 2,  /**< Request ship mode. */
	bq25180_reset_shipment_mode_Reset = 3,	   /**< Request a hardware reset. */
};

/**
 * @brief Complete SHIP_RST register reset value.
 * @details Maps to register 0x09 and supplies the initial push-button,
 *          wake-timer, power-mode, and reset fields.
 */
#define BQ25180_SHIP_RST_DEFAULT (0x11)

/**
 * @brief BQ25180 SHIP_RST register representation.
 * @details Maps to register 0x09 and controls push-button wake timing,
 *          long-press actions, ship/shutdown entry, and reset requests.
 */
union bq25180_SHIP_RST_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief SHIP_RST bit-field mapping.
	 * @details Maps all push-button, power-mode, and reset fields in
	 *          register 0x09.
	 */
	struct bq25180_SHIP_RST_register_bits {
		unsigned int bEnablePush : 1;				 /**< Enable push-button detection. */
		unsigned int bWake2TimerSet : 1;			 /**< WAKE2 timer encoding. */
		unsigned int bWake1TimerSet : 1;			 /**< WAKE1 timer encoding. */
		unsigned int bPushbuttonLongPressAction : 2; /**< Long-press action encoding. */
		unsigned int bEnableShipModeAndReset : 2;	 /**< Ship, shutdown, or reset command. */
		unsigned int bSoftwareReset : 1;			 /**< Request a software reset. */
	} bits;
};

/**
 * @brief BQ25180 system rail power-mode encoding.
 * @details Maps to SYS_REG.SYS_MODE, register 0x0A bits 3:2.
 */
enum bq25180_sys_power_mode_type {
	bq25180_sys_power_mode_VIN_or_VBAT = 0,		   /**< Power SYS from VIN or VBAT. */
	bq25180_sys_power_mode_VBAT = 1,			   /**< Power SYS from VBAT only. */
	bq25180_sys_power_mode_DisconnectFloating = 2, /**< Disconnect SYS and leave it floating. */
	bq25180_sys_power_mode_DisconnectPulldown = 3, /**< Disconnect SYS with pull-down enabled. */
};

/**
 * @brief BQ25180 system rail regulation-voltage encoding.
 * @details Maps to SYS_REG.SYS_REG, register 0x0A bits 7:5.
 */
enum bq25180_sys_regulation_voltage_type {
	bq25180_sys_regulation_voltage_BatteryTrack = 0, /**< Track battery voltage. */
	bq25180_sys_regulation_voltage_4V4 = 1,			 /**< Regulate at 4.4 V. */
	bq25180_sys_regulation_voltage_4V5 = 2,			 /**< Regulate at 4.5 V. */
	bq25180_sys_regulation_voltage_4V6 = 3,			 /**< Regulate at 4.6 V. */
	bq25180_sys_regulation_voltage_4V7 = 4,			 /**< Regulate at 4.7 V. */
	bq25180_sys_regulation_voltage_4V8 = 5,			 /**< Regulate at 4.8 V. */
	bq25180_sys_regulation_voltage_4V9 = 6,			 /**< Regulate at 4.9 V. */
	bq25180_sys_regulation_voltage_VIN = 7,			 /**< Track VIN. */
};

/**
 * @brief Complete SYS_REG register reset value.
 * @details Maps to register 0x0A and supplies the initial system DPPM,
 *          watchdog, power-mode, and regulation-voltage fields.
 */
#define BQ25180_SYS_REG_DEFAULT (0x40)

/**
 * @brief BQ25180 SYS_REG register representation.
 * @details Maps to register 0x0A and controls system DPPM, I2C watchdog
 *          enablement, rail source/disconnect behavior, and SYS voltage.
 */
union bq25180_SYS_REG_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief SYS_REG bit-field mapping.
	 * @details Maps all system rail and watchdog fields in register 0x0A.
	 */
	struct bq25180_SYS_REG_register_bits {
		unsigned int bVDPPMEnable : 1;			/**< Enable battery/system DPPM. */
		unsigned int bI2CWatchdogEnable : 1;	/**< Enable the I2C watchdog. */
		unsigned int bSYSPowerMode : 2;			/**< System power-mode encoding. */
		unsigned int : 1;						/**< Reserved. */
		unsigned int bSYSRegulationVoltage : 3; /**< System voltage encoding. */
	} bits;
};

/**
 * @brief BQ25180 cold battery-temperature threshold encoding.
 * @details Maps to TS_CONTROL.TS_COLD, register 0x0B bits 5:4.
 */
enum bq25180_cold_threshold_type {
	bq25180_cold_threshold_0 = 0,  /**< 0 degrees C. */
	bq25180_cold_threshold_3 = 1,  /**< 3 degrees C. */
	bq25180_cold_threshold_5 = 2,  /**< 5 degrees C. */
	bq25180_cold_threshold_m3 = 3, /**< -3 degrees C. */
};

/**
 * @brief BQ25180 hot battery-temperature threshold encoding.
 * @details Maps to TS_CONTROL.TS_HOT, register 0x0B bits 7:6.
 */
enum bq25180_hot_threshold_type {
	bq25180_hot_threshold_60 = 0, /**< 60 degrees C. */
	bq25180_hot_threshold_65 = 1, /**< 65 degrees C. */
	bq25180_hot_threshold_50 = 2, /**< 50 degrees C. */
	bq25180_hot_threshold_45 = 3, /**< 45 degrees C. */
};

/**
 * @brief Complete TS_CONTROL register reset value.
 * @details Maps to register 0x0B and supplies the initial battery-temperature
 *          response and threshold fields.
 */
#define BQ25180_TS_CONTROL_DEFAULT (0x00)

/**
 * @brief BQ25180 TS_CONTROL register representation.
 * @details Maps to register 0x0B and controls warm-region voltage reduction,
 *          thermal current reduction, and cool/warm/cold/hot thresholds.
 */
union bq25180_TS_CONTROL_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief TS_CONTROL bit-field mapping.
	 * @details Maps all battery-temperature response fields in register 0x0B.
	 */
	struct bq25180_TS_CONTROL_register_bits {
		unsigned int bReducedBatteryVoltage : 1; /**< Reduce VBAT regulation when warm. */
		unsigned int bFastChargeCurrentReductionInThermalSystem
			: 1;									  /**< Reduce current in cool/warm regions. */
		unsigned int bThermalSystemCoolThreshold : 1; /**< Cool-region threshold encoding. */
		unsigned int bThermalSystemWarmThreshold : 1; /**< Warm-region threshold encoding. */
		unsigned int bThermalSystemColdThreshold : 2; /**< Cold threshold encoding. */
		unsigned int bThermalSystemHotThreshold : 2;  /**< Hot threshold encoding. */
	} bits;
};

/**
 * @brief Complete MASK_ID register reset value.
 * @details Maps to register 0x0C and represents the initial device-ID and
 *          interrupt-mask bits expected by the driver.
 */
#define BQ25180_MASK_ID_DEFAULT (0xC0)

/**
 * @brief BQ25180 MASK_ID register representation.
 * @details Maps to register 0x0C. Bits 3:0 identify the device and bits 7:4
 *          mask power, battery, thermal-regulation, and shutdown interrupts.
 */
union bq25180_MASK_ID_register_t {
	uint8_t value; /**< Complete raw register value. */
	/**
	 * @brief MASK_ID bit-field mapping.
	 * @details Maps the device-ID and interrupt-mask fields in register 0x0C.
	 */
	struct bq25180_MASK_ID_register_bits {
		unsigned int bDeviceID : 4;						  /**< Device identification code. */
		unsigned int bPowerGoodMaskInterrupt : 1;		  /**< Mask power-good interrupt. */
		unsigned int bBatteryMaskInterrupt : 1;			  /**< Mask battery interrupt. */
		unsigned int bThermalRegulationMaskInterrupt : 1; /**< Mask thermal-regulation interrupt. */
		unsigned int bThermalShutdownMaskInterrupt : 1;	  /**< Mask thermal-shutdown interrupt. */
	} bits;
};

/**
 * \brief The device id field in the BQ25180 MASK_ID register has a fixed value of
 * zero that can be used to verify communication with the device.
 */
#define BQ25180_HW_DEVICE_ID (0u)

#endif /* BQ25180_REGISTERS_H */
