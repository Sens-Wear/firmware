#ifndef BQ25180_H_
#define BQ25180_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#ifndef BQ25180_I2C_TIMEOUT
#define BQ25180_I2C_TIMEOUT (100) // in ms
#endif

#define BQ25180_PWR_KILL_GPIO_NODE DT_NODELABEL(gpio2)
#define BQ25180_PWR_KILL_PIN (4)

// DEFAULT CONFIG for LIPO battery and USB TYPE C charging
#define BQ25180_DEFAULT_BATTERY_VOLTAGE (4200u) // mV
// fast charge current limit for the battery -- max is 1.5A for pack -- check this
//TODO: check this value
#define BQ25180_DEFAULT_BATTERY_CHARGE_CURRENT (100u) // mA --
#define BQ25180_DEFAULT_TERMINATION_CURRENT \
    (bq25180_termination_current_20Percent)
#define BQ25180_DEFAULT_PRECHARGE_CURRENT \
    (bq25180_precharge_current_Termination)
#define BQ25180_DEFAULT_RECHARGE_VOLTAGE_THRESHOLD \
    (bq25180_recharge_voltage_threshold_200mV)
#define BQ25180_DEFAULT_PRECHARGE_VOLTAGE_THRESHOLD \
    (bq25180_precharge_voltage_threshold_3V0)
#define BQ25180_DEFAULT_UVLO_THRESHOLD (bq25180_battery_UVLO_threshold_2V8)
#define BQ25180_DEFAULT_BATTERY_DISCHARGE_CURRENT_LIMIT \
    (bq25180_battery_discharge_current_limit_Disabled)
#define BQ25180_DEFAULT_INPUT_CURRENT_LIMIT (bq25180_input_current_limit_1100mA)
#define BQ25180_DEFAULT_VINDPM_LEVEL (bq25180_VINDPM_level_4V7)

enum bq25180_register_type {
    bq25180_register_STAT0 = 0x0,
    bq25180_register_STAT1 = 0x1,
    bq25180_register_FLAG0 = 0x2,
    bq25180_register_VBAT_CTRL = 0x3,
    bq25180_register_ICHG_CTRL = 0x4,
    bq25180_register_CHARGECTRL0 = 0x5,
    bq25180_register_CHARGECTRL1 = 0x6,
    bq25180_register_IC_CTRL = 0x7,
    bq25180_register_TMR_ILIM = 0x8,
    bq25180_register_SHIP_RST = 0x9,
    bq25180_register_SYS_REG = 0xA,
    bq25180_register_TS_CONTROL = 0xB,
    bq25180_register_MASK_ID = 0xC,
};

enum bq25180_charging_status_type {
    bq25180_charging_status_NotCharging = 0,
    bq25180_charging_status_ChargingConstantCurrent = 1,
    bq25180_charging_status_ChargingConstantVoltage = 2,
    bq25180_charging_status_ChargingDone = 3,
};

union bq25180_STAT0_register_t {
    unsigned int value;
    struct bq25180_STAT0_register_bits {
        unsigned int bVinPgoodStat: 1;
        unsigned int bThermalRegulationActiveStat: 1;
        unsigned int bVINDPPMActiveStat: 1;
        unsigned int bVDPPMActiveStat: 1;
        unsigned int bILIMActiveStat: 1;
        unsigned int bChgStat: 2;
        unsigned int bTSOpenStat: 1;
    } bits;
};

enum bq25180_ts_status_type {
    bq25180_ts_status_Normal = 0,
    bq25180_ts_status_ColdOrHot = 1,
    bq25180_ts_status_Cool = 2,
    bq25180_ts_status_Warm = 3,
};
union bq25180_STAT1_register_t {
    unsigned int value;
    struct bq25180_STAT1_register_bits {
        unsigned int bWake2Flag: 1;
        unsigned int bWake1Flag: 1;
        unsigned int bSafetyTimerFaultFlag: 1;
        unsigned int bTSStatus: 2;
        unsigned int _reserved: 1;
        unsigned int bBattUVLOStatus: 1;
        unsigned int bVinOVPFault: 1;
    } bits;
};

union bq25180_FLAG0_register_t {
    unsigned int value;
    struct bq25180_FLAG0_register_bits {
        unsigned int bBattOCPFault: 1;
        unsigned int bBattUVLOFault: 1;
        unsigned int bVinOVPFault: 1;
        unsigned int bThermalRegulationFlag: 1;
        unsigned int bVinDPMFlag: 1;
        unsigned int bVDPPMFlag: 1;
        unsigned int bILIMFlag: 1;
        unsigned int bTSFault: 1;
    } bits;
};

#define BQ25180_VBAT_REG_VAL_TO_OUT(__val) (uint16_t)(3500 + (__val * 10))
#define BQ25180_VBAT_REG_VAL_FROM_OUT(__val)                                   \
    (uint8_t)((__val > 3500 && __val <= 4650) ? (((__val - 3500) / 10) & 0x7F) \
                                              : 0)
#define BQ25180_VBAT_REG_DEFAULT (70)
#define BQ25180_VBAT_OUT_DEFAULT \
    BQ25180_VBAT_REG_VAL_TO_OUT(BQ25180_VBAT_REG_DEFAULT)
#define BQ25180_VBAT_CTRL_DEFAULT (70)

union bq25180_VBAT_CTRL_register_t {
    unsigned int value;
    struct bq25180_VBAT_CTRL_register_bits {
        unsigned int bVBattReg: 7;
        unsigned int : 1;
    } bits;
};

#define BQ25180_ICHG_VAL_TO_OUT(__val) \
    ((__val <= 35) ? (__val + 5) : (((__val - 31) * 10) + 40))
#define BQ25180_ICHG_OUT_TO_VAL(__val)                       \
    ((__val < 90u) ? ((__val <= 40u) ? ((__val - 5u)) : 36u) \
                   : (((__val - 40u) / 10u) + 31u))
#define BQ25180_ICHG_VAL_DEFAULT (5)
#define BQ25180_ICHG_OUT_DEFAULT BQ25180_ICHG_VAL_TO_OUT(5)
#define BQ25180_ICHG_CTRL_DEFAULT (5)

union bq25180_ICHG_CTRL_register_t {
    unsigned int value;
    struct bq25180_ICHG_CTRL_register_bits {
        unsigned int bICHG: 7;
        unsigned int bChargeDisable: 1;
    } bits;
};

enum bq25180_thermal_regulation_threshold_type {
    bq25180_thermal_regulation_threshold_100C = 0,
    bq25180_thermal_regulation_threshold_Disabled = 3,
};

enum bq25180_VINDPM_level_type {
    bq25180_VINDPM_level_4V2 = 0,
    bq25180_VINDPM_level_4V5 = 1,
    bq25180_VINDPM_level_4V7 = 2,
    bq25180_VINDPM_level_Disabled = 3,
};

enum bq25180_termination_current_type {
    bq25180_termination_current_Disabled = 0,
    bq25180_termination_current_5Percent = 1,
    bq25180_termination_current_10Percent = 2,
    bq25180_termination_current_20Percent = 3,
};

enum bq25180_precharge_current_type {
    bq25180_precharge_current_2xTermination = 0,
    bq25180_precharge_current_Termination = 1
};

#define BQ25180_CHARGECTRL0_DEFAULT (0x2C)
union bq25180_CHARGECTRL0_register_t {
    uint8_t value;
    struct bq25180_CHARGECTRL0_register_bits {
        unsigned int bThermalRegulationThresold: 2;
        unsigned int bVINDPMLvel: 2;
        unsigned int bTerminationCurrent: 2;
        unsigned int bPrechargeCurrent: 2;
        unsigned int : 1;
    } bits;
};

enum bq25180_battery_UVLO_threshold_type {
    bq25180_battery_UVLO_threshold_3V0 = 0,
    bq25180_battery_UVLO_threshold_3V0_1 = 1,
    bq25180_battery_UVLO_threshold_3V0_2 = 2,
    bq25180_battery_UVLO_threshold_2V8 = 3,
    bq25180_battery_UVLO_threshold_2V6 = 4,
    bq25180_battery_UVLO_threshold_2V4 = 5,
    bq25180_battery_UVLO_threshold_2V2 = 6,
    bq25180_battery_UVLO_threshold_2V0 = 7,
};
enum bq25180_battery_discharge_current_limit_type {
    bq25180_battery_discharge_current_limit_500mA = 0,
    bq25180_battery_discharge_current_limit_1000mA = 1,
    bq25180_battery_discharge_current_limit_1500mA = 2,
    bq25180_battery_discharge_current_limit_Disabled = 3,
};

#define BQ25180_CHARGECTRL1_DEFAULT (0x56)

union bq25180_CHARGECTRL1_register_t {
    uint8_t value;
    struct bq25180_CHARGECTRL1_register_bits {
        unsigned int bMaskVINDPMInterrupt: 1;
        unsigned int bMaskILIMInterrupt: 1;
        unsigned int bMaskChargingStatusInterrupt: 1;
        unsigned int bBatteryUVLOThreshold: 3;
        unsigned int bBatteryDischargeCurrentLimit: 2;
    } bits;
};

enum bq25180_watchdog_selection_type {
    bq25180_watchdog_selection_160sDefaultRegisterValues = 0,
    bq25180_watchdog_selection_160sHardwareReset = 1,
	bq25180_watchdog_selection_40sHardwareReset = 2,
    bq25180_watchdog_selection_Disable = 3,
};

enum bq25180_fast_charge_time_type {
    bq25180_fast_charge_time_3h = 0,
    bq25180_fast_charge_time_6h = 1,
    bq25180_fast_charge_time_12h = 2,
    bq25180_fast_charge_time_Disable = 3,
};

enum bq25180_recharge_voltage_threshold_type {
    bq25180_recharge_voltage_threshold_100mV = 0,
    bq25180_recharge_voltage_threshold_200mV = 1,
};

enum bq25180_precharge_voltage_threshold_type {
    bq25180_precharge_voltage_threshold_2V8 = 0,
    bq25180_precharge_voltage_threshold_3V0 = 1,
};

#define BQ25180_IC_CTRL_DEFAULT (0x84)
union bq25180_IC_CTRL_register_t {
    uint8_t value;
    struct bq25180_IC_CTRL_register_bits {
        unsigned int bWatchdogSelection: 2;
        unsigned int bSafetyFastChargeTimer: 2;
        unsigned int b2XTimerEnable: 1;
        unsigned int bRechargeVoltage: 1;
        unsigned int bPrechargeVoltageThreshold: 1;
        unsigned int bTSAutoFunctionEnable: 1;
    } bits;
};

enum bq25180_input_current_limit_type {
    bq25180_input_current_limit_50mA = 0,
    bq25180_input_current_limit_100mA = 1,
    bq25180_input_current_limit_200mA = 2,
    bq25180_input_current_limit_300mA = 3,
    bq25180_input_current_limit_400mA = 4,
    bq25180_input_current_limit_500mA = 5,
    bq25180_input_current_limit_700mA = 6,
    bq25180_input_current_limit_1100mA = 7,
};
enum bq25180_auto_wakeup_timer_restart_type {
    bq25180_auto_wakeup_timer_restart_0p5s = 0,
    bq25180_auto_wakeup_timer_restart_1s = 1,
    bq25180_auto_wakeup_timer_restart_2s = 2,
    bq25180_auto_wakeup_timer_restart_4s = 3,
};

enum bq25180_pb_long_press_duration_type {
    bq25180_pb_long_press_duration_5s = 0,
    bq25180_pb_long_press_duration_10s = 1,
    bq25180_pb_long_press_duration_15s = 2,
    bq25180_pb_long_press_duration_20s = 3,
};

#define BQ25180_TMR_ILIM_DEFAULT (0x4D)
union bq25180_TMR_ILIM_register_t {
    uint8_t value;
    struct bq25180_TMR_ILIM_register_bits {
        unsigned int bInputCurrentLimit: 3;
        unsigned int bAutoWakeupTimer: 2;
        unsigned int bHardwareResetCondition: 1;
        unsigned int bLongPressDuration: 2;
    } bits;
};

enum bq25180_wake2_timer_set_type {
    bq25180_wake2_timer_set_2s = 0,
    bq25180_wake2_timer_set_3s = 1,
};

enum bq25180_wake1_timer_set_type {
    bq25180_wake1_timer_set_300ms = 0,
    bq25180_wake1_timer_set_1s = 1,
};
enum bq25180_long_press_action_type {
    bq25180_long_press_action_DoNothing = 0,
    bq25180_long_press_action_HardwareReset = 1,
    bq25180_long_press_action_ShipMode = 2,
    bq25180_long_press_action_Shutdown = 3,
};

enum bq25180_reset_shipment_mode_type {
    bq25180_reset_shipment_mode_DoNothing = 0,
    bq25180_reset_shipment_mode_Shutdown = 1,
    bq25180_reset_shipment_mode_ShipMode = 2,
    bq25180_reset_shipment_mode_Reset = 3,
};

#define BQ25180_SHIP_RST_DEFAULT (0x11)
union bq25180_SHIP_RST_register_t {
    uint8_t value;
    struct bq25180_SHIP_RST_register_bits {
        unsigned int bEnablePush: 1;
        unsigned int bWake2TimerSet: 1;
        unsigned int bWake1TimerSet: 1;
        unsigned int bPushbuttonLongPressAction: 2;
        unsigned int bEnableShipModeAndReset: 2;
        unsigned int bSoftwareReset: 1;
    } bits;
};

enum bq25180_sys_power_mode_type {
    bq25180_sys_power_mode_VIN_or_VBAT = 0,
    bq25180_sys_power_mode_VBAT = 1,
    bq25180_sys_power_mode_DisconnectFloating = 2,
    bq25180_sys_power_mode_DisconnectPulldown = 3,
};

enum bq25180_sys_regulation_voltage_type {
    bq25180_sys_regulation_voltage_BatteryTrack = 0,
    bq25180_sys_regulation_voltage_4V4 = 1,
    bq25180_sys_regulation_voltage_4V5 = 2,
    bq25180_sys_regulation_voltage_4V6 = 3,
    bq25180_sys_regulation_voltage_4V7 = 4,
    bq25180_sys_regulation_voltage_4V8 = 5,
    bq25180_sys_regulation_voltage_4V9 = 6,
    bq25180_sys_regulation_voltage_VIN = 7,
};

#define BQ25180_SYS_REG_DEFAULT (0x40)
union bq25180_SYS_REG_register_t {
    uint8_t value;
    struct bq25180_SYS_REG_register_bits {
        unsigned int bVDPPMEnable: 1;
        unsigned int bI2CWatchdogEnable: 1;
        unsigned int bSYSPowerMode: 2;
        unsigned int : 1;
        unsigned int bSYSRegulationVoltage: 3;
    } bits;
};

enum bq25180_cold_threshold_type {
    bq25180_cold_threshold_0 = 0,
    bq25180_cold_threshold_3 = 1,
    bq25180_cold_threshold_5 = 2,
    bq25180_cold_threshold_m3 = 3,
};
enum bq25180_hot_threshold_type {
    bq25180_hot_threshold_60 = 0,
    bq25180_hot_threshold_65 = 1,
    bq25180_hot_threshold_50 = 2,
    bq25180_hot_threshold_45 = 3,
};

#define BQ25180_TS_CONTROL_DEFAULT (0x00)
union bq25180_TS_CONTROL_register_t {
    uint8_t value;
    struct bq25180_TS_CONTROL_register_bits {
        unsigned int bReducedBatteryVoltage: 1;
        unsigned int bFastChargeCurrentReductionInThermalSystem: 1;
        unsigned int bThermalSystemCoolThreshold: 1;
        unsigned int bThermalSystemWarmThreshold: 1;
        unsigned int bThermalSystemColdThreshold: 2;
        unsigned int bThermalSystemHotThreshold: 2;
    } bits;
};

#define BQ25180_MASK_ID_DEFAULT (0xC0)
union bq25180_MASK_ID_register_t {
    uint8_t value;
    struct bq25180_MASK_ID_register_buts {
        unsigned int bDeviceID: 4;
        unsigned int bPowerGoodMaskInterrupt: 1;
        unsigned int bBatteryMaskInterrupt: 1;
        unsigned int bThermalRegulationMaskInterrupt: 1;
        unsigned int bThermalShutdownMaskInterrupt: 1;
    } bits;
};

enum bq25180_event_type {
    bq25180_event_Invalid = -1,
    bq25180_event_PowerGood = 0,
    bq25180_event_Charging = 1,
    bq25180_event_ChargingDone = 2,
    bq25180_event_ThermalRegulation = 3,
    bq25180_event_VINOverVoltageProtection = 4,
    bq25180_event_BatteryUnderVoltageLockOut = 5,
    bq25180_event_SafetyTimerExpired = 6,
    bq25180_event_ThermalSystemFault = 7,
    bq25180_event_BatteryUndervoltageLockoutFault = 8,
    bq25180_event_BatteryOverCurrentProtectionFault = 9,
};

union bq25180_charger_state_t {
    int value;
    struct bq25180_charger_state_bits {
        unsigned int bButtonPressed: 1;
        unsigned int bWake1: 1;
        unsigned int bWake2: 1;
        unsigned int bShipmentMode: 1;
        unsigned int bShutdownMode: 1;
        unsigned int bPowerGood: 1;
        unsigned int bCharging: 1;
        unsigned int bCharged: 1;
        unsigned int bThermalRegulation: 1;
        unsigned int bBatteryUVLO: 1;
        unsigned int bThermalNormal: 1;
        unsigned int bThermalWarmOrHot: 1;
        unsigned int bThermalWarm: 1;
        unsigned int bThermalCool: 1;
        unsigned int bSafetyTimerFault: 1;
        unsigned int bThermalSystemFault: 1;
        unsigned int bBatteryUVLOFault: 1;
        unsigned int bBatteryOCPFault: 1;
    } bits;
};

struct bq25180_config_t {
    uint16_t charge_voltage;
    uint16_t charge_current;

    enum bq25180_input_current_limit_type input_current;
    enum bq25180_VINDPM_level_type vin_dpm_level; //+

    enum bq25180_battery_UVLO_threshold_type battery_uvlo;                 //+
    enum bq25180_battery_discharge_current_limit_type battery_ocp_limit; //+

    enum bq25180_termination_current_type termination_current; // +
    enum bq25180_precharge_current_type precharge_current;       //+

    enum bq25180_precharge_voltage_threshold_type precharge_threshold;         //+
    enum bq25180_recharge_voltage_threshold_type recharge_voltage_threshold; //+

    enum bq25180_pb_long_press_duration_type long_press_duration;
};

union bq25180_state_t {
    unsigned int value;
    struct bq25180_state_bits {
        unsigned int bInitialized: 1;
        unsigned int bConfigured: 1;
        unsigned int bPendingShutdown: 1;
        unsigned int bPendingShipMode: 1;
    } bits;
};

struct bq25180_t {
    struct i2c_dt_spec device;
    struct bq25180_config_t config;
    union bq25180_state_t state;
    union bq25180_charger_state_t charger_state;
};

/**
 * \brief Initializes BQ25180 driver
 *
 * \param i2c I2C device to be used by the driver
 */
bool bq25180_init(void);
bool bq25180_is_available(void);

/**
 * \brief Configures BQ25180 is with the specified configuration.
 * \details This function also enables some interrupts.
 *
 * \param config The configuration to be loaded.
 */
bool bq25180_config(struct bq25180_config_t *config);

/**
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values loaded to the registers.
 *
 * \param config The destination configuration
 */
void bq25180_get_default_config(struct bq25180_config_t *config);

/**
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values optimized for LiPo battery of 450mA capacity charged
 * over USB type C.
 *
 * \param config The destination configuration
 */
void bq25180_get_default_lipo_usb_charger_config(struct bq25180_config_t *config);

/**
 * \brief Updates the state of BQ25180 driver from the device
 *
 * \return The detected event as a result of handled IRQ
 */
bool bq25180_update_state(union bq25180_charger_state_t *state);

/**
 * \brief Enables shipment mode and resets the charger
 * 
 */
bool bq25180_shipment_mode_enable(void);

/**
 * \brief Disables shipment mode
 * 
 */
bool bq25180_shipment_mode_disable(void);
/**
 * \brief Enables charger shutdown mode 
 * 
 */
bool bq25180_shutdown_enable(void);

/**
 * \brief Disables charger shutdown mode.
 * 
 */
bool bq25180_shutdown_disable(void);

/**
 * \brief Resets the BQ25180 device and reconfigures it with given parameters
 * 
 * \details The reset is performed by setting software reset bit in SHIP_RST register.
 *          After reset, all registers are reverted to default values and device must be
 *          reconfigured to maintain desired functionality. The function will retry reset
 *          up to 3 times before failing. After successful reset, the given configuration
 *          is applied via bq25180_config() function.
 *
 * \param config [in] New configuration to apply after reset. If NULL, default configuration 
 *                    will be used from bq25180_get_default_config().
 * \return true on success, false if reset or configuration fails
 */
bool bq25180_reset(struct bq25180_config_t *config);

bool bq25180_enable_charging(bool enable);

/**
 * \brief Prints the charger state of the device
 */
void bq25180_print_state(void);

#endif //! BQ25180_H_
