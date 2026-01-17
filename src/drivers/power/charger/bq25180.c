/*
 * bq25180.c
 *
 *  Created on: 28 Apr 2023
 *      Author: husey
 */

#include "bq25180.h"
#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bq25180, CONFIG_LOG_DEFAULT_LEVEL);
static struct bq25180_t bq25180 = {0};

#define BQ25180_NODE DT_NODELABEL(bq25180)
#define BQ25180_IRQ_NODE DT_NODELABEL(bq25180_irq)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(BQ25180_NODE);
static const struct gpio_dt_spec bq25180_int =
        GPIO_DT_SPEC_GET(BQ25180_IRQ_NODE, interrupt_gpios);
static struct gpio_callback bq25180_int_cb;
static struct k_work bq25180_int_work;
static bool bq25180_last_power_good;
static bool bq25180_irq_initialized;

static void bq25180_int_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    union bq25180_charger_state_t state;
    if (!bq25180_update_state(&state)) {
        LOG_WRN("BQ25180 state update failed");
        return;
    }

    if (state.bits.bPowerGood && !bq25180_last_power_good) {
        if (!state.bits.bCharged) {
            bq25180_enable_charging(true);
            LOG_INF("BQ25180 VIN detected, charging enabled");
        } else {
            bq25180_enable_charging(false);
            LOG_INF("BQ25180 VIN detected, battery full");
        }
    } else if (!state.bits.bPowerGood && bq25180_last_power_good) {
        bq25180_enable_charging(false);
        LOG_INF("BQ25180 VIN removed, charging disabled");
    } else if (state.bits.bCharged) {
        bq25180_enable_charging(false);
        LOG_INF("BQ25180 battery full, charging disabled");
    }

    bq25180_last_power_good = state.bits.bPowerGood;
}

static void bq25180_int_callback(const struct device *dev,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_work_submit(&bq25180_int_work);
}

static int bq25180_irq_init(void)
{
    int ret;

    if (!device_is_ready(bq25180_int.port)) {
        LOG_WRN("BQ25180 interrupt GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&bq25180_int, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("BQ25180 interrupt pin config failed (%d)", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&bq25180_int,
                                          GPIO_INT_EDGE_FALLING);
    if (ret) {
        LOG_ERR("BQ25180 interrupt config failed (%d)", ret);
        return ret;
    }

    gpio_init_callback(&bq25180_int_cb, bq25180_int_callback,
                       BIT(bq25180_int.pin));
    ret = gpio_add_callback(bq25180_int.port, &bq25180_int_cb);
    if (ret) {
        LOG_ERR("BQ25180 interrupt callback add failed (%d)", ret);
        return ret;
    }

    k_work_init(&bq25180_int_work, bq25180_int_work_fn);
    return 0;
}

static inline int
bq25180_i2c_write_register(enum bq25180_register_type address, uint8_t value) {
     return i2c_burst_write_dt(&bq25180.device,
                              (uint8_t)address,
                              &value,
                              sizeof(value));
}

static inline int
bq25180_i2c_read_register(enum bq25180_register_type address, uint8_t *value) {
    return i2c_burst_read_dt(&bq25180.device,
                             (uint8_t)address,
                             value,
                             sizeof(*value));
}

/*
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values loaded to the registers.
 *
 * \param config The destination configuraiton
 */
void bq25180_get_default_config(struct bq25180_config_t *config) {
    union bq25180_CHARGECTRL0_register_t chrgctrl0 = {
            .value = BQ25180_CHARGECTRL0_DEFAULT};
    union bq25180_CHARGECTRL1_register_t chrgctrl1 = {
            .value = BQ25180_CHARGECTRL1_DEFAULT};
    union bq25180_IC_CTRL_register_t ic_ctrl = {.value =
    BQ25180_IC_CTRL_DEFAULT};
    union bq25180_TMR_ILIM_register_t tmr_ilim = {.value =
    BQ25180_TMR_ILIM_DEFAULT};

    config->charge_voltage = BQ25180_VBAT_OUT_DEFAULT;
    config->charge_current = BQ25180_ICHG_OUT_DEFAULT;

    config->termination_current = (enum bq25180_termination_current_type)
            chrgctrl0.bits.bTerminationCurrent;
    config->precharge_current =
            (enum bq25180_precharge_current_type) chrgctrl0.bits.bPrechargeCurrent;
    config->vin_dpm_level =
            (enum bq25180_VINDPM_level_type) chrgctrl0.bits.bVINDPMLvel;

    config->battery_ocp_limit =
            (enum bq25180_battery_discharge_current_limit_type)
                    chrgctrl1.bits.bBatteryDischargeCurrentLimit;
    config->battery_uvlo = (enum bq25180_battery_UVLO_threshold_type)
            chrgctrl1.bits.bBatteryUVLOThreshold;

    config->precharge_threshold =
            (enum bq25180_precharge_voltage_threshold_type)
                    ic_ctrl.bits.bPrechargeVoltageThreshold;
    config->recharge_voltage_threshold =
            (enum bq25180_recharge_voltage_threshold_type)
                    ic_ctrl.bits.bRechargeVoltage;

    config->input_current = (enum bq25180_input_current_limit_type)
            tmr_ilim.bits.bInputCurrentLimit;
    config->long_press_duration = (enum bq25180_pb_long_press_duration_type)
            tmr_ilim.bits.bLongPressDuration;
}

/*
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values optimized for LiPo battery of 450mA capacity charged
 * over USB type C.
 *
 * \param config The destination configuraiton
 */
void bq25180_get_default_lipo_usb_charger_config(struct bq25180_config_t *config) {
    config->charge_voltage = BQ25180_DEFAULT_BATTERY_VOLTAGE;
    config->charge_current = BQ25180_DEFAULT_BATTERY_CHARGE_CURRENT;

    config->termination_current = BQ25180_DEFAULT_TERMINATION_CURRENT;
    config->precharge_current = BQ25180_DEFAULT_PRECHARGE_CURRENT;
    config->vin_dpm_level = BQ25180_DEFAULT_VINDPM_LEVEL;

    config->battery_ocp_limit = bq25180_battery_discharge_current_limit_Disabled; //bq25180_battery_discharge_current_limit_1500mA; //BQ25180_DEFAULT_BATTERY_DISCHARGE_CURRENT_LIMIT;
    config->battery_uvlo = bq25180_battery_UVLO_threshold_2V0; //BQ25180_DEFAULT_UVLO_THRESHOLD;

    // TODO: Check this.
    config->precharge_threshold = BQ25180_DEFAULT_PRECHARGE_VOLTAGE_THRESHOLD;
    config->recharge_voltage_threshold =
            BQ25180_DEFAULT_RECHARGE_VOLTAGE_THRESHOLD;

    config->input_current = BQ25180_DEFAULT_INPUT_CURRENT_LIMIT;
    config->long_press_duration = bq25180_pb_long_press_duration_5s;
}

bool bq25180_init(void) {
    bq25180.device = dev_i2c;
    if (!device_is_ready(bq25180.device.bus)) {
            LOG_ERR("I2C bus not ready");
            return -ENODEV;
    }
    bq25180_get_default_config(&(bq25180.config));
    bq25180.state.bits.bInitialized = 1;
    return true;
}

/*
 * \brief Configures BQ25180 is with the specified configuration.
 * \details This function also enables some interrupts.
 *
 * \param config The configuration to be loaded.
 */
bool bq25180_config(struct bq25180_config_t *config) {
    union bq25180_VBAT_CTRL_register_t vbatCtrl = {
            .value = BQ25180_VBAT_CTRL_DEFAULT};
    union bq25180_ICHG_CTRL_register_t ichgCtrl = {
            .value = BQ25180_ICHG_CTRL_DEFAULT};
    union bq25180_CHARGECTRL0_register_t chrgctrl0 = {
            .value = BQ25180_CHARGECTRL0_DEFAULT};
    union bq25180_CHARGECTRL1_register_t chrgctrl1 = {
            .value = BQ25180_CHARGECTRL1_DEFAULT};
    union bq25180_IC_CTRL_register_t icCtrl = {.value =
    BQ25180_IC_CTRL_DEFAULT};
    union bq25180_TMR_ILIM_register_t tmrIlim = {.value =
    BQ25180_TMR_ILIM_DEFAULT};
    union bq25180_SHIP_RST_register_t shipRst = {.value =
    BQ25180_SHIP_RST_DEFAULT};
    union bq25180_SYS_REG_register_t sysReg = {
            .value = 0 /*BQ25180_SYS_REG_DEFAULT*/};
    union bq25180_TS_CONTROL_register_t tsControlReg = {
            .value = BQ25180_TS_CONTROL_DEFAULT};
    //	union bq25180_MASK_ID_register_t maskId = {.value = BQ25180_MASK_ID_DEFAULT};

    assert(bq25180.state.bits.bInitialized != 0);
    if (config == NULL) {
        bq25180_get_default_lipo_usb_charger_config(&(bq25180.config));
    } else {
        memcpy(&(bq25180.config), config, sizeof(struct bq25180_config_t));
    }

    // load the configuration to the registers
    vbatCtrl.bits.bVBattReg =
            BQ25180_VBAT_REG_VAL_FROM_OUT(bq25180.config.charge_voltage);

    ichgCtrl.bits.bICHG =
            BQ25180_ICHG_OUT_TO_VAL(bq25180.config.charge_current) & 0x7f;
    ichgCtrl.bits.bChargeDisable = 0;

    chrgctrl0.bits.bPrechargeCurrent = bq25180.config.precharge_current;
    chrgctrl0.bits.bTerminationCurrent = bq25180.config.termination_current;
    chrgctrl0.bits.bVINDPMLvel = bq25180.config.vin_dpm_level;

    chrgctrl1.bits.bBatteryDischargeCurrentLimit =
            bq25180.config.battery_ocp_limit;
    chrgctrl1.bits.bBatteryUVLOThreshold = bq25180.config.battery_uvlo;
    chrgctrl1.bits.bMaskVINDPMInterrupt = 1; //these interrupts can be masked.

    icCtrl.bits.bPrechargeVoltageThreshold = bq25180.config.precharge_threshold;
    icCtrl.bits.bRechargeVoltage = bq25180.config.recharge_voltage_threshold;
    icCtrl.bits.bSafetyFastChargeTimer = (int) bq25180_fast_charge_time_3h;
    icCtrl.bits.bTSAutoFunctionEnable = 0;
    tmrIlim.bits.bInputCurrentLimit = bq25180.config.input_current;
    tmrIlim.bits.bLongPressDuration = bq25180.config.long_press_duration;
    tmrIlim.bits.bHardwareResetCondition = 1; // long press and vin

    shipRst.bits.bEnablePush = 1;
    shipRst.bits.bPushbuttonLongPressAction =
            bq25180_long_press_action_DoNothing;
    shipRst.bits.bEnableShipModeAndReset =
            bq25180_reset_shipment_mode_DoNothing;

    tsControlReg.bits.bThermalSystemColdThreshold = bq25180_cold_threshold_m3;
    sysReg.bits.bSYSPowerMode = bq25180_sys_power_mode_VIN_or_VBAT;

    // bool bRet = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (bRet == false) {
    //     return false;
    // }

    bq25180_i2c_write_register(bq25180_register_VBAT_CTRL,
                               (uint8_t) vbatCtrl.value);
    bq25180_i2c_write_register(bq25180_register_ICHG_CTRL,
                               (uint8_t) ichgCtrl.value);
    bq25180_i2c_write_register(bq25180_register_CHARGECTRL0, chrgctrl0.value);
    bq25180_i2c_write_register(bq25180_register_CHARGECTRL1, chrgctrl1.value);
    bq25180_i2c_write_register(bq25180_register_IC_CTRL, icCtrl.value);
    bq25180_i2c_write_register(bq25180_register_TMR_ILIM, tmrIlim.value);
    bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value);
    bq25180_i2c_write_register(bq25180_register_TS_CONTROL, tsControlReg.value);
    bq25180_i2c_write_register(bq25180_register_SYS_REG, sysReg.value);

    bq25180.state.bits.bConfigured = 1;

    //TODO: Enable the interrupt after making necessary changes to the design.
    // union bq25180_charger_state_t state;
    // bq25180_update_state(&state);
    // bq25180_last_power_good = state.bits.bPowerGood;
    // bq25180_enable_charging(bq25180_last_power_good && !state.bits.bCharged);

    // if (!bq25180_irq_initialized) {
    //     int ret = bq25180_irq_init();
    //     if (ret == 0) {
    //         bq25180_irq_initialized = true;
    //         bq25180_int_work_fn(&bq25180_int_work);
    //     }
    // }

    // BQ25180_I2C_UNLOCK(&bq25180);
    return true;
}

/*
 * \brief Handles BQ25180 interrupt request
 *
 * \return The detected event as a result of handled IRQ
 */
bool bq25180_update_state(union bq25180_charger_state_t *state) {
    assert(bq25180.state.bits.bInitialized != 0);
    assert(bq25180.state.bits.bConfigured != 0);

    union bq25180_STAT0_register_t stat0;
    union bq25180_STAT1_register_t stat1;
    union bq25180_FLAG0_register_t flag0;
    union bq25180_SHIP_RST_register_t shipRst;

    //	enum bq25180_event_type event = bq25180_event_Invalid;
    enum bq25180_charging_status_type chargingStatus;
    enum bq25180_reset_shipment_mode_type operationMode;
    union bq25180_charger_state_t currentState = {.value = 0};

    // bool bRet = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (bRet == false) {
    //     return false;
    // }

    bq25180_i2c_read_register(bq25180_register_STAT0,
                              (uint8_t *) &(stat0.value));
    bq25180_i2c_read_register(bq25180_register_STAT1,
                              (uint8_t *) &(stat1.value));
    bq25180_i2c_read_register(bq25180_register_FLAG0,
                              (uint8_t *) &(flag0.value));
    bq25180_i2c_read_register(bq25180_register_SHIP_RST,
                              (uint8_t *) &(shipRst.value));
    // BQ25180_I2C_UNLOCK(&bq25180);

    chargingStatus = (enum bq25180_charging_status_type) stat0.bits.bChgStat;
    operationMode = (enum bq25180_reset_shipment_mode_type)
            shipRst.bits.bEnableShipModeAndReset;
    switch (operationMode) {
        case bq25180_reset_shipment_mode_ShipMode:
            currentState.bits.bShipmentMode = 1;
            currentState.bits.bShutdownMode = 0;
            bq25180.state.bits.bPendingShipMode = 1;
            bq25180.state.bits.bPendingShutdown = 0;
            break;
        case bq25180_reset_shipment_mode_Shutdown:
            currentState.bits.bShipmentMode = 0;
            currentState.bits.bShutdownMode = 1;
            bq25180.state.bits.bPendingShipMode = 0;
            bq25180.state.bits.bPendingShutdown = 1;
            break;
        case bq25180_reset_shipment_mode_Reset:
        case bq25180_reset_shipment_mode_DoNothing:
        default:
            currentState.bits.bShipmentMode = 0;
            currentState.bits.bShutdownMode = 0;
            bq25180.state.bits.bPendingShipMode = 0;
            bq25180.state.bits.bPendingShutdown = 0;
            break;
    }

    currentState.bits.bPowerGood = stat0.bits.bVinPgoodStat == 0 ? 0 : 1;
    currentState.bits.bButtonPressed = stat0.bits.bTSOpenStat == 0 ? 0 : 1;
    currentState.bits.bWake1 = stat1.bits.bWake1Flag == 0 ? 0 : 1;
    currentState.bits.bWake2 = stat1.bits.bWake2Flag == 0 ? 0 : 1;
    if (chargingStatus == bq25180_charging_status_NotCharging) {
        currentState.bits.bCharging = 0;
        currentState.bits.bCharged = 0;
    } else if (chargingStatus == bq25180_charging_status_ChargingDone) {
        currentState.bits.bCharging = 0;
        currentState.bits.bCharged = 1;
    } else {
        currentState.bits.bCharging = 1;
        currentState.bits.bCharged = 0;
    }

    currentState.bits.bThermalRegulation =
            stat0.bits.bThermalRegulationActiveStat == 0 ? 0 : 1;
    currentState.bits.bBatteryUVLO = (stat1.bits.bBattUVLOStatus == 0) ? 0 : 1;
    if (stat1.bits.bTSStatus == (int) bq25180_ts_status_ColdOrHot) {
        currentState.bits.bThermalWarmOrHot = 1;
    } else if (stat1.bits.bTSStatus == (int) bq25180_ts_status_Cool) {
        currentState.bits.bThermalCool = 1;
    } else if (stat1.bits.bTSStatus == (int) bq25180_ts_status_Warm) {
        currentState.bits.bThermalWarm = 1;
    } else {
        currentState.bits.bThermalNormal = 1;
    }

    currentState.bits.bSafetyTimerFault = (stat1.bits.bSafetyTimerFaultFlag == 0) ? 0
                                                                                  : 1;
    currentState.bits.bThermalSystemFault = (flag0.bits.bTSFault == 0) ? 0 : 1;
    currentState.bits.bBatteryUVLOFault = (flag0.bits.bBattUVLOFault == 0) ? 0 : 1;
    currentState.bits.bBatteryOCPFault = (flag0.bits.bBattOCPFault == 0) ? 0 : 1;
	if(state != NULL){
		state->value = currentState.value;
	}
	if(currentState.value != bq25180.charger_state.value) {
		bq25180.charger_state.value = currentState.value;
	}
    return true;
}

/*
 * \brief Enables shipment mode and resets the charger
 *
 */
bool bq25180_shipment_mode_enable(void) {
    union bq25180_SHIP_RST_register_t shipRst;
    assert(bq25180.state.bits.bConfigured != 0);
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }
    bq25180_i2c_read_register(bq25180_register_SHIP_RST,
                              (uint8_t *) &(shipRst.value));
    shipRst.bits.bEnablePush = 1;
    shipRst.bits.bPushbuttonLongPressAction =
            bq25180_long_press_action_ShipMode;
    shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_ShipMode;
    bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value);
    // BQ25180_I2C_UNLOCK(&bq25180);
    return true;
}

/*
 * \brief Disables shipment mode
 *
 */
bool bq25180_shipment_mode_disable(void) {
    union bq25180_SHIP_RST_register_t shipRst;
    assert(bq25180.state.bits.bConfigured != 0);
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }
    bq25180_i2c_read_register(bq25180_register_SHIP_RST,
                              (uint8_t *) &(shipRst.value));
    shipRst.bits.bEnablePush = 0;
    shipRst.bits.bPushbuttonLongPressAction =
            bq25180_long_press_action_DoNothing;
    shipRst.bits.bEnableShipModeAndReset =
            bq25180_reset_shipment_mode_DoNothing;
    bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value);
    // BQ25180_I2C_UNLOCK(&bq25180);
    return true;
}

/*
 * \brief Enables charger shutdown mode
 *
 */
bool bq25180_shutdown_enable(void) {
    union bq25180_SHIP_RST_register_t shipRst;
    assert(bq25180.state.bits.bConfigured != 0);
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }
    bq25180_i2c_read_register(bq25180_register_SHIP_RST,
                              (uint8_t *) &(shipRst.value));
    shipRst.bits.bEnablePush = 1;
    shipRst.bits.bPushbuttonLongPressAction =
            bq25180_long_press_action_Shutdown;
    //	shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_Shutdown;
    bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value);
    // BQ25180_I2C_UNLOCK(&bq25180);

    return true;
}

/*
 * \brief Disables charger shutdown mode.
 *
 */
bool bq25180_shutdown_disable(void) {
    union bq25180_SHIP_RST_register_t shipRst;
    assert(bq25180.state.bits.bConfigured != 0);
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }
    bq25180_i2c_read_register(bq25180_register_SHIP_RST,
                              (uint8_t *) &(shipRst.value));
    shipRst.bits.bEnablePush = 0;
    shipRst.bits.bPushbuttonLongPressAction =
            bq25180_long_press_action_DoNothing;
    shipRst.bits.bEnableShipModeAndReset =
            bq25180_reset_shipment_mode_DoNothing;
    bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value);
    // BQ25180_I2C_UNLOCK(&bq25180);
    return true;
}

/*
 * \brief Prints the charger state of the device
 */
void bq25180_print_state(void) {
    union bq25180_charger_state_t state;
    state.value = bq25180.charger_state.value;
    LOG_INF("Charger state: PG=%d Charging=%d Charged=%d Shipmode=%d "
                    "Shutdown=%d BT=%d Wake1=%d Wake2=%d ThermalReg=%d UVLO=%d "
                    "TNormal=%d TWarmHot=%d TWarm=%d TCool=%d TimerFault=%d TFault=%d "
                    "UVLOFault=%d OCPFault=%d\r\n",
                    state.bits.bPowerGood,
                    state.bits.bCharging,
                    state.bits.bCharged,
                    state.bits.bShipmentMode,
                    state.bits.bShutdownMode,
                    state.bits.bButtonPressed,
                    state.bits.bWake1,
                    state.bits.bWake2,
                    state.bits.bThermalRegulation,
                    state.bits.bBatteryUVLO,
                    state.bits.bThermalNormal,
                    state.bits.bThermalWarmOrHot,
                    state.bits.bThermalWarm,
                    state.bits.bThermalCool,
                    state.bits.bSafetyTimerFault,
                    state.bits.bThermalSystemFault,
                    state.bits.bBatteryUVLOFault,
                    state.bits.bBatteryOCPFault);
}



/*
 * \brief Issues a software-reset command to the BQ25180.
 *
 * After the reset the IC returns to its POR register defaults.
 * The function returns   true  if the command could be sent,
 *                        false otherwise (lock or I²C failure).
 */
bool bq25180_reset(struct bq25180_config_t *config)
{
    /* 1. Ensure the driver was already initialised */
    assert(bq25180.state.bits.bInitialized != 0);

    /* 2. Prepare the SHIP_RST register value
     *      – keep previously configured options
     *      – change only the ‘EnableShipModeAndReset’ field so that the
     *        device performs a reset (no ship-mode request).
     */
    union bq25180_SHIP_RST_register_t ship_rst = {
            .value = BQ25180_SHIP_RST_DEFAULT
    };

    /* 3. Take the lock, send the command, release the lock */
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }

    bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t *) &(ship_rst.value));
    ship_rst.bits.bEnablePush                  = 1; /* keep push-button */
    bq25180_i2c_write_register(bq25180_register_SHIP_RST,
                               (uint8_t)ship_rst.value);

    // BQ25180_I2C_UNLOCK(&bq25180);


    /* 4. Local state is no longer valid; mark as not configured */
    bq25180.state.bits.bConfigured = 0;

    return bq25180_config(config);
}

bool bq25180_enable_charging(bool enable) {

    /* The driver has to be initialised & configured first */
    assert(bq25180.state.bits.bInitialized != 0);
    assert(bq25180.state.bits.bConfigured != 0);

    /* Obtain exclusive access to the I²C bus */
    // bool ret = BQ25180_I2C_LOCK(&bq25180, &bq25180.i2c_config, BQ25180_I2C_TIMEOUT);
    // if (ret == false) {
    //     return false;
    // }

    /* 1. Read current ICHG_CTRL value ---------------------------------- */
    uint8_t value = 0u;
    bq25180_i2c_read_register(bq25180_register_ICHG_CTRL, &value);

    union bq25180_ICHG_CTRL_register_t ctrlReg = {.value = value};
    bool prevValue = ctrlReg.bits.bChargeDisable != 0;
    /* 2. Toggle the Charge-Disable flag if it is set -------------------- */
    if (prevValue != enable) {
        ctrlReg.bits.bChargeDisable = (enable == false) ? 1u : 0u;
        bq25180_i2c_write_register(bq25180_register_ICHG_CTRL,
                                   (uint8_t) ctrlReg.value);
    }

    // BQ25180_I2C_UNLOCK(&bq25180);
    return true;
}
