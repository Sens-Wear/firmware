/**
 * bq25180.c
 *
 * @file bq25180.c
 * @brief SenseWear BQ25180 charger driver implementation.
 * @details The driver is a board-level singleton. Its connection is derived
 *          from `DT_NODELABEL(bq25180)`, while all transfers are routed through
 *          @ref sensewear_sys_i2c. Public hardware operations own the shared
 *          bus for their complete register sequence and finish with
 *          sys_i2c_release().
 *
 * Register helpers in this file intentionally do not lock. They may only be
 * called from a high-level operation that already owns the shared bus.
 *
 * The optional GPIO interrupt path defers policy work to the system work queue.
 * It is implemented but currently not enabled by bq25180_config().
 */

#include "bq25180.h"
#include "sys_i2c.h"
#include "device_events.h"
#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bq25180, CONFIG_LOG_DEFAULT_LEVEL);
/**
 * @brief Devicetree node identifier for the board's BQ25180 instance.
 * @details Maps the `bq25180` node label to the node used for constructing the
 *          shared-I2C specification and interrupt GPIO specification.
 */
#define BQ25180_NODE DT_NODELABEL(bq25180)

/** Internal driver lifecycle flags (private to the implementation). */
union bq25180_state_t {
	unsigned int value; /**< Complete packed lifecycle state. */
	/** Individual internal lifecycle flags. */
	struct bq25180_state_bits {
		unsigned int bInitialized : 1;	   /**< Probe completed successfully. */
		unsigned int bConfigured : 1;	   /**< Register configuration was applied. */
		unsigned int bProbed : 1;		   /**< Device presence probe has been attempted. */
		unsigned int bDeviceFound : 1;	   /**< Device presence was successfully detected. */
		unsigned int bPendingShutdown : 1; /**< Shutdown request is pending. */
		unsigned int bPendingShipMode : 1; /**< Ship-mode request is pending. */
	} bits;
};

/**
 * @brief Internal singleton driver context.
 * @details Stores all private runtime data used by the BQ25180 driver,
 *          including bus bindings, interrupt wiring, cached configuration,
 *          lifecycle flags, latest charger status, and deferred IRQ handling
 *          objects. This context is private to this implementation unit.
 */
static struct bq25180_t {
	/** Identifier used when posting to the device event manager. */
	uint32_t device_id;
	/** Shared-I2C connection and ownership token derived from devicetree. */
	struct sys_i2c_dt_spec device;
	/** Interrupt GPIO specification derived from devicetree. */
	struct gpio_dt_spec irq_gpio;
	/** Kill GPIO specification derived from devicetree. */
	struct gpio_dt_spec kill_gpio;
	/** Last successfully requested configuration. */
	struct bq25180_config_t config;
	/** Driver lifecycle state. */
	union bq25180_state_t state;
	/** Most recently decoded charger state. */
	union bq25180_charger_state_t charger_state;
	/** GPIO callback instance registered for charger interrupt events. */
	struct gpio_callback irq_cb;
	/** Work item used to defer interrupt processing from ISR context. */
	struct k_work irq_work;
} bq25180 = {
	.device = SYS_I2C_DT_SPEC_GET(BQ25180_NODE),
	.irq_gpio = GPIO_DT_SPEC_GET(BQ25180_NODE, int_gpios),
	.kill_gpio = GPIO_DT_SPEC_GET(BQ25180_NODE, kill_gpios),
	/* All remaining members (config, state, charger_state, irq_cb, irq_work)
	 * are zero-initialised by static storage duration. */
};

/** Acquire shared-I2C ownership for one high-level charger operation. */
static inline bool bq25180_bus_lock(void) {
	int ret = sys_i2c_lock(&bq25180.device, K_MSEC(BQ25180_I2C_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("Failed to lock SYS_I2C (%d)", ret);
		return false;
	}

	return true;
}

/**
 * Complete a high-level operation and fully release nested bus ownership.
 *
 * @param operation_succeeded Result of the register operation.
 * @retval true The operation and ownership release both succeeded.
 * @retval false The operation failed or ownership could not be released.
 */
static inline bool bq25180_bus_unlock(void) {
	int ret = sys_i2c_release(&bq25180.device);

	if (ret != 0) {
		LOG_ERR("Failed to release SYS_I2C ownership (%d)", ret);
		return false;
	}

	return true;
}

/** Write one BQ25180 register. The caller must own the shared bus. */
static inline int bq25180_i2c_write_register(enum bq25180_register_type address, uint8_t value) {
	uint8_t tx[] = {(uint8_t) address, value};

	return sys_i2c_write(&bq25180.device, tx, sizeof(tx));
}

/** Read one BQ25180 register. The caller must own the shared bus. */
static inline int bq25180_i2c_read_register(enum bq25180_register_type address, uint8_t* value) {
	uint8_t reg = (uint8_t) address;

	return sys_i2c_write_read(&bq25180.device, &reg, sizeof(reg), value, sizeof(*value));
}

/** Probe for an I2C response by reading MASK_ID. The caller must own the bus. */
static bool bq25180_probe(void) {
	uint8_t mask_id = 0xffu;
	int ret = bq25180_i2c_read_register(bq25180_register_MASK_ID, &mask_id);

	if (ret != 0) {
		LOG_WRN("BQ25180 not detected on I2C bus (%d)", ret);
		return false;
	}
	bq25180.state.bits.bProbed = 1;
	union bq25180_MASK_ID_register_t mask_id_reg = {.value = mask_id};
	if (mask_id_reg.bits.bDeviceID == BQ25180_DEVICE_ID) {
		LOG_INF("BQ25180 detected on I2C bus");
		bq25180.state.bits.bDeviceFound = 1;
	} else {
		LOG_WRN("BQ25180 probe failed: unexpected MASK_ID value 0x%02x", mask_id);
		bq25180.state.bits.bDeviceFound = 0;
	}
	return true;
}

/** Deferred interrupt policy handler. Currently dormant until IRQ setup is enabled. */
static void bq25180_irq_work_fn(struct k_work* work) {
	ARG_UNUSED(work);
	union bq25180_charger_state_t state = bq25180.charger_state;
	union bq25180_charger_state_t newState;
	// lets get the new state. How we handle the charging depends on the AC power
	// and the charging status of the battery.
	if (!bq25180_update_state(&newState)) {
		LOG_WRN("BQ25180 state update failed");
		return;
	}

	if (state.bits.bPowerGood != newState.bits.bPowerGood) {
		// when they are not equal, we have to check how to handle the change
		if (state.bits.bPowerGood && !state.bits.bPowerGood) {
			// when the charger indicates power good
			if (!state.bits.bCharged) {
				bq25180_enable_charging(true);
				LOG_INF("BQ25180 VIN detected, charging enabled");
			} else {
				bq25180_enable_charging(false);
				LOG_INF("BQ25180 VIN detected, battery full. Charging disabled!");
			}
		} else {
			bq25180_enable_charging(false);
			LOG_INF("BQ25180 VIN removed, charging disabled");
		}
	}

	// we must check what has changed and generate events accordingly.
	union bq25180_charger_state_t changed = {.value = state.value ^ newState.value};
	if (changed.bits.bPowerGood != 0) {
		enum bq25180_event_type eventType = newState.bits.bPowerGood ? bq25180_event_Plugged
																	 : bq25180_event_Unplugged;
		// power good, we can generate a charger connected event.
		device_event_post(bq25180.device_id, eventType, 0, NULL, K_MSEC(BQ25180_I2C_TIMEOUT));
	}
	if (changed.bits.bCharged != 0) {
		enum bq25180_event_type eventType = newState.bits.bCharged ? bq25180_event_ChargingDone
																   : bq25180_event_Charging;
		// battery is fully charged, we can generate a battery full event.
		device_event_post(bq25180.device_id, eventType, 0, NULL, K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bBatteryOCPFault != 0 && newState.bits.bBatteryOCPFault != 0) {
		// battery overcurrent fault, we can generate a battery OCP event.
		device_event_post(bq25180.device_id,
						  bq25180_event_BatteryOverCurrentProtectionFault,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bBatteryUVLOFault != 0 && newState.bits.bBatteryUVLOFault != 0) {
		// battery UVLO fault, we can generate a battery UVLO event.
		device_event_post(bq25180.device_id,
						  bq25180_event_BatteryUndervoltageLockoutFault,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bBatteryUVLO != 0 && newState.bits.bBatteryUVLO != 0) {
		// battery UVLO status active, we can generate a battery UVLO status event.
		device_event_post(bq25180.device_id,
						  bq25180_event_BatteryUnderVoltageLockOut,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bThermalRegulation != 0 && newState.bits.bThermalRegulation != 0) {
		// thermal regulation active, we can generate a thermal regulation event.
		device_event_post(bq25180.device_id,
						  bq25180_event_ThermalRegulation,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bSafetyTimerFault != 0 && newState.bits.bSafetyTimerFault != 0) {
		// safety timer fault, we can generate a safety timer expired event.
		device_event_post(bq25180.device_id,
						  bq25180_event_SafetyTimerExpired,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bThermalSystemFault != 0 && newState.bits.bThermalSystemFault != 0) {
		// thermal system fault, we can generate a thermal system fault event.
		device_event_post(bq25180.device_id,
						  bq25180_event_ThermalSystemFault,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bWake1 != 0 && newState.bits.bWake1 != 0) {
		// WAKE1 event detected, we can generate a WAKE1 event.
		device_event_post(bq25180.device_id,
						  bq25180_event_Wake1,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}
	if (changed.bits.bWake2 != 0 && newState.bits.bWake2 != 0) {
		// WAKE2 event detected, we can generate a WAKE2 event.
		device_event_post(bq25180.device_id,
						  bq25180_event_Wake2,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}

	if (changed.bits.bButtonPressed != 0 && newState.bits.bButtonPressed != 0) {
		// button activity detected, we can generate a button pressed event.
		device_event_post(bq25180.device_id,
						  bq25180_event_ButtonPressed,
						  0,
						  NULL,
						  K_MSEC(BQ25180_I2C_TIMEOUT));
	}
	bq25180.charger_state = newState;
}

/** GPIO ISR that schedules charger policy work outside interrupt context. */
static void bq25180_irq_callback(const struct device* dev,
								 struct gpio_callback* cb,
								 uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_submit(&bq25180.irq_work);
}

/** Configure the active-low charger interrupt and its deferred work item. */
static int bq25180_irq_init(void) {
	if (bq25180.state.bits.bInitialized == 0) {
		int ret;
		// configure the interrupt with open drain configuration
		// since we have a pull-up resistor on the board, we don't need to configure
		// it to have internal pull-ups enabled.
		if (!device_is_ready(bq25180.irq_gpio.port)) {
			LOG_WRN("BQ25180 interrupt GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&bq25180.irq_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("BQ25180 interrupt pin config failed (%d)", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&bq25180.irq_gpio, GPIO_INT_EDGE_FALLING);
		if (ret) {
			LOG_ERR("BQ25180 interrupt config failed (%d)", ret);
			return ret;
		}
		// configure the irq callback
		gpio_init_callback(&bq25180.irq_cb, bq25180_irq_callback, BIT(bq25180.irq_gpio.pin));
		ret = gpio_add_callback(bq25180.irq_gpio.port, &bq25180.irq_cb);
		if (ret) {
			LOG_ERR("BQ25180 interrupt callback add failed (%d)", ret);
			return ret;
		}
		// configure the deferred work item for the interrupt callback
		k_work_init(&bq25180.irq_work, bq25180_irq_work_fn);
		return 0;
	}
	return 0;
}
/** Initialize the kill GPIO for the BQ25180. */
static int bq25180_kill_init(void) {
	if (bq25180.state.bits.bInitialized == 0) {
		int ret;
		if (!device_is_ready(bq25180.kill_gpio.port)) {
			LOG_WRN("BQ25180 kill GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&bq25180.kill_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			LOG_ERR("BQ25180 kill pin config failed (%d)", ret);
			return ret;
		}
		return 0;
	}
	return 0;
}
/** Set the kill GPIO for the BQ25180. */
static inline void bq25180_kill(bool enable) {
	if (enable) {
		gpio_pin_set_dt(&bq25180.kill_gpio, GPIO_OUTPUT_ACTIVE);
	} else {
		gpio_pin_set_dt(&bq25180.kill_gpio, GPIO_OUTPUT_INACTIVE);
	}
}

/* Check if the BQ25180 is available. */
bool bq25180_is_ready(void) {
	assert(bq25180.state.bits.bInitialized != 0);
	if (bq25180.state.bits.bProbed == 0) {
		bq25180_bus_lock();
		bq25180_probe();
		bq25180_bus_unlock();
	}
	return bq25180.state.bits.bProbed != 0 && bq25180.state.bits.bDeviceFound != 0;
}

/*
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values loaded to the registers.
 *
 * \param config The destination configuraiton
 */
void bq25180_get_default_config(struct bq25180_config_t* config) {
	union bq25180_CHARGECTRL0_register_t chrgctrl0 = {.value = BQ25180_CHARGECTRL0_DEFAULT};
	union bq25180_CHARGECTRL1_register_t chrgctrl1 = {.value = BQ25180_CHARGECTRL1_DEFAULT};
	union bq25180_IC_CTRL_register_t ic_ctrl = {.value = BQ25180_IC_CTRL_DEFAULT};
	union bq25180_TMR_ILIM_register_t tmr_ilim = {.value = BQ25180_TMR_ILIM_DEFAULT};

	config->charge_voltage = BQ25180_VBAT_OUT_DEFAULT;
	config->charge_current = BQ25180_ICHG_OUT_DEFAULT;

	config->termination_current =
		(enum bq25180_termination_current_type) chrgctrl0.bits.bTerminationCurrent;
	config->precharge_current =
		(enum bq25180_precharge_current_type) chrgctrl0.bits.bPrechargeCurrent;
	config->vin_dpm_level = (enum bq25180_VINDPM_level_type) chrgctrl0.bits.bVINDPMLvel;

	config->battery_ocp_limit = (enum bq25180_battery_discharge_current_limit_type)
									chrgctrl1.bits.bBatteryDischargeCurrentLimit;
	config->battery_uvlo =
		(enum bq25180_battery_UVLO_threshold_type) chrgctrl1.bits.bBatteryUVLOThreshold;

	config->precharge_threshold =
		(enum bq25180_precharge_voltage_threshold_type) ic_ctrl.bits.bPrechargeVoltageThreshold;
	config->recharge_voltage_threshold =
		(enum bq25180_recharge_voltage_threshold_type) ic_ctrl.bits.bRechargeVoltage;

	config->input_current =
		(enum bq25180_input_current_limit_type) tmr_ilim.bits.bInputCurrentLimit;
	config->long_press_duration =
		(enum bq25180_pb_long_press_duration_type) tmr_ilim.bits.bLongPressDuration;
}

/*
 * \brief Returns the default configuration of BQ25180. This configuration
 * is composed of values optimized for LiPo battery of 450mA capacity charged
 * over USB type C.
 *
 * \param config The destination configuraiton
 */
void bq25180_get_default_lipo_usb_charger_config(struct bq25180_config_t* config) {
	config->charge_voltage = BQ25180_DEFAULT_BATTERY_VOLTAGE;
	config->charge_current = BQ25180_DEFAULT_BATTERY_CHARGE_CURRENT;

	config->termination_current = BQ25180_DEFAULT_TERMINATION_CURRENT;
	config->precharge_current = BQ25180_DEFAULT_PRECHARGE_CURRENT;
	config->vin_dpm_level = BQ25180_DEFAULT_VINDPM_LEVEL;

	config->battery_ocp_limit =
		bq25180_battery_discharge_current_limit_Disabled; // bq25180_battery_discharge_current_limit_1500mA;
														  // //BQ25180_DEFAULT_BATTERY_DISCHARGE_CURRENT_LIMIT;
	config->battery_uvlo = bq25180_battery_UVLO_threshold_2V0; // BQ25180_DEFAULT_UVLO_THRESHOLD;

	config->precharge_threshold = BQ25180_DEFAULT_PRECHARGE_VOLTAGE_THRESHOLD;
	config->recharge_voltage_threshold = BQ25180_DEFAULT_RECHARGE_VOLTAGE_THRESHOLD;

	config->input_current = BQ25180_DEFAULT_INPUT_CURRENT_LIMIT;
	config->long_press_duration = bq25180_pb_long_press_duration_5s;
}

/* Initialize the BQ25180 charger. */
bool bq25180_init(uint32_t device_id) {
	if (bq25180.state.bits.bInitialized != 0) {
		LOG_WRN("BQ25180 already initialized!");
		return true;
	}

	bq25180.device_id = device_id;

	// lets check whether the i2c bus is ready
	// before we try to acquire the bus lock.
	if (!sys_i2c_is_ready(&bq25180.device)) {
		LOG_ERR("SYS_I2C bus not ready");
		return false;
	}
	bool deviceReady = bq25180_is_ready();
	if (!deviceReady) {
		LOG_ERR("BQ25180 device not found during initialization");
		return false;
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
bool bq25180_config(struct bq25180_config_t* config) {
	assert(bq25180.state.bits.bInitialized != 0);
	// for an initialized, we can apply the configuration.
	union bq25180_VBAT_CTRL_register_t vbatCtrl = {.value = BQ25180_VBAT_CTRL_DEFAULT};
	union bq25180_ICHG_CTRL_register_t ichgCtrl = {.value = BQ25180_ICHG_CTRL_DEFAULT};
	union bq25180_CHARGECTRL0_register_t chrgctrl0 = {.value = BQ25180_CHARGECTRL0_DEFAULT};
	union bq25180_CHARGECTRL1_register_t chrgctrl1 = {.value = BQ25180_CHARGECTRL1_DEFAULT};
	union bq25180_IC_CTRL_register_t icCtrl = {.value = BQ25180_IC_CTRL_DEFAULT};
	union bq25180_TMR_ILIM_register_t tmrIlim = {.value = BQ25180_TMR_ILIM_DEFAULT};
	union bq25180_SHIP_RST_register_t shipRst = {.value = BQ25180_SHIP_RST_DEFAULT};
	union bq25180_SYS_REG_register_t sysReg = {.value = 0 /*BQ25180_SYS_REG_DEFAULT*/};
	union bq25180_TS_CONTROL_register_t tsControlReg = {.value = BQ25180_TS_CONTROL_DEFAULT};
	//	union bq25180_MASK_ID_register_t maskId = {.value = BQ25180_MASK_ID_DEFAULT};

	if (!bq25180_is_ready() || (bq25180.state.bits.bInitialized == 0)) {
		return false;
	}
	if (config == NULL) {
		bq25180_get_default_lipo_usb_charger_config(&(bq25180.config));
	} else {
		memcpy(&(bq25180.config), config, sizeof(struct bq25180_config_t));
	}

	// load the configuration to the registers
	vbatCtrl.bits.bVBattReg = BQ25180_VBAT_REG_VAL_FROM_OUT(bq25180.config.charge_voltage);

	ichgCtrl.bits.bICHG = BQ25180_ICHG_OUT_TO_VAL(bq25180.config.charge_current) & 0x7f;
	ichgCtrl.bits.bChargeDisable = 0;

	chrgctrl0.bits.bPrechargeCurrent = bq25180.config.precharge_current;
	chrgctrl0.bits.bTerminationCurrent = bq25180.config.termination_current;
	chrgctrl0.bits.bVINDPMLvel = bq25180.config.vin_dpm_level;

	chrgctrl1.bits.bBatteryDischargeCurrentLimit = bq25180.config.battery_ocp_limit;
	chrgctrl1.bits.bBatteryUVLOThreshold = bq25180.config.battery_uvlo;
	chrgctrl1.bits.bMaskVINDPMInterrupt = 1; // these interrupts can be masked.

	icCtrl.bits.bPrechargeVoltageThreshold = bq25180.config.precharge_threshold;
	icCtrl.bits.bRechargeVoltage = bq25180.config.recharge_voltage_threshold;
	icCtrl.bits.bSafetyFastChargeTimer = (int) bq25180_fast_charge_time_3h;
	icCtrl.bits.bTSAutoFunctionEnable = 0;
	tmrIlim.bits.bInputCurrentLimit = bq25180.config.input_current;
	tmrIlim.bits.bLongPressDuration = bq25180.config.long_press_duration;
	tmrIlim.bits.bHardwareResetCondition = 1; // long press and vin

	shipRst.bits.bEnablePush = 1;
	shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_DoNothing;
	shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_DoNothing;

	tsControlReg.bits.bThermalSystemColdThreshold = bq25180_cold_threshold_m3;
	sysReg.bits.bSYSPowerMode = bq25180_sys_power_mode_VIN_or_VBAT;

	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret =
		(bq25180_i2c_write_register(bq25180_register_VBAT_CTRL, (uint8_t) vbatCtrl.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_ICHG_CTRL, (uint8_t) ichgCtrl.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_CHARGECTRL0, chrgctrl0.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_CHARGECTRL1, chrgctrl1.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_IC_CTRL, icCtrl.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_TMR_ILIM, tmrIlim.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_TS_CONTROL, tsControlReg.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_SYS_REG, sysReg.value) == 0);

	ret &= bq25180_bus_unlock();
	if (!ret) {
		return false;
	}
	// now everything is configured, and we can set up the interrupt if it is not set up yet.
	if (bq25180.state.bits.bConfigured == 0) {
		int iRet = bq25180_irq_init();
		if (iRet != 0) {
			LOG_ERR("BQ25180 IRQ initialization failed (%d)", iRet);
			return false;
		}
		iRet = bq25180_kill_init();
		if (iRet != 0) {
			LOG_ERR("BQ25180 kill GPIO initialization failed (%d)", iRet);
			return false;
		}
	}
	bq25180.state.bits.bConfigured = 1;

	return true;
}

/*
 * \brief Handles BQ25180 interrupt request
 *
 * \return The detected event as a result of handled IRQ
 */
bool bq25180_update_state(union bq25180_charger_state_t* state) {
	assert(bq25180.state.bits.bConfigured != 0);

	union bq25180_STAT0_register_t stat0;
	union bq25180_STAT1_register_t stat1;
	union bq25180_FLAG0_register_t flag0;
	union bq25180_SHIP_RST_register_t shipRst;

	//	enum bq25180_event_type event = bq25180_event_Invalid;
	enum bq25180_charging_status_type chargingStatus;
	enum bq25180_reset_shipment_mode_type operationMode;
	union bq25180_charger_state_t currentState = {.value = 0};

	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret =
		(bq25180_i2c_read_register(bq25180_register_STAT0, (uint8_t*) &(stat0.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_STAT1, (uint8_t*) &(stat1.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_FLAG0, (uint8_t*) &(flag0.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &(shipRst.value)) == 0);

	ret &= bq25180_bus_unlock();
	if (!ret) {
		if (state != NULL) {
			state->value = 0;
		}
		return false;
	}

	chargingStatus = (enum bq25180_charging_status_type) stat0.bits.bChgStat;
	operationMode = (enum bq25180_reset_shipment_mode_type) shipRst.bits.bEnableShipModeAndReset;
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

	currentState.bits.bThermalRegulation = stat0.bits.bThermalRegulationActiveStat == 0 ? 0 : 1;
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

	currentState.bits.bSafetyTimerFault = (stat1.bits.bSafetyTimerFaultFlag == 0) ? 0 : 1;
	currentState.bits.bThermalSystemFault = (flag0.bits.bTSFault == 0) ? 0 : 1;
	currentState.bits.bBatteryUVLOFault = (flag0.bits.bBattUVLOFault == 0) ? 0 : 1;
	currentState.bits.bBatteryOCPFault = (flag0.bits.bBattOCPFault == 0) ? 0 : 1;
	if (state != NULL) {
		state->value = currentState.value;
	}
	if (currentState.value != bq25180.charger_state.value) {
		bq25180.charger_state.value = currentState.value;
	}
	return true;
}

/*
 * \brief Enables shipment mode and resets the charger
 *
 */
bool bq25180_shipment_mode_enable(void) {
	assert(bq25180.state.bits.bConfigured != 0);
	union bq25180_SHIP_RST_register_t shipRst;
	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret = bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &shipRst.value) == 0;
	if (ret) {
		shipRst.bits.bEnablePush = 1;
		shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_ShipMode;
		shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_ShipMode;
		ret = bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value) == 0;
	}

	ret &= bq25180_bus_unlock();
	return ret;
}

/*
 * \brief Disables shipment mode
 *
 */
bool bq25180_shipment_mode_disable(void) {
	assert(bq25180.state.bits.bConfigured != 0);
	union bq25180_SHIP_RST_register_t shipRst;

	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret = bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &shipRst.value) == 0;
	if (ret) {
		shipRst.bits.bEnablePush = 0;
		shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_DoNothing;
		shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_DoNothing;
		ret = bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value) == 0;
	}
	ret &= bq25180_bus_unlock();
	return ret;
}

/*
 * \brief Enables charger shutdown mode
 *
 */
bool bq25180_shutdown_enable(void) {
	assert(bq25180.state.bits.bConfigured != 0);
	union bq25180_SHIP_RST_register_t shipRst;
	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret = bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &shipRst.value) == 0;
	if (ret) {
		shipRst.bits.bEnablePush = 1;
		shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_Shutdown;
		ret = bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value) == 0;
	}

	ret &= bq25180_bus_unlock();
	return ret;
}

/*
 * \brief Disables charger shutdown mode.
 *
 */
bool bq25180_shutdown_disable(void) {
	assert(bq25180.state.bits.bConfigured != 0);
	union bq25180_SHIP_RST_register_t shipRst;
	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret = bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &shipRst.value) == 0;
	if (ret) {
		shipRst.bits.bEnablePush = 0;
		shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_DoNothing;
		shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_DoNothing;
		ret = bq25180_i2c_write_register(bq25180_register_SHIP_RST, shipRst.value) == 0;
	}

	ret &= bq25180_bus_unlock();
	return ret;
}

/*
 * \brief Prints the charger state of the device
 */
void bq25180_print_state(void) {
	assert(bq25180.state.bits.bConfigured != 0);

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
bool bq25180_reset(struct bq25180_config_t* config) {
	/* 1. Ensure the driver was already initialised */
	assert(bq25180.state.bits.bInitialized != 0);
	/* 2. Prepare the SHIP_RST register value
	 *      – keep previously configured options
	 *      – change only the ‘EnableShipModeAndReset’ field so that the
	 *        device performs a reset (no ship-mode request).
	 */
	union bq25180_SHIP_RST_register_t ship_rst = {.value = BQ25180_SHIP_RST_DEFAULT};

	/* 3. Take the lock, send the command, release the lock */
	if (!bq25180_bus_lock()) {
		return false;
	}

	bool ret = bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &ship_rst.value) ==
			   0;
	if (ret) {
		ship_rst.bits.bEnablePush = 1;
		ret = bq25180_i2c_write_register(bq25180_register_SHIP_RST, (uint8_t) ship_rst.value) == 0;
	}

	ret &= bq25180_bus_unlock();
	if (!ret) {
		return false;
	}

	/* 4. Local configuration is no longer valid; mark as not configured */
	bq25180.state.bits.bConfigured = 0;
	// configure if a valid configuration is provided
	if (config != NULL) {
		return bq25180_config(config);
	}
	return true;
}

bool bq25180_enable_charging(bool enable) {
	assert(bq25180.state.bits.bConfigured != 0);

	/* Obtain exclusive access to the I²C bus */
	if (!bq25180_bus_lock()) {
		return false;
	}

	/* 1. Read current ICHG_CTRL value ---------------------------------- */
	uint8_t value = 0u;
	bool ret = bq25180_i2c_read_register(bq25180_register_ICHG_CTRL, &value) == 0;
	if (ret) {
		union bq25180_ICHG_CTRL_register_t ctrlReg = {.value = value};
		bool charge_disabled = ctrlReg.bits.bChargeDisable != 0;

		if (charge_disabled != !enable) {
			ctrlReg.bits.bChargeDisable = enable ? 0u : 1u;
			ret = bq25180_i2c_write_register(bq25180_register_ICHG_CTRL, (uint8_t) ctrlReg.value) ==
				  0;
		}
	}
	ret &= bq25180_bus_unlock();
	return ret;
}
