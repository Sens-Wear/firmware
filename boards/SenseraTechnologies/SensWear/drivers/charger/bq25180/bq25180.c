/**
 * bq25180.c
 *
 * @file bq25180.c
 * @brief SensWear BQ25180 charger driver implementation.
 * @details The driver is a board-level singleton. Its connection is derived
 *          from `DT_NODELABEL(bq25180)`, while all transfers are routed through
 *          @ref senswear_sys_i2c. Public hardware operations own the shared
 *          bus for their complete register sequence and finish with
 *          sys_i2c_release().
 *
 * Register helpers in this file intentionally do not lock. They may only be
 * called from a high-level operation that already owns the shared bus.
 *
 * The GPIO interrupt path records the most recent interrupt timestamp and
 * posts an ISR-safe notification event. Timestamps are sampled from
 * `SYS_CLOCK_REALTIME` through `rtc_get_timestamp_us()`, expressed as Unix
 * epoch microseconds since `1970-01-01 00:00:00 UTC`, and truncated to whole
 * microseconds. Callers should run bq25180_update_state() from thread context
 * to read the IC, refresh the cached charger state snapshot, and process the
 * charger state machine.
 */

#include "bq25180.h"
#include "rtc.h"
#include "sys_i2c.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"
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
		unsigned int bIrqConfigured : 1;   /**< Interrupt GPIO callback was registered. */
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
 *          lifecycle flags, the latest decoded charger status, the most recent
 *          interrupt timestamp, and deferred IRQ handling objects. This
 *          context is private to this implementation unit.
 */
static struct bq25180_t {
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
	union bq25180_charger_state charger_state;
	/** GPIO callback instance registered for charger interrupt events. */
	struct gpio_callback irq_cb;
	/**< Timestamp of the last interrupt event. */
	time_t last_irq_time;
} bq25180 = {
	.device = SYS_I2C_DT_SPEC_GET(BQ25180_NODE),
	.irq_gpio = GPIO_DT_SPEC_GET(BQ25180_NODE, int_gpios),
	.kill_gpio = GPIO_DT_SPEC_GET(BQ25180_NODE, kill_gpios),
	.last_irq_time = -1,
	/* All remaining members (config, state, charger_state, irq_cb, last_irq_time)
	 * are zero-initialised by static storage duration. */
};

static const char* const bq25180_event_names[bq25180_event_Count] = {
	[bq25180_event_Plugged] = "Plugged",
	[bq25180_event_Unplugged] = "Unplugged",
	[bq25180_event_Charging] = "Charging",
	[bq25180_event_ChargingDone] = "ChargingDone",
	[bq25180_event_ThermalRegulation] = "ThermalRegulation",
	[bq25180_event_VIN_OverVoltageProtection] = "VIN_OverVoltageProtection",
	[bq25180_event_BatteryUnderVoltageLockOut] = "BatteryUnderVoltageLockOut",
	[bq25180_event_SafetyTimerExpired] = "SafetyTimerExpired",
	[bq25180_event_ThermalSystemFault] = "ThermalSystemFault",
	[bq25180_event_BatteryUndervoltageLockoutFault] = "BatteryUndervoltageLockoutFault",
	[bq25180_event_BatteryOverCurrentProtectionFault] = "BatteryOverCurrentProtectionFault",
	[bq25180_event_Wake1] = "Wake1",
	[bq25180_event_Wake2] = "Wake2",
	[bq25180_event_ButtonPressed] = "ButtonPressed",
	[bq25180_event_Irq] = "InterruptDetected",
};

const char* bq25180_event_name(uint32_t event_id) {
	if (event_id >= (uint32_t) bq25180_event_Count || bq25180_event_names[event_id] == NULL) {
		return "Unknown";
	}

	return bq25180_event_names[event_id];
}

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

static inline void bq25180_post_event(enum bq25180_event_type event) {
	(void) device_driver_event_post(BQ25180_DEVICE_DTS_ID,
									(uint32_t) event,
									0,
									(uintptr_t) NULL,
									K_MSEC(BQ25180_I2C_TIMEOUT));
}

static inline void bq25180_post_event_isr(enum bq25180_event_type event, uint32_t vParam) {
	(void) device_driver_event_post_isr(BQ25180_DEVICE_DTS_ID,
										(uint32_t) event,
										vParam,
										(uintptr_t) NULL);
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
	if (mask_id_reg.bits.bDeviceID == BQ25180_HW_DEVICE_ID) {
		LOG_INF("BQ25180 detected on I2C bus");
		bq25180.state.bits.bDeviceFound = 1;
	} else {
		LOG_WRN("BQ25180 probe failed: unexpected MASK_ID value 0x%02x", mask_id);
		bq25180.state.bits.bDeviceFound = 0;
	}
	return true;
}

static void bq25180_process_state_change(union bq25180_charger_state state,
										 union bq25180_charger_state newState) {
	if (state.bits.bPowerGood != newState.bits.bPowerGood) {
		// power-good changed; decide how to handle it from the new state
		if (newState.bits.bPowerGood) {
			// VIN detected: keep charging enabled unconditionally, even when the
			// battery already reads full. The BQ25180 terminates and auto-recharges
			// internally, so leaving CE asserted tops the battery back up below the
			// recharge threshold rather than overcharging.
			bq25180_enable_charging(true);
			LOG_INF("BQ25180 VIN detected, charging enabled");
		} else {
			// VIN removed: leave CE asserted so charging resumes immediately when
			// power returns. Without input power the charger draws no charge current
			// regardless, so this never stops charging on its own.
			LOG_INF("BQ25180 VIN removed; charging left enabled for auto-resume");
		}
	}

	// we must check what has changed and generate events accordingly.
	union bq25180_charger_state changed = {.value = state.value ^ newState.value};
	if (changed.bits.bPowerGood != 0) {
		enum bq25180_event_type eventType = newState.bits.bPowerGood ? bq25180_event_Plugged
																	 : bq25180_event_Unplugged;
		// power good, we can generate a charger connected event.
		bq25180_post_event(eventType);
	}
	if (changed.bits.bCharged != 0) {
		enum bq25180_event_type eventType = newState.bits.bCharged ? bq25180_event_ChargingDone
																   : bq25180_event_Charging;
		// battery is fully charged, we can generate a battery full event.
		bq25180_post_event(eventType);
	}

	if (changed.bits.bBatteryOCPFault != 0 && newState.bits.bBatteryOCPFault != 0) {
		// battery overcurrent fault, we can generate a battery OCP event.
		bq25180_post_event(bq25180_event_BatteryOverCurrentProtectionFault);
	}

	if (changed.bits.bBatteryUVLOFault != 0 && newState.bits.bBatteryUVLOFault != 0) {
		// battery UVLO fault, we can generate a battery UVLO event.
		bq25180_post_event(bq25180_event_BatteryUndervoltageLockoutFault);
	}

	if (changed.bits.bBatteryUVLO != 0 && newState.bits.bBatteryUVLO != 0) {
		// battery UVLO status active, we can generate a battery UVLO status event.
		bq25180_post_event(bq25180_event_BatteryUnderVoltageLockOut);
	}

	if (changed.bits.bThermalRegulation != 0 && newState.bits.bThermalRegulation != 0) {
		// thermal regulation active, we can generate a thermal regulation event.
		bq25180_post_event(bq25180_event_ThermalRegulation);
	}

	if (changed.bits.bSafetyTimerFault != 0 && newState.bits.bSafetyTimerFault != 0) {
		// safety timer fault, we can generate a safety timer expired event.
		bq25180_post_event(bq25180_event_SafetyTimerExpired);
	}

	if (changed.bits.bThermalSystemFault != 0 && newState.bits.bThermalSystemFault != 0) {
		// thermal system fault, we can generate a thermal system fault event.
		bq25180_post_event(bq25180_event_ThermalSystemFault);
	}

	if (changed.bits.bWake1 != 0 && newState.bits.bWake1 != 0) {
		// WAKE1 event detected, we can generate a WAKE1 event.
		bq25180_post_event(bq25180_event_Wake1);
	}
	if (changed.bits.bWake2 != 0 && newState.bits.bWake2 != 0) {
		// WAKE2 event detected, we can generate a WAKE2 event.
		bq25180_post_event(bq25180_event_Wake2);
	}

	if (changed.bits.bButtonPressed != 0 && newState.bits.bButtonPressed != 0) {
		// button activity detected, we can generate a button pressed event.
		bq25180_post_event(bq25180_event_ButtonPressed);
	}
	bq25180.charger_state = newState;
}

/** GPIO ISR that posts the charger interrupt notification event. */
static void bq25180_irq_callback(const struct device* dev,
								 struct gpio_callback* cb,
								 uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	bq25180.last_irq_time = rtc_get_timestamp_us();
	bq25180_post_event_isr(bq25180_event_Irq, pins);
}

/** Configure the active-low charger interrupt. */
static int bq25180_irq_init(void) {
	if (bq25180.state.bits.bIrqConfigured != 0) {
		return 0;
	}

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

	bq25180.state.bits.bIrqConfigured = 1;
	return 0;
}

/** Set the kill GPIO for the BQ25180. */
static inline void bq25180_kill(bool enable) {
	/* kill-gpios is active-low and open-drain: logical 1 asserts PWR_KILL (pulls
	 * the shared TS/MR line low to request shutdown), logical 0 releases it so R4
	 * holds it high and power stays on. gpio_pin_set_dt() takes a logical level. */
	gpio_pin_set_dt(&bq25180.kill_gpio, enable ? 1 : 0);
}

/** Initialize the kill GPIO for the BQ25180. */
static int bq25180_kill_init(void) {
	if (bq25180.state.bits.bInitialized == 0) {
		int ret;
		if (!device_is_ready(bq25180.kill_gpio.port)) {
			LOG_WRN("BQ25180 kill GPIO not ready");
			return -ENODEV;
		}

		// Initialize released (inactive): the open-drain line floats high via R4,
		// keeping power on. Initializing active would pull PWR_KILL low and request
		// shutdown. The DT flags (active-low, open-drain) are merged in here.
		ret = gpio_pin_configure_dt(&bq25180.kill_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			LOG_ERR("BQ25180 kill pin config failed (%d)", ret);
			return ret;
		}
		// reaffirm the released (power-on) state
		bq25180_kill(false);
		return 0;
	}
	return 0;
}

/* Check if the BQ25180 is available. */
bool bq25180_is_ready(void) {
	if (bq25180.state.bits.bInitialized == 0) {
		return false;
	}
	if (bq25180.state.bits.bProbed == 0) {
		bq25180_bus_lock();
		bq25180_probe();
		bq25180_bus_unlock();
	}
	return bq25180.state.bits.bProbed != 0 && bq25180.state.bits.bDeviceFound != 0 &&
		   bq25180.state.bits.bConfigured != 0;
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
bool bq25180_init(void) {
	if (bq25180.state.bits.bInitialized != 0) {
		LOG_WRN("BQ25180 already initialized!");
		return true;
	}

	// lets check whether the i2c bus is ready
	// before we try to acquire the bus lock.
	if (!sys_i2c_is_ready(&bq25180.device)) {
		LOG_ERR("SYS_I2C bus not ready");
		return false;
	}
	// we must check whether the device is accessible
	bq25180_bus_lock();
	bq25180_probe();
	bq25180_bus_unlock();
	if (bq25180.state.bits.bProbed == 0 || bq25180.state.bits.bDeviceFound == 0) {
		LOG_ERR("BQ25180 device not found during initialization");
		return false;
	}
	// lets initialize the GPIOs
	int ret = bq25180_irq_init();
	if (ret != 0) {
		LOG_ERR("BQ25180 interrupt initialization failed (%d)", ret);
		return false;
	}
	ret = bq25180_kill_init();
	if (ret != 0) {
		LOG_ERR("BQ25180 kill GPIO initialization failed (%d)", ret);
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
bool bq25180_config(const struct bq25180_config_t* config) {
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
	union bq25180_MASK_ID_register_t maskId = {.value = BQ25180_MASK_ID_DEFAULT};

	if ((bq25180.state.bits.bInitialized == 0) || (bq25180.state.bits.bProbed == 0) ||
		(bq25180.state.bits.bDeviceFound == 0)) {
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
	/* Never stop charging: disable the fast-charge safety timer so a long charge
	 * cannot fault out, and disable the I2C watchdog so a host that never services
	 * it cannot silently restore the default registers (which would re-arm the timer
	 * and revert this configuration). The BQ25180 still manages CV/termination and
	 * auto-recharge on its own. */
	icCtrl.bits.bSafetyFastChargeTimer = (int) bq25180_fast_charge_time_Disable;
	icCtrl.bits.bWatchdogSelection = (int) bq25180_watchdog_selection_Disable;
	icCtrl.bits.bTSAutoFunctionEnable = 0;
	tmrIlim.bits.bInputCurrentLimit = bq25180.config.input_current;
	tmrIlim.bits.bLongPressDuration = bq25180.config.long_press_duration;
	tmrIlim.bits.bHardwareResetCondition = 1; // long press and vin

	shipRst.bits.bEnablePush = 0;
	shipRst.bits.bPushbuttonLongPressAction = bq25180_long_press_action_DoNothing;
	shipRst.bits.bEnableShipModeAndReset = bq25180_reset_shipment_mode_DoNothing;

	tsControlReg.bits.bThermalSystemColdThreshold = bq25180_cold_threshold_m3;
	sysReg.bits.bSYSPowerMode = bq25180_sys_power_mode_VIN_or_VBAT;

	// Configure which conditions assert the INT pin (1 = masked). Keep
	// power-good (plug/unplug) and battery/charge interrupts enabled; mask the
	// chatty thermal-regulation and thermal-shutdown sources. The device-ID
	// nibble is read-only and ignored on write.
	maskId.bits.bPowerGoodMaskInterrupt = 0;
	maskId.bits.bBatteryMaskInterrupt = 0;
	maskId.bits.bThermalRegulationMaskInterrupt = 1;
	maskId.bits.bThermalShutdownMaskInterrupt = 1;

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
		(bq25180_i2c_write_register(bq25180_register_SYS_REG, sysReg.value) == 0) &&
		(bq25180_i2c_write_register(bq25180_register_MASK_ID, maskId.value) == 0);

	ret &= bq25180_bus_unlock();
	if (!ret) {
		return false;
	}
	// now everything is configured.
	bq25180.state.bits.bConfigured = 1;

	return true;
}

/**
 * @brief Return the most recent charger-interrupt timestamp.
 * @details The timestamp is captured in ISR context with rtc_get_timestamp_us()
 *          and is expressed as Unix epoch microseconds.
 */
time_t bq25180_get_last_irq_timestamp_ms(void) {
	__ASSERT(bq25180.state.bits.bInitialized != 0, "BQ25180 driver not initialized");
	__ASSERT(bq25180.state.bits.bIrqConfigured != 0, "BQ25180 IRQ not configured");
	__ASSERT(bq25180.state.bits.bConfigured != 0, "BQ25180 not configured");
	return bq25180.last_irq_time;
}

int bq25180_update_state(struct bq25180_charger_state_t* state) {
	assert(bq25180.state.bits.bConfigured != 0);

	union bq25180_STAT0_register_t stat0;
	union bq25180_STAT1_register_t stat1;
	union bq25180_FLAG0_register_t flag0;
	union bq25180_SHIP_RST_register_t shipRst;

	//	enum bq25180_event_type event = bq25180_event_Invalid;
	enum bq25180_charging_status_type chargingStatus;
	enum bq25180_reset_shipment_mode_type operationMode;
	union bq25180_charger_state currentState = {.value = 0};

	if (!bq25180_bus_lock()) {
		LOG_ERR("Failed to lock I2C bus for BQ25180 state update");
		return -EIO;
	}

	bool ret =
		(bq25180_i2c_read_register(bq25180_register_STAT0, (uint8_t*) &(stat0.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_STAT1, (uint8_t*) &(stat1.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_FLAG0, (uint8_t*) &(flag0.value)) == 0) &&
		(bq25180_i2c_read_register(bq25180_register_SHIP_RST, (uint8_t*) &(shipRst.value)) == 0);

	if (ret == false) {
		LOG_ERR("Failed to read BQ25180 registers for state update");
	} else if (state != NULL) {
		state->last_update_time = rtc_get_timestamp_us();
		state->last_irq_time = bq25180.last_irq_time;
	}
	ret &= bq25180_bus_unlock();
	if (!ret && (state != NULL)) {
		state->state.value = 0;
		return -EIO;
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
		state->state.value = currentState.value;
	}
	if (currentState.value != bq25180.charger_state.value) {
		bq25180_process_state_change(bq25180.charger_state, currentState);
	}
	return 0;
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
void bq25180_print_state(union bq25180_charger_state* state) {
	assert(bq25180.state.bits.bConfigured != 0);

	if (state == NULL) {
		state = &bq25180.charger_state;
	}
	LOG_INF("Charger state: PG=%d Charging=%d Charged=%d Shipmode=%d "
			"Shutdown=%d BT=%d Wake1=%d Wake2=%d ThermalReg=%d UVLO=%d "
			"TNormal=%d TWarmHot=%d TWarm=%d TCool=%d TimerFault=%d TFault=%d "
			"UVLOFault=%d OCPFault=%d\r\n",
			state->bits.bPowerGood,
			state->bits.bCharging,
			state->bits.bCharged,
			state->bits.bShipmentMode,
			state->bits.bShutdownMode,
			state->bits.bButtonPressed,
			state->bits.bWake1,
			state->bits.bWake2,
			state->bits.bThermalRegulation,
			state->bits.bBatteryUVLO,
			state->bits.bThermalNormal,
			state->bits.bThermalWarmOrHot,
			state->bits.bThermalWarm,
			state->bits.bThermalCool,
			state->bits.bSafetyTimerFault,
			state->bits.bThermalSystemFault,
			state->bits.bBatteryUVLOFault,
			state->bits.bBatteryOCPFault);
}

/*
 * \brief Issues a software-reset command to the BQ25180.
 *
 * After the reset the IC returns to its POR register defaults.
 * The function returns   true  if the command could be sent,
 *                        false otherwise (lock or I²C failure).
 */
bool bq25180_reset(const struct bq25180_config_t* config) {
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
