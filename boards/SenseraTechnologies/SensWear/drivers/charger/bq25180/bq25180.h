/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bq25180.h
 * @brief SenseWear BQ25180 charger and power-path controller API.
 *
 * @defgroup sensewear_bq25180 SenseWear BQ25180 charger
 * @ingroup io_interfaces
 * @{
 *
 * The BQ25180 manages battery charging, input-current limiting, battery
 * protection thresholds, power-path behavior, ship mode, and shutdown behavior
 * for the SenseWear main board. It also exposes a cached charger-state
 * snapshot and the most recent interrupt timestamp for policy code that needs
 * to react to charger events.
 *
 * Timestamp values in this driver are captured from `SYS_CLOCK_REALTIME` via
 * `rtc_get_timestamp_us()`, are expressed as Unix epoch time in microseconds
 * since `1970-01-01 00:00:00 UTC`, and truncate the sub-microsecond portion of
 * the clock.
 *
 * The driver uses the board's @ref sensewear_sys_i2c ownership wrapper rather
 * than calling Zephyr's I2C API directly:
 *
 * @code{.text}
 * application / power manager
 *          |
 *          | bq25180_config(), bq25180_update_state(), ...
 *          v
 * BQ25180 driver
 *          |
 *          | sys_i2c_lock(), transfer(s), sys_i2c_release()
 *          v
 * SenseWear shared system I2C bus
 * @endcode
 *
 * Every high-level hardware operation acquires the shared bus once and calls
 * sys_i2c_release() on exit. Private register helpers assume that ownership has
 * already been acquired.
 *
 * @section sensewear_bq25180_devicetree Devicetree representation
 *
 * The charger remains a standard child of the physical I2C controller:
 *
 * @code{.dts}
 * sys_i2c_peripheral: &i2c21 {
 *     bq25180: charger@6a {
 *         compatible = "sensewear,bq25180";
 *         reg = <0x6a>;
 *         int-gpios = <&gpio1 5 GPIO_ACTIVE_LOW>;
 *         kill-gpios = <&gpio1 8 GPIO_ACTIVE_HIGH>;
 *         status = "okay";
 *     };
 * };
 * @endcode
 *
 * `reg` supplies the target address used by SYS_I2C_DT_SPEC_GET(). The
 * interrupt GPIO is also read from this node. `kill-gpios` is reserved for
 * power-hold control but is not yet consumed by the current implementation.
 *
 * @section sensewear_bq25180_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call bq25180_init() to verify the shared bus and probe the charger.
 * 2. Prepare a struct bq25180_config_t using one of the default helpers.
 * 3. Call bq25180_config() to program the device registers.
 * 4. Call bq25180_update_state() in response to IRQs or whenever the decoded
 *    charger state needs to be refreshed.
 * 5. Use bq25180_enable_charging(), the power-mode operations, and
 *    bq25180_get_last_irq_timestamp_ms() as required.
 *
 * Initialization and configuration are deliberately separate. A successful
 * probe does not imply that the desired battery parameters have been applied.
 *
 * @section sensewear_bq25180_example Typical usage
 *
 * @code{.c}
 * struct bq25180_config_t config;
 * union bq25180_charger_state_t state;
 *
 * if (!bq25180_init()) {
 *     // Charger absent or shared bus unavailable.
 *     return;
 * }
 *
 * bq25180_get_default_lipo_usb_charger_config(&config);
 * config.battery_uvlo = bq25180_battery_UVLO_threshold_2V8;
 *
 * if (!bq25180_config(&config)) {
 *     return;
 * }
 *
 * if (bq25180_update_state(&state) == 0) {
 *     bool may_charge = state.state.bits.bPowerGood && !state.state.bits.bCharged;
 *     (void)bq25180_enable_charging(may_charge);
 * }
 * @endcode
 *
 * @section sensewear_bq25180_interrupts Interrupt status
 *
 * The GPIO callback for `int-gpios` posts bq25180_event_InterruptDetected from
 * ISR context. Call bq25180_update_state() from thread context to read the IC,
 * refresh the cached decoded state, and publish state-change events. The
 * returned snapshot stores the refresh time in microseconds since the Unix
 * epoch, and bq25180_get_last_irq_timestamp_ms() exposes the most recent IRQ
 * timestamp captured in the callback.
 */

#ifndef BQ25180_H_
#define BQ25180_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <zephyr/drivers/gpio.h>
#include <bq25180_registers.h>

/**
 * @brief Maximum time allowed for acquiring the shared I2C bus.
 * @details Expressed in milliseconds and passed to K_MSEC() by the driver.
 *          Applications may override it before including this header.
 */
#ifndef BQ25180_I2C_TIMEOUT
#define BQ25180_I2C_TIMEOUT (100)
#endif

/**
 * @brief Driver-level BQ25180 event identifiers.
 * @details This is a software event namespace rather than a hardware register
 *          encoding. Values map decoded STAT0, STAT1, and FLAG0 conditions to
 *          stable event identifiers for higher-level policy code.
 */
enum bq25180_event_type {
	bq25180_event_Invalid = -1,							  /**< No valid event. */
	bq25180_event_Plugged = 0,							  /**< Valid input power detected. */
	bq25180_event_Unplugged = 1,						  /**< Input power lost. */
	bq25180_event_Charging = 2,							  /**< Battery is charging. */
	bq25180_event_ChargingDone = 3,						  /**< Charge cycle completed. */
	bq25180_event_ThermalRegulation = 4,				  /**< Thermal regulation became active. */
	bq25180_event_VIN_OverVoltageProtection = 5,		  /**< VIN overvoltage protection event. */
	bq25180_event_BatteryUnderVoltageLockOut = 6,		  /**< Battery UVLO status event. */
	bq25180_event_SafetyTimerExpired = 7,				  /**< Charge safety timer expired. */
	bq25180_event_ThermalSystemFault = 8,				  /**< Battery TS fault. */
	bq25180_event_BatteryUndervoltageLockoutFault = 9,	  /**< Latched battery UVLO fault. */
	bq25180_event_BatteryOverCurrentProtectionFault = 10, /**< Latched battery OCP fault. */
	bq25180_event_Wake1 = 11,							  /**< WAKE1 event detected. */
	bq25180_event_Wake2 = 12,							  /**< WAKE2 event detected. */
	bq25180_event_ButtonPressed = 13,					  /**< Button pressed event detected. */
	bq25180_event_Irq = 14,								  /**< INT pin assertion detected. */
	bq25180_event_Count,								  /**< Number of valid event identifiers. */
};

/**
 * @brief Decoded charger state exposed to callers.
 * @details This software datatype does not map one-to-one to a hardware
 *          register. bq25180_update_state() combines STAT0 (0x00), STAT1
 *          (0x01), FLAG0 (0x02), and SHIP_RST (0x09) into logical flags for
 *          application and power-policy code.
 */
union bq25180_charger_state {
	unsigned int value; /**< Complete packed state; zero means no flags are asserted. */
	/**
	 * @brief Decoded charger-state bit mapping.
	 * @details Each field maps one or more hardware status bits to a normalized
	 *          boolean condition; it is not written back to the BQ25180.
	 */
	struct bq25180_charger_state_bits {
		unsigned int bButtonPressed : 1;	 /**< Push-button activity detected. */
		unsigned int bWake1 : 1;			 /**< WAKE1 event detected. */
		unsigned int bWake2 : 1;			 /**< WAKE2 event detected. */
		unsigned int bShipmentMode : 1;		 /**< Ship-mode request is encoded. */
		unsigned int bShutdownMode : 1;		 /**< Shutdown request is encoded. */
		unsigned int bPowerGood : 1;		 /**< VIN is power-good. */
		unsigned int bCharging : 1;			 /**< Constant-current or constant-voltage charging. */
		unsigned int bCharged : 1;			 /**< Charge cycle is complete. */
		unsigned int bThermalRegulation : 1; /**< Thermal regulation is active or latched. */
		unsigned int bBatteryUVLO : 1;		 /**< Battery UVLO status is active. */
		unsigned int bThermalNormal : 1;	 /**< Battery temperature is normal. */
		unsigned int bThermalWarmOrHot
			: 1;					   /**< Battery temperature is outside normal on warm side. */
		unsigned int bThermalWarm : 1; /**< Battery temperature is warm. */
		unsigned int bThermalCool : 1; /**< Battery temperature is cool. */
		unsigned int bSafetyTimerFault : 1;	  /**< Charge safety timer expired. */
		unsigned int bThermalSystemFault : 1; /**< Battery temperature fault is latched. */
		unsigned int bBatteryUVLOFault : 1;	  /**< Battery UVLO fault is latched. */
		unsigned int bBatteryOCPFault : 1;	  /**< Battery overcurrent fault is latched. */
	} bits;
};

/**
 * @brief Cached charger-state snapshot returned by bq25180_update_state().
 * @details `last_update_time` is populated with the wall-clock timestamp of the
 *          most recent successful refresh, sampled from `SYS_CLOCK_REALTIME`
 *          through `rtc_get_timestamp_us()`. The value is Unix epoch time in
 *          microseconds since `1970-01-01 00:00:00 UTC` and truncates the
 *          sub-microsecond portion of the clock.
 */
struct bq25180_charger_state_t {
	time_t last_update_time; /**< Timestamp of the last state update in microseconds since epoch. */
	time_t last_irq_time; /**< Timestamp of the last INT pin assertion in microseconds since epoch.
							 -1 if no interrupt has been detected. */
	union bq25180_charger_state state; /**< Current charger state. */
};

/**
 * @brief High-level BQ25180 charger configuration.
 * @details This software datatype maps engineering-unit and enum selections to
 *          VBAT_CTRL, ICHG_CTRL, CHARGECTRL0, CHARGECTRL1, IC_CTRL, TMR_ILIM,
 *          SHIP_RST, SYS_REG, and TS_CONTROL during bq25180_config(). It is not
 *          a raw register image.
 */
struct bq25180_config_t {
	/** Battery regulation voltage in millivolts. */
	uint16_t charge_voltage;
	/** Fast-charge current in milliamperes. */
	uint16_t charge_current;

	/** Input current limit. */
	enum bq25180_input_current_limit_type input_current;
	/** Input voltage dynamic power-management threshold. */
	enum bq25180_VINDPM_level_type vin_dpm_level;

	/** Battery undervoltage lockout threshold. */
	enum bq25180_battery_UVLO_threshold_type battery_uvlo;
	/** Battery discharge overcurrent limit. */
	enum bq25180_battery_discharge_current_limit_type battery_ocp_limit;

	/** Charge-termination current ratio. */
	enum bq25180_termination_current_type termination_current;
	/** Precharge current selection. */
	enum bq25180_precharge_current_type precharge_current;

	/** Voltage threshold below which precharge is used. */
	enum bq25180_precharge_voltage_threshold_type precharge_threshold;
	/** Voltage drop below regulation voltage that restarts charging. */
	enum bq25180_recharge_voltage_threshold_type recharge_voltage_threshold;

	/** Push-button long-press duration. */
	enum bq25180_pb_long_press_duration_type long_press_duration;
};

/**
 * @brief Initialize and probe the BQ25180.
 *
 * The function copies the devicetree-derived shared-I2C specification into the
 * driver context, verifies bus readiness, and reads the MASK_ID register to
 * confirm that a device responds.
 *
 * This function does not program the charger configuration.
 *
 * @retval true The shared bus was ready and the charger responded.
 * @retval false The bus was unavailable, ownership could not be acquired, or
 *         the probe transfer failed.
 */
bool bq25180_init(void);

/**
 * @brief Report whether the BQ25180 is initialized and configured.
 *
 * @retval true The charger has been detected and its register configuration was applied.
 * @retval false Initialization, probing, or configuration has not succeeded.
 */
bool bq25180_is_ready(void);

/**
 * @brief Program the charger operating parameters.
 *
 * The complete register sequence is protected by one shared-I2C ownership
 * scope. Passing NULL selects the LiPo/USB charging defaults.
 *
 * @param config Configuration to apply, or NULL for the LiPo/USB defaults.
 * @retval true All configuration registers were written and ownership released.
 * @retval false The driver was not initialized, locking failed, a transfer
 *         failed, or ownership could not be released.
 */
bool bq25180_config(struct bq25180_config_t* config);

/**
 * @brief Return the printable name for a BQ25180 event identifier.
 *
 * @param event_id Event identifier from enum bq25180_event_type.
 * @return Constant string for the event, or "Unknown" when @p event_id is not valid.
 */
const char* bq25180_event_name(uint32_t event_id);

/**
 * @brief Populate a configuration from BQ25180 register reset defaults.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void bq25180_get_default_config(struct bq25180_config_t* config);

/**
 * @brief Populate the SenseWear LiPo/USB charging defaults.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void bq25180_get_default_lipo_usb_charger_config(struct bq25180_config_t* config);

/**
 * @brief Read and decode the current charger state.
 *
 * STAT0, STAT1, FLAG0, and SHIP_RST are read under one ownership scope.
 *
 * @param state Optional destination for the decoded state. When non-NULL, the
 *        driver writes the decoded charger flags to @p state->state and the
 *        refresh timestamp to @p state->last_update_time.
 * @return 0 The registers were read and decoded.
 * @return A negative errno-style code on failure. When @p state is non-NULL it
 *         is cleared on failure.
 */
int bq25180_update_state(struct bq25180_charger_state_t* state);

/**
 * @brief Read the timestamp of the most recent charger interrupt.
 *
 * The value is captured in ISR context when the `int-gpios` callback fires by
 * sampling `SYS_CLOCK_REALTIME` through `rtc_get_timestamp_us()`. It is Unix
 * epoch time in microseconds since `1970-01-01 00:00:00 UTC` and truncates the
 * sub-microsecond portion of the clock.
 *
 * @return Microseconds since the Unix epoch, or -1 before the first interrupt
 *         or if the RTC timestamp could not be read.
 */
time_t bq25180_get_last_irq_timestamp_ms(void);

/**
 * @brief Request ship mode through the SHIP_RST register.
 *
 * @retval true The read-modify-write sequence completed.
 * @retval false The driver is unconfigured or the bus operation failed.
 */
bool bq25180_shipment_mode_enable(void);

/**
 * @brief Clear the ship-mode request and long-press ship action.
 *
 * @retval true The read-modify-write sequence completed.
 * @retval false The driver is unconfigured or the bus operation failed.
 */
bool bq25180_shipment_mode_disable(void);

/**
 * @brief Configure a push-button long press to request shutdown.
 *
 * @retval true The read-modify-write sequence completed.
 * @retval false The driver is unconfigured or the bus operation failed.
 */
bool bq25180_shutdown_enable(void);

/**
 * @brief Clear the shutdown long-press action.
 *
 * @retval true The read-modify-write sequence completed.
 * @retval false The driver is unconfigured or the bus operation failed.
 */
bool bq25180_shutdown_disable(void);

/**
 * @brief Reissue the SHIP_RST control value and reapply configuration.
 *
 * The driver first performs a SHIP_RST read-modify-write, marks its cached
 * configuration invalid, and then calls bq25180_config().
 *
 * @param config Configuration to reapply, or NULL for LiPo/USB defaults.
 * @retval true The control write and reconfiguration succeeded.
 * @retval false The driver was unavailable or a bus/configuration step failed.
 */
bool bq25180_reset(struct bq25180_config_t* config);

/**
 * @brief Enable or disable battery charging.
 *
 * The function updates only ICHG_CTRL.ChargeDisable and preserves the remaining
 * register fields.
 *
 * @param enable true to enable charging; false to disable it.
 * @retval true The requested state was already active or was written successfully.
 * @retval false The driver was unconfigured or the bus operation failed.
 */
bool bq25180_enable_charging(bool enable);

/**
 * @brief Log the most recently cached charger state.
 *
 * @param state Pointer to the charger state to log. If NULL, the internal cached state is logged.
 */
void bq25180_print_state(union bq25180_charger_state* state);

/** @} */

#endif //! BQ25180_H_
