/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bq27427.h
 * @brief SenseWear BQ27427 single-cell Li-Ion fuel-gauge API.
 *
 * @defgroup sensewear_bq27427_driver SenseWear BQ27427 fuel gauge
 * @ingroup io_interfaces
 * @{
 *
 * The BQ27427 is an Impedance Track gas gauge that reports battery voltage,
 * current, temperature, state of charge, and remaining/full capacity for the
 * SenseWear main board. The driver programs the gauge's design parameters
 * (capacity, energy, chemistry, current thresholds) into data flash and then
 * reads the decoded battery state on demand.
 *
 * Unlike the @ref sensewear_bq25180 charger, this driver talks to the device
 * directly through Zephyr's I2C API using an I2C_DT_SPEC_GET() specification:
 *
 * @code{.text}
 * application / power manager
 *          |
 *          | bq27427_config(), bq27427_update_state(), ...
 *          v
 * BQ27427 driver
 *          |
 *          | i2c_burst_write_dt(), i2c_burst_read_dt(), ...
 *          v
 * Zephyr I2C controller
 * @endcode
 *
 * @section sensewear_bq27427_devicetree Devicetree representation
 *
 * The gauge is a standard child of the I2C controller. The driver resolves it
 * through DT_NODELABEL(bq27427), so the node must use that label:
 *
 * @code{.dts}
 * bq27427: fuel-gauge@55 {
 *     compatible = "sensewear,bq27427";
 *     reg = <0x55>;
 *     int-gpios = <&gpio1 7 GPIO_ACTIVE_LOW>;
 *     status = "okay";
 * };
 * @endcode
 *
 * `reg` supplies the target address used by I2C_DT_SPEC_GET(). The `int-gpios`
 * (GAUGE_IRQ) property is declared for the GPOUT/SOC_INT line but is not yet
 * consumed by the current implementation; battery state is polled.
 *
 * @section sensewear_bq27427_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call bq27427_init() to resolve the devicetree specification, verify the
 *    bus, probe the DeviceType register, and seed the default configuration.
 * 2. Prepare a struct bq27427_config_t, typically via
 *    bq27427_get_default_config().
 * 3. Call bq27427_config() to unseal the gauge, program the design parameters
 *    into data flash, and re-seal it.
 * 4. Call bq27427_update_state() periodically to read the latest battery state.
 *
 * @warning Configuration is a long, blocking operation. Unsealing, entering
 *          CONFIG UPDATE mode, programming data flash, and re-inserting the
 *          battery involve multiple multi-second k_msleep() delays, so
 *          bq27427_config() and bq27427_reset() must be called from a context
 *          where blocking for several seconds is acceptable.
 *
 * @section sensewear_bq27427_example Typical usage
 *
 * @code{.c}
 * struct bq27427_config_t config;
 * struct bq27427_battery_state_t state;
 *
 * if (!bq27427_init(MY_GAUGE_DEVICE_ID)) {
 *     // Gauge absent or bus unavailable.
 *     return;
 * }
 *
 * bq27427_get_default_config(&config);
 *
 * if (!bq27427_config(&config)) {
 *     return;
 * }
 *
 * if (bq27427_update_state(&state)) {
 *     int soc = state.state_of_charge; // tenths of a percent
 *     (void)soc;
 * }
 * @endcode
 */

#ifndef BQ27427_H_
#define BQ27427_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/i2c.h>
#include "bq27427_registers.h"

/**
 * @brief Maximum time allowed for acquiring the shared I2C bus.
 * @details Expressed in milliseconds. Reserved for the shared-bus locking path,
 *          which is currently disabled in the driver, and may be overridden by
 *          the application before including this header.
 */
#ifndef BQ27427_I2C_TIMEOUT
#define BQ27427_I2C_TIMEOUT (100)
#endif
/**
 * \brief Event types posted by the BQ27427 driver to the device event manager.
 *
 */
enum bq27427_event_type {
	bq27427_event_Invalid = -1,
	bq27427_event_BatteryLow = 0,
	bq27427_event_StateUpdated = 1
};

/**
 * @brief High-level BQ27427 design configuration.
 * @details This software datatype maps engineering-unit selections to data-flash
 *          parameters programmed during bq27427_config(). The trailing comments
 *          give the data-flash subclass and byte offset each field is written
 *          to. It is not a raw register image.
 */
struct bq27427_config_t {
	/** Battery chemistry profile to select. */
	enum bq27427_chemistry_type battery_type;
	/* Gas Gauging class -- State subclass (82) */
	uint16_t battery_capacity;			  /**< Design capacity in mAh (offset 6). */
	uint16_t battery_energy;			  /**< Design energy in mWh (offset 8). */
	uint16_t battery_termination_voltage; /**< Terminate voltage in mV (offset 10). */
	uint16_t taper_rate;				  /**< Taper rate, charge-current divisor (offset 21). */
	uint16_t gauge_sleep_current;		  /**< Sleep-current threshold in mA (offset 23). */
	/* Gas Gauging class -- Current Thresholds subclass (81) */
	uint16_t discharge_current_threshold; /**< Discharge current threshold in mA (offset 0). */
	uint16_t charge_current_threshold;	  /**< Charge current threshold in mA (offset 2). */
	uint16_t quit_current_threshold;	  /**< Quit current threshold in mA (offset 4). */
	/* Chemistry Info class -- Chem Data subclass (109) */
	uint16_t
		voltage_at_charge_termination; /**< Cell voltage at charge termination in mV (offset 6). */
	uint16_t taper_voltage;			   /**< Taper voltage in mV (offset 8). */
	/* Ra Tables class -- Ra0 RAM (89): not currently programmed. */
};

/**
 * @brief Decoded battery state exposed to callers.
 * @details Populated by bq27427_update_state() from the gauge's standard
 *          commands. Values use the gauge's native units as listed per field.
 */
struct bq27427_battery_state_t {
	int temperature;				/**< Battery temperature in tenths of a degree Celsius. */
	int voltage;					/**< Cell voltage in millivolts. */
	int average_current;			/**< Average current in milliamperes (signed). */
	int average_power;				/**< Average power in milliwatts (signed). */
	int state_of_charge;			/**< State of charge in tenths of a percent. */
	int nominal_available_capacity; /**< Nominal available capacity in mAh. */
	int full_battery_capacity;		/**< Full available capacity in mAh. */
	int remaining_capacity;			/**< Remaining capacity in mAh. */
	bool learning_in_progress;		/**< True while the gauge is still qualifying capacity. */
};

/**
 * @brief Initialize and probe the BQ27427.
 *
 * Resolves the DT_NODELABEL(bq27427) specification, verifies the I2C bus is
 * ready, and reads the DeviceType control subcommand to confirm a gauge
 * responds with @ref BQ27427_DEVICE_TYPE. On success it seeds the default
 * configuration, signals battery insertion, and issues a soft reset.
 *
 * This function does not program the gauge design parameters.
 *
 * @param device_id Identifier reserved for the device event manager (see
 *        device_driver_events.h). It is stored but not yet used to post events.
 * @retval true The bus was ready and the gauge responded with the expected
 *         device type.
 * @retval false The bus was not ready or the device-type probe failed.
 */
bool bq27427_init(uint32_t device_id);

/**
 * @brief Report whether bq27427_init() successfully detected the gauge.
 *
 * @retval true The gauge is available.
 * @retval false Initialization has not succeeded.
 */
bool bq27427_is_ready(void);

/**
 * @brief Program the gauge design parameters.
 *
 * Unseals the gauge, enters CONFIG UPDATE mode, programs the chemistry profile
 * and the data-flash design parameters, exits CONFIG UPDATE mode, and re-seals
 * the gauge. The battery is removed and re-inserted so the gauge re-evaluates
 * its state with the new configuration.
 *
 * @param config Configuration to apply. Must not be NULL; it is dereferenced
 *        immediately.
 * @retval true The gauge was configured and the cached state marked configured.
 * @retval false The driver was not initialized.
 *
 * @warning Blocks for several seconds (see the file-level warning).
 */
bool bq27427_config(struct bq27427_config_t* config);

/**
 * @brief Reset the gauge and optionally reapply a configuration.
 *
 * @details Issues the RESET (0x0041) control subcommand, waits for the gauge to
 *          reboot and the bus to become ready, and clears the cached configured
 *          flag. When @p config is non-NULL it then reprograms the design
 *          parameters as bq27427_config() would.
 *
 * @param config Configuration to apply after reset, or NULL to reset only.
 * @retval true Reset (and reconfiguration when requested) succeeded.
 * @retval false The driver was not initialized.
 *
 * @warning Blocks for several seconds (see the file-level warning).
 */
bool bq27427_reset(struct bq27427_config_t* config);

/**
 * @brief Populate a configuration with the SenseWear battery defaults.
 *
 * @details Fills @p config from the BQ27427_DEFAULT_* values in
 *          bq27427_registers.h.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void bq27427_get_default_config(struct bq27427_config_t* config);

/**
 * @brief Read and decode the current battery state.
 *
 * Reads temperature, voltage, current, power, state of charge, and the capacity
 * values, updating the driver's cached battery state. A pending POR/reset
 * condition is reported as a failure.
 *
 * @param state Optional destination for the decoded state. The internal cached
 *        state is updated regardless; @p state is cleared on failure when
 *        non-NULL.
 * @retval true The state was read and decoded.
 * @retval false The driver is unavailable or unconfigured, or a POR/reset was
 *         detected.
 */
bool bq27427_update_state(struct bq27427_battery_state_t* state);

/**
 * @brief Log the most recently cached battery state.
 */
void bq27427_print_state(void);

/** @} */

#endif //! BQ27427_H_
