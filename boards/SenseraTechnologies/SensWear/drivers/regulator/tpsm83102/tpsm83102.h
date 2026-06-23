/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file tpsm83102.h
 * @brief SenseWear TPSM83102 regulator-driver declarations.
 *
 * @defgroup sensewear_tpsm83102 SenseWear TPSM83102 regulator
 * @ingroup io_interfaces
 * @{
 *
 * The board driver exposes the TPSM83102 through Zephyr's regulator API. Use
 * DEVICE_DT_GET(DT_NODELABEL(tpsm83102)) with regulator_enable(),
 * regulator_disable(), regulator_set_voltage(), and regulator_get_voltage().
 * Register transfers are serialized through the SenseWear shared-I2C wrapper.
 *
 * @section sensewear_tpsm83102_regulator_api Zephyr regulator API
 *
 * This driver intentionally has @b no bespoke public control surface: it is a
 * standard Zephyr regulator device and is meant to be driven entirely through
 * the Zephyr regulator framework. For the meaning and contract of every
 * operation it implements (regulator_enable(), regulator_disable(),
 * regulator_set_voltage(), regulator_get_voltage(), regulator_is_enabled(),
 * the voltage-window semantics, and the devicetree regulator-* constraints),
 * consult Zephyr's regulator documentation rather than re-deriving behavior
 * from this header:
 *   https://docs.zephyrproject.org/latest/hardware/peripherals/regulators.html
 *   include/zephyr/drivers/regulator.h
 *
 * @section sensewear_tpsm83102_events Device-event reporting
 *
 * In addition to the regulator API, the driver publishes lifecycle events into
 * the SenseWear centralized device-event manager (see device_driver_events.h)
 * so power-policy code can observe the rail without polling.
 *
 * Unlike the SenseWear I2C drivers (for example bq25180 / bq27427), which take
 * a device identifier as an argument to their @c _init() call, this driver does
 * @b not require the caller to supply one. Because it is a devicetree-defined
 * regulator (instantiated by DEVICE_DT_DEFINE, not an application init call),
 * there is no init entry point to pass an id to. The driver therefore tags
 * every event it posts with its own build-time identifier, taken @b
 * automatically from the generated @c device_driver_dts_ids.h
 * (@c TPSM83102_DEVICE_DTS_ID, derived from the @c tpsm83102 devicetree label).
 * Consumers match on that same macro; they never receive an id chosen by an
 * application caller.
 *
 * Events are posted from thread context with a @c K_NO_WAIT timeout: if the
 * event queue is full the event is dropped rather than blocking the regulator
 * operation, and the regulator call still succeeds. Event delivery is therefore
 * best-effort and must not be relied on for correctness.
 */
#ifndef SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_
#define SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_

#include <stdint.h>

/** Maximum time allowed for acquiring the shared I2C bus, in milliseconds. */
#ifndef TPSM83102_I2C_TIMEOUT
#define TPSM83102_I2C_TIMEOUT (100)
#endif

/**
 * @brief Driver-level TPSM83102 event identifiers.
 * @details Posted to the SenseWear device-event manager under
 *          @c TPSM83102_DEVICE_DTS_ID. These are software lifecycle events, not
 *          a hardware register encoding.
 */
enum tpsm83102_event_type {
	/**
	 * @brief The converter was enabled (regulator_enable() succeeded).
	 * @details @c v_param carries the current VOUT setting in microvolts.
	 */
	tpsm83102_event_Enabled = 0,
	/**
	 * @brief The output-voltage setpoint changed (regulator_set_voltage()).
	 * @details @c v_param carries the new VOUT setting in microvolts.
	 */
	tpsm83102_event_VOUT_UPDATED = 1,
	/**
	 * @brief The converter was disabled (regulator_disable() succeeded).
	 * @details @c v_param carries the last VOUT setting in microvolts.
	 */
	tpsm83102_event_Disabled = 2,
};

/** @} */

#endif /* SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_ */
