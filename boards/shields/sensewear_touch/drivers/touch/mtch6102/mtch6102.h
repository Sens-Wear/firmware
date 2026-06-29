/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mtch6102.h
 * @brief SenseWear MTCH6102 capacitive touch controller API.
 *
 * @defgroup sensewear_mtch6102 SenseWear MTCH6102 touch controller
 * @ingroup io_interfaces
 * @{
 *
 * The MTCH6102 is the touch controller on the SenseWear touch daughter board.
 * It reports touch coordinates, touch-state flags, and gesture codes through
 * I2C. The driver probes the part, populates a default configuration object,
 * and uses the daughter-board connector arbiter for the INT and SYNC pins
 * rather than describing those pins directly in devicetree.
 *
 * Like the MAX30101 PPG driver, the MTCH6102 does not expose a per-driver
 * callback. Its INT-pin GPIO callback posts ::mtch6102_Irq from ISR context,
 * and a consumer calls mtch6102_irq_handler() from thread context to read the
 * touch/gesture state and publish decoded `mtch6102_event_*` identifiers through
 * the shared @ref sensewear_device_driver_events manager:
 *
 * @code{.text}
 * application / touch bridge
 *          |
 *          | mtch6102_init(), mtch6102_config(), mtch6102_start(),
 *          | mtch6102_irq_handler()
 *          v
 * MTCH6102 driver
 *          |
 *          | I2C transfers, daughter_if GPIO ownership, device_driver_event_post()
 *          v
 * SenseWear touch daughter board
 * @endcode
 *
 * @section sensewear_mtch6102_devicetree Devicetree representation
 *
 * The controller is declared as a standard I2C child of the shared system bus:
 *
 * @code{.dts}
 * &sys_i2c_peripheral {
 *     mtch6102: mtch6102@25 {
 *         compatible = "sensewear,daughter-i2c-device", "i2c-device";
 *         reg = <0x25>;
 *         label = "MTCH6102";
 *         vin-supply = <&tpsm83102>;
 *         status = "okay";
 *     };
 * };
 * @endcode
 *
 * The node is also wired into the board aliases `sensewear-daughter` and
 * `sensewear-touch`, both pointing at this instance.
 *
 * `reg` supplies the target address used by the I2C helper functions.
 * `vin-supply` names the shared daughter-connector rail (VDD_DAUGHTER, the
 * `tpsm83102` regulator) and is mandatory: like the DRV2605 and MAX30101 drivers
 * the MTCH6102 resolves the regulator from devicetree at build time (a missing
 * `vin-supply` is a build error) and owns the regulator handle directly, so none
 * is passed in at run time. The driver drives the rail to its fixed
 * ::MTCH6102_SUPPLY_VOLTAGE_UV (2.8 V) and enables it when acquisition starts; an
 * already-live shared rail must already sit at that voltage. Keep this rail
 * enabled while the touch controller is in use: powering the MTCH6102 down can
 * leave the device holding the I2C bus until the rail is restored. The driver
 * resolves its INT and SYNC GPIOs at run time from the @ref sensewear_daughter_if
 * arbiter, using `daughter_if_GPIO3` for INT and `daughter_if_GPIO2` for SYNC.
 * The MTCH6102 INT pin is open-collector, active-low; SYNC is an active-high
 * output, so the driver treats both connector lines as inputs and only arms an
 * interrupt on INT.
 *
 * @section sensewear_mtch6102_lifecycle Driver lifecycle
 *
 * The expected lifecycle is:
 *
 * 1. Call mtch6102_init() to verify the shared bus, resolve and validate the
 *    devicetree supply rail, probe the controller, claim and configure the SYNC
 *    and INT lines, arm the INT interrupt, and populate the default
 *    configuration object. The rail is enabled later, when acquisition starts.
 * 2. Optionally adjust a mtch6102_config_t and pass it to mtch6102_config()
 *    to program the touch controller registers in one ownership scope.
 * 3. Call mtch6102_start() to power the supply rail. If the device has not yet
 *    been configured, start() applies the stored default config on demand
 *    first. The INT line is already armed by mtch6102_init(); with the rail
 *    powered the device begins asserting INT. Do not disable the rail while the
 *    driver is active: the MTCH6102 can hold the I2C bus in that state until the
 *    supply is restored.
 * 4. On every INT assertion the driver posts ::mtch6102_Irq from ISR context.
 *    A consumer then calls mtch6102_irq_handler() from thread context, which
 *    reads the touch position and gesture state, classifies the sample with
 *    mtch6102_sample_event(), and publishes the decoded `mtch6102_event_*`
 *    identifier through the shared device-event manager.
 * 5. Call mtch6102_stop() and mtch6102_deinit() when finished.
 *
 * The INT line is armed once in mtch6102_init() and stays armed until
 * mtch6102_deinit(); acquisition is gated by the supply rail, which
 * mtch6102_start() powers and mtch6102_stop() removes. Keep the rail enabled
 * while the controller is connected to the bus; if the part is powered down it
 * may continue to hold the I2C lines until the rail is brought back up.
 *
 * @section sensewear_mtch6102_events Interrupt and event model
 *
 * The driver does not depend on Bluetooth or any consumer subsystem, and it does
 * not expose a per-driver callback. It only turns INT-pin activity into stable
 * software events delivered through the shared
 * @ref sensewear_device_driver_events manager.
 *
 * The daughter-board interrupt callback posts ::mtch6102_Irq from ISR context. A
 * consumer then calls mtch6102_irq_handler() from thread context, which reads the
 * touch/gesture state, classifies it, and publishes one decoded
 * `mtch6102_event_*` identifier. Each published event carries the raw
 * `TOUCH_STATE` byte in `v_param` and a pointer to the decoded
 * ::touch_sensor_sample (held in the driver context) in `p_param`.
 *
 * The driver exposes normalized event identifiers for raw interrupts, touch
 * state, and the gesture codes documented by the MTCH6102 user guide:
 *
 * - ::mtch6102_Irq is the raw INT pin assertion, posted from the ISR.
 * - ::mtch6102_event_TouchDetected and ::mtch6102_event_TouchReleased describe
 *   the raw touch-present bit in `TOUCH_STATE`.
 * - ::mtch6102_event_SingleClick, ::mtch6102_event_ClickAndHold,
 *   ::mtch6102_event_DoubleClick, ::mtch6102_event_DownSwipe,
 *   ::mtch6102_event_DownSwipeAndHold, ::mtch6102_event_RightSwipe,
 *   ::mtch6102_event_RightSwipeAndHold, ::mtch6102_event_UpSwipe,
 *   ::mtch6102_event_UpSwipeAndHold, ::mtch6102_event_LeftSwipe, and
 *   ::mtch6102_event_LeftSwipeAndHold map the MTCH6102 gesture codes.
 *
 * `mtch6102_sample_event()` classifies a decoded `struct touch_sensor_sample` into
 * one of those event identifiers, and `mtch6102_event_name()` returns a printable
 * name for logging or higher-level policy.
 *
 * @section sensewear_mtch6102_config Configuration model
 *
 * The driver programs the MTCH6102 through one `mtch6102_config_t` object.
 * On the SenseWear shield the touch surface is a single row of 15 pads, with
 * the last three pads routed to the Y axis; the default configuration keeps
 * that 12 X / 3 Y split explicit in the register block.
 * `mtch6102_config()` writes only the writable registers needed for touch and
 * gesture decoding, under one I2C ownership scope: the configuration register
 * block (NumberOfXChannels through VerticalGestureAngle, in register order),
 * the MODE and MODECON decode-control registers, and finally the CMD register
 * with the `cfg` bit set to apply the block. The read-only firmware-ID
 * (0x00-0x03) and touch/gesture status (0x10-0x15) registers are never written,
 * and the I2CAddr register is skipped so the device keeps its fixed address.
 *
 * `mtch6102_get_default_config()` fills the structure with the SenseWear
 * defaults. Passing NULL to `mtch6102_config()` applies the same defaults.
 *
 * @section sensewear_mtch6102_example Typical usage
 *
 * @code{.c}
 * if (mtch6102_init() != 0) {
 *     return;
 * }
 *
 * if (mtch6102_config(NULL) != 0) {
 *     return;
 * }
 * mtch6102_start();
 *
 * // From the device-manager thread, in response to ::mtch6102_Irq:
 * mtch6102_irq_handler();
 * @endcode
 *
 * Legacy `touch_sensor_*` lifecycle entry points remain available as
 * compatibility wrappers for the current bridge code.
 */

#ifndef MTCH6102_H_
#define MTCH6102_H_

#include "mtch6102_registers.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/i2c.h>

/**
 * @brief Fixed supply-rail voltage required by the MTCH6102, in microvolts.
 * @details The driver drives its `vin-supply` regulator to this voltage and
 *          enables it when acquisition starts, mirroring the fixed-voltage rail
 *          handling in the DRV2605 and MAX30101 drivers.
 */
#define MTCH6102_SUPPLY_VOLTAGE_UV (2800000)

/**
 * @brief Driver-level MTCH6102 event identifiers.
 * @details This is a software event namespace rather than a hardware register
 *          encoding. ::mtch6102_Irq is the raw INT-pin assertion; the
 *          `mtch6102_event_*` values map decoded touch state and gesture codes
 *          to stable identifiers for higher-level policy code.
 */
enum mtch6102_event_type {
	mtch6102_event_Invalid = -1,	  /**< No valid event. */
	mtch6102_Irq = 0,				  /**< INT pin assertion detected (raw ISR-level signal). */
	mtch6102_event_TouchDetected,	  /**< Touch present bit asserted in TOUCH_STATE. */
	mtch6102_event_TouchReleased,	  /**< Touch present bit cleared in TOUCH_STATE. */
	mtch6102_event_SingleClick,		  /**< Gesture code 0x10. */
	mtch6102_event_ClickAndHold,	  /**< Gesture code 0x11. */
	mtch6102_event_DoubleClick,		  /**< Gesture code 0x20. */
	mtch6102_event_DownSwipe,		  /**< Gesture code 0x31. */
	mtch6102_event_DownSwipeAndHold,  /**< Gesture code 0x32. */
	mtch6102_event_RightSwipe,		  /**< Gesture code 0x41. */
	mtch6102_event_RightSwipeAndHold, /**< Gesture code 0x42. */
	mtch6102_event_UpSwipe,			  /**< Gesture code 0x51. */
	mtch6102_event_UpSwipeAndHold,	  /**< Gesture code 0x52. */
	mtch6102_event_LeftSwipe,		  /**< Gesture code 0x61. */
	mtch6102_event_LeftSwipeAndHold,  /**< Gesture code 0x62. */
	mtch6102_event_Count,			  /**< Number of valid event identifiers. */
};

/**
 * @brief Decoded touch sample payload.
 * @details The structure holds the raw touch position and the raw gesture code
 *          read from the device. The helper mtch6102_sample_event() can map this
 *          payload to a stable event identifier, while the application may still
 *          inspect the raw fields directly if it needs the unmodified sensor
 *          state.
 */
struct touch_sensor_sample {
	struct mtch6102_position position; /**< Decoded touch position and raw touch-state byte. */
	uint8_t gesture_state;			   /**< Raw GESTURE_STATE register value. */
};

/** Number of bytes in the MTCH6102 configuration register block. */
#define MTCH6102_CONFIGURATION_REGISTER_COUNT \
	((uint8_t) (mtch6102_config_I2CAddr - mtch6102_config_NumberOfXChannels + 1U))

/** Convert a configuration-register ID into an index within the config array. */
#define MTCH6102_CONFIGURATION_INDEX(register_id) \
	((uint8_t) ((register_id) - mtch6102_config_NumberOfXChannels))

/**
 * @brief MTCH6102 driver configuration object.
 * @details This is a register-content container, not a raw I2C image. The
 *          driver writes the configuration array in register order, then
 *          applies the core control registers, and finally uses the sampling
 *          rate for the periodic work item.
 */
struct mtch6102_config_t {
	union mtch6102_cmd_register_t cmd;	 /**< Command register image; CMD is written last with the
											`cfg` bit set to apply the block below. */
	union mtch6102_mode_register_t mode; /**< Decode mode register image written to MODE. */
	union mtch6102_modecon_register_t modecon; /**< Raw-ADC control image written to MODECON. */
	/**
	 * Configuration register block written from NumberOfXChannels through
	 * VerticalGestureAngle, in register order. The I2CAddr register is part of
	 * the block but is never written, so the device keeps its fixed address.
	 */
	uint8_t configuration[MTCH6102_CONFIGURATION_REGISTER_COUNT];
};

/**
 * @brief Return the printable name for a MTCH6102 event identifier.
 *
 * @param event_id Event identifier from enum mtch6102_event_type.
 * @return Constant string for the event, or "Unknown" when @p event_id is invalid.
 */
const char* mtch6102_event_name(enum mtch6102_event_type event_id);

/**
 * @brief Classify one decoded sample into a stable MTCH6102 event identifier.
 *
 * The helper maps the raw gesture code first, so swipe and click events are
 * reported even when the touch-present bit has already cleared. If no gesture
 * code is present, the helper falls back to touch-present and touch-released
 * events.
 *
 * @param sample Decoded touch sample, or NULL.
 * @return A value from enum mtch6102_event_type.
 */
enum mtch6102_event_type mtch6102_sample_event(const struct touch_sensor_sample* sample);

/**
 * @brief Report whether mtch6102_init() has successfully probed the part.
 *
 * @retval true The controller is available.
 * @retval false Initialization has not succeeded.
 */
bool mtch6102_is_ready(void);

/**
 * @brief Read the MTCH6102 touch position registers.
 *
 * This is a low-level helper kept for compatibility with the existing touch
 * bridge and test code. It transfers over the shared @ref sensewear_sys_i2c
 * bus, owning it only for the duration of the read.
 *
 * @param pos Destination position structure.
 * @retval 0 Touch data was read and decoded.
 * @retval -EINVAL @p pos was NULL.
 * @retval -EIO The shared bus could not be acquired.
 * @retval -ENODATA The controller reported no touch at the time of the read.
 * @return A negative errno if the I2C transfer failed.
 */
int mtch6102_get_position(struct mtch6102_position* pos);

/**
 * @brief Populate the SenseWear default touch configuration.
 * @details Fills the MTCH6102 configuration object with the default register
 *          contents used by the driver.
 *
 * @param config Destination configuration. Must not be NULL.
 */
void mtch6102_get_default_config(struct mtch6102_config_t* config);

/**
 * @brief Program the MTCH6102 touch configuration.
 *
 * The complete register sequence is applied under one programming scope.
 * Passing NULL selects the SenseWear defaults.
 *
 * @param config Configuration to apply, or NULL for the defaults.
 * @retval 0 All configuration registers were written.
 * @retval -EINVAL The driver was not initialized.
 * @retval -EBUSY The driver is currently sampling.
 * @return A negative errno if a register transfer failed.
 */
int mtch6102_config(struct mtch6102_config_t* config);

/**
 * @brief Initialize and probe the MTCH6102.
 *
 * Verifies that the shared bus is ready, resolves and validates the devicetree
 * supply rail, probes the firmware identifier, claims and configures the
 * daughter-board SYNC and INT lines, arms the INT interrupt, and populates the
 * default configuration object. The rail is not enabled here; the driver powers
 * it when acquisition starts.
 *
 * @retval 0 The shared bus was ready and the sensor responded (or the driver was
 *         already initialized).
 * @retval -ENODEV The bus was unavailable, the GPIO controller was not ready,
 *         or the sensor did not respond.
 * @retval -EIO A configuration, regulator, or GPIO transfer failed.
 */
int mtch6102_init(void);

/**
 * @brief Start interrupt-driven touch acquisition.
 *
 * Initializes the sensor on demand if necessary, applies the stored
 * configuration if it has not yet been programmed, and powers the supply rail to
 * ::MTCH6102_SUPPLY_VOLTAGE_UV. The INT line was already armed by
 * mtch6102_init(), so once the rail is powered each INT assertion posts
 * ::mtch6102_Irq, and the consumer drains it through mtch6102_irq_handler().
 *
 * @retval 0 Acquisition was started.
 * @return A negative errno propagated from mtch6102_init(), mtch6102_config(),
 *         or regulator setup.
 */
int mtch6102_start(void);

/** @brief Stop acquisition and power down the rail. */
void mtch6102_stop(void);

/** @brief Stop acquisition and mark the driver uninitialized. */
void mtch6102_deinit(void);

/**
 * @brief Handle an MTCH6102 interrupt.
 *
 * Reads the touch position and gesture state, classifies the resulting sample
 * with mtch6102_sample_event(), and publishes the decoded `mtch6102_event_*`
 * identifier through the shared device-event manager. The raw `TOUCH_STATE`
 * byte travels in `v_param` and a pointer to the decoded
 * ::touch_sensor_sample (held in the driver context) in `p_param`.
 *
 * Call from thread context in response to ::mtch6102_Irq.
 *
 * @retval 0 The touch state was read and the decoded event published.
 * @retval -EAGAIN The driver is not initialized.
 * @return A negative errno if the I2C transfer failed.
 */
int mtch6102_irq_handler(void);

static inline bool touch_sensor_is_ready(void) {
	return mtch6102_is_ready();
}

static inline int touch_sensor_init(void) {
	return mtch6102_init();
}

static inline int touch_sensor_start(void) {
	return mtch6102_start();
}

static inline void touch_sensor_stop(void) {
	mtch6102_stop();
}

static inline void touch_sensor_deinit(void) {
	mtch6102_deinit();
}

/** @} */

#endif /* MTCH6102_H_ */
