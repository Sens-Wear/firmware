/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mtch6102.c
 * @brief SenseWear MTCH6102 touch controller driver implementation.
 * @details The driver is a daughter-board singleton. Its connection is derived
 *          from `DT_ALIAS(sensewear_touch)`, while all transfers are routed
 *          through @ref sensewear_sys_i2c. Public hardware operations own the
 *          shared bus for their complete register sequence and finish with
 *          sys_i2c_release(). The INT and SYNC lines are claimed from the
 *          daughter-board GPIO arbiter so ownership stays centralized.
 *
 * Register helpers in this file intentionally do not lock. They may only be
 * called from a high-level operation that already owns the shared bus.
 *
 * The driver never reprograms the MTCH6102 I2C address: the address is fixed
 * by the devicetree `reg` property, and mtch6102_apply_config() skips the
 * I2CAddr configuration register so no configuration can move the device off
 * its default address.
 *
 * Like the MAX30101 PPG driver, the MTCH6102 does not expose a per-driver
 * callback. Its INT-pin GPIO callback posts ::mtch6102_Irq from ISR context, and
 * mtch6102_irq_handler() runs from thread context to read the touch/gesture
 * state and publish decoded events through the shared device-event manager. The
 * implementation is organized into explicit probe, GPIO, regulator,
 * configuration, event-classification, and interrupt sections so it reads like
 * the other SenseWear board drivers.
 */

#include "mtch6102.h"
#include "rtc.h"
#include "sys_i2c.h"
#include "daughter_if.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mtch6102, CONFIG_LOG_DEFAULT_LEVEL);

/** Devicetree node identifier for the shield's MTCH6102 instance. */
#define MTCH6102_NODE DT_ALIAS(sensewear_touch)
BUILD_ASSERT(DT_NODE_HAS_STATUS(MTCH6102_NODE, okay),
			 "Touch firmware requires the sensewear_touch shield");
/* The driver owns the supply rail through the devicetree vin-supply phandle. */
BUILD_ASSERT(DT_NODE_HAS_PROP(MTCH6102_NODE, vin_supply),
			 "Invalid regulator device specified for MTCH6102");

/** Daughter-board connector line carrying the MTCH6102 INT signal. */
#define MTCH6102_IRQ_LINE daughter_if_GPIO3
/** Daughter-board connector line carrying the MTCH6102 SYNC signal. */
#define MTCH6102_SYNC_LINE daughter_if_GPIO2

/** Rail ramp and power-on settle time before the device is accessed. */
#define MTCH6102_SUPPLY_RAMP_DELAY_MS 50

/** Maximum time to wait for shared-bus ownership, in milliseconds. */
#define MTCH6102_I2C_TIMEOUT 100

/**
 * @brief Internal driver lifecycle flags.
 * @details Packs the driver's progress through init, configuration, and
 *          acquisition into a single word. The union lets the whole flag set be
 *          read or cleared as one word, while the bit-field view is used to test
 *          and update individual milestones.
 */
union mtch6102_state_t {
	unsigned int value; /**< All lifecycle flags as one word; used to clear them together. */
	struct mtch6102_state_bits {
		unsigned int bInitialized : 1;	  /**< mtch6102_init() completed successfully. */
		unsigned int bProbed : 1;		  /**< A firmware-ID probe has been attempted. */
		unsigned int bDeviceFound : 1;	  /**< The probe matched a supported MTCH6102. */
		unsigned int bConfigured : 1;	  /**< The register configuration has been applied. */
		unsigned int bSampling : 1;		  /**< Interrupt-driven acquisition is active. */
		unsigned int bIrqConfigured : 1;  /**< The INT line is claimed and its callback armed. */
		unsigned int bSyncConfigured : 1; /**< The SYNC line is claimed and configured as input. */
		unsigned int bSupplyEnabled : 1;  /**< This driver enabled the supply rail. */
	} bits;
};

/**
 * @brief Internal singleton driver context.
 * @details Stores the I2C specification, the supply regulator, connector GPIO
 *          ownership, and the most recently decoded sample published with each
 *          event.
 */
static struct mtch6102_t {
	struct sys_i2c_dt_spec device;		 /**< Shared-bus target spec from devicetree. */
	const struct device* regulator;		 /**< Dedicated supply-rail regulator handle. */
	const struct gpio_dt_spec* irq_gpio; /**< INT line, claimed from the daughter-board arbiter. */
	const struct gpio_dt_spec*
		sync_gpio;					 /**< SYNC line, claimed from the daughter-board arbiter. */
	struct gpio_callback irq_cb;	 /**< GPIO callback registered on the INT line. */
	union mtch6102_state_t state;	 /**< Lifecycle flags. */
	struct mtch6102_config_t config; /**< Cached register configuration to apply. */
	struct touch_sensor_sample_t
		last_sample; /**< Most recently decoded sample, published per event. */
	time_t
		irq_timestamp; /**< Timestamp of the last interrupt in microseconds since the Unix epoch. */
} mtch6102 = {
	.device = SYS_I2C_DT_SPEC_GET(MTCH6102_NODE),
	.regulator = DEVICE_DT_GET(DT_PHANDLE(MTCH6102_NODE, vin_supply)),
	.irq_timestamp = -1, /**< No interrupt has been received yet. */
	/* All remaining members are zero-initialized by static storage duration. */
};

/** Printable name for each event identifier, indexed by ::mtch6102_event_type. */
static const char* const mtch6102_event_names[mtch6102_event_Count] = {
	[mtch6102_Irq] = "Irq",
	[mtch6102_event_TouchDetected] = "TouchDetected",
	[mtch6102_event_TouchReleased] = "TouchReleased",
	[mtch6102_event_SingleClick] = "SingleClick",
	[mtch6102_event_ClickAndHold] = "ClickAndHold",
	[mtch6102_event_DoubleClick] = "DoubleClick",
	[mtch6102_event_DownSwipe] = "DownSwipe",
	[mtch6102_event_DownSwipeAndHold] = "DownSwipeAndHold",
	[mtch6102_event_RightSwipe] = "RightSwipe",
	[mtch6102_event_RightSwipeAndHold] = "RightSwipeAndHold",
	[mtch6102_event_UpSwipe] = "UpSwipe",
	[mtch6102_event_UpSwipeAndHold] = "UpSwipeAndHold",
	[mtch6102_event_LeftSwipe] = "LeftSwipe",
	[mtch6102_event_LeftSwipeAndHold] = "LeftSwipeAndHold",
};

const char* mtch6102_event_name(enum mtch6102_event_type event_id) {
	if (event_id < 0 || event_id >= mtch6102_event_Count ||
		mtch6102_event_names[event_id] == NULL) {
		return "Unknown";
	}

	return mtch6102_event_names[event_id];
}

/**
 * @brief Map a raw GESTURE_STATE byte to a driver event identifier.
 * @details The switch maps the ::mtch6102_gesture_type codes onto the driver's
 *          software event namespace. ::mtch6102_gesture_None (touch only, no
 *          gesture) is reported as ::mtch6102_event_TouchDetected; every other
 *          recognized code maps to its click or swipe event.
 *
 * @param gesture_state Raw GESTURE_STATE register value.
 * @return The matching ::mtch6102_event_type, or ::mtch6102_event_Invalid for an
 *         unrecognized code.
 */
static enum mtch6102_event_type mtch6102_decode_gesture(uint8_t gesture_state) {
	switch (gesture_state) {
	case mtch6102_gesture_None:
		return mtch6102_event_TouchDetected;
	case mtch6102_gesture_SingleClick:
		return mtch6102_event_SingleClick;
	case mtch6102_gesture_ClickAndHold:
		return mtch6102_event_ClickAndHold;
	case mtch6102_gesture_DoubleClick:
		return mtch6102_event_DoubleClick;
	case mtch6102_gesture_DownSwipe:
		return mtch6102_event_DownSwipe;
	case mtch6102_gesture_DownSwipeAndHold:
		return mtch6102_event_DownSwipeAndHold;
	case mtch6102_gesture_RightSwipe:
		return mtch6102_event_RightSwipe;
	case mtch6102_gesture_RightSwipeAndHold:
		return mtch6102_event_RightSwipeAndHold;
	case mtch6102_gesture_UpSwipe:
		return mtch6102_event_UpSwipe;
	case mtch6102_gesture_UpSwipeAndHold:
		return mtch6102_event_UpSwipeAndHold;
	case mtch6102_gesture_LeftSwipe:
		return mtch6102_event_LeftSwipe;
	case mtch6102_gesture_LeftSwipeAndHold:
		return mtch6102_event_LeftSwipeAndHold;
	default:
		return mtch6102_event_Invalid;
	}
}

enum mtch6102_event_type mtch6102_sample_event(const struct touch_sensor_sample_t* sample) {
	enum mtch6102_event_type event;

	if (sample == NULL) {
		return mtch6102_event_Invalid;
	}

	event = mtch6102_decode_gesture(sample->gesture_state);
	if (event != mtch6102_event_Invalid && event != mtch6102_event_TouchDetected) {
		return event;
	}

	if (sample->position.touched) {
		return mtch6102_event_TouchDetected;
	}

	return mtch6102_event_TouchReleased;
}

/**
 * @brief Decode a raw touch-register burst into a ::mtch6102_position.
 * @details Expects the four bytes read starting at ::mtch6102_touch_TOUCHSTATE:
 *          rx[0] TOUCHSTATE, rx[1] TOUCHX MSB, rx[2] TOUCHY MSB, and rx[3] the
 *          packed X/Y low-nibble byte. The 12-bit coordinates are reassembled as
 *          `(MSB << 4) | low nibble`; when no touch is present the coordinates
 *          are forced to zero.
 *
 * @param pos Destination position structure.
 * @param rx  Four-byte touch-register burst starting at TOUCHSTATE.
 */
static void mtch6102_decode_position(struct mtch6102_position* pos, const uint8_t rx[4]) {
	union mtch6102_touchstate_register_t touch_state = {.value = rx[0]};

	pos->touch_state = (uint8_t) touch_state.value;
	pos->touched = touch_state.bits.tch != 0U;

	if (!pos->touched) {
		pos->x = 0U;
		pos->y = 0U;
		return;
	}

	pos->x = ((uint16_t) rx[1] << 4) | (uint16_t) (rx[3] & 0x0FU);
	pos->y = ((uint16_t) rx[2] << 4) | (uint16_t) ((rx[3] >> 4) & 0x0FU);
}

/** Acquire shared-I2C ownership for one high-level touch operation. */
static inline bool mtch6102_bus_lock(void) {
	int ret = sys_i2c_lock(&mtch6102.device, K_MSEC(MTCH6102_I2C_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("MTCH6102 failed to lock SYS_I2C (%d)", ret);
		return false;
	}

	return true;
}

/** Fully release nested bus ownership held for one high-level operation. */
static inline bool mtch6102_bus_unlock(void) {
	int ret = sys_i2c_release(&mtch6102.device);

	if (ret != 0) {
		LOG_ERR("MTCH6102 failed to release SYS_I2C ownership (%d)", ret);
		return false;
	}

	return true;
}

/** Write a single register byte. The caller must own the shared bus. */
static inline int mtch6102_write_register(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = {reg, value};

	return sys_i2c_write(&mtch6102.device, tx, sizeof(tx));
}

/**
 * @brief Probe for the MTCH6102 firmware identifier.
 * @details Reads the four core ID bytes (FWMajor, FWMinor, APPIDH, APPIDL) and
 *          accepts the observed firmware family and application ID; the minor
 *          revision is allowed to vary between parts. Updates the `bProbed` and
 *          `bDeviceFound` state flags as a side effect. The caller must already
 *          own the shared bus.
 *
 * @retval true  A supported MTCH6102 responded with a matching ID.
 * @retval false The read failed or the ID did not match.
 */
static bool mtch6102_probe(void) {
	uint8_t core[4] = {0U};
	uint8_t start_reg = mtch6102_core_FWMajor;
	int ret;

	mtch6102.state.bits.bProbed = 1U;

	ret = sys_i2c_write_read(&mtch6102.device, &start_reg, 1, core, sizeof(core));
	if (ret != 0) {
		LOG_WRN("MTCH6102 probe failed (%d)", ret);
		mtch6102.state.bits.bDeviceFound = 0U;
		return false;
	}

	/* Accept the observed MTCH6102 firmware family and application ID.
	 * The minor firmware revision can vary between parts and revisions. */
	mtch6102.state.bits.bDeviceFound =
		(core[0] == 0x02U && core[2] == 0x00U && core[3] == 0x12U) ? 1U : 0U;
	if (mtch6102.state.bits.bDeviceFound == 0U) {
		LOG_WRN("MTCH6102 firmware ID mismatch: %02x %02x %02x %02x",
				core[0],
				core[1],
				core[2],
				core[3]);
	} else if (core[1] != 0x05U) {
		LOG_INF("MTCH6102 firmware revision %02x %02x %02x %02x",
				core[0],
				core[1],
				core[2],
				core[3]);
	}

	return mtch6102.state.bits.bDeviceFound != 0U;
}

/* Defined in the interrupt section below; registered by mtch6102_gpio_init(). */
static void mtch6102_irq_callback(const struct device* dev,
								  struct gpio_callback* cb,
								  uint32_t pins);

/**
 * @brief Claim, configure, and arm the daughter-board SYNC and INT lines.
 * @details Claims both connector lines from the @ref sensewear_daughter_if
 *          arbiter (SYNC first, then INT), configures each as an input, registers
 *          the INT-pin GPIO callback, and arms the falling-edge interrupt (the
 *          MTCH6102 INT pin is open-collector and active-low, so the callback is
 *          in place before the interrupt is enabled). On any failure every line
 *          claimed within the call is released and the cached handles are
 *          cleared, so the driver is left with no partially owned GPIOs.
 *
 * @retval 0 Both lines were claimed and configured and the INT interrupt armed.
 * @retval -ENODEV A connector line could not be claimed.
 * @return A negative errno from the GPIO driver if pin configuration, callback
 *         registration, or interrupt arming failed.
 */
static int mtch6102_gpio_init(void) {
	int ret;

	if (mtch6102.sync_gpio == NULL) {
		mtch6102.sync_gpio = daughter_if_gpio_claim(MTCH6102_SYNC_LINE);
		if (mtch6102.sync_gpio == NULL) {
			LOG_ERR("MTCH6102 could not claim daughter-board SYNC line");
			return -ENODEV;
		}
	}

	if (mtch6102.irq_gpio == NULL) {
		mtch6102.irq_gpio = daughter_if_gpio_claim(MTCH6102_IRQ_LINE);
		if (mtch6102.irq_gpio == NULL) {
			LOG_ERR("MTCH6102 could not claim daughter-board INT line");
			(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
			mtch6102.sync_gpio = NULL;
			return -ENODEV;
		}
	}

	ret = gpio_pin_configure_dt(mtch6102.sync_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("MTCH6102 SYNC pin config failed (%d)", ret);
		(void) daughter_if_gpio_release(MTCH6102_IRQ_LINE);
		(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
		mtch6102.irq_gpio = NULL;
		mtch6102.sync_gpio = NULL;
		return ret;
	}

	mtch6102.state.bits.bSyncConfigured = 1U;

	ret = gpio_pin_configure_dt(mtch6102.irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("MTCH6102 INT pin config failed (%d)", ret);
		(void) daughter_if_gpio_release(MTCH6102_IRQ_LINE);
		(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
		mtch6102.irq_gpio = NULL;
		mtch6102.sync_gpio = NULL;
		mtch6102.state.bits.bSyncConfigured = 0U;
		return ret;
	}

	gpio_init_callback(&mtch6102.irq_cb, mtch6102_irq_callback, BIT(mtch6102.irq_gpio->pin));
	ret = gpio_add_callback(mtch6102.irq_gpio->port, &mtch6102.irq_cb);
	if (ret != 0) {
		LOG_ERR("MTCH6102 interrupt callback add failed (%d)", ret);
		(void) daughter_if_gpio_release(MTCH6102_IRQ_LINE);
		(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
		mtch6102.irq_gpio = NULL;
		mtch6102.sync_gpio = NULL;
		mtch6102.state.bits.bSyncConfigured = 0U;
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(mtch6102.irq_gpio, GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		LOG_ERR("MTCH6102 interrupt arm failed (%d)", ret);
		gpio_remove_callback(mtch6102.irq_gpio->port, &mtch6102.irq_cb);
		(void) daughter_if_gpio_release(MTCH6102_IRQ_LINE);
		(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
		mtch6102.irq_gpio = NULL;
		mtch6102.sync_gpio = NULL;
		mtch6102.state.bits.bSyncConfigured = 0U;
		return ret;
	}

	mtch6102.state.bits.bIrqConfigured = 1U;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Supply-rail ownership                                                      */
/* ------------------------------------------------------------------------- */

/**
 * @brief Validate the dedicated MTCH6102 supply rail and cache its regulator.
 * @details Mirrors the DRV2605 / MAX30101 power-up checks: an already-live shared
 *          rail must already sit at the fixed ::MTCH6102_SUPPLY_VOLTAGE_UV the
 *          MTCH6102 requires, otherwise driving it later risks damaging the part
 *          or another device on the rail. When the rail is not yet enabled the
 *          output voltage is programmed once here so a later enable brings it up
 *          at the correct level.
 *
 * @retval 0 The rail is ready (or no regulator is configured).
 * @retval -ENODEV The regulator device is not ready.
 * @retval -EINVAL The rail is already live at the wrong voltage.
 * @return A negative errno from the regulator API on read/set failure.
 */
static int mtch6102_supply_init(void) {
	int32_t current_uv;
	int ret;

	__ASSERT(mtch6102.regulator != NULL, "MTCH6102 regulator not specified");
	if (mtch6102.regulator == NULL) {
		return 0;
	}

	if (!device_is_ready(mtch6102.regulator)) {
		LOG_ERR("MTCH6102 regulator %s not ready", mtch6102.regulator->name);
		return -ENODEV;
	}

	if (regulator_is_enabled(mtch6102.regulator)) {

		ret = regulator_get_voltage(mtch6102.regulator, &current_uv);
		if (ret < 0) {
			LOG_ERR("Failed to read MTCH6102 regulator %s voltage (%d)",
					mtch6102.regulator->name,
					ret);
			return ret;
		}

		if (current_uv != MTCH6102_SUPPLY_VOLTAGE_UV) {
			LOG_ERR("MTCH6102 regulator %s already enabled at %d uV, requires fixed %d uV",
					mtch6102.regulator->name,
					current_uv,
					MTCH6102_SUPPLY_VOLTAGE_UV);
			return -EINVAL;
		}
		return 0;
	}
	// lets configure the output voltage once.
	ret = regulator_set_voltage(mtch6102.regulator,
								MTCH6102_SUPPLY_VOLTAGE_UV,
								MTCH6102_SUPPLY_VOLTAGE_UV);
	if (ret < 0) {
		LOG_ERR("Failed to set MTCH6102 rail to %d uV (%d)", MTCH6102_SUPPLY_VOLTAGE_UV, ret);
		return ret;
	}
	return 0;
}

/**
 * @brief Enable the dedicated supply rail and wait for it to settle.
 * @details Idempotent: a rail this driver already enabled is left untouched. If
 *          the rail is found already enabled by some other owner the call fails,
 *          since the driver cannot guarantee the correct voltage. After enabling
 *          it waits ::MTCH6102_SUPPLY_RAMP_DELAY_MS for the rail to ramp and the
 *          device to power up before returning.
 *
 * @retval 0 The rail is enabled and settled (or no regulator is configured).
 * @retval -EINVAL The rail was already enabled by another owner.
 * @return A negative errno from the regulator API on enable failure.
 */
static int mtch6102_supply_on(void) {
	int ret;

	if (mtch6102.regulator == NULL || mtch6102.state.bits.bSupplyEnabled) {
		return 0;
	}

	if (regulator_is_enabled(mtch6102.regulator)) {
		LOG_ERR("MTCH6102 regulator %s already enabled, but this driver did not enable it",
				mtch6102.regulator->name);
		return -EINVAL;
	}

	ret = regulator_enable(mtch6102.regulator);
	if (ret < 0) {
		LOG_ERR("Failed to enable MTCH6102 rail (%d)", ret);
		return ret;
	}

	k_msleep(MTCH6102_SUPPLY_RAMP_DELAY_MS);
	mtch6102.state.bits.bSupplyEnabled = 1U;
	return 0;
}

/**
 * @brief Disable the dedicated supply rail if this driver enabled it.
 * @details A no-op when no regulator is configured or the rail was not enabled by
 *          this driver, so it never disturbs a rail owned elsewhere.
 */
static void mtch6102_supply_off(void) {
	if (mtch6102.regulator == NULL || !mtch6102.state.bits.bSupplyEnabled) {
		return;
	}

	(void) regulator_disable(mtch6102.regulator);
	mtch6102.state.bits.bSupplyEnabled = 0U;
}

int mtch6102_start(void) {
	int ret;

	if (!mtch6102.state.bits.bInitialized) {
		ret = mtch6102_init();
		if (ret != 0) {
			return ret;
		}
	}

	if (mtch6102.state.bits.bSampling != 0U) {
		return 0;
	}

	if (mtch6102.state.bits.bConfigured == 0U) {
		ret = mtch6102_config(&mtch6102.config);
		if (ret != 0) {
			LOG_ERR("MTCH6102 configuration failed (%d)", ret);
			return ret;
		}
	}

	ret = mtch6102_supply_on();
	if (ret != 0) {
		LOG_ERR("MTCH6102 supply power-on failed (%d)", ret);
		return ret;
	}

	/* The INT line is armed once in mtch6102_init(); powering the rail is
	 * all that is needed to begin receiving touch interrupts. */
	mtch6102.state.bits.bSampling = 1U;
	return 0;
}
/* ------------------------------------------------------------------------- */
/* Event publication                                                          */
/* ------------------------------------------------------------------------- */

/**
 * @brief Publish a decoded event through the shared device-event manager.
 * @details Thread-context variant. Posts with K_NO_WAIT and drops the return
 *          value: event delivery is best-effort and never blocks the caller.
 *
 * @param event   Decoded ::mtch6102_event_type to publish.
 * @param v_param Scalar payload (the raw TOUCH_STATE byte).
 * @param p_param Pointer payload (the decoded ::touch_sensor_sample).
 */
static inline void mtch6102_post_event(enum mtch6102_event_type event,
									   uint32_t v_param,
									   uintptr_t p_param) {
	(void) device_driver_event_post(MTCH6102_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									p_param,
									K_NO_WAIT);
}

/**
 * @brief Publish an event from ISR context.
 * @details ISR-safe variant used by the GPIO callback to post ::mtch6102_Irq.
 *          Carries no pointer payload and drops the return value.
 *
 * @param event   Event identifier to publish, ::mtch6102_Irq at INT time.
 * @param v_param Scalar payload (the interrupting pin mask).
 */
static inline void mtch6102_post_event_isr(enum mtch6102_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(MTCH6102_DEVICE_DTS_ID,
										(uint32_t) event,
										v_param,
										(uintptr_t) NULL);
}

/** GPIO ISR that posts the raw interrupt notification event. */
static void mtch6102_irq_callback(const struct device* dev,
								  struct gpio_callback* cb,
								  uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	mtch6102.irq_timestamp = rtc_get_timestamp_us();
	mtch6102_post_event_isr(mtch6102_Irq, pins);
}

/**
 * @brief Read and decode a full touch/gesture sample.
 * @details Acquires the shared bus, reads the six-byte burst from
 *          ::mtch6102_touch_TOUCHSTATE (position plus GESTURE_STATE), decodes the
 *          position into @p sample, stores the raw gesture byte, and releases the
 *          bus. Unlike the register helpers, this routine owns the bus for its
 *          own transfer.
 *
 * @param sample Destination sample structure.
 * @retval 0 The sample was read and decoded.
 * @retval -EINVAL @p sample was NULL.
 * @retval -EIO The shared bus could not be acquired.
 * @return A negative errno if the I2C transfer failed.
 */
static int mtch6102_read_sample(struct touch_sensor_sample_t* sample) {
	uint8_t gesture_state;
	uint8_t start_reg = mtch6102_touch_TOUCHSTATE;
	uint8_t rx[6] = {0U};
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	if (!mtch6102_bus_lock()) {
		return -EIO;
	}

	ret = sys_i2c_write_read(&mtch6102.device, &start_reg, 1, rx, sizeof(rx));
	if (ret != 0) {
		LOG_ERR("MTCH6102 I2C touch read failed (%d)", ret);
		mtch6102_bus_unlock();
		return ret;
	}

	mtch6102_decode_position(&sample->position, rx);
	gesture_state = rx[4];
	sample->gesture_state = gesture_state;
	sample->timestamp = (mtch6102.irq_timestamp == -1) ? rtc_get_timestamp_us()
													   : mtch6102.irq_timestamp;

	mtch6102_bus_unlock();
	return 0;
}

int mtch6102_irq_handler(void) {
	enum mtch6102_event_type event;
	int ret;

	if (!mtch6102.state.bits.bInitialized) {
		return -EAGAIN;
	}

	ret = mtch6102_read_sample(&mtch6102.last_sample);
	if (ret != 0) {
		LOG_ERR("MTCH6102 sample read failed: %d", ret);
		return ret;
	}

	event = mtch6102_sample_event(&mtch6102.last_sample);
	LOG_DBG("MTCH6102 sample event: %s", mtch6102_event_name(event));

	if (event != mtch6102_event_Invalid) {
		mtch6102_post_event(event,
							mtch6102.last_sample.position.touch_state,
							(uintptr_t) &mtch6102.last_sample);
	}

	return 0;
}

void mtch6102_get_default_config(struct mtch6102_config_t* config) {
	if (config == NULL) {
		return;
	}

	memset(config, 0, sizeof(*config));

	config->cmd.bits.cfg = 1U;
	config->mode.bits.mode = 0x03U;
	config->modecon.value = 0U;

	/* SenseWear uses a single row of 15 pads, wired as 12 X channels and
	 * 3 Y channels on the trailing end of the row. */
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_NumberOfXChannels)] = 0x0CU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_NumberOfYChannels)] = 0x03U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_ScanCount)] = 0x06U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TouchThreshX)] = 0x37U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TouchThreshY)] = 0x28U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_ActivePeriodL)] = 0x85U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_ActivePeriodH)] = 0x02U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_IdlePeriodL)] = 0x4CU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_IdlePeriodH)] = 0x06U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_IdleTimeout)] = 0x10U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_Hysteresis)] = 0x04U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DebounceUp)] = 0x01U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DebounceDown)] = 0x01U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BaseIntervalL)] = 0x0AU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BaseIntervalH)] = 0x00U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BasePosFilter)] = 0x14U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BaseNegFilter)] = 0x14U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_FilterType)] = 0x02U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_FilterStrength)] = 0x01U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BaseFilterType)] = 0x01U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_BaseFilterStrength)] = 0x05U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_LargeActivationThreshL)] =
		0x00U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_LargeActivationThreshH)] =
		0x00U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_HorizontalSwipeDistance)] =
		0x40U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_VerticalSwipeDistance)] =
		0x40U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_SwipeHoldBoundary)] = 0x19U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TapDistance)] = 0x19U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DistanceBetweenTaps)] =
		0x40U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TapHoldTimeL)] = 0x32U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TapHoldTimeH)] = 0x00U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_GestureClickTime)] = 0x0CU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_SwipeHoldThresh)] = 0x20U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_MinSwipeVelocity)] = 0x04U;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_HorizontalGestureAngle)] =
		0x2DU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_VerticalGestureAngle)] =
		0x2DU;
	config->configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_I2CAddr)] = 0x25U;
}

/**
 * @brief Write a configuration object to the device registers.
 * @details Writes the configuration register block (NumberOfXChannels through
 *          VerticalGestureAngle, in register order), then the MODE and MODECON
 *          decode-control registers, and finally the CMD register with the `cfg`
 *          bit set to latch the block into the running configuration. The I2CAddr
 *          register is skipped so the device keeps its devicetree-fixed address,
 *          and the read-only firmware-ID and status registers are never touched.
 *          The caller must already own the shared bus.
 *
 * @param config Configuration to apply.
 * @retval 0 Every register was written.
 * @retval -EINVAL @p config was NULL.
 * @return A negative errno if a register transfer failed.
 */
static int mtch6102_apply_config(const struct mtch6102_config_t* config) {
	union mtch6102_cmd_register_t cmd;
	union mtch6102_mode_register_t mode;
	union mtch6102_modecon_register_t modecon;
	uint8_t reg;
	int ret;

	if (config == NULL) {
		return -EINVAL;
	}

	for (uint8_t idx = 0U; idx < MTCH6102_CONFIGURATION_REGISTER_COUNT; ++idx) {
		reg = (uint8_t) (mtch6102_config_NumberOfXChannels + idx);
		/*
		 * Never reprogram the I2C address: the device address is fixed by
		 * devicetree, so no configuration may move the part off its default.
		 */
		if (reg == (uint8_t) mtch6102_config_I2CAddr) {
			continue;
		}
		ret = mtch6102_write_register(reg, config->configuration[idx]);
		if (ret != 0) {
			LOG_ERR("MTCH6102 configuration register 0x%02x write failed (%d)", reg, ret);
			return ret;
		}
	}

	/* Select the touch/gesture decode mode and raw-ADC control. The firmware
	 * ID (0x00-0x03) and touch/gesture status (0x10-0x15) registers are
	 * read-only. */
	mode = config->mode;
	ret = mtch6102_write_register(mtch6102_core_MODE, (uint8_t) mode.value);
	if (ret != 0) {
		LOG_ERR("MTCH6102 MODE write failed (%d)", ret);
		return ret;
	}

	modecon = config->modecon;
	ret = mtch6102_write_register(mtch6102_core_MODECON, (uint8_t) modecon.value);
	if (ret != 0) {
		LOG_ERR("MTCH6102 MODECON write failed (%d)", ret);
		return ret;
	}

	/* Apply the configuration last: the CMD `cfg` bit latches the register
	 * block written above into the running configuration. */
	cmd = config->cmd;
	cmd.bits.cfg = 1U;
	ret = mtch6102_write_register(mtch6102_core_CMD, (uint8_t) cmd.value);
	if (ret != 0) {
		LOG_ERR("MTCH6102 CMD write failed (%d)", ret);
		return ret;
	}

	return 0;
}

int mtch6102_config(struct mtch6102_config_t* config) {
	int ret;

	if (!mtch6102.state.bits.bInitialized) {
		LOG_ERR("MTCH6102 not initialized");
		return -EINVAL;
	}

	if (mtch6102.state.bits.bSampling != 0U) {
		LOG_ERR("MTCH6102 is currently sampling");
		return -EBUSY;
	}

	if (config == NULL) {
		mtch6102_get_default_config(&mtch6102.config);
	} else {
		memcpy(&mtch6102.config, config, sizeof(mtch6102.config));
	}

	ret = mtch6102_supply_on();
	if (ret != 0) {
		LOG_ERR("MTCH6102 supply power-on failed (%d)", ret);
		return ret;
	}

	if (!mtch6102_bus_lock()) {
		LOG_ERR("MTCH6102 failed to lock SYS_I2C bus");
		return -EIO;
	}

	ret = mtch6102_apply_config(&mtch6102.config);
	mtch6102_bus_unlock();
	if (ret != 0) {
		LOG_ERR("MTCH6102 configuration failed (%d)", ret);
		return ret;
	}

	mtch6102.state.bits.bConfigured = 1U;
	mtch6102.state.bits.bSampling = 0U;
	return 0;
}

bool mtch6102_is_ready(void) {
	return mtch6102.state.bits.bInitialized && mtch6102.state.bits.bDeviceFound &&
		   mtch6102.state.bits.bConfigured;
}

int mtch6102_get_position(struct mtch6102_position* pos) {
	uint8_t start_reg = mtch6102_touch_TOUCHSTATE;
	uint8_t rx[4] = {0U};
	int ret;

	if (pos == NULL) {
		LOG_ERR("MTCH6102 position pointer is NULL");
		return -EINVAL;
	}

	if (!mtch6102_bus_lock()) {
		LOG_ERR("MTCH6102 failed to lock SYS_I2C bus");
		return -EIO;
	}

	ret = sys_i2c_write_read(&mtch6102.device, &start_reg, 1, rx, sizeof(rx));
	mtch6102_bus_unlock();
	if (ret != 0) {
		LOG_ERR("MTCH6102 failed reading touch position (%d)", ret);
		return ret;
	}

	mtch6102_decode_position(pos, rx);
	if (!pos->touched) {
		LOG_WRN("MTCH6102 no touch detected");
		return -ENODATA;
	}

	return 0;
}

void mtch6102_stop(void) {
	mtch6102.state.bits.bSampling = 0U;
	mtch6102_supply_off();
}

int mtch6102_init(void) {
	int ret;

	if (mtch6102.state.bits.bInitialized) {
		return 0;
	}

	if (!sys_i2c_is_ready(&mtch6102.device)) {
		LOG_ERR("MTCH6102 SYS_I2C bus is not ready");
		return -ENODEV;
	}

	ret = mtch6102_supply_init();
	if (ret != 0) {
		LOG_ERR("MTCH6102 supply init failed (%d)", ret);
		return ret;
	}
	// we must enable the supply before we probe the device, otherwise the probe will fail
	ret = mtch6102_supply_on();
	if (ret != 0) {
		LOG_ERR("MTCH6102 supply on failed (%d) from init", ret);
		return ret;
	}
	if (!mtch6102_bus_lock()) {
		LOG_ERR("MTCH6102 failed to lock SYS_I2C bus");
		return -EIO;
	}
	ret = mtch6102_probe() ? 0 : -ENODEV;
	mtch6102_bus_unlock();
	if (ret != 0) {
		LOG_ERR("MTCH6102 probe failed (%d)", ret);
		return ret;
	}

	ret = mtch6102_gpio_init();
	if (ret != 0) {
		LOG_ERR("MTCH6102 GPIO init failed (%d)", ret);
		return ret;
	}

	mtch6102_get_default_config(&mtch6102.config);
	mtch6102.state.bits.bInitialized = 1U;
	mtch6102.state.bits.bConfigured = 0U;
	return 0;
}
