/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mtch6102.c
 * @brief SensWear MTCH6102 acquisition and host slider decoding.
 *
 * SYNC frame completion schedules bounded, coalesced thread-context reads.
 * Public operations serialize through mtch6102_mutex; register helpers run
 * only while their caller owns SYS_I2C. Events own immutable pool samples.
 * See mtch6102.h and the shield README for mapping, lifecycle and ownership.
 */

#include "mtch6102.h"
#include "mtch6102_slider.h"
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
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(mtch6102, CONFIG_LOG_DEFAULT_LEVEL);

/** Devicetree node identifier for the shield's MTCH6102 instance. */
#define MTCH6102_NODE DT_ALIAS(senswear_touch)
BUILD_ASSERT(DT_NODE_HAS_STATUS(MTCH6102_NODE, okay),
			 "Touch firmware requires the senswear_touch shield");
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

/* Recovery polling is independent of the shared event queue. No I2C in ISR. */
#define MTCH6102_RECOVERY_MS 100
K_MUTEX_DEFINE(mtch6102_mutex);
K_MEM_SLAB_DEFINE(mtch6102_samples, sizeof(struct touch_sensor_sample_t), 8, 8);
static atomic_t sampling;
static atomic_t irq_pending;
static atomic_t frame_sequence;
static void mtch6102_recovery_tick(struct k_timer* timer);
K_TIMER_DEFINE(mtch6102_recovery_timer, mtch6102_recovery_tick, NULL);

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
		unsigned int bIrqConfigured : 1;  /**< The SYNC frame callback is armed. */
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
	struct gpio_callback irq_cb;	 /**< GPIO callback registered on SYNC. */
	union mtch6102_state_t state;	 /**< Lifecycle flags. */
	struct mtch6102_config_t config; /**< Cached register configuration to apply. */
	struct touch_sensor_sample_t
		last_sample; /**< Most recently decoded sample, published per event. */
	struct mtch6102_slider_config slider_config;
	struct mtch6102_slider_state slider;
	atomic_val_t last_frame_sequence;
	int64_t last_read_ms;
	bool sample_valid;
} mtch6102 = {
	.device = SYS_I2C_DT_SPEC_GET(MTCH6102_NODE),
	.regulator = DEVICE_DT_GET(DT_PHANDLE(MTCH6102_NODE, vin_supply)),
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
 * @retval 0 A supported MTCH6102 responded with a matching ID.
 * @retval -ENODEV The firmware ID did not match.
 * @return The original negative errno if the I2C read failed.
 */
static int mtch6102_probe(void) {
	uint8_t core[4] = {0U};
	uint8_t start_reg = mtch6102_core_FWMajor;
	int ret;

	mtch6102.state.bits.bProbed = 1U;

	ret = sys_i2c_write_read(&mtch6102.device, &start_reg, 1, core, sizeof(core));
	if (ret != 0) {
		LOG_WRN("MTCH6102 probe failed (%d)", ret);
		mtch6102.state.bits.bDeviceFound = 0U;
		return ret;
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

	return mtch6102.state.bits.bDeviceFound != 0U ? 0 : -ENODEV;
}

/* Defined in the interrupt section below; registered by mtch6102_gpio_init(). */
static void mtch6102_irq_callback(const struct device* dev,
								  struct gpio_callback* cb,
								  uint32_t pins);

/**
 * @brief Claim INT/SYNC as inputs and arm the SYNC frame-complete callback.
 * On failure, release all connector lines claimed by this call.
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

	/* SYNC falls after EVERY acquisition frame, even when the controller's
	 * 2D decoder does not see a touch. INT alone misses single-bank touches.
	 * Microchip DS40001750A section 5.3 explicitly supports host decoding. */
	gpio_init_callback(&mtch6102.irq_cb, mtch6102_irq_callback, BIT(mtch6102.sync_gpio->pin));
	ret = gpio_add_callback(mtch6102.sync_gpio->port, &mtch6102.irq_cb);
	if (ret != 0) {
		LOG_ERR("MTCH6102 SYNC callback add failed (%d)", ret);
		(void) daughter_if_gpio_release(MTCH6102_IRQ_LINE);
		(void) daughter_if_gpio_release(MTCH6102_SYNC_LINE);
		mtch6102.irq_gpio = NULL;
		mtch6102.sync_gpio = NULL;
		mtch6102.state.bits.bSyncConfigured = 0U;
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(mtch6102.sync_gpio, GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		LOG_ERR("MTCH6102 SYNC interrupt arm failed (%d)", ret);
		gpio_remove_callback(mtch6102.sync_gpio->port, &mtch6102.irq_cb);
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
 * @details An already-live shared rail must already sit at the board-specific
 *          ::MTCH6102_SUPPLY_VOLTAGE_UV. Retuning another owner's live rail could
 *          violate its device requirements. When the rail is not yet enabled the
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

/* Coalesce frame notifications so a slow consumer cannot fill the event queue
 * with redundant reads. A failed post is retried by the next frame/timer. */
static void mtch6102_request_sample(void) {
	if (!atomic_get(&sampling) || !atomic_cas(&irq_pending, 0, 1)) {
		return;
	}
	int ret = device_driver_event_post_isr(MTCH6102_DEVICE_DTS_ID, mtch6102_Irq, 0, 0);
	if (ret != 0) {
		atomic_clear(&irq_pending);
	}
}

static void mtch6102_recovery_tick(struct k_timer* timer) {
	ARG_UNUSED(timer);
	mtch6102_request_sample();
}

static void mtch6102_irq_callback(const struct device* dev,
								  struct gpio_callback* cb,
								  uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	atomic_inc(&frame_sequence);
	mtch6102_request_sample();
}

static int mtch6102_start_locked(void) {
	int ret = mtch6102_init();
	if (ret != 0 || atomic_get(&sampling)) {
		return ret;
	}
	if (!mtch6102.state.bits.bConfigured) {
		ret = mtch6102_config(&mtch6102.config);
		if (ret != 0) {
			return ret;
		}
	}
	if (!mtch6102_bus_lock()) {
		return -EIO;
	}
	ret = mtch6102_write_register(mtch6102_core_MODE, (uint8_t) mtch6102.config.mode.value);
	if (!mtch6102_bus_unlock() && ret == 0) {
		ret = -EIO;
	}
	if (ret != 0) {
		return ret;
	}
	mtch6102_slider_reset(&mtch6102.slider);
	mtch6102.sample_valid = false;
	mtch6102.last_read_ms = k_uptime_get();
	mtch6102.last_frame_sequence = atomic_get(&frame_sequence);
	mtch6102.state.bits.bSampling = 1U;
	atomic_set(&sampling, 1);
	k_timer_start(&mtch6102_recovery_timer,
				  K_MSEC(MTCH6102_RECOVERY_MS),
				  K_MSEC(MTCH6102_RECOVERY_MS));
	mtch6102_request_sample();
	return 0;
}

/* All reads occur between SYNC frames. Reject a burst if acquisition overlapped
 * it, rather than mixing electrode values from different frames. */
static int mtch6102_read_sample(struct touch_sensor_sample_t* sample) {
	uint8_t status[6];
	uint8_t sensors[MTCH6102_SLIDER_ELECTRODES];
	uint8_t reg = mtch6102_touch_TOUCHSTATE;
	atomic_val_t sequence = atomic_get(&frame_sequence);
	int64_t now_ms = k_uptime_get();
	int ret;

	if (mtch6102.sample_valid && sequence == mtch6102.last_frame_sequence &&
		now_ms - mtch6102.last_read_ms < MTCH6102_RECOVERY_MS) {
		return -EAGAIN;
	}
	if (!mtch6102_bus_lock()) {
		return -EIO;
	}
	ret = gpio_pin_get_raw(mtch6102.sync_gpio->port, mtch6102.sync_gpio->pin);
	if (ret > 0) {
		ret = -EAGAIN;
	} else if (ret == 0) {
		ret = sys_i2c_write_read(&mtch6102.device, &reg, 1, status, sizeof(status));
		if (ret == 0) {
			reg = mtch6102_acquisition_SENSORVALUES_RX0;
			ret = sys_i2c_write_read(&mtch6102.device, &reg, 1, sensors, sizeof(sensors));
		}
		if (ret == 0) {
			ret = gpio_pin_get_raw(mtch6102.sync_gpio->port, mtch6102.sync_gpio->pin);
			if (ret > 0 || sequence != atomic_get(&frame_sequence)) {
				ret = -EAGAIN;
			}
		}
	}
	if (!mtch6102_bus_unlock() && ret == 0) {
		ret = -EIO;
	}
	if (ret != 0) {
		return ret;
	}

	struct mtch6102_slider_output output;
	/* A long bus/consumer outage must not turn a stale finger into a hold or
	 * join two unrelated contacts into a swipe or double tap. */
	bool reset = now_ms - mtch6102.last_read_ms > 4 * MTCH6102_RECOVERY_MS;
	mtch6102_slider_process(&mtch6102.slider,
							&mtch6102.slider_config,
							sensors,
							status[0],
							(uint64_t) now_ms,
							reset,
							&output);
	sample->timestamp = rtc_get_timestamp_us();
	sample->position = output.position;
	sample->gesture_state = output.gesture;
	mtch6102.last_frame_sequence = sequence;
	mtch6102.last_read_ms = now_ms;
	return 0;
}

void mtch6102_release_sample(const struct touch_sensor_sample_t* sample) {
	if (sample != NULL) {
		k_mem_slab_free(&mtch6102_samples, (void*) sample);
	}
}

static int mtch6102_irq_handler_locked(void) {
	struct touch_sensor_sample_t sample;
	struct touch_sensor_sample_t* queued_sample;
	int ret;

	atomic_clear(&irq_pending);
	if (!atomic_get(&sampling)) {
		return -EAGAIN;
	}
	/* Reserve ownership before advancing the gesture state. */
	ret = k_mem_slab_alloc(&mtch6102_samples, (void**) &queued_sample, K_NO_WAIT);
	if (ret != 0) {
		return ret;
	}
	ret = mtch6102_read_sample(&sample);
	if (ret != 0) {
		mtch6102_release_sample(queued_sample);
		if (ret != -EAGAIN) {
			LOG_WRN("MTCH6102 sample read failed (%d); acquisition will retry", ret);
		}
		return ret;
	}
	bool publish = !mtch6102.sample_valid || sample.position.touched ||
				   mtch6102.last_sample.position.touched ||
				   sample.gesture_state != mtch6102_gesture_None;
	mtch6102.last_sample = sample;
	mtch6102.sample_valid = true;
	if (!publish) {
		mtch6102_release_sample(queued_sample);
		return 0;
	}
	*queued_sample = sample;
	ret = device_driver_event_post(MTCH6102_DEVICE_DTS_ID,
								   mtch6102_sample_event(&sample),
								   sample.position.touch_state,
								   (uintptr_t) queued_sample,
								   K_NO_WAIT);
	if (ret != 0) {
		mtch6102_release_sample(queued_sample);
		/* Retry the latest state even if the lost event was a release. */
		mtch6102.sample_valid = false;
	}
	return ret;
}

void mtch6102_get_default_config(struct mtch6102_config_t* config) {
	if (config == NULL) {
		return;
	}

	memset(config, 0, sizeof(*config));

	config->cmd.bits.cfg = 1U;
	config->mode.bits.mode = 0x03U;
	config->modecon.value = 0U;

	/* SensWear uses a single row of 15 pads, wired as 12 X channels and
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
	/* CFG self-clears on completion (DS40001750A section 8). Wait before
	 * reporting readiness to the application. */
	int64_t deadline = k_uptime_get() + 250;
	uint8_t pending;
	reg = mtch6102_core_CMD;
	do {
		ret = sys_i2c_write_read(&mtch6102.device, &reg, 1, &pending, 1);
		if (ret != 0) {
			LOG_ERR("MTCH6102 configuration completion read failed (%d)", ret);
			return ret;
		}
		if ((pending & BIT(5)) == 0) {
			return 0;
		}
		k_msleep(5);
	} while (k_uptime_get() < deadline);
	LOG_ERR("MTCH6102 configuration completion timed out");
	return -ETIMEDOUT;
}

static int mtch6102_config_locked(const struct mtch6102_config_t* config) {
	struct mtch6102_config_t next;
	struct mtch6102_slider_config slider;
	int ret;

	if (!mtch6102.state.bits.bInitialized) {
		/* Retry initialization with configuration after a transient probe failure.
		 * The public entry point uses the same recursive driver mutex. */
		ret = mtch6102_init();
		if (ret != 0) {
			return ret;
		}
	}
	if (atomic_get(&sampling)) {
		return -EBUSY;
	}
	if (config == NULL) {
		mtch6102_get_default_config(&next);
	} else {
		next = *config;
	}
	mtch6102_slider_config_defaults(&slider);
#define CFG(reg) next.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_##reg)]
	/* This shield wires all 15 channels; the controller requires >=3 per
	 * bank. Reject configurations that silently disable physical pads. */
	slider.x_channels = CFG(NumberOfXChannels);
	if (slider.x_channels < 3 || CFG(NumberOfYChannels) < 3 ||
		slider.x_channels + CFG(NumberOfYChannels) != MTCH6102_SLIDER_ELECTRODES ||
		next.mode.value < 1 || next.mode.value > 3 || (next.cmd.value & ~BIT(5)) != 0) {
		LOG_ERR("MTCH6102 invalid configuration: X=%u Y=%u MODE=0x%x CMD=0x%x",
				slider.x_channels,
				CFG(NumberOfYChannels),
				next.mode.value,
				next.cmd.value);
		return -EINVAL;
	}
	slider.threshold_x = CFG(TouchThreshX);
	slider.threshold_y = CFG(TouchThreshY);
	slider.hysteresis = CFG(Hysteresis);
	slider.debounce_down = CFG(DebounceDown);
	slider.debounce_up = CFG(DebounceUp);
	slider.tap_distance = CFG(TapDistance);
	slider.double_tap_distance = CFG(DistanceBetweenTaps);
	slider.swipe_distance = CFG(HorizontalSwipeDistance);
	slider.swipe_hold_boundary = CFG(SwipeHoldBoundary);
	/* Host gesture timing is in monotonic milliseconds, independent of
	 * active/idle scan periods. See the slider defaults and shield README. */
#undef CFG
	if (!mtch6102_slider_config_valid(&slider)) {
		LOG_ERR("MTCH6102 invalid slider thresholds or gesture distances");
		return -EINVAL;
	}
	ret = mtch6102_supply_on();
	if (ret != 0) {
		return ret;
	}
	if (!mtch6102_bus_lock()) {
		return -EIO;
	}
	mtch6102.state.bits.bConfigured = 0U;
	ret = mtch6102_apply_config(&next);
	if (!mtch6102_bus_unlock() && ret == 0) {
		ret = -EIO;
	}
	if (ret != 0) {
		return ret;
	}
	mtch6102.config = next;
	mtch6102.slider_config = slider;
	mtch6102_slider_reset(&mtch6102.slider);
	mtch6102.sample_valid = false;
	mtch6102.state.bits.bConfigured = 1U;
	return 0;
}

bool mtch6102_is_ready(void) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	bool ready = mtch6102.state.bits.bInitialized && mtch6102.state.bits.bDeviceFound &&
				 mtch6102.state.bits.bConfigured;
	k_mutex_unlock(&mtch6102_mutex);
	return ready;
}

int mtch6102_get_position(struct mtch6102_position* pos) {
	if (pos == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	int ret = -EAGAIN;
	if (mtch6102.sample_valid && atomic_get(&sampling)) {
		*pos = mtch6102.last_sample.position;
		ret = pos->touched ? 0 : -ENODATA;
	}
	k_mutex_unlock(&mtch6102_mutex);
	return ret;
}

void mtch6102_stop(void) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	atomic_clear(&sampling);
	k_timer_stop(&mtch6102_recovery_timer);
	mtch6102.state.bits.bSampling = 0U;
	mtch6102.sample_valid = false;
	mtch6102_slider_reset(&mtch6102.slider);
	/* Keep the rail powered: the unpowered controller can hold shared I2C.
	 * Standby (DS40001750A section 7) stops sensing and baseline updates. */
	if (mtch6102.state.bits.bInitialized) {
		int ret = -EIO;
		if (mtch6102_bus_lock()) {
			ret = mtch6102_write_register(mtch6102_core_MODE, 0U);
			if (!mtch6102_bus_unlock() && ret == 0) {
				ret = -EIO;
			}
		}
		if (ret != 0) {
			LOG_ERR("MTCH6102 standby failed (%d)", ret);
		}
	}
	k_mutex_unlock(&mtch6102_mutex);
}

static int mtch6102_init_locked(void) {
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
	ret = mtch6102_probe();
	if (!mtch6102_bus_unlock() && ret == 0) {
		ret = -EIO;
	}
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

/* Lifecycle and frame processing may be requested by different threads. The
 * recursive Zephyr mutex also permits start -> init/config safely. */
int mtch6102_init(void) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	int ret = mtch6102_init_locked();
	k_mutex_unlock(&mtch6102_mutex);
	return ret;
}

int mtch6102_config(const struct mtch6102_config_t* config) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	int ret = mtch6102_config_locked(config);
	k_mutex_unlock(&mtch6102_mutex);
	return ret;
}

int mtch6102_start(void) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	int ret = mtch6102_start_locked();
	k_mutex_unlock(&mtch6102_mutex);
	return ret;
}

int mtch6102_irq_handler(void) {
	k_mutex_lock(&mtch6102_mutex, K_FOREVER);
	int ret = mtch6102_irq_handler_locked();
	k_mutex_unlock(&mtch6102_mutex);
	return ret;
}
