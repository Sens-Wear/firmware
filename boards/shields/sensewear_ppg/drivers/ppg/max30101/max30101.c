/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30101.c
 * @brief SenseWear MAX30101 PPG sensor driver implementation.
 * @details The driver is a daughter-board singleton. Its connection is derived
 *          from `DT_NODELABEL(max30101)`, while all transfers are routed through
 *          @ref sensewear_sys_i2c. Public hardware operations own the shared bus
 *          for their complete register sequence and finish with sys_i2c_release().
 *
 * Register helpers in this file intentionally do not lock. They may only be
 * called from a high-level operation that already owns the shared bus.
 *
 * The interrupt line is obtained from the daughter-board connector arbiter
 * (@ref sensewear_daughter_if, line ::daughter_if_GPIO1). Its GPIO callback posts
 * ::max30101_Irq from ISR context; max30101_irq_handler() then runs from thread
 * context to decode the interrupt sources and drain the FIFO into the internal
 * sample stream.
 */

#include "max30101.h"
#include "max30101_config.h"
#include "max30101_registers.h"
#include "rtc.h"
#include "sys_i2c.h"
#include "daughter_if.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <string.h>
#include <time.h>
#include <errno.h>

LOG_MODULE_REGISTER(max30101, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Devicetree node identifier for the shield's MAX30101 instance.
 * @details Maps the `max30101` node label to the node used for constructing the
 *          shared-I2C specification.
 */
#define MAX30101_NODE DT_NODELABEL(max30101)
// check the regulator property is present and valid; the driver uses it to power the rail
BUILD_ASSERT(DT_NODE_HAS_PROP(MAX30101_NODE, vin_supply),
			 "Invalid regulator device specified for MAX30101");

/** Connector line carrying the MAX30101 INT signal. */
#define MAX30101_IRQ_LINE daughter_if_GPIO1

/** Number of bytes the FIFO emits per active LED channel. */
#define MAX30101_BYTES_PER_CHANNEL (3)
/** Depth of the device FIFO in samples. */
#ifndef MAX30101_FIFO_DEPTH
#define MAX30101_FIFO_DEPTH (32)
#endif
BUILD_ASSERT(MAX30101_FIFO_DEPTH <= 32, "MAX30101 FIFO depth must be <= 32");
BUILD_ASSERT(MAX30101_FIFO_DEPTH > 0, "MAX30101 FIFO depth must be greater than 0");
#define MAX30101_FIFO_READ_SAMPLES (MAX30101_FIFO_DEPTH * MAX30101_BYTES_PER_CHANNEL)
/** Largest single register burst this driver writes (address + payload). */
#define MAX30101_I2C_TX_MAX (4)

/** Minimum and maximum voltages for each LED channel. @{*/
#define MAX30101_LED_VOLTAGE_RED_MIN_UV (3100000u)
#define MAX30101_LED_VOLTAGE_RED_MAX_UV (5000000u)
#define MAX30101_LED_VOLTAGE_IR_MIN_UV (3100000u)
#define MAX30101_LED_VOLTAGE_IR_MAX_UV (5000000u)
#define MAX30101_LED_VOLTAGE_GREEN_MIN_UV (4500000u)
#define MAX30101_LED_VOLTAGE_GREEN_MAX_UV (5500000u)
/** @} */
/** Minimum and maximum voltages for the PPG LEDs as a group. @{*/
#define MAX30101_PPG_VOLTAGE_MIN_UV (MAX30101_LED_VOLTAGE_RED_MIN_UV)
#define MAX30101_PPG_VOLTAGE_MAX_UV (MAX30101_LED_VOLTAGE_GREEN_MAX_UV)
/** @} */

BUILD_ASSERT(DT_NODE_HAS_STATUS(MAX30101_NODE, okay),
			 "PPG firmware requires the sensewear_ppg shield");

/** Internal driver lifecycle flags (private to the implementation). */
union max30101_state_t {
	unsigned int value; /**< Complete packed lifecycle state. */
	/** Individual internal lifecycle flags. */
	struct max30101_state_bits {
		unsigned int bInitialized : 1;	 /**< Probe and IRQ setup completed. */
		unsigned int bProbed : 1;		 /**< Device presence probe attempted. */
		unsigned int bDeviceFound : 1;	 /**< Device presence detected. */
		unsigned int bIrqConfigured : 1; /**< The INT line is claimed and its callback armed. */
		unsigned int bConfigured : 1;	 /**< Register configuration applied. */
		unsigned int bLedsPowered : 1;	 /**< LED supply is enabled. */
		unsigned int bDetectingProximity : 1; /**< Proximity detection is active. */
		unsigned int bSampling : 1;			  /**< Acquisition is running. */
	} bits;
};

/**
 * @brief Internal singleton driver context.
 * @details Stores all private runtime data: the shared-bus binding, interrupt
 *          wiring, lifecycle flags, and the cached acquisition geometry. This
 *          context is private to this implementation unit.
 */
static struct max30101_t {
	/** Shared-I2C connection and ownership token derived from devicetree. */
	struct sys_i2c_dt_spec device;
	/** Optional regulator powering the sensor's supply rail, or NULL. */
	const struct device* regulator;
	/** Interrupt GPIO specification obtained from the daughter-board arbiter. */
	const struct gpio_dt_spec* irq_gpio;
	/** GPIO callback instance registered for the interrupt line. */
	struct gpio_callback irq_cb;
	/** Driver lifecycle state. */
	union max30101_state_t state;
	/** Most recently applied acquisition configuration. */
	struct max30101_config_t config;
	/** Number of LED channels active in the current mode. */
	int led_count;
	/** Effective per-channel sampling rate in hertz. */
	float sampling_rate;
	/** Running sum used while averaging the proximity LED reading. */
	uint32_t proximity_led_value_sum;
	/** Count of proximity readings accumulated so far. */
	int proximity_led_read_count;

	struct max30101_ppg_sample_t samples[MAX30101_FIFO_DEPTH];
	size_t sample_count;

	/** Timestamp of the last interrupt in microseconds since the Unix epoch. */
	time_t irq_timestamp;
} max30101 = {
	.device = SYS_I2C_DT_SPEC_GET(MAX30101_NODE),
	.regulator = DEVICE_DT_GET(DT_PHANDLE(MAX30101_NODE, vin_supply)),
	.irq_timestamp = -1, /**< No interrupt has been received yet. */
	/* All remaining members are zero-initialised by static storage duration. */
};

static const char* const max30101_event_names[max30101_event_Count] = {
	[max30101_Irq] = "Irq",
	[max30101_event_PowerReady] = "PowerReady",
	[max30101_event_Proximity] = "Proximity",
	[max30101_event_AmbientLightCancelOverflow] = "AmbientLightCancelOverflow",
	[max30101_event_DieTemperatureReady] = "DieTemperatureReady",
	[max30101_event_FifoDataReady] = "FifoDataReady",
};

const char* max30101_event_name(uint32_t event_id) {
	if (event_id >= (uint32_t) max30101_event_Count || max30101_event_names[event_id] == NULL) {
		return "Unknown";
	}

	return max30101_event_names[event_id];
}

/* ------------------------------------------------------------------------- */
/* Shared-bus ownership and register helpers                                 */
/* ------------------------------------------------------------------------- */

/** Acquire shared-I2C ownership for one high-level sensor operation. */
static inline bool max30101_bus_lock(void) {
	int ret = sys_i2c_lock(&max30101.device, K_MSEC(MAX30101_I2C_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("Failed to lock SYS_I2C (%d)", ret);
		return false;
	}

	return true;
}

/** Fully release nested bus ownership held for one high-level operation. */
static inline bool max30101_bus_unlock(void) {
	int ret = sys_i2c_release(&max30101.device);

	if (ret != 0) {
		LOG_ERR("Failed to release SYS_I2C ownership (%d)", ret);
		return false;
	}

	return true;
}

/** Write @p count bytes starting at @p reg. The caller must own the shared bus. */
static inline int max30101_i2c_write(enum max30101_register_type reg,
									 const uint8_t* value,
									 size_t count) {
	uint8_t tx[MAX30101_I2C_TX_MAX];

	if (count + 1 > sizeof(tx)) {
		return -EINVAL;
	}

	tx[0] = (uint8_t) reg;
	memcpy(&tx[1], value, count);

	return sys_i2c_write(&max30101.device, tx, count + 1);
}

/** Write a single register byte. The caller must own the shared bus. */
static inline int max30101_i2c_write_byte(enum max30101_register_type reg, uint8_t value) {
	return max30101_i2c_write(reg, &value, 1);
}

/** Read @p count bytes starting at @p reg. The caller must own the shared bus. */
static inline int max30101_i2c_read(enum max30101_register_type reg, uint8_t* value, size_t count) {
	uint8_t addr = (uint8_t) reg;

	return sys_i2c_write_read(&max30101.device, &addr, sizeof(addr), value, count);
}

/* ------------------------------------------------------------------------- */
/* Event publication                                                          */
/* ------------------------------------------------------------------------- */

static inline void max30101_post_event(enum max30101_event_type event,
									   uint32_t v_param,
									   uintptr_t p_param) {
	(void) device_driver_event_post(MAX30101_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									p_param,
									K_MSEC(MAX30101_I2C_TIMEOUT));
}

static inline void max30101_post_event_isr(enum max30101_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(MAX30101_DEVICE_DTS_ID,
										(uint32_t) event,
										v_param,
										(uintptr_t) NULL);
}

/* ------------------------------------------------------------------------- */
/* FIFO sample helpers                                                        */
/* ------------------------------------------------------------------------- */

static uint32_t max30101_unpack_sample(const uint8_t* sample) {
	return ((uint32_t) sample[0] << 16 | (uint32_t) sample[1] << 8 | (uint32_t) sample[2]) &
		   0x03FFFF;
}

/* ------------------------------------------------------------------------- */
/* Probe and interrupt wiring                                                 */
/* ------------------------------------------------------------------------- */

/** Probe for an I2C response by reading PART_ID. The caller must own the bus. */
static bool max30101_probe(void) {
	uint8_t part_id = 0u;
	int ret = max30101_i2c_read(max30101_register_PartID, &part_id, sizeof(part_id));

	max30101.state.bits.bProbed = 1;
	if (ret != 0) {
		LOG_WRN("MAX30101 not detected on I2C bus (%d)", ret);
		max30101.state.bits.bDeviceFound = 0;
		return false;
	}
	if (part_id == MAX30101_PART_ID) {
		LOG_INF("MAX30101 detected on I2C bus");
		max30101.state.bits.bDeviceFound = 1;
	} else {
		LOG_WRN("MAX30101 probe failed: unexpected PART_ID 0x%02x", part_id);
		max30101.state.bits.bDeviceFound = 0;
	}
	return max30101.state.bits.bDeviceFound != 0;
}

/**
 * @brief GPIO ISR that posts the raw interrupt notification event.
 * @details Captures the IRQ arrival time with rtc_get_timestamp_us() so FIFO
 *          sample timestamps can later be interpolated from this anchor.
 */
static void max30101_irq_callback(const struct device* dev,
								  struct gpio_callback* cb,
								  uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	max30101.irq_timestamp = rtc_get_timestamp_us();
	max30101_post_event_isr(max30101_Irq, pins);
}

/** Claim and configure the daughter-board interrupt line (active low). */
static int max30101_irq_init(void) {
	if (max30101.state.bits.bIrqConfigured != 0) {
		return 0;
	}

	const struct gpio_dt_spec* irq = daughter_if_gpio_claim(MAX30101_IRQ_LINE);
	if (irq == NULL) {
		LOG_ERR("MAX30101 could not claim daughter-board interrupt line");
		return -ENODEV;
	}
	max30101.irq_gpio = irq;

	int ret = gpio_pin_configure_dt(irq, GPIO_INPUT);
	if (ret) {
		LOG_ERR("MAX30101 interrupt pin config failed (%d)", ret);
		daughter_if_gpio_release(MAX30101_IRQ_LINE);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(irq, GPIO_INT_EDGE_FALLING);
	if (ret) {
		LOG_ERR("MAX30101 interrupt config failed (%d)", ret);
		daughter_if_gpio_release(MAX30101_IRQ_LINE);
		return ret;
	}

	gpio_init_callback(&max30101.irq_cb, max30101_irq_callback, BIT(irq->pin));
	ret = gpio_add_callback(irq->port, &max30101.irq_cb);
	if (ret) {
		LOG_ERR("MAX30101 interrupt callback add failed (%d)", ret);
		daughter_if_gpio_release(MAX30101_IRQ_LINE);
		return ret;
	}

	max30101.state.bits.bIrqConfigured = 1;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Public lifecycle                                                           */
/* ------------------------------------------------------------------------- */

bool max30101_is_ready(void) {
	if (max30101.state.bits.bProbed == 0) {
		if (!max30101_bus_lock()) {
			return false;
		}
		max30101_probe();
		max30101_bus_unlock();
	}
	return max30101.state.bits.bProbed != 0 && max30101.state.bits.bDeviceFound != 0;
}

/**
 * @brief Power the sensor's supply rail through @p regulatorDev.
 * @details Sets the rail to @p voltage_uv and enables it, then waits for the rail
 *          and the MAX30101 power-on reset to settle. A NULL @p regulatorDev is a
 *          no-op success: the caller is responsible for powering the sensor.
 */
static bool max30101_led_power_on(int32_t voltage_uv) {
	if (max30101.state.bits.bLedsPowered != 0) {
		__ASSERT(regulator_dev != max30101.regulator,
				 "MAX30101 regulator device is different than the one currently in use, "
				 "but the LED supply is already flagged enabled");
		__ASSERT(regulator_is_enabled(regulator_dev) == false,
				 "MAX30101 LED supply is already enabled, but the regulator is disabled");
		if (voltage_uv != max30101.config.ppg_voltage_uv) {
			LOG_WRN("MAX30101 LED supply voltage already set to %d uV, requested %d uV",
					max30101.config.ppg_voltage_uv,
					voltage_uv);
			regulator_set_voltage(max30101.regulator, voltage_uv, voltage_uv);
			max30101.config.ppg_voltage_uv = voltage_uv;
		}
		return true;
	}
	if (max30101.regulator == NULL) {
		return true;
	}

	if (!device_is_ready(max30101.regulator)) {
		LOG_ERR("MAX30101 regulator %s not ready", max30101.regulator->name);
		return false;
	}
	// check whether the regulator is already active, if it is, we must make sure
	// its output voltage the requested voltage.
	// -- OTHERWISE-- we risk damaging either MAX30101 or the other device
	// powered through the regulator.
	if (regulator_is_enabled(max30101.regulator)) {
		int32_t current_voltage;
		int ret = regulator_get_voltage(max30101.regulator, &current_voltage);
		if (ret < 0) {
			LOG_ERR("Failed to get MAX30101 regulator %s voltage (%d)",
					max30101.regulator->name,
					ret);
			return false;
		}
		// in this case we must stop the application here!
		// This ia a configuration error, the application must be fixed to avoid
		// damaging the MAX30101 or the other device powered through the regulator.
		__ASSERT(current_voltage == voltage_uv,
				 "MAX30101 regulator %s already enabled at %d uV, requested %d uV",
				 max30101.regulator->name,
				 current_voltage,
				 voltage_uv);
	}
	int ret = regulator_set_voltage(max30101.regulator, voltage_uv, voltage_uv);
	if (ret < 0) {
		LOG_ERR("Failed to set MAX30101 rail to %d uV (%d)", voltage_uv, ret);
		return false;
	}

	ret = regulator_enable(max30101.regulator);
	if (ret < 0) {
		LOG_ERR("Failed to enable MAX30101 rail (%d)", ret);
		return false;
	}

	k_msleep(MAX30101_PPG_RAMP_DELAY_MS);
	// Update the cached configuration to reflect the new voltage.
	max30101.config.ppg_voltage_uv = voltage_uv;
	max30101.state.bits.bLedsPowered = 1;
	return true;
}

static void max30101_led_power_off(void) {
	__ASSERT(max30101.regulator != NULL, "MAX30101 regulator is NULL.");
	if (max30101.state.bits.bLedsPowered == 0) {
		return;
	}
	if (max30101.regulator != NULL) {
		regulator_disable(max30101.regulator);
	} else {
		LOG_WRN("MAX30101 regulator is NULL, but the LED supply is flagged enabled");
	}
	max30101.state.bits.bLedsPowered = 0;
}

static bool max30101_validate_led_voltage(int32_t voltage_uv) {
	uint32_t min_uv = MAX30101_PPG_VOLTAGE_MIN_UV;
	uint32_t max_uv = MAX30101_PPG_VOLTAGE_MAX_UV;
	if (voltage_uv < min_uv || voltage_uv > max_uv) {
		LOG_ERR("MAX30101 PPG supply voltage %d uV is out of range [%d, %d]",
				voltage_uv,
				MAX30101_PPG_VOLTAGE_MIN_UV,
				MAX30101_PPG_VOLTAGE_MAX_UV);
		return false;
	}
	// now check LED configuration vs voltage
	if (max30101.config.ir_led_pulse_amplitude_config > 0) {
		if (min_uv > MAX30101_LED_VOLTAGE_IR_MIN_UV) {
			min_uv = MAX30101_LED_VOLTAGE_IR_MIN_UV;
		}
		if (max_uv > MAX30101_LED_VOLTAGE_IR_MAX_UV) {
			max_uv = MAX30101_LED_VOLTAGE_IR_MAX_UV;
		}
	}
	if (max30101.config.red_led_pulse_amplitude_config > 0) {
		if (min_uv > MAX30101_LED_VOLTAGE_RED_MIN_UV) {
			min_uv = MAX30101_LED_VOLTAGE_RED_MIN_UV;
		}
		if (max_uv > MAX30101_LED_VOLTAGE_RED_MAX_UV) {
			max_uv = MAX30101_LED_VOLTAGE_RED_MAX_UV;
		}
	}
	if (max30101.config.green_led_pulse_amplitude_config > 0) {
		if (min_uv > MAX30101_LED_VOLTAGE_GREEN_MIN_UV) {
			min_uv = MAX30101_LED_VOLTAGE_GREEN_MIN_UV;
		}
		if (max_uv > MAX30101_LED_VOLTAGE_GREEN_MAX_UV) {
			max_uv = MAX30101_LED_VOLTAGE_GREEN_MAX_UV;
		}
	}
	if (voltage_uv < min_uv || voltage_uv > max_uv) {
		LOG_ERR("MAX30101 PPG supply voltage %d uV is out of range [%d, %d] for the current LED "
				"configuration",
				voltage_uv,
				min_uv,
				max_uv);
		return false;
	}
	return true;
}

int max30101_init(void) {
	__ASSERT(max30101.regulator != NULL,
			 "Regulator must be initialized by default, but it is NULL!");
	if (max30101.state.bits.bInitialized != 0) {
		LOG_WRN("MAX30101 already initialized!");
		return 0;
	}

	if (!sys_i2c_is_ready(&max30101.device)) {
		LOG_ERR("SYS_I2C bus not ready");
		return false;
	}

	max30101_get_default_config(&max30101.config);

	// at init, we must have LED power off, so we can safely set the voltage to the default value.
	if (regulator_is_enabled(max30101.regulator)) {
		uint32_t currentVoltage;
		int ret = regulator_get_voltage(max30101.regulator, &currentVoltage);
		if (ret < 0) {
			LOG_ERR("Failed to get MAX30101 regulator %s voltage (%d)",
					max30101.regulator->name,
					ret);
			return -EIO;
		}
		// validate that the current voltage is within the the allowed range.
		if (currentVoltage < MAX30101_PPG_VOLTAGE_MIN_UV ||
			currentVoltage > MAX30101_PPG_VOLTAGE_MAX_UV) {
			LOG_ERR("MAX30101 regulator %s voltage %d uV is out of range [%d, %d]",
					max30101.regulator->name,
					currentVoltage,
					MAX30101_PPG_VOLTAGE_MIN_UV,
					MAX30101_PPG_VOLTAGE_MAX_UV);
			return -EIO;
		}
	}
	// here, either voltage is within the allowed range, or the regulator is not
	// enabled, so we can safely set the voltage to the default value.
	//--------------------------------------------------------------------------

	// we can now probe the device
	if (!max30101_bus_lock()) {
		return -EIO;
	}
	max30101_probe();
	max30101_bus_unlock();
	if (max30101.state.bits.bDeviceFound == 0) {
		LOG_ERR("MAX30101 device not found during initialization");
		return -EIO;
	}
	// -------------------------------------------------------------------------
	// now, we can shutdown the device.
	max30101_shutdown();
	// -------------------------------------------------------------------------
	int ret = max30101_irq_init();
	if (ret != 0) {
		LOG_ERR("MAX30101 interrupt initialization failed (%d)", ret);
		return -EIO;
	}

	max30101.state.bits.bInitialized = 1;
	return 0;
}

void max30101_get_default_config(struct max30101_config_t* config) {
	memset(config, 0, sizeof(*config));

	/* Multi-LED acquisition with three channels in FIFO order: IR, Red, Green. */
	config->mode_config.bits.mode = max30101_mode_MultiLed;

	config->fifo_config.bits.fifo_a_full =
		MAX30101_FIFO_DEPTH - MAX30101_FIFO_ALMOST_FULL_THRESHOLD;
	config->fifo_config.bits.fifo_roll_over_en = MAX30101_FIFO_ROLLOVER;
	config->fifo_config.bits.sample_average = MAX30101_AVERAGED_SAMPLES;

	config->spo2_config.bits.led_pw = MAX30101_LED_PULSEWIDTH;
	config->spo2_config.bits.spo2_adc_range = MAX30101_ADC_RANGE;
	config->spo2_config.bits.spo2_sr = MAX30101_SAMPLE_RATE;

	config->multi_led_config.bits.slot1 = (int) max30101_led_IR;
	config->multi_led_config.bits.slot2 = (int) max30101_led_Red;
	config->multi_led_config.bits.slot3 = (int) max30101_led_Green;
	config->multi_led_config.bits.slot4 = 0;

	config->ir_led_pulse_amplitude_config = MAX30101_IR_LED_PULSE_AMPLITUDE;
	config->red_led_pulse_amplitude_config = MAX30101_RED_LED_PULSE_AMPLITUDE;
	config->green_led_pulse_amplitude_config = MAX30101_GREEN_LED_PULSE_AMPLITUDE;
	config->proximity_led_pulse_amplitude_config = MAX30101_PROXIMITY_MODE_LED_PULSE_AMPLITUDE;
	config->proximity_int_threshold = MAX30101_PROXIMITY_THRESHOLD;

	/* Default interrupt enable: FIFO almost full drives streaming. */
	config->interrupts.value = 0;

	/* Supply rail applied when max30101_init() is given a regulator. */
	config->ppg_voltage_uv = MAX30101_PPG_VOLTAGE_UV;
}

/** Compute the effective per-channel sampling rate from a configuration. */
static float max30101_compute_sampling_rate(const struct max30101_config_t* config) {
	/* SPO2_SR is a non-linear enum index, not a value in hertz: the steps are
	 * 50, 100, 200, 400, 800, 1000, 1600, 3200 Hz. */
	static const float sample_rate_hz[] = {
		[max30101_sample_rate_50Hz] = 50.0f,
		[max30101_sample_rate_100Hz] = 100.0f,
		[max30101_sample_rate_200Hz] = 200.0f,
		[max30101_sample_rate_400Hz] = 400.0f,
		[max30101_sample_rate_800Hz] = 800.0f,
		[max30101_sample_rate_1000Hz] = 1000.0f,
		[max30101_sample_rate_1600Hz] = 1600.0f,
		[max30101_sample_rate_3200Hz] = 3200.0f,
	};
	float rate = sample_rate_hz[config->spo2_config.bits.spo2_sr];

	return rate / (float) (1 << config->fifo_config.bits.sample_average);
}

/** Count the active LED channels implied by a configuration. */
static int max30101_count_leds(const struct max30101_config_t* config) {
	switch (config->mode_config.bits.mode) {
	case max30101_mode_HeartRate:
		return 1;
	case max30101_mode_SpO2:
		return 2;
	case max30101_mode_MultiLed:
	default: {
		int count = 0;
		count += config->multi_led_config.bits.slot1 != 0 ? 1 : 0;
		count += config->multi_led_config.bits.slot2 != 0 ? 1 : 0;
		count += config->multi_led_config.bits.slot3 != 0 ? 1 : 0;
		count += config->multi_led_config.bits.slot4 != 0 ? 1 : 0;
		return count;
	}
	}
}

/** Issue a software reset and wait for it to clear. The caller must own the bus. */
static bool max30101_reset_locked(void) {
	union max30101_mode_configuration_t mode = {.value = 0};
	mode.bits.reset = 1;
	mode.bits.mode = max30101_mode_MultiLed;

	if (max30101_i2c_write_byte(max30101_register_ModeConfiguration, (uint8_t) mode.value) != 0) {
		return false;
	}

	/* Poll until the device clears the reset bit. */
	for (int attempts = 0; attempts < 16; ++attempts) {
		uint8_t value = 0;
		if (max30101_i2c_read(max30101_register_ModeConfiguration, &value, 1) != 0) {
			return false;
		}
		mode.value = value;
		if (mode.bits.reset == 0) {
			return true;
		}
	}
	LOG_ERR("MAX30101 reset did not clear");
	return false;
}

int max30101_config(struct max30101_config_t* config) {
	if (max30101.state.bits.bInitialized == 0) {
		LOG_ERR("MAX30101 not initialized");
		return -EINVAL;
	}
	if (max30101.state.bits.bDeviceFound == 0) {
		LOG_ERR("MAX30101 not found");
		return -ENODEV;
	}

	if (max30101.state.bits.bSampling != 0) {
		LOG_ERR("MAX30101 is sampling, stop sampling before reconfiguring");
		return -EBUSY;
	}
	// power off the LED supply before reconfiguring, to avoid damaging the device.
	max30101_led_power_off();

	if (config == NULL) {
		max30101_get_default_config(&max30101.config);
	} else {
		memcpy(&max30101.config, config, sizeof(max30101.config));
	}

	// check the regulator voltage configuration against the LED configuration.
	if (!max30101_validate_led_voltage(max30101.config.ppg_voltage_uv)) {
		return -EINVAL;
	}

	const struct max30101_config_t* cfg = &max30101.config;

	uint8_t led_amplitudes[3] = {
		cfg->ir_led_pulse_amplitude_config,
		cfg->red_led_pulse_amplitude_config,
		cfg->green_led_pulse_amplitude_config,
	};
	uint8_t fifo_clear[3] = {0, 0, 0};
	uint8_t multi_led[2] = {
		(uint8_t) (cfg->multi_led_config.value & 0xFF),
		(uint8_t) ((cfg->multi_led_config.value >> 8) & 0xFF),
	};
	uint8_t int_enable[2] = {
		(uint8_t) (cfg->interrupts.value & 0xFF),
		(uint8_t) ((cfg->interrupts.value >> 8) & 0xFF),
	};
	union max30101_mode_configuration_t mode = {.value = 0};
	mode.bits.mode = cfg->mode_config.bits.mode;

	if (!max30101_bus_lock()) {
		return -EIO;
	}

	bool ret =
		max30101_reset_locked() &&
		(max30101_i2c_write_byte(max30101_register_FIFO_Configuration,
								 (uint8_t) cfg->fifo_config.value) == 0) &&
		(max30101_i2c_write_byte(max30101_register_SpO2Configuration,
								 (uint8_t) cfg->spo2_config.value) == 0) &&
		(max30101_i2c_write(max30101_register_LED1_PA, led_amplitudes, sizeof(led_amplitudes)) ==
		 0) &&
		(max30101_i2c_write_byte(max30101_register_ProxModeLED_PA,
								 cfg->proximity_led_pulse_amplitude_config) == 0) &&
		(max30101_i2c_write_byte(max30101_register_ProxIntThreshold,
								 cfg->proximity_int_threshold) == 0) &&
		(max30101_i2c_write(max30101_register_ModeControlReg1, multi_led, sizeof(multi_led)) ==
		 0) &&
		(max30101_i2c_write(max30101_register_FIFO_WritePointer, fifo_clear, sizeof(fifo_clear)) ==
		 0) &&
		(max30101_i2c_write(max30101_register_InterruptEnable1, int_enable, sizeof(int_enable)) ==
		 0) &&
		(max30101_i2c_write_byte(max30101_register_ModeConfiguration, (uint8_t) mode.value) == 0);

	ret &= max30101_bus_unlock();
	if (!ret) {
		return -EIO;
	}

	max30101.led_count = max30101_count_leds(cfg);
	max30101.sampling_rate = max30101_compute_sampling_rate(cfg);
	max30101.state.bits.bDetectingProximity = 0;
	max30101.state.bits.bSampling = 0;
	max30101.state.bits.bConfigured = 1;
	return 0;
}

int max30101_get_led_count(void) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -1;
	}
	return max30101.led_count;
}

float max30101_get_sampling_rate(void) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -1.0f;
	}
	return max30101.sampling_rate;
}

int max30101_read_fifo(void* buffer, size_t buffer_size) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -EAGAIN;
	}
	if (buffer == NULL) {
		LOG_ERR("Buffer is NULL");
		return -EINVAL;
	}
	if (buffer_size < (size_t) (MAX30101_FIFO_DEPTH / 8) * MAX30101_BYTES_PER_CHANNEL) {
		LOG_ERR("Buffer size is too small");
		return -EINVAL;
	}

	uint8_t pointers[3] = {0};
	bool ret = max30101_i2c_read(max30101_register_FIFO_WritePointer, pointers, sizeof(pointers)) ==
			   0;
	if (!ret) {
		return -EIO;
	}

	uint8_t write_ptr = pointers[0] & 0x1F;
	uint8_t overflow_counter = pointers[1] & 0x1F;
	uint8_t read_ptr = pointers[2] & 0x1F;
	uint8_t available_samples = (write_ptr - read_ptr) & 0x1F;

	if (overflow_counter > 0) {
		LOG_WRN("FIFO overflow detected: overflow=%d, readPtr=%d, writePtr=%d",
				overflow_counter,
				read_ptr,
				write_ptr);
		available_samples = MAX30101_FIFO_DEPTH;
	}

	if (available_samples == 0) {
		return 0;
	}

	size_t byte_count =
		(size_t) available_samples * MAX30101_BYTES_PER_CHANNEL * max30101.led_count;
	size_t sample_size = (size_t) MAX30101_BYTES_PER_CHANNEL * max30101.led_count;

	/* Read only as many whole samples as the destination buffer can hold. */
	while (byte_count > buffer_size && byte_count >= sample_size) {
		byte_count -= sample_size;
	}

	ret = max30101_i2c_read(max30101_register_FIFO_DataRegister, buffer, byte_count) == 0;
	if (!ret) {
		return -EIO;
	}

	if (overflow_counter > 0) {
		/* Discard possibly-corrupted data after an overflow. */
		return 0;
	}
	return byte_count;
}

/* ------------------------------------------------------------------------- */
/* Internal decoded-sample stream                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief Drain the FIFO, decode each record, and append it to the internal stream.
 * @details Each decoded sample is stamped relative to the most recent IRQ time
 *          and the configured sampling rate, then stored in the internal buffer
 *          that is republished with ::max30101_event_FifoDataReady.
 */
static int max30101_drain_fifo(void) {
	uint8_t buffer[MAX30101_FIFO_READ_SAMPLES * 3 * MAX30101_BYTES_PER_CHANNEL] = {0};

	int actual = max30101_read_fifo(buffer, sizeof(buffer));
	if (actual <= 0) {
		return -EIO;
	}

	size_t sample_size = (size_t) max30101.led_count * MAX30101_BYTES_PER_CHANNEL;
	size_t offset = 0;
	size_t samples = 0;

	while ((sample_size > 0) && (offset + sample_size <= actual)) {
		const uint8_t* record = &buffer[offset];
		struct max30101_ppg_sample_t sample = {.timestamp = max30101.irq_timestamp};

		if (max30101.led_count >= 1) {
			sample.ir = max30101_unpack_sample(record);
			record += MAX30101_BYTES_PER_CHANNEL;
		}
		if (max30101.led_count >= 2) {
			sample.red = max30101_unpack_sample(record);
			record += MAX30101_BYTES_PER_CHANNEL;
		}
		if (max30101.led_count >= 3) {
			sample.green = max30101_unpack_sample(record);
		}

		if (max30101.sample_count < MAX30101_FIFO_DEPTH) {
			max30101.samples[max30101.sample_count++] = sample;
		} else {
			LOG_WRN("MAX30101 internal sample buffer overflow, dropping sample");
		}
		offset += sample_size;
		samples++;
	}

	// lets update the sample time stamps in microseconds, based on the sampling rate and the number
	// of samples read.
	time_t delta = (time_t) ((float) 1.0e6f / max30101.sampling_rate);
	time_t t0 = (max30101.irq_timestamp < 0) ? rtc_get_timestamp_us() : max30101.irq_timestamp;
	time_t t = t0 - samples * delta;
	for (size_t i = 0; i < samples; i++) {
		max30101.samples[i].timestamp = t;
		t += delta;
	}

	return samples;
}

static int max30101_read_die_temperature(float* temperature_c) {
	if (temperature_c == NULL) {
		return -EINVAL;
	}

	uint8_t temp_data[2] = {0};
	if (!max30101_bus_lock()) {
		return -EIO;
	}
	bool ret = max30101_i2c_read(max30101_register_DieTempInt, temp_data, sizeof(temp_data)) == 0;
	if (!ret) {
		return -EIO;
	}

	int8_t temp_int = (int8_t) temp_data[0];
	uint8_t temp_frac = temp_data[1] & 0x0F;						   // 4 bits of fractional part
	*temperature_c = (float) temp_int + ((float) temp_frac * 0.0625f); // Each LSB is 0.0625°C
	return 0;
}

time_t max30101_last_irq_timestamp(void) {
	__ASSERT(max30101.state.bits.bInitialized != 0, "MAX30101 not initialized");
	__ASSERT(max30101.state.bits.bDeviceFound != 0, "MAX30101 not found");
	__ASSERT(max30101.state.bits.bConfigured != 0, "MAX30101 not configured");
	return max30101.irq_timestamp;
}

int max30101_irq_handler(void) {
	if (max30101.state.bits.bConfigured == 0) {
		return -EAGAIN;
	}

	uint8_t status[2] = {0};
	if (!max30101_bus_lock()) {
		return -EIO;
	}
	bool ret = max30101_i2c_read(max30101_register_InterruptStatus1, status, sizeof(status)) == 0;
	if (!ret) {
		max30101_bus_unlock();
		return -EIO;
	}

	union max30101_interrupt_status_t int_status = {
		.value = (int) ((uint32_t) status[0] | ((uint32_t) status[1] << 8))};

	/* Power-on / brown-out: the device returns to its reset defaults and must be
	 * reconfigured before it will sample again. */
	if (int_status.bits.pwr_rdy) {
		max30101.state.bits.bConfigured = 0;
		max30101_post_event(max30101_event_PowerReady, (uint32_t) int_status.value, 0);
	}
	if (int_status.bits.prox_int) {
		max30101_post_event(max30101_event_Proximity, (uint32_t) int_status.value, 0);
	}
	if (int_status.bits.alc_ovf) {
		LOG_WRN("MAX30101 ambient-light-cancellation overflow");
		max30101_post_event(max30101_event_AmbientLightCancelOverflow,
							(uint32_t) int_status.value,
							0);
	}
	if (int_status.bits.die_tamp_ready) {
		// get the die temperature reading
		float die_temp_c = 0.0f;
		if (max30101_read_die_temperature(&die_temp_c) == 0) {
			LOG_INF("MAX30101 die temperature: %.2f °C", (double) die_temp_c);
			max30101_post_event(max30101_event_DieTemperatureReady, (uint32_t) die_temp_c, 0);
		} else {
			LOG_ERR("Failed to read MAX30101 die temperature");
		}
	}

	/* New-data and almost-full both mean there are records to read out. Drain
	 * the FIFO into the internal stream and report how many samples arrived. */
	if (int_status.bits.ppg_rdy || int_status.bits.a_full) {
		max30101.sample_count = 0; // Reset sample count before draining
		int ret = max30101_drain_fifo();
		if (ret > 0) {
			max30101_post_event(max30101_event_FifoDataReady,
								max30101.sample_count,
								(uintptr_t) max30101.samples);
		}
	}

	max30101_bus_unlock();
	return 0;
}

int max30101_enable_wrist_hr_sampling(bool per_sample_irq) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -EAGAIN;
	}
	if (max30101.state.bits.bSampling != 0) {
		return -EBUSY;
	}

	struct max30101_config_t cfg = max30101.config;
	cfg.mode_config.bits.mode = max30101_mode_MultiLed;
	cfg.multi_led_config.value = 0;
	cfg.multi_led_config.bits.slot1 = (int) max30101_led_IR;
	cfg.multi_led_config.bits.slot2 = (int) max30101_led_Red;
	cfg.multi_led_config.bits.slot3 = (int) max30101_led_Green;

	/* per_sample_irq selects the FIFO notification cadence: PPG_RDY fires one
	 * interrupt per new sample, A_FULL fires once per FIFO almost-full batch. */
	cfg.interrupts.value = 0;
	cfg.interrupts.bits.alc_ovf = 1;
	cfg.interrupts.bits.ppg_rdy = per_sample_irq ? 1 : 0;
	cfg.interrupts.bits.a_full = per_sample_irq ? 0 : 1;

	int ret = max30101_config(&cfg);
	if (ret != 0) {
		return ret;
	}

	// now, we can enable the LED regulator
	if (!max30101_led_power_on(max30101.config.ppg_voltage_uv)) {
		LOG_ERR("Failed to power on MAX30101 LED supply");
		return -EINVAL;
	}
	// indicate that we are now sampling
	max30101.state.bits.bSampling = 1;
	return 0;
}

int max30101_enable_sampling(enum max30101_operation_mode_type mode, bool per_sample_irq) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -EAGAIN;
	}

	if (max30101.state.bits.bDetectingProximity != 0 || max30101.state.bits.bSampling != 0) {
		return -EBUSY;
	}

	struct max30101_config_t cfg = max30101.config;
	cfg.multi_led_config.value = 0;

	switch (mode) {
	case max30101_mode_HeartRate:
		cfg.mode_config.bits.mode = max30101_mode_HeartRate;
		cfg.multi_led_config.bits.slot1 = (int) max30101_led_Red;
		break;
	case max30101_mode_SpO2:
		cfg.mode_config.bits.mode = max30101_mode_SpO2;
		cfg.multi_led_config.bits.slot1 = (int) max30101_led_Red;
		cfg.multi_led_config.bits.slot2 = (int) max30101_led_IR;
		break;
	case max30101_mode_MultiLed:
		cfg.mode_config.bits.mode = max30101_mode_MultiLed;
		cfg.multi_led_config.bits.slot1 = (int) max30101_led_IR;
		cfg.multi_led_config.bits.slot2 = (int) max30101_led_Red;
		cfg.multi_led_config.bits.slot3 = (int) max30101_led_Green;
		break;
	default:
		LOG_ERR("Invalid mode %d", mode);
		return -EINVAL;
	}

	/* per_sample_irq selects the FIFO notification cadence: PPG_RDY fires one
	 * interrupt per new sample, A_FULL fires once per FIFO almost-full batch. */
	cfg.interrupts.value = 0;
	cfg.interrupts.bits.ppg_rdy = per_sample_irq ? 1 : 0;
	cfg.interrupts.bits.a_full = per_sample_irq ? 0 : 1;

	int ret = max30101_config(&cfg);
	if (ret != 0) {
		return ret;
	}

	// now, we can enable the LED regulator
	if (!max30101_led_power_on(max30101.config.ppg_voltage_uv)) {
		LOG_ERR("Failed to power on MAX30101 LED supply");
		return -EINVAL;
	}
	// indicate that we are now sampling
	max30101.state.bits.bSampling = 1;
	return 0;
}

int max30101_disable_sampling(void) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -EAGAIN;
	}

	if (max30101.state.bits.bDetectingProximity != 0 && max30101.state.bits.bSampling != 0) {
		LOG_ERR("MAX30101 is both sampling and detecting proximity, cannot disable sampling");
		return -EBUSY;
	}

	if (max30101.state.bits.bSampling == 0) {
		LOG_WRN("MAX30101 is not sampling, nothing to disable");
		return 0;
	}
	// indicate that we are no longer sampling
	max30101.state.bits.bSampling = 0;
	if (max30101.state.bits.bDetectingProximity != 0) {
		struct max30101_config_t cfg = max30101.config;
		cfg.mode_config.bits.mode = max30101_mode_MultiLed;
		cfg.multi_led_config.value = 0;
		cfg.multi_led_config.bits.slot1 = (int) max30101_led_IR;
		cfg.interrupts.value = 0;
		cfg.interrupts.bits.prox_int = 1;
		cfg.interrupts.bits.ppg_rdy = 1;

		if (!max30101_config(&cfg)) {
			return -EIO;
		}
	} else {
		max30101_shutdown();
	}
	return 0;
}

int max30101_enable_proximity(void) {
	if (max30101.state.bits.bConfigured == 0) {
		LOG_ERR("MAX30101 not configured");
		return -EAGAIN;
	}

	if (max30101.state.bits.bDetectingProximity != 0) {
		LOG_WRN("MAX30101 already detecting proximity");
		return -EBUSY;
	}

	if (max30101.state.bits.bSampling != 0) {
		LOG_ERR("MAX30101 is sampling, cannot enable proximity detection");
		return -EBUSY;
	}

	struct max30101_config_t cfg = max30101.config;
	cfg.mode_config.bits.mode = max30101_mode_MultiLed;
	cfg.multi_led_config.value = 0;
	cfg.multi_led_config.bits.slot1 = (int) max30101_led_IR;
	cfg.interrupts.value = 0;
	cfg.interrupts.bits.prox_int = 1;
	cfg.interrupts.bits.ppg_rdy = 1;

	if (!max30101_config(&cfg)) {
		return -EIO;
	}

	max30101.proximity_led_value_sum = 0;
	max30101.proximity_led_read_count = 0;
	max30101.state.bits.bDetectingProximity = 1;
	max30101.state.bits.bSampling = 0;

	// now, we can enable the LED regulator
	if (!max30101_led_power_on(max30101.config.ppg_voltage_uv)) {
		LOG_ERR("Failed to power on MAX30101 LED supply");
		return -EIO;
	}
	return 0;
}

void max30101_shutdown(void) {

	union max30101_mode_configuration_t mode = {.value = 0};
	mode.bits.mode = max30101_mode_MultiLed;
	mode.bits.shdn = 1;

	if (!max30101_bus_lock()) {
		return;
	}
	(void) max30101_i2c_write_byte(max30101_register_ModeConfiguration, (uint8_t) mode.value);
	max30101_bus_unlock();

	max30101.state.bits.bDetectingProximity = 0;
	max30101.state.bits.bSampling = 0;
	// now we can shutdown the LED supply, if it is enabled.
	max30101_led_power_off();
}
