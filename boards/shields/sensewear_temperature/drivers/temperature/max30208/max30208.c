/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30208.c
 * @brief SenseWear MAX30208 temperature sensor driver implementation.
 * @details The driver is a daughter-board singleton. Its connection is derived
 *          from `DT_ALIAS(sensewear_temperature)`, while all transfers are routed
 *          through @ref sensewear_sys_i2c. Public hardware operations own the
 *          shared bus for their complete register sequence and finish with
 *          sys_i2c_release().
 *
 * Register helpers in this file intentionally do not lock. They may only be
 * called from a high-level operation that already owns the shared bus.
 *
 * The temperature daughter board exposes neither a regulator nor a
 * daughter-connector interrupt line to this driver. A periodic software timer
 * substitutes for the missing INT pin: it posts ::max30208_TimerIrq each
 * sampling period, and the consumer responds by calling max30208_get_samples(),
 * which performs the conversion, fills the internal sample buffer, and publishes
 * ::max30208_event_SampleReady.
 *
 * Each drained temperature sample is stamped with the conversion time captured
 * from rtc_get_timestamp_us(); all samples returned from one drain share that
 * same timestamp.
 */

#include "max30208.h"
#include "max30208_config.h"
#include "max30208_registers.h"
#include "rtc.h"
#include "sys_i2c.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(max30208, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Devicetree node identifier for the shield's MAX30208 instance.
 * @details Maps the `sensewear_temperature` alias to the node used for
 *          constructing the shared-I2C specification.
 */
#define MAX30208_NODE DT_ALIAS(sensewear_temperature)

BUILD_ASSERT(DT_NODE_HAS_STATUS(MAX30208_NODE, okay),
			 "Temperature firmware requires the sensewear_temperature shield");

/** Largest single register burst this driver writes (address + payload). */
#define MAX30208_I2C_TX_MAX (3)

/** Depth of the driver's internal decoded-sample buffer. */
#define MAX30208_SAMPLE_BUFFER_DEPTH MAX30208_FIFO_DEPTH

/** Internal driver lifecycle flags (private to the implementation). */
union max30208_state_t {
	unsigned int value; /**< Complete packed lifecycle state. */
	/** Individual internal lifecycle flags. */
	struct max30208_state_bits {
		unsigned int bInitialized : 1; /**< Probe and FIFO setup completed. */
		unsigned int bProbed : 1;	   /**< Device presence probe attempted. */
		unsigned int bDeviceFound : 1; /**< Device presence detected. */
		unsigned int bConfigured : 1;  /**< Register configuration applied. */
		unsigned int bSampling : 1;	   /**< Sampling timer is running. */
	} bits;
};

/**
 * @brief Internal singleton driver context.
 * @details Stores all private runtime data: the shared-bus binding, sampling
 *          timer, lifecycle flags, and the decoded-sample buffer republished
 *          with each ::max30208_event_SampleReady. This context is private to
 *          this implementation unit.
 */
static struct max30208_t {
	/** Shared-I2C connection and ownership token derived from devicetree. */
	struct sys_i2c_dt_spec device;
	/** Driver lifecycle state. */
	union max30208_state_t state;
	/** Periodic timer pacing conversions; substitutes for the missing INT line. */
	struct k_timer sample_timer;
	/** Per-sensor sampling rate in hertz. */
	uint16_t sampling_rate_hz;
	/** Most recently applied acquisition configuration. */
	struct max30208_config_t config;
	/** Internal decoded-sample buffer, republished as event payload. */
	struct temperature_sample_t samples[MAX30208_SAMPLE_BUFFER_DEPTH];
	/** Number of valid entries in @ref samples. */
	size_t sample_count;
} max30208 = {
	.device = SYS_I2C_DT_SPEC_GET(MAX30208_NODE),
	.sampling_rate_hz = MAX30208_DEFAULT_SAMPLING_RATE_HZ,
	/* All remaining members are zero-initialised by static storage duration. */
};

static const char* const max30208_event_names[max30208_event_Count] = {
	[max30208_TimerIrq] = "TimerIrq",
	[max30208_event_SampleReady] = "SampleReady",
	[max30208_event_SamplingStarted] = "SamplingStarted",
	[max30208_event_SamplingStopped] = "SamplingStopped",
};

const char* max30208_event_name(uint32_t event_id) {
	if (event_id >= (uint32_t) max30208_event_Count || max30208_event_names[event_id] == NULL) {
		return "Unknown";
	}

	return max30208_event_names[event_id];
}

/* ------------------------------------------------------------------------- */
/* Event publication                                                          */
/* ------------------------------------------------------------------------- */

static inline void max30208_post_event(enum max30208_event_type event,
									   uint32_t v_param,
									   uintptr_t p_param) {
	(void) device_driver_event_post(MAX30208_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									p_param,
									K_MSEC(MAX30208_I2C_TIMEOUT));
}

static inline void max30208_post_event_isr(enum max30208_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(MAX30208_DEVICE_DTS_ID,
										(uint32_t) event,
										v_param,
										(uintptr_t) NULL);
}

/* ------------------------------------------------------------------------- */
/* Shared-bus ownership and register helpers                                 */
/* ------------------------------------------------------------------------- */

/** Acquire shared-I2C ownership for one high-level sensor operation. */
static inline bool max30208_bus_lock(void) {
	int ret = sys_i2c_lock(&max30208.device, K_MSEC(MAX30208_I2C_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("Failed to lock SYS_I2C (%d)", ret);
		return false;
	}

	return true;
}

/** Fully release nested bus ownership held for one high-level operation. */
static inline bool max30208_bus_unlock(void) {
	int ret = sys_i2c_release(&max30208.device);

	if (ret != 0) {
		LOG_ERR("Failed to release SYS_I2C ownership (%d)", ret);
		return false;
	}

	return true;
}

/** Write @p count bytes starting at @p reg. The caller must own the shared bus. */
static inline int max30208_i2c_write(enum max30208_register_type reg,
									 const uint8_t* value,
									 size_t count) {
	uint8_t tx[MAX30208_I2C_TX_MAX];

	if (count + 1 > sizeof(tx)) {
		return -EINVAL;
	}

	tx[0] = (uint8_t) reg;
	memcpy(&tx[1], value, count);

	return sys_i2c_write(&max30208.device, tx, count + 1);
}

/** Write a single register byte. The caller must own the shared bus. */
static inline int max30208_i2c_write_byte(enum max30208_register_type reg, uint8_t value) {
	return max30208_i2c_write(reg, &value, 1);
}

/** Read @p count bytes starting at @p reg. The caller must own the shared bus. */
static inline int max30208_i2c_read(enum max30208_register_type reg, uint8_t* value, size_t count) {
	uint8_t addr = (uint8_t) reg;

	return sys_i2c_write_read(&max30208.device, &addr, sizeof(addr), value, count);
}

/** Write a single register byte under its own ownership scope. */
static int max30208_write_locked(enum max30208_register_type reg, uint8_t value) {
	if (!max30208_bus_lock()) {
		return -EIO;
	}
	int ret = max30208_i2c_write_byte(reg, value);
	max30208_bus_unlock();
	return ret != 0 ? -EIO : 0;
}

/** Read @p count bytes under its own ownership scope. */
static int max30208_read_locked(enum max30208_register_type reg, uint8_t* value, size_t count) {
	if (!max30208_bus_lock()) {
		return -EIO;
	}
	int ret = max30208_i2c_read(reg, value, count);
	max30208_bus_unlock();
	return ret != 0 ? -EIO : 0;
}

/* ------------------------------------------------------------------------- */
/* Probe and configuration                                                    */
/* ------------------------------------------------------------------------- */

/** Probe for an I2C response by reading PART_ID. The caller must own the bus. */
static bool max30208_probe(void) {
	uint8_t part_id = 0u;
	int ret = max30208_i2c_read(max30208_register_PartID, &part_id, sizeof(part_id));

	max30208.state.bits.bProbed = 1;
	if (ret != 0) {
		LOG_WRN("MAX30208 not detected on I2C bus (%d)", ret);
		max30208.state.bits.bDeviceFound = 0;
		return false;
	}
	if (part_id == MAX30208_PART_ID) {
		LOG_INF("MAX30208 detected on I2C bus");
		max30208.state.bits.bDeviceFound = 1;
	} else {
		LOG_WRN("MAX30208 probe failed: unexpected PART_ID 0x%02x", part_id);
		max30208.state.bits.bDeviceFound = 0;
	}
	return max30208.state.bits.bDeviceFound != 0;
}

void max30208_get_default_config(struct max30208_config_t* config) {
	memset(config, 0, sizeof(*config));

	config->fifo_config1.bits.a_full = MAX30208_FIFO_A_FULL_THRESHOLD;

	config->fifo_config2.bits.fifo_ro = MAX30208_FIFO_ROLLOVER;
	config->fifo_config2.bits.a_full_type = MAX30208_FIFO_A_FULL_TYPE;
	config->fifo_config2.bits.fifo_stat_clr = MAX30208_FIFO_STATUS_CLEAR;

	config->interrupts.bits.temp_ready_en = MAX30208_INT_TEMP_READY_ENABLE;
	config->interrupts.bits.a_full_en = MAX30208_INT_A_FULL_ENABLE;

	config->gpio_setup.value = MAX30208_GPIO_SETUP_DEFAULT;

	config->alarm_high_counts = (int16_t) MAX30208_ALARM_HIGH_DEFAULT;
	config->alarm_low_counts = (int16_t) MAX30208_ALARM_LOW_DEFAULT;
}

/** Issue a software reset and wait for it to clear. The caller must own the bus. */
static bool max30208_reset_locked(void) {
	union max30208_system_control_register_t sysctl = {.value = 0};
	sysctl.bits.reset = 1;

	if (max30208_i2c_write_byte(max30208_register_SystemControl, sysctl.value) != 0) {
		return false;
	}

	/* Poll until the device clears the self-clearing reset bit. */
	for (int attempts = 0; attempts < 16; ++attempts) {
		uint8_t value = 0;
		if (max30208_i2c_read(max30208_register_SystemControl, &value, sizeof(value)) != 0) {
			return false;
		}
		sysctl.value = value;
		if (sysctl.bits.reset == 0) {
			return true;
		}
	}
	LOG_ERR("MAX30208 reset did not clear");
	return false;
}

int max30208_config(struct max30208_config_t* config) {
	if (max30208.state.bits.bInitialized == 0) {
		LOG_ERR("MAX30208 not initialized");
		return -EINVAL;
	}

	if (config == NULL) {
		max30208_get_default_config(&max30208.config);
	} else {
		memcpy(&max30208.config, config, sizeof(max30208.config));
	}

	const struct max30208_config_t* cfg = &max30208.config;

	uint8_t alarm_high[2] = {
		(uint8_t) ((uint16_t) cfg->alarm_high_counts >> 8),
		(uint8_t) ((uint16_t) cfg->alarm_high_counts & 0xFF),
	};
	uint8_t alarm_low[2] = {
		(uint8_t) ((uint16_t) cfg->alarm_low_counts >> 8),
		(uint8_t) ((uint16_t) cfg->alarm_low_counts & 0xFF),
	};

	if (!max30208_bus_lock()) {
		return -EIO;
	}

	bool ret =
		max30208_reset_locked() &&
		(max30208_i2c_write_byte(max30208_register_FifoConfig1, cfg->fifo_config1.value) == 0) &&
		(max30208_i2c_write_byte(max30208_register_FifoConfig2, cfg->fifo_config2.value) == 0) &&
		(max30208_i2c_write_byte(max30208_register_InterruptEnable, cfg->interrupts.value) == 0) &&
		(max30208_i2c_write(max30208_register_AlarmHighMsb, alarm_high, sizeof(alarm_high)) == 0) &&
		(max30208_i2c_write(max30208_register_AlarmLowMsb, alarm_low, sizeof(alarm_low)) == 0) &&
		(max30208_i2c_write_byte(max30208_register_GpioSetup, cfg->gpio_setup.value) == 0);

	ret &= max30208_bus_unlock();
	if (!ret) {
		return -EIO;
	}

	max30208.state.bits.bConfigured = 1;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Acquisition                                                                */
/* ------------------------------------------------------------------------- */

/** Effective sampling period in milliseconds, derived from the sampling rate. */
static uint32_t max30208_sample_period_ms(void) {
	uint32_t hz = max30208.sampling_rate_hz;

	if (hz == 0u) {
		hz = 1u;
	}

	return MAX(1u, 1000u / hz);
}

/** Decode one raw 16-bit FIFO word into milli-degrees Celsius. */
static int32_t max30208_decode_temperature(const uint8_t* raw) {
	int16_t counts = (int16_t) (((uint16_t) raw[0] << 8) | raw[1]);

	return (int32_t) counts * MAX30208_TEMP_LSB_MDEG_C;
}

/**
 * @brief Trigger one conversion and drain the FIFO into the internal buffer.
 *
 * The shared bus is acquired per register access rather than held across the
 * conversion wait, so the poll delay does not block other consumers of the bus.
 *
 * @retval >=0 Number of samples appended to the internal buffer.
 * @retval -EIO Locking or a register transfer failed.
 * @retval -ETIMEDOUT The conversion did not complete in time.
 */
static int max30208_drain_fifo(void) {
	union max30208_temp_setup_register_t setup = {.value = MAX30208_TEMP_SETUP_CONVERT};

	int ret = max30208_write_locked(max30208_register_TempSetup, setup.value);
	if (ret != 0) {
		return ret;
	}

	/* Poll the status register until the conversion-ready bit asserts. The bus is
	 * released between polls so the conversion delay does not starve other
	 * devices on the shared system I2C bus. */
	union max30208_status_register_t status = {.value = 0};
	uint32_t waited_ms = 0;
	do {
		k_msleep(MAX30208_CONVERSION_POLL_MS);
		waited_ms += MAX30208_CONVERSION_POLL_MS;

		ret = max30208_read_locked(max30208_register_Status, &status.value, sizeof(status.value));
		if (ret != 0) {
			return ret;
		}
	} while (status.bits.temp_ready == 0 && waited_ms < MAX30208_CONVERSION_TIMEOUT_MS);

	if (status.bits.temp_ready == 0) {
		LOG_WRN("MAX30208 conversion timed out");
		return -ETIMEDOUT;
	}

	union max30208_fifo_data_count_register_t data_count = {.value = 0};
	ret = max30208_read_locked(max30208_register_FifoDataCount,
							   &data_count.value,
							   sizeof(data_count.value));
	if (ret != 0) {
		return ret;
	}

	size_t available = data_count.bits.count;
	if (available == 0) {
		LOG_INF("MAX30208 FIFO is empty");
		return 0;
	}

	if (available > MAX30208_SAMPLE_BUFFER_DEPTH) {
		LOG_WRN("MAX30208 FIFO has %zu samples, but internal buffer can only hold %d; dropping "
				"oldest samples",
				available,
				MAX30208_SAMPLE_BUFFER_DEPTH);
		available = MAX30208_SAMPLE_BUFFER_DEPTH;
	}
	if (available > 1) {
		LOG_WRN("MAX30208 FIFO has %zu samples. All samples will have the same timestamp.",
				available);
	}

	uint8_t raw[MAX30208_SAMPLE_BUFFER_DEPTH * MAX30208_FIFO_SAMPLE_BYTES] = {0};
	ret = max30208_read_locked(max30208_register_FifoData,
							   raw,
							   available * MAX30208_FIFO_SAMPLE_BYTES);
	if (ret != 0) {
		return ret;
	}

	time_t now = rtc_get_timestamp_us();
	for (size_t i = 0; i < available; i++) {
		if (max30208.sample_count >= MAX30208_SAMPLE_BUFFER_DEPTH) {
			LOG_WRN("MAX30208 internal sample buffer overflow, dropping sample");
			break;
		}
		max30208.samples[max30208.sample_count].temperature_mdeg_c =
			max30208_decode_temperature(&raw[i * MAX30208_FIFO_SAMPLE_BYTES]);
		max30208.samples[max30208.sample_count].timestamp = now;
		max30208.sample_count++;
	}

	return (int) available;
}

/** Sampling timer expiry: notify the consumer to drain the next sample(s). */
static void max30208_sample_timer_expiry(struct k_timer* timer) {
	ARG_UNUSED(timer);
	max30208_post_event_isr(max30208_TimerIrq, 0);
}

/* ------------------------------------------------------------------------- */
/* Public lifecycle                                                           */
/* ------------------------------------------------------------------------- */

int max30208_init(void) {
	if (max30208.state.bits.bInitialized != 0) {
		return 0;
	}

	if (!sys_i2c_is_ready(&max30208.device)) {
		LOG_ERR("SYS_I2C bus not ready");
		return -ENODEV;
	}

	max30208_get_default_config(&max30208.config);

	if (!max30208_bus_lock()) {
		return -EIO;
	}
	bool found = max30208_probe();
	max30208_bus_unlock();

	if (!found) {
		LOG_ERR("MAX30208 not connected");
		return -ENODEV;
	}

	k_timer_init(&max30208.sample_timer, max30208_sample_timer_expiry, NULL);
	max30208.state.bits.bInitialized = 1;
	return 0;
}

bool max30208_is_ready(void) {
	return max30208.state.bits.bInitialized != 0 && max30208.state.bits.bDeviceFound != 0 &&
		   max30208.state.bits.bConfigured != 0;
}

int max30208_start(void) {
	if (max30208.state.bits.bInitialized == 0) {
		int ret = max30208_init();

		if (ret != 0) {
			return ret;
		}
	}

	/* Apply the acquisition configuration if it has not been programmed yet. */
	if (max30208.state.bits.bConfigured == 0) {
		int ret = max30208_config(NULL);

		if (ret != 0) {
			return ret;
		}
	}

	/* Flush the internal buffer when a new acquisition session begins. */
	max30208.sample_count = 0;

	uint32_t period_ms = max30208_sample_period_ms();
	k_timer_start(&max30208.sample_timer, K_MSEC(period_ms), K_MSEC(period_ms));
	max30208.state.bits.bSampling = 1;
	max30208_post_event(max30208_event_SamplingStarted, 0, 0);
	return 0;
}

void max30208_stop(void) {
	bool was_sampling = max30208.state.bits.bSampling != 0;

	k_timer_stop(&max30208.sample_timer);
	max30208.state.bits.bSampling = 0;

	if (was_sampling) {
		max30208_post_event(max30208_event_SamplingStopped, 0, 0);
	}
}

void max30208_deinit(void) {
	max30208_stop();
	max30208.state.bits.bInitialized = 0;
	max30208.state.bits.bConfigured = 0;
}

void max30208_set_sampling_rate(uint16_t new_sampling_rate) {
	if (new_sampling_rate == 0u) {
		return;
	}

	max30208.sampling_rate_hz = new_sampling_rate;

	if (max30208.state.bits.bSampling != 0) {
		uint32_t period_ms = max30208_sample_period_ms();
		k_timer_start(&max30208.sample_timer, K_MSEC(period_ms), K_MSEC(period_ms));
	}
}

int max30208_get_samples(struct temperature_sample_t* samples, size_t max_samples) {
	if (max30208.state.bits.bInitialized == 0 || max30208.state.bits.bSampling == 0) {
		return -EAGAIN;
	}

	/* Refresh the internal buffer with the latest conversion(s). */
	max30208.sample_count = 0;
	int drained = max30208_drain_fifo();
	if (drained < 0) {
		return drained;
	}

	max30208_post_event(max30208_event_SampleReady,
						(uint32_t) max30208.sample_count,
						(uintptr_t) max30208.samples);

	size_t copied = 0;
	if (samples != NULL) {
		copied = MIN(max30208.sample_count, max_samples);
		memcpy(samples, max30208.samples, copied * sizeof(*samples));
	}

	return (int) copied;
}
