/*
 * Copyright 2024 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DRV2605 Datasheet: https://www.ti.com/lit/gpn/drv2605
 *
 * SensWear local copy:
 * Copied from Zephyr/NCS v3.3.0
 *   zephyr/drivers/haptics/drv2605.c
 * This file is intentionally vendored as the patch target for SensWear-specific
 * DRV2605 integration. Keep local changes documented in PATCHED_FROM_ZEPHYR.md.
 */

#define DT_DRV_COMPAT senswear_drv2605

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "daughter_if.h"
#include "device_driver_dts_ids.h"
#include "device_driver_events.h"
#include "drv2605.h"
#include "sys_i2c.h"

LOG_MODULE_REGISTER(DRV2605, CONFIG_LOG_DEFAULT_LEVEL);

#define DRV2605_REG_STATUS 0x0
#define DRV2605_DEVICE_ID GENMASK(7, 5)
#define DRV2605_DEVICE_ID_DRV2605 0x3
#define DRV2605_DEVICE_ID_DRV2605L 0x7
#define DRV2605_DIAG_RESULT BIT(3)
#define DRV2605_FB_STS BIT(2)
#define DRV2605_OVER_TEMP BIT(1)
#define DRV2605_OC_DETECT BIT(0)

#define DRV2605_REG_MODE 0x1
#define DRV2605_DEV_RESET BIT(7)
#define DRV2605_STANDBY BIT(6)
#define DRV2605_MODE GENMASK(2, 0)

#define DRV2605_REG_RT_PLAYBACK_INPUT 0x2

#define DRV2605_REG_UNNAMED 0x3
#define DRV2605_HI_Z_OUTPUT BIT(4)
#define DRV2605_LIBRARY_SEL GENMASK(2, 0)

#define DRV2605_REG_WAVEFORM_SEQUENCER 0x4
#define DRV2605_WAIT BIT(7)
#define DRV2605_WAV_FRM_SEQ GENMASK(6, 0)

#define DRV2605_REG_GO 0xc
#define DRV2605_GO BIT(0)

#define DRV2605_REG_OVERDRIVE_TIME_OFFSET 0xd

#define DRV2605_REG_SUSTAIN_TIME_OFFSET_POS 0xe

#define DRV2605_REG_SUSTAIN_TIME_OFFSET_NEG 0xf

#define DRV2605_REG_BRAKE_TIME_OFFSET 0x10

#define DRV2605_TIME_STEP_MS 5

#define DRV2605_REG_AUDIO_TO_VIBE_CONTROL 0x11
#define DRV2605_ATH_PEAK_TIME GENMASK(3, 2)
#define DRV2605_ATH_FILTER GENMASK(1, 0)

#define DRV2605_REG_AUDIO_TO_VIBE_MIN_INPUT_LEVEL 0x12

#define DRV2605_REG_AUDIO_TO_VIBE_MAX_INPUT_LEVEL 0x13

#define DRV2605_ATH_INPUT_STEP_UV (1800000 / 255)

#define DRV2605_REG_AUDIO_TO_VIBE_MIN_OUTPUT_DRIVE 0x14

#define DRV2605_REG_AUDIO_TO_VIBE_MAX_OUTPUT_DRIVE 0x15

#define DRV2605_ATH_OUTPUT_DRIVE_PCT (100 * 255)

#define DRV2605_REG_RATED_VOLTAGE 0x16

#define DRV2605_REG_OVERDRIVE_CLAMP_VOLTAGE 0x17

#define DRV2605_REG_AUTO_CAL_COMP_RESULT 0x18

#define DRV2605_REG_AUTO_CAL_BACK_EMF_RESULT 0x19

#define DRV2605_REG_FEEDBACK_CONTROL 0x1a
#define DRV2605_N_ERM_LRA BIT(7)
#define DRV2605_FB_BRAKE_FACTOR GENMASK(6, 4)
#define DRV2605_LOOP_GAIN GENMASK(3, 2)
#define DRV2605_BEMF_GAIN GENMASK(1, 0)

#define DRV2605_ACTUATOR_MODE_ERM 0
#define DRV2605_ACTUATOR_MODE_LRA 1

#define DRV2605_REG_CONTROL1 0x1b
#define DRV2605_STARTUP_BOOST BIT(7)
#define DRV2605_AC_COUPLE BIT(5)
#define DRV2605_DRIVE_TIME GENMASK(4, 0)

#define DRV2605_REG_CONTROL2 0x1c
#define DRV2605_BIDIR_INPUT BIT(7)
#define DRV2605_BRAKE_STABILIZER BIT(6)
#define DRV2605_SAMPLE_TIME GENMASK(5, 4)
#define DRV2605_BLANKING_TIME GENMASK(3, 2)
#define DRV2605_IDISS_TIME GENMASK(1, 0)

#define DRV2605_REG_CONTROL3 0x1d
#define DRV2605_NG_THRESH GENMASK(7, 6)
#define DRV2605_ERM_OPEN_LOOP BIT(5)
#define DRV2605_SUPPLY_COMP_DIS BIT(4)
#define DRV2605_DATA_FORMAT_RTP BIT(3)
#define DRV2605_LRA_DRIVE_MODE BIT(2)
#define DRV2605_N_PWM_ANALOG BIT(1)
#define DRV2605_LRA_OPEN_LOOP BIT(0)

#define DRV2605_REG_CONTROL4 0x1e
#define DRV2605_ZERO_CROSSING_TIME GENMASK(7, 6)
#define DRV2605_AUTO_CAL_TIME GENMASK(5, 4)
#define DRV2605_OTP_STATUS BIT(2)
#define DRV2605_OTP_PROGRAM BIT(0)

#define DRV2605_REG_BATT_VOLTAGE_MONITOR 0x21
#define DRV2605_VBAT_STEP_UV (5600000 / 255)

#define DRV2605_REG_LRA_RESONANCE_PERIOD 0x22

#define DRV2605_POWER_UP_DELAY_US 250
#define DRV2605_I2C_TIMEOUT_MS 100
#define DRV2605_RTP_ACTIVE_EVENT_PERIOD K_SECONDS(1)
#define DRV2605_AUTO_CAL_POLL_MS 10
#define DRV2605_AUTO_CAL_TIMEOUT_MS 1500

/* Fixed feedback timing used by the closed-loop LRA configuration. */
#define DRV2605_LRA_SAMPLE_TIME_FIELD 3
#define DRV2605_LRA_SAMPLE_TIME_US 300
#define DRV2605_LRA_BLANKING_TIME_FIELD 1
#define DRV2605_LRA_IDISS_TIME_FIELD 1
#define DRV2605_LRA_AUTO_CAL_TIME_FIELD 3

/* Voltage step sizes from the DRV2605/DRV2605L data-sheet equations. */
#define DRV2605_LRA_RATED_STEP_UV 20710U
#define DRV2605L_LRA_RATED_STEP_UV 20580U
#define DRV2605_ERM_RATED_STEP_UV 21330U
#define DRV2605L_ERM_RATED_STEP_UV 21180U
#define DRV2605_OD_CLAMP_STEP_UV 21960U
#define DRV2605L_OD_CLAMP_STEP_UV 21220U

/* Board-level VDD rail for the driver IC; this is not the actuator voltage. */
#define DRV2605_SUPPLY_VOLTAGE_UV (3600000)
/* Rail ramp and power-on settle time before the device is accessed. */
#define DRV2605_SUPPLY_RAMP_DELAY_MS 50

struct drv2605_config {
	struct sys_i2c_dt_spec i2c;
	struct gpio_dt_spec in_trig_gpio;
	const struct device* regulator;
	uint8_t feedback_brake_factor;
	uint8_t loop_gain;
	uint16_t rated_voltage_mv;
	uint16_t overdrive_clamp_voltage_mv;
	uint16_t resonant_frequency_hz;
	bool actuator_mode;
};

struct drv2605_data {
	const struct device* dev;
	struct k_work rtp_work;
	struct k_timer rtp_active_timer;
	const struct drv2605_rtp_data* rtp_data;
	enum drv2605_mode mode;
	const struct gpio_dt_spec* en_gpio;
	const struct device* regulator;
	uint8_t device_id;
	bool supply_enabled;
	atomic_t rtp_active;
	atomic_t rtp_active_seconds;
	atomic_t rtp_stop_requested;
};

static void drv2605_gpio_release(const struct device* dev);

static const char* const drv2605_event_names[drv2605_event_Count] = {
	[drv2605_event_Starting] = "Starting",
	[drv2605_event_Stopped] = "Stopped",
	[drv2605_event_PlaybackActive] = "PlaybackActive",
	[drv2605_event_Error] = "Error",
};

const char* drv2605_event_name(enum drv2605_event_type event_id) {
	if ((event_id >= drv2605_event_Count) || (drv2605_event_names[event_id] == NULL)) {
		return "Unknown";
	}

	return drv2605_event_names[event_id];
}

bool drv2605_rtp_is_active(const struct device* dev) {
	struct drv2605_data* data = dev->data;

	return atomic_get(&data->rtp_active) != 0;
}

static inline void drv2605_post_event(enum drv2605_event_type event, uint32_t v_param) {
	(void) device_driver_event_post(DRV2605_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									(uintptr_t) NULL,
									K_MSEC(DRV2605_I2C_TIMEOUT_MS));
}

static inline void drv2605_post_event_isr(enum drv2605_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(DRV2605_DEVICE_DTS_ID,
										(uint32_t) event,
										v_param,
										(uintptr_t) NULL);
}

static void drv2605_rtp_active_timer_handler(struct k_timer* timer) {
	struct drv2605_data* data = CONTAINER_OF(timer, struct drv2605_data, rtp_active_timer);
	uint32_t elapsed_s;

	if (!atomic_get(&data->rtp_active)) {
		return;
	}

	elapsed_s = (uint32_t) atomic_inc(&data->rtp_active_seconds) + 1U;
	drv2605_post_event_isr(drv2605_event_PlaybackActive, elapsed_s);
}

static int drv2605_bus_lock(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	int ret = sys_i2c_lock(&config->i2c, K_MSEC(DRV2605_I2C_TIMEOUT_MS));

	if (ret != 0) {
		LOG_ERR("Failed to lock DRV2605 SYS_I2C ownership: %d", ret);
	}

	return ret;
}

static int drv2605_bus_release(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	int ret = sys_i2c_release(&config->i2c);

	if (ret != 0) {
		LOG_ERR("Failed to release DRV2605 SYS_I2C ownership: %d", ret);
	}

	return ret;
}

static int drv2605_i2c_write_register(const struct device* dev, uint8_t reg, uint8_t value) {
	const struct drv2605_config* config = dev->config;
	uint8_t tx[] = {reg, value};

	return sys_i2c_write(&config->i2c, tx, sizeof(tx));
}

static int drv2605_i2c_read_register(const struct device* dev, uint8_t reg, uint8_t* value) {
	const struct drv2605_config* config = dev->config;

	return sys_i2c_write_read(&config->i2c, &reg, sizeof(reg), value, sizeof(*value));
}

static int drv2605_i2c_update_register(const struct device* dev,
									   uint8_t reg,
									   uint8_t mask,
									   uint8_t value) {
	uint8_t current;
	int ret;

	ret = drv2605_i2c_read_register(dev, reg, &current);
	if (ret < 0) {
		return ret;
	}

	current = (current & ~mask) | (value & mask);
	return drv2605_i2c_write_register(dev, reg, current);
}

/* Integer square root, used to evaluate the LRA rated-voltage equation without
 * pulling floating-point support into the firmware. */
static uint32_t drv2605_isqrt(uint32_t value) {
	uint32_t result = 0;
	uint32_t bit = BIT(30);

	while (bit > value) {
		bit >>= 2;
	}

	while (bit != 0U) {
		if (value >= (result + bit)) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}
		bit >>= 2;
	}

	return result;
}

static int drv2605_calculate_drive_time(const struct drv2605_config* config, uint8_t* drive_time) {
	uint32_t half_period_us;
	uint32_t field;

	if ((config->resonant_frequency_hz < 125U) || (config->resonant_frequency_hz > 300U)) {
		LOG_ERR("Unsupported LRA resonant frequency: %u Hz", config->resonant_frequency_hz);
		return -EINVAL;
	}

	half_period_us = DIV_ROUND_CLOSEST(500000U, config->resonant_frequency_hz);
	if (half_period_us <= 500U) {
		field = 0U;
	} else {
		field = DIV_ROUND_CLOSEST(half_period_us - 500U, 100U);
	}

	*drive_time = (uint8_t) MIN(field, (uint32_t) DRV2605_DRIVE_TIME);
	return 0;
}

static int drv2605_calculate_voltage_registers(const struct device* dev,
											   uint8_t* rated_voltage,
											   uint8_t* overdrive_clamp_voltage) {
	const struct drv2605_config* config = dev->config;
	const struct drv2605_data* data = dev->data;
	const bool is_drv2605l = data->device_id == DRV2605_DEVICE_ID_DRV2605L;
	uint32_t rated_step_uv;
	uint32_t clamp_step_uv;
	uint32_t rated_value;
	uint32_t clamp_value;

	clamp_step_uv = is_drv2605l ? DRV2605L_OD_CLAMP_STEP_UV : DRV2605_OD_CLAMP_STEP_UV;
	clamp_value =
		DIV_ROUND_CLOSEST((uint32_t) config->overdrive_clamp_voltage_mv * 1000U, clamp_step_uv);

	if (config->actuator_mode == DRV2605_ACTUATOR_MODE_LRA) {
		uint32_t feedback_window_ppm;
		uint32_t correction_sqrt;

		feedback_window_ppm =
			(4U * DRV2605_LRA_SAMPLE_TIME_US + 300U) * config->resonant_frequency_hz;
		if (feedback_window_ppm >= 1000000U) {
			return -EINVAL;
		}

		/* sqrt(1 - (4 * t_sample + 300 us) * f_LRA), represented in ppm. */
		correction_sqrt = drv2605_isqrt(1000000U - feedback_window_ppm);
		rated_step_uv = is_drv2605l ? DRV2605L_LRA_RATED_STEP_UV : DRV2605_LRA_RATED_STEP_UV;
		rated_value =
			DIV_ROUND_CLOSEST((uint32_t) config->rated_voltage_mv * correction_sqrt, rated_step_uv);
	} else {
		rated_step_uv = is_drv2605l ? DRV2605L_ERM_RATED_STEP_UV : DRV2605_ERM_RATED_STEP_UV;
		rated_value = DIV_ROUND_CLOSEST((uint32_t) config->rated_voltage_mv * 1000U, rated_step_uv);
	}

	if ((rated_value == 0U) || (rated_value > UINT8_MAX) || (clamp_value == 0U) ||
		(clamp_value > UINT8_MAX)) {
		LOG_ERR("Actuator voltage configuration is out of range: rated=%u mV, clamp=%u mV",
				config->rated_voltage_mv,
				config->overdrive_clamp_voltage_mv);
		return -EINVAL;
	}

	*rated_voltage = (uint8_t) rated_value;
	*overdrive_clamp_voltage = (uint8_t) clamp_value;
	return 0;
}

int drv2605_is_active(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	uint8_t value;
	int ret;
	int release_ret;

	/* RTP streaming state is tracked in software, so answer it without any bus
	 * traffic and before touching the device. */
	if (atomic_get(&data->rtp_active) != 0) {
		return 1;
	}

	/* ROM waveform-sequencer playback (internal trigger) has no software
	 * completion signal: the device clears the GO bit when the sequence ends.
	 * Read it back to tell whether a sequence is still running. Other GO-driven
	 * modes (diagnostics, auto-calibration) are not haptic playback and are not
	 * reported here. */
	if (data->mode != DRV2605_MODE_INTERNAL_TRIGGER) {
		return 0;
	}

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_GO, &value);

	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		return ret;
	}
	if (release_ret < 0) {
		return release_ret;
	}

	return FIELD_GET(DRV2605_GO, value) ? 1 : 0;
}

/*
 * Validate the dedicated DRV2605 supply rail and cache its regulator handle in
 * the driver object. Mirrors the MAX30101 power-up checks: a NULL regulator
 * means the caller powers the device; an already-live shared rail must already
 * sit at the fixed voltage the DRV2605 requires, otherwise driving it later
 * risks damaging the part.
 */
static int drv2605_supply_init(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	struct drv2605_data* data = dev->data;
	int32_t current_uv;
	int ret;

	__ASSERT(config->regulator != NULL, "DRV2605 regulator not specified");
	data->regulator = config->regulator;
	if (data->regulator == NULL) {
		return 0;
	}

	if (!device_is_ready(data->regulator)) {
		LOG_ERR("DRV2605 regulator %s not ready", data->regulator->name);
		return -ENODEV;
	}

	if (!regulator_is_enabled(data->regulator)) {
		return 0;
	}

	ret = regulator_get_voltage(data->regulator, &current_uv);
	if (ret < 0) {
		LOG_ERR("Failed to read DRV2605 regulator %s voltage: %d", data->regulator->name, ret);
		return ret;
	}

	if (current_uv != DRV2605_SUPPLY_VOLTAGE_UV) {
		LOG_ERR("DRV2605 regulator %s already enabled at %d uV, requires fixed %d uV",
				data->regulator->name,
				current_uv,
				DRV2605_SUPPLY_VOLTAGE_UV);
		return -EINVAL;
	}

	return 0;
}

/*
 * Bring the dedicated supply rail to the fixed DRV2605 voltage and enable it.
 * Idempotent: a rail this driver already enabled is left untouched. The shared
 * bus must not be held here, the regulator transport locks it itself.
 */
static int drv2605_supply_on(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	int ret;
	__ASSERT(data->regulator != NULL, "DRV2605 regulator not specified");
	if (data->regulator == NULL || data->supply_enabled) {
		return 0;
	}
	if (regulator_is_enabled(data->regulator)) {
		LOG_ERR("DRV2605 regulator %s already enabled, but this driver did not enable it",
				data->regulator->name);
		return -EINVAL;
	}
	ret = regulator_set_voltage(data->regulator,
								DRV2605_SUPPLY_VOLTAGE_UV,
								DRV2605_SUPPLY_VOLTAGE_UV);
	if (ret < 0) {
		LOG_ERR("Failed to set DRV2605 rail to %d uV: %d", DRV2605_SUPPLY_VOLTAGE_UV, ret);
		return ret;
	}

	ret = regulator_enable(data->regulator);
	if (ret < 0) {
		LOG_ERR("Failed to enable DRV2605 rail: %d", ret);
		return ret;
	}

	k_msleep(DRV2605_SUPPLY_RAMP_DELAY_MS);
	data->supply_enabled = true;
	return 0;
}

#ifdef CONFIG_PM_DEVICE
/* Disable the dedicated supply rail if this driver enabled it. */
static void drv2605_supply_off(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	int ret;

	if (data->regulator == NULL || !data->supply_enabled) {
		return;
	}

	ret = regulator_disable(data->regulator);
	if (ret < 0) {
		LOG_ERR("Failed to disable DRV2605 rail: %d", ret);
		return;
	}

	data->supply_enabled = false;
}
#endif

/*
 * The drv2605_haptic_config_* helpers perform register transactions and
 * require the caller to hold SYS_I2C ownership for this device.
 */
static inline int drv2605_haptic_config_audio(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	int ret;

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL3,
									  DRV2605_N_PWM_ANALOG,
									  DRV2605_N_PWM_ANALOG);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL1,
									  DRV2605_AC_COUPLE,
									  DRV2605_AC_COUPLE);
	if (ret < 0) {
		return ret;
	}

	data->mode = DRV2605_MODE_AUDIO_TO_VIBE;

	return 0;
}

static inline int drv2605_haptic_config_pwm_analog(const struct device* dev, const bool analog) {
	struct drv2605_data* data = dev->data;
	uint8_t value = 0;
	int ret;

	if (analog) {
		value = DRV2605_N_PWM_ANALOG;
	}

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_CONTROL3, DRV2605_N_PWM_ANALOG, value);
	if (ret < 0) {
		return ret;
	}

	data->mode = DRV2605_MODE_PWM_ANALOG_INPUT;

	return 0;
}

static int drv2605_write_rtp_input(const struct device* dev, uint8_t value, bool cancel_on_stop) {
	struct drv2605_data* data = dev->data;
	bool canceled = false;
	int ret;
	int release_ret;

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	/* Re-check after acquiring the bus. If stop_output() won the lock and
	 * already cleared RTP_INPUT, do not re-energize the actuator with a frame
	 * that passed the worker's earlier stop check. */
	if (cancel_on_stop && atomic_get(&data->rtp_stop_requested)) {
		canceled = true;
		ret = 0;
	} else {
		ret = drv2605_i2c_write_register(dev, DRV2605_REG_RT_PLAYBACK_INPUT, value);
	}
	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		return ret;
	}
	if (release_ret < 0) {
		return release_ret;
	}

	return canceled ? 1 : 0;
}

static void drv2605_rtp_work_handler(struct k_work* work) {
	struct drv2605_data* data = CONTAINER_OF(work, struct drv2605_data, rtp_work);
	const struct drv2605_rtp_data* rtp_data = data->rtp_data;
	int error = 0;
	int ret;
	int i;

	k_timer_start(&data->rtp_active_timer,
				  DRV2605_RTP_ACTIVE_EVENT_PERIOD,
				  DRV2605_RTP_ACTIVE_EVENT_PERIOD);

	for (i = 0; i < rtp_data->size; i++) {
		if (atomic_get(&data->rtp_stop_requested)) {
			break;
		}

		ret = drv2605_write_rtp_input(data->dev, rtp_data->rtp_input[i], true);
		if (ret > 0) {
			break;
		}
		if (ret < 0) {
			LOG_ERR("Failed to write DRV2605 RTP frame %d: %d", i, ret);
			error = ret;
			break;
		}

		k_usleep(rtp_data->rtp_hold_us[i]);
	}

	ret = drv2605_write_rtp_input(data->dev, 0, false);
	if ((ret < 0) && (error == 0)) {
		LOG_ERR("Failed to clear DRV2605 RTP input: %d", ret);
		error = ret;
	}

	k_timer_stop(&data->rtp_active_timer);
	if (error < 0) {
		drv2605_post_event(drv2605_event_Error, (uint32_t) -error);
	}
	drv2605_post_event(drv2605_event_Stopped, (uint32_t) DRV2605_MODE_RTP);

	atomic_set(&data->rtp_active, 0);
	atomic_set(&data->rtp_stop_requested, 0);
}

static inline int drv2605_haptic_config_rtp(const struct device* dev,
											const struct drv2605_rtp_data* rtp_data) {
	struct drv2605_data* data = dev->data;
	int ret;

	ret = drv2605_i2c_write_register(dev, DRV2605_REG_RT_PLAYBACK_INPUT, 0);
	if (ret < 0) {
		return ret;
	}

	/* The public API treats RTP bytes as an unsigned 0..255 amplitude. Use
	 * closed-loop unidirectional playback so 0 is off and 255 is full scale. */
	ret = drv2605_i2c_update_register(dev, DRV2605_REG_CONTROL2, DRV2605_BIDIR_INPUT, 0);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL3,
									  DRV2605_DATA_FORMAT_RTP,
									  DRV2605_DATA_FORMAT_RTP);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_MODE,
									  DRV2605_MODE,
									  (uint8_t) DRV2605_MODE_RTP);
	if (ret < 0) {
		return ret;
	}

	data->rtp_data = rtp_data;
	data->mode = DRV2605_MODE_RTP;

	return 0;
}

static inline int drv2605_haptic_config_rom(const struct device* dev,
											const struct drv2605_rom_data* rom_data) {
	uint8_t reg_addr = DRV2605_REG_WAVEFORM_SEQUENCER;
	struct drv2605_data* data = dev->data;
	int i, ret;

	switch (rom_data->trigger) {
	case DRV2605_MODE_INTERNAL_TRIGGER:
	case DRV2605_MODE_EXTERNAL_EDGE_TRIGGER:
	case DRV2605_MODE_EXTERNAL_LEVEL_TRIGGER:
		break;
	default:
		return -EINVAL;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_UNNAMED,
									  DRV2605_LIBRARY_SEL,
									  (uint8_t) rom_data->library);
	if (ret < 0) {
		return ret;
	}

	for (i = 0; i < DRV2605_WAVEFORM_SEQUENCER_MAX; i++) {
		ret = drv2605_i2c_write_register(dev, reg_addr, rom_data->seq_regs[i]);
		if (ret < 0) {
			return ret;
		}

		reg_addr++;

		if (rom_data->seq_regs[i] == 0U) {
			break;
		}
	}

	ret = drv2605_i2c_write_register(dev,
									 DRV2605_REG_OVERDRIVE_TIME_OFFSET,
									 rom_data->overdrive_time);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_write_register(dev,
									 DRV2605_REG_SUSTAIN_TIME_OFFSET_POS,
									 rom_data->sustain_pos_time);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_write_register(dev,
									 DRV2605_REG_SUSTAIN_TIME_OFFSET_NEG,
									 rom_data->sustain_neg_time);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_write_register(dev, DRV2605_REG_BRAKE_TIME_OFFSET, rom_data->brake_time);
	if (ret < 0) {
		return ret;
	}

	/* The on-chip TS2200 LRA library is encoded for bidirectional playback. */
	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL2,
									  DRV2605_BIDIR_INPUT,
									  DRV2605_BIDIR_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_CONTROL3, DRV2605_DATA_FORMAT_RTP, 0);
	if (ret < 0) {
		return ret;
	}

	/* Select the trigger only after the complete sequence is programmed. */
	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_MODE,
									  DRV2605_MODE,
									  (uint8_t) rom_data->trigger);
	if (ret < 0) {
		return ret;
	}

	data->mode = rom_data->trigger;
	return 0;
}

int drv2605_haptic_config(const struct device* dev,
						  enum drv2605_haptics_source source,
						  const union drv2605_config_data* config_data) {
	struct drv2605_data* data = dev->data;
	uint8_t go;
	int ret;
	int release_ret;

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	if (atomic_get(&data->rtp_active) != 0) {
		ret = -EBUSY;
		goto release_bus;
	}

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_GO, &go);
	if (ret < 0) {
		goto release_bus;
	}
	if (FIELD_GET(DRV2605_GO, go) != 0U) {
		ret = -EBUSY;
		goto release_bus;
	}

	switch (source) {
	case DRV2605_HAPTICS_SOURCE_ROM:
		ret = (config_data == NULL) || (config_data->rom_data == NULL)
				  ? -EINVAL
				  : drv2605_haptic_config_rom(dev, config_data->rom_data);
		break;
	case DRV2605_HAPTICS_SOURCE_RTP:
		ret = (config_data == NULL) || (config_data->rtp_data == NULL) ||
				  (config_data->rtp_data->size == 0U) ||
				  (config_data->rtp_data->rtp_hold_us == NULL) ||
					  (config_data->rtp_data->rtp_input == NULL)
				  ? -EINVAL
				  : drv2605_haptic_config_rtp(dev, config_data->rtp_data);
		break;
	case DRV2605_HAPTICS_SOURCE_AUDIO:
		ret = drv2605_haptic_config_audio(dev);
		break;
	case DRV2605_HAPTICS_SOURCE_PWM:
		ret = drv2605_haptic_config_pwm_analog(dev, false);
		break;
	case DRV2605_HAPTICS_SOURCE_ANALOG:
		ret = drv2605_haptic_config_pwm_analog(dev, true);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

release_bus:
	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		return ret;
	}

	return release_ret;
}

static inline int drv2605_edge_mode_event(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&config->in_trig_gpio, 1);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_set_dt(&config->in_trig_gpio, 0);
}

static int drv2605_stop_output(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	struct drv2605_data* data = dev->data;
	uint8_t value;
	enum drv2605_mode stopped_mode = data->mode;
	bool rtp_worker_will_post_stopped = false;
	int ret;
	int release_ret;

	if (data->mode == DRV2605_MODE_RTP && atomic_get(&data->rtp_active)) {
		atomic_set(&data->rtp_stop_requested, 1);
	}

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	switch (data->mode) {
	case DRV2605_MODE_DIAGNOSTICS:
	case DRV2605_MODE_AUTO_CAL:
		ret = drv2605_i2c_read_register(dev, DRV2605_REG_GO, &value);
		if (ret < 0) {
			break;
		}

		if (FIELD_GET(DRV2605_GO, value)) {
			LOG_DBG("Playback mode: %d is uninterruptible", data->mode);
			ret = -EBUSY;
			break;
		}

		break;
	case DRV2605_MODE_INTERNAL_TRIGGER:
		ret = drv2605_i2c_update_register(dev, DRV2605_REG_GO, DRV2605_GO, 0);
		break;
	case DRV2605_MODE_EXTERNAL_EDGE_TRIGGER:
		ret = drv2605_edge_mode_event(dev);
		break;
	case DRV2605_MODE_EXTERNAL_LEVEL_TRIGGER:
		ret = gpio_pin_set_dt(&config->in_trig_gpio, 0);
		break;
	case DRV2605_MODE_PWM_ANALOG_INPUT:
	case DRV2605_MODE_AUDIO_TO_VIBE:
		ret = drv2605_i2c_update_register(dev,
										  DRV2605_REG_MODE,
										  DRV2605_MODE,
										  (uint8_t) DRV2605_MODE_INTERNAL_TRIGGER);
		if (ret < 0) {
			break;
		}

		ret = drv2605_i2c_update_register(dev, DRV2605_REG_GO, DRV2605_GO, 0);
		break;
	case DRV2605_MODE_RTP:
		ret = drv2605_i2c_write_register(dev, DRV2605_REG_RT_PLAYBACK_INPUT, 0);
		if (ret < 0) {
			break;
		}

		ret = k_work_cancel(&data->rtp_work);
		if (ret == 0) {
			k_timer_stop(&data->rtp_active_timer);
			atomic_set(&data->rtp_active, 0);
			atomic_set(&data->rtp_stop_requested, 0);
		} else {
			rtp_worker_will_post_stopped = true;
		}
		ret = 0;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		return ret;
	}

	if ((release_ret == 0) && !rtp_worker_will_post_stopped) {
		drv2605_post_event(drv2605_event_Stopped, (uint32_t) stopped_mode);
	}

	// drv2605_supply_off(dev);

	return release_ret;
}

static int drv2605_start_output(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	struct drv2605_data* data = dev->data;
	enum drv2605_mode started_mode = data->mode;
	int ret;
	int release_ret;

	/* Bring the dedicated rail up before any bus traffic; the regulator
	 * transport locks the shared bus itself, so it cannot run under our lock. */
	ret = drv2605_supply_on(dev);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	switch (data->mode) {
	case DRV2605_MODE_DIAGNOSTICS:
	case DRV2605_MODE_AUTO_CAL:
	case DRV2605_MODE_INTERNAL_TRIGGER:
		ret = drv2605_i2c_update_register(dev, DRV2605_REG_GO, DRV2605_GO, DRV2605_GO);
		break;
	case DRV2605_MODE_EXTERNAL_EDGE_TRIGGER:
		ret = drv2605_edge_mode_event(dev);
		break;
	case DRV2605_MODE_EXTERNAL_LEVEL_TRIGGER:
		ret = gpio_pin_set_dt(&config->in_trig_gpio, 1);
		break;
	case DRV2605_MODE_AUDIO_TO_VIBE:
	case DRV2605_MODE_PWM_ANALOG_INPUT:
		ret =
			drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_MODE, (uint8_t) data->mode);
		break;
	case DRV2605_MODE_RTP:
		if (!atomic_cas(&data->rtp_active, 0, 1)) {
			ret = -EBUSY;
			break;
		}

		atomic_set(&data->rtp_active_seconds, 0);
		atomic_set(&data->rtp_stop_requested, 0);
		ret = k_work_submit(&data->rtp_work);
		if (ret < 0) {
			atomic_set(&data->rtp_active, 0);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		return ret;
	}

	if (release_ret == 0) {
		drv2605_post_event(drv2605_event_Starting, (uint32_t) started_mode);
	}

	return release_ret;
}

#ifdef CONFIG_PM_DEVICE
static int drv2605_pm_action(const struct device* dev, enum pm_device_action action) {
	struct drv2605_data* data = dev->data;
	int ret;
	int release_ret;

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		return ret;
	}

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_STANDBY, 0);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_STANDBY, DRV2605_STANDBY);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		if (data->en_gpio != NULL) {
			ret = gpio_pin_set_dt(data->en_gpio, 0);
		} else {
			ret = 0;
		}

		break;
	case PM_DEVICE_ACTION_TURN_ON:
		if (data->en_gpio != NULL) {
			ret = gpio_pin_set_dt(data->en_gpio, 1);
			if (ret == 0) {
				k_usleep(DRV2605_POWER_UP_DELAY_US);
			}
		} else {
			ret = 0;
		}

		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	release_ret = drv2605_bus_release(dev);

	/* The regulator transport locks the shared bus itself, so cut the rail only
	 * after the device bus lock is released. */
	if (ret == 0 && action == PM_DEVICE_ACTION_TURN_OFF) {
		drv2605_supply_off(dev);
	}

	if (ret < 0) {
		return ret;
	}

	return release_ret;
}
#endif

static int drv2605_auto_calibrate(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	uint8_t status;
	uint8_t go = DRV2605_GO;
	uint8_t compensation;
	uint8_t back_emf;
	uint8_t feedback;
	int elapsed_ms;
	int ret;
	int mode_ret;

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL4,
									  DRV2605_AUTO_CAL_TIME,
									  FIELD_PREP(DRV2605_AUTO_CAL_TIME,
												 DRV2605_LRA_AUTO_CAL_TIME_FIELD));
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_MODE,
									  DRV2605_MODE,
									  (uint8_t) DRV2605_MODE_AUTO_CAL);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_GO, DRV2605_GO, DRV2605_GO);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("Auto-calibrating %s for the fitted LRA",
			data->device_id == DRV2605_DEVICE_ID_DRV2605L ? "DRV2605L" : "DRV2605");
	for (elapsed_ms = 0; elapsed_ms < DRV2605_AUTO_CAL_TIMEOUT_MS;
		 elapsed_ms += DRV2605_AUTO_CAL_POLL_MS) {
		k_msleep(DRV2605_AUTO_CAL_POLL_MS);
		ret = drv2605_i2c_read_register(dev, DRV2605_REG_GO, &go);
		if (ret < 0) {
			return ret;
		}
		if (FIELD_GET(DRV2605_GO, go) == 0U) {
			break;
		}
	}

	if (FIELD_GET(DRV2605_GO, go) != 0U) {
		LOG_ERR("DRV2605 auto-calibration timed out");
		return -ETIMEDOUT;
	}

	/* DIAG_RESULT is valid after GO clears and is cleared by reading STATUS. */
	ret = drv2605_i2c_read_register(dev, DRV2605_REG_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_AUTO_CAL_COMP_RESULT, &compensation);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_AUTO_CAL_BACK_EMF_RESULT, &back_emf);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_FEEDBACK_CONTROL, &feedback);
	if (ret < 0) {
		return ret;
	}

	mode_ret = drv2605_i2c_update_register(dev,
										   DRV2605_REG_MODE,
										   DRV2605_MODE,
										   (uint8_t) DRV2605_MODE_INTERNAL_TRIGGER);
	if (mode_ret < 0) {
		return mode_ret;
	}
	data->mode = DRV2605_MODE_INTERNAL_TRIGGER;

	if ((status & (DRV2605_DIAG_RESULT | DRV2605_OVER_TEMP | DRV2605_OC_DETECT)) != 0U) {
		LOG_ERR("DRV2605 auto-calibration failed (STATUS=0x%02x)", status);
		return -EIO;
	}

	LOG_INF("DRV2605 calibration passed: A_CAL_COMP=0x%02x, A_CAL_BEMF=0x%02x, "
			"BEMF_GAIN=%u",
			compensation,
			back_emf,
			(unsigned int) FIELD_GET(DRV2605_BEMF_GAIN, feedback));
	return 0;
}

static int drv2605_hw_config(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	uint8_t rated_voltage;
	uint8_t overdrive_clamp_voltage;
	uint8_t drive_time;
	uint8_t mask, value;
	int ret;

	value = FIELD_PREP(DRV2605_N_ERM_LRA, config->actuator_mode) |
			FIELD_PREP(DRV2605_FB_BRAKE_FACTOR, config->feedback_brake_factor) |
			FIELD_PREP(DRV2605_LOOP_GAIN, config->loop_gain);

	mask = DRV2605_N_ERM_LRA | DRV2605_FB_BRAKE_FACTOR | DRV2605_LOOP_GAIN;

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_FEEDBACK_CONTROL, mask, value);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_calculate_voltage_registers(dev, &rated_voltage, &overdrive_clamp_voltage);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_write_register(dev, DRV2605_REG_RATED_VOLTAGE, rated_voltage);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_write_register(dev,
									 DRV2605_REG_OVERDRIVE_CLAMP_VOLTAGE,
									 overdrive_clamp_voltage);
	if (ret < 0) {
		return ret;
	}

	mask = DRV2605_ERM_OPEN_LOOP | DRV2605_LRA_DRIVE_MODE | DRV2605_LRA_OPEN_LOOP;
	if (config->actuator_mode == DRV2605_ACTUATOR_MODE_ERM) {
		value = DRV2605_ERM_OPEN_LOOP;
	} else {
		/* Library 6 is a closed-loop LRA library. Keep auto-resonance enabled
		 * and update its amplitude once per cycle for a symmetric waveform. */
		value = 0;
	}

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_CONTROL3, mask, value);
	if (ret < 0) {
		return ret;
	}

	if (config->actuator_mode != DRV2605_ACTUATOR_MODE_LRA) {
		return 0;
	}

	ret = drv2605_calculate_drive_time(config, &drive_time);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev,
									  DRV2605_REG_CONTROL1,
									  DRV2605_DRIVE_TIME,
									  FIELD_PREP(DRV2605_DRIVE_TIME, drive_time));
	if (ret < 0) {
		return ret;
	}

	mask = DRV2605_BIDIR_INPUT | DRV2605_SAMPLE_TIME | DRV2605_BLANKING_TIME | DRV2605_IDISS_TIME;
	value = DRV2605_BIDIR_INPUT | FIELD_PREP(DRV2605_SAMPLE_TIME, DRV2605_LRA_SAMPLE_TIME_FIELD) |
			FIELD_PREP(DRV2605_BLANKING_TIME, DRV2605_LRA_BLANKING_TIME_FIELD) |
			FIELD_PREP(DRV2605_IDISS_TIME, DRV2605_LRA_IDISS_TIME_FIELD);
	ret = drv2605_i2c_update_register(dev, DRV2605_REG_CONTROL2, mask, value);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("LRA config: %u Hz, RATED_VOLTAGE=0x%02x, OD_CLAMP=0x%02x, DRIVE_TIME=0x%02x",
			config->resonant_frequency_hz,
			rated_voltage,
			overdrive_clamp_voltage,
			drive_time);

	return drv2605_auto_calibrate(dev);
}

static int drv2605_reset(const struct device* dev) {
	int retries = 5, ret;
	uint8_t value;

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_STANDBY, 0);
	if (ret < 0) {
		return ret;
	}

	ret = drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_DEV_RESET, DRV2605_DEV_RESET);
	if (ret < 0) {
		return ret;
	}

	k_msleep(100);

	while (retries > 0) {
		retries--;

		ret = drv2605_i2c_read_register(dev, DRV2605_REG_MODE, &value);
		if (ret < 0) {
			k_usleep(10000);
			continue;
		}

		if ((value & DRV2605_DEV_RESET) == 0U) {
			return drv2605_i2c_update_register(dev, DRV2605_REG_MODE, DRV2605_STANDBY, 0);
		}
	}

	return -ETIMEDOUT;
}

static int drv2605_check_devid(const struct device* dev) {
	struct drv2605_data* data = dev->data;
	uint8_t value;
	int ret;

	ret = drv2605_i2c_read_register(dev, DRV2605_REG_STATUS, &value);
	if (ret < 0) {
		return ret;
	}

	value = FIELD_GET(DRV2605_DEVICE_ID, value);

	switch (value) {
	case DRV2605_DEVICE_ID_DRV2605:
		LOG_INF("Found DRV2605 (device ID 0x%x)", value);
		break;
	case DRV2605_DEVICE_ID_DRV2605L:
		LOG_INF("Found DRV2605L (device ID 0x%x)", value);
		break;
	default:
		LOG_ERR("Unsupported DRV2605-family device ID: 0x%x", value);
		return -ENOTSUP;
	}

	data->device_id = value;
	return 0;
}

static int drv2605_gpio_config(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	struct drv2605_data* data = dev->data;
	int ret;

	data->en_gpio = daughter_if_gpio_claim(daughter_if_GPIO0);
	if (data->en_gpio == NULL) {
		LOG_ERR("DRV2605 enable requires daughter_if GPIO0, but that line is already owned. "
				"Check enabled daughter boards and whether they use GPIO0.");
		__ASSERT(false, "DRV2605 daughter_if GPIO0 claim failed; check enabled daughter boards");
		return -EBUSY;
	}

	ret = gpio_pin_configure_dt(data->en_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		(void) daughter_if_gpio_release(daughter_if_GPIO0);
		data->en_gpio = NULL;
		return ret;
	}
	k_usleep(DRV2605_POWER_UP_DELAY_US);

	if (config->in_trig_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->in_trig_gpio)) {
			drv2605_gpio_release(dev);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->in_trig_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			drv2605_gpio_release(dev);
			return ret;
		}
	}

	return 0;
}

static void drv2605_gpio_release(const struct device* dev) {
	struct drv2605_data* data = dev->data;

	if (data->en_gpio == NULL) {
		return;
	}

	(void) gpio_pin_set_dt(data->en_gpio, 0);
	(void) daughter_if_gpio_release(daughter_if_GPIO0);
	data->en_gpio = NULL;
}

static int drv2605_init(const struct device* dev) {
	const struct drv2605_config* config = dev->config;
	struct drv2605_data* data = dev->data;
	int ret;
	int release_ret;

	data->dev = dev;
	LOG_INF("Initializing DRV2605 device %s", dev->name);

	if (!sys_i2c_is_ready(&config->i2c)) {
		LOG_ERR("DRV2605 I2C bus %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	ret = drv2605_supply_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize DRV2605 supply: %d", ret);
		return ret;
	}

	ret = drv2605_supply_on(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enable DRV2605 supply: %d", ret);
		return ret;
	}

	k_work_init(&data->rtp_work, drv2605_rtp_work_handler);
	k_timer_init(&data->rtp_active_timer, drv2605_rtp_active_timer_handler, NULL);

	ret = drv2605_gpio_config(dev);
	if (ret < 0) {
		LOG_ERR("Failed to allocate GPIOs: %d", ret);
		return ret;
	}

	ret = drv2605_bus_lock(dev);
	if (ret < 0) {
		LOG_ERR("Failed to lock DRV2605 SYS_I2C ownership: %d", ret);
		drv2605_gpio_release(dev);
		return ret;
	}

	ret = drv2605_check_devid(dev);
	if (ret < 0) {
		LOG_ERR("Failed to check DRV2605 device ID: %d", ret);
		goto release_bus;
	}

	ret = drv2605_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset device: %d", ret);
		goto release_bus;
	}

	ret = drv2605_hw_config(dev);
	if (ret < 0) {
		LOG_ERR("Failed to configure device: %d", ret);
		goto release_bus;
	}

release_bus:
	release_ret = drv2605_bus_release(dev);
	if (ret < 0) {
		drv2605_gpio_release(dev);
		return ret;
	}

	if (release_ret < 0) {
		drv2605_gpio_release(dev);
		return release_ret;
	}

	LOG_INF("DRV2605 device %s initialized successfully", dev->name);
	return 0;
}

static DEVICE_API(haptics, drv2605_driver_api) = {
	.start_output = &drv2605_start_output,
	.stop_output = &drv2605_stop_output,
};

#define HAPTICS_DRV2605_DEFINE(inst)                                                 \
                                                                                     \
	static const struct drv2605_config drv2605_config_##inst = {                     \
		.i2c = SYS_I2C_DT_SPEC_GET(DT_DRV_INST(inst)),                               \
		.in_trig_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, in_trig_gpios, {}),           \
		.regulator = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, vin_supply),            \
								 (DEVICE_DT_GET(DT_INST_PHANDLE(inst, vin_supply))), \
								 (NULL)),                                            \
		.feedback_brake_factor = DT_INST_ENUM_IDX(inst, feedback_brake_factor),      \
		.loop_gain = DT_INST_ENUM_IDX(inst, loop_gain),                              \
		.actuator_mode = DT_INST_ENUM_IDX(inst, actuator_mode),                      \
		.rated_voltage_mv = DT_INST_PROP(inst, vib_rated_mv),                        \
		.overdrive_clamp_voltage_mv = DT_INST_PROP(inst, vib_overdrive_mv),          \
		.resonant_frequency_hz = DT_INST_PROP(inst, vib_resonant_hz),                \
	};                                                                               \
                                                                                     \
	static struct drv2605_data drv2605_data_##inst = {                               \
		.mode = DRV2605_MODE_INTERNAL_TRIGGER,                                       \
	};                                                                               \
                                                                                     \
	PM_DEVICE_DT_INST_DEFINE(inst, drv2605_pm_action);                               \
                                                                                     \
	DEVICE_DT_INST_DEFINE(inst,                                                      \
						  drv2605_init,                                              \
						  PM_DEVICE_DT_INST_GET(inst),                               \
						  &drv2605_data_##inst,                                      \
						  &drv2605_config_##inst,                                    \
						  POST_KERNEL,                                               \
						  CONFIG_HAPTICS_INIT_PRIORITY,                              \
						  &drv2605_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HAPTICS_DRV2605_DEFINE)
