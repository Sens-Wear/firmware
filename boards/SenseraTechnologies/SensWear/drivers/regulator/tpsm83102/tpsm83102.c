/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file tpsm83102.c
 * @brief SenseWear TPSM83102 regulator driver implementation.
 * @details The driver remains a Zephyr regulator device so existing consumers
 *          can use regulator_enable(), regulator_disable(), and voltage APIs.
 *          All register sequences use the SenseWear shared-I2C ownership
 *          wrapper. Private register helpers require the caller to own the bus.
 */

#define DT_DRV_COMPAT ti_tpsm83102

#include "tpsm83102.h"
#include "sys_i2c.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(tpsm83102, CONFIG_REGULATOR_LOG_LEVEL);

#define TPSM83102_ENUM_IDX_OR(node_id, prop, default_val) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop),       \
		    (DT_ENUM_IDX(node_id, prop)),           \
		    (default_val))

/** Static configuration for one devicetree TPSM83102 instance. */
struct tpsm83102_config {
	struct regulator_common_config common;
	struct sys_i2c_dt_spec device;
	struct gpio_dt_spec en_gpio;
	bool has_en_gpio;
	bool cfg_fpwm;
	bool cfg_fast_ramp_en;
	uint8_t cfg_discharge_mode;
	bool cfg_cl_ramp_min_high;
	uint8_t cfg_td_ramp;
	bool cfg_enable_scp;
	bool cfg_fast_dvs;
	bool strict_reset_defaults;
};

/** Software lifecycle flags for one TPSM83102 instance. */
union tpsm83102_state_t {
	uint8_t value;
	struct {
		uint8_t initialized : 1;
		uint8_t configured : 1;
		uint8_t probed : 1;
		uint8_t device_found : 1;
		uint8_t enabled : 1;
	} bits;
};

/** Runtime state for one TPSM83102 regulator device. */
struct tpsm83102_data {
	struct regulator_common_data common;
	union tpsm83102_state_t state;
	uint32_t output_uv;
};

/** Acquire shared-I2C ownership for one high-level regulator operation. */
static int tpsm83102_bus_lock(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	int ret = sys_i2c_lock(&cfg->device, K_MSEC(TPSM83102_I2C_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("Failed to lock SYS_I2C (%d)", ret);
	}
	return ret;
}

/** Fully release shared-I2C ownership after a high-level operation. */
static int tpsm83102_bus_release(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	int ret = sys_i2c_release(&cfg->device);

	if (ret != 0) {
		LOG_ERR("Failed to release SYS_I2C ownership (%d)", ret);
	}
	return ret;
}

/** Write one register. The caller must own the shared bus. */
static int tpsm83102_write_register(const struct device *dev,
				    enum tpsm83102_register_type reg,
				    uint8_t value)
{
	const struct tpsm83102_config *cfg = dev->config;
	uint8_t tx[] = {(uint8_t)reg, value};

	return sys_i2c_write(&cfg->device, tx, sizeof(tx));
}

/** Read one register. The caller must own the shared bus. */
static int tpsm83102_read_register(const struct device *dev,
				   enum tpsm83102_register_type reg,
				   uint8_t *value)
{
	const struct tpsm83102_config *cfg = dev->config;
	uint8_t address = (uint8_t)reg;

	return sys_i2c_write_read(&cfg->device, &address, sizeof(address),
				  value, sizeof(*value));
}

/** Update selected register bits. The caller must own the shared bus. */
static int tpsm83102_update_register(const struct device *dev,
				     enum tpsm83102_register_type reg,
				     uint8_t mask,
				     uint8_t value)
{
	uint8_t current;
	int ret = tpsm83102_read_register(dev, reg, &current);

	if (ret != 0) {
		return ret;
	}

	uint8_t next = (current & (uint8_t)~mask) | (value & mask);

	if (next == current) {
		return 0;
	}
	return tpsm83102_write_register(dev, reg, next);
}

/**
 * Probe using the defined register set.
 *
 * TPSM83102 has no device-ID register, so normal probing verifies I2C access
 * and CONTROL1's fixed-zero bits. The optional strict mode additionally
 * requires all three registers to contain their power-on reset values.
 */
static int tpsm83102_probe(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	struct tpsm83102_data *data = dev->data;
	union tpsm83102_CONTROL1_register_t control1 = {.value = 0U};
	union tpsm83102_VOUT_register_t vout = {.value = 0U};
	union tpsm83102_CONTROL2_register_t control2 = {.value = 0U};
	int ret;

	data->state.bits.probed = 1;
	data->state.bits.device_found = 0;

	ret = tpsm83102_read_register(dev, tpsm83102_register_CONTROL1, &control1.value);
	if (ret == 0) {
		ret = tpsm83102_read_register(dev, tpsm83102_register_VOUT, &vout.value);
	}
	if (ret == 0) {
		ret = tpsm83102_read_register(dev, tpsm83102_register_CONTROL2,
					     &control2.value);
	}
	if (ret != 0) {
		LOG_WRN("TPSM83102 did not respond to register probe (%d)", ret);
		return ret;
	}

	if ((control1.value & TPSM83102_CONTROL1_RESERVED_MASK) != 0U) {
		LOG_WRN("TPSM83102 CONTROL1 reserved bits are invalid: 0x%02x",
			control1.value);
		return -ENODEV;
	}

	if (cfg->strict_reset_defaults &&
	    ((control1.value != TPSM83102_CONTROL1_RESET) ||
	     (vout.value != TPSM83102_VOUT_RESET) ||
	     (control2.value != TPSM83102_CONTROL2_RESET))) {
		LOG_WRN("TPSM83102 reset defaults mismatch: C1=0x%02x VOUT=0x%02x C2=0x%02x",
			control1.value, vout.value, control2.value);
		return -ENODEV;
	}

	data->state.bits.device_found = 1;
	data->state.bits.enabled = control1.bits.bConverterEnable;
	LOG_INF("TPSM83102 detected: C1=0x%02x VOUT=0x%02x C2=0x%02x",
		control1.value, vout.value, control2.value);
	return 0;
}

/** Drive the external active-level EN signal when present. */
static int tpsm83102_set_hardware_enable(const struct device *dev, bool enabled)
{
	const struct tpsm83102_config *cfg = dev->config;

	if (!cfg->has_en_gpio) {
		return 0;
	}
	return gpio_pin_set_dt(&cfg->en_gpio, enabled ? 1 : 0);
}

/** Convert a VOUT code to microvolts. */
static uint32_t tpsm83102_code_to_uv(uint8_t code)
{
	if (code >= TPSM83102_VOUT_CODE_CLAMP_5V5) {
		return TPSM83102_VOUT_MAX_UV;
	}
	return TPSM83102_VOUT_MIN_UV +
	       ((uint32_t)code * TPSM83102_VOUT_STEP_UV);
}

/**
 * Select the lowest supported voltage inside a regulator API voltage window.
 */
static int tpsm83102_voltage_window_to_code(int32_t min_uv,
					    int32_t max_uv,
					    uint8_t *code)
{
	if ((code == NULL) || (min_uv > max_uv) ||
	    (max_uv < (int32_t)TPSM83102_VOUT_MIN_UV) ||
	    (min_uv > (int32_t)TPSM83102_VOUT_MAX_UV)) {
		return -EINVAL;
	}

	uint32_t target = MAX((uint32_t)MAX(min_uv, 0),
			      TPSM83102_VOUT_MIN_UV);
	uint32_t steps = DIV_ROUND_UP(target - TPSM83102_VOUT_MIN_UV,
				      TPSM83102_VOUT_STEP_UV);
	uint32_t selected = TPSM83102_VOUT_MIN_UV +
			    (steps * TPSM83102_VOUT_STEP_UV);

	if ((selected > TPSM83102_VOUT_MAX_UV) ||
	    (selected > (uint32_t)max_uv)) {
		return -EINVAL;
	}

	*code = selected == TPSM83102_VOUT_MAX_UV
			? TPSM83102_VOUT_CODE_CLAMP_5V5
			: (uint8_t)steps;
	return 0;
}

/** Apply the board policy encoded by optional devicetree properties. */
static int tpsm83102_apply_config(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	union tpsm83102_CONTROL1_register_t control1 = {.value = 0U};
	union tpsm83102_CONTROL2_register_t control2 = {.value = 0U};

	control1.bits.bEnableSCP = cfg->cfg_enable_scp;
	control1.bits.bEnableFastDVS = cfg->cfg_fast_dvs;

	control2.bits.bForcedPWM = cfg->cfg_fpwm;
	control2.bits.bFastRampEnable = cfg->cfg_fast_ramp_en;
	control2.bits.bDischargeMode = cfg->cfg_discharge_mode;
	control2.bits.bCurrentLimitHigh = cfg->cfg_cl_ramp_min_high;
	control2.bits.bRampTime = cfg->cfg_td_ramp;

	int ret = tpsm83102_update_register(dev, tpsm83102_register_CONTROL1,
					   TPSM83102_CONTROL1_EN_SCP |
						   TPSM83102_CONTROL1_EN_FAST_DVS,
					   control1.value);

	if (ret == 0) {
		ret = tpsm83102_update_register(dev, tpsm83102_register_CONTROL2,
					       TPSM83102_CONTROL2_FPWM |
						       TPSM83102_CONTROL2_FAST_RAMP_EN |
						       TPSM83102_CONTROL2_DISCH_MASK |
						       TPSM83102_CONTROL2_CL_RAMP_MIN |
						       TPSM83102_CONTROL2_TD_RAMP_MASK,
					       control2.value);
	}
	return ret;
}

static int tpsm83102_enable(const struct device *dev)
{
	struct tpsm83102_data *data = dev->data;
	int ret = tpsm83102_set_hardware_enable(dev, true);

	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_bus_lock(dev);
	if (ret != 0) {
		(void)tpsm83102_set_hardware_enable(dev, false);
		return ret;
	}

	ret = tpsm83102_update_register(dev, tpsm83102_register_CONTROL1,
					TPSM83102_CONTROL1_CONVERTER_EN,
					TPSM83102_CONTROL1_CONVERTER_EN);
	int release_ret = tpsm83102_bus_release(dev);

	if (ret == 0) {
		ret = release_ret;
	}
	if (ret != 0) {
		(void)tpsm83102_set_hardware_enable(dev, false);
		return ret;
	}

	data->state.bits.enabled = 1;
	return 0;
}

static int tpsm83102_disable(const struct device *dev)
{
	struct tpsm83102_data *data = dev->data;
	int ret = tpsm83102_bus_lock(dev);

	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_update_register(dev, tpsm83102_register_CONTROL1,
					TPSM83102_CONTROL1_CONVERTER_EN, 0U);
	int release_ret = tpsm83102_bus_release(dev);

	if (ret == 0) {
		ret = release_ret;
	}
	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_set_hardware_enable(dev, false);
	if (ret == 0) {
		data->state.bits.enabled = 0;
	}
	return ret;
}

static int tpsm83102_set_voltage(const struct device *dev,
				 int32_t min_uv,
				 int32_t max_uv)
{
	struct tpsm83102_data *data = dev->data;
	union tpsm83102_VOUT_register_t vout = {.value = 0U};
	int ret = tpsm83102_voltage_window_to_code(min_uv, max_uv, &vout.value);

	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_bus_lock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_write_register(dev, tpsm83102_register_VOUT, vout.value);
	int release_ret = tpsm83102_bus_release(dev);

	if (ret == 0) {
		ret = release_ret;
	}
	if (ret == 0) {
		data->output_uv = tpsm83102_code_to_uv(vout.bits.bOutputVoltage);
	}
	return ret;
}

static int tpsm83102_get_voltage(const struct device *dev, int32_t *uv)
{
	struct tpsm83102_data *data = dev->data;
	union tpsm83102_VOUT_register_t vout = {.value = 0U};
	int ret;

	if (uv == NULL) {
		return -EINVAL;
	}

	ret = tpsm83102_bus_lock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_read_register(dev, tpsm83102_register_VOUT, &vout.value);
	int release_ret = tpsm83102_bus_release(dev);

	if (ret == 0) {
		ret = release_ret;
	}
	if (ret == 0) {
		data->output_uv = tpsm83102_code_to_uv(vout.bits.bOutputVoltage);
		*uv = (int32_t)data->output_uv;
	}
	return ret;
}

static const struct regulator_driver_api tpsm83102_api = {
	.enable = tpsm83102_enable,
	.disable = tpsm83102_disable,
	.set_voltage = tpsm83102_set_voltage,
	.get_voltage = tpsm83102_get_voltage,
};

static int tpsm83102_init(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	struct tpsm83102_data *data = dev->data;
	union tpsm83102_VOUT_register_t vout = {.value = 0U};
	int ret;

	regulator_common_data_init(dev);
	data->state.value = 0U;
	data->output_uv = 0U;

	if (!sys_i2c_is_ready(&cfg->device)) {
		LOG_ERR("SYS_I2C device not ready");
		return -ENODEV;
	}

	if (cfg->has_en_gpio) {
		if (!device_is_ready(cfg->en_gpio.port)) {
			LOG_ERR("EN GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->en_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure EN GPIO (%d)", ret);
			return ret;
		}
		k_msleep(100);
	}

	ret = tpsm83102_bus_lock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = tpsm83102_probe(dev);
	if (ret == 0) {
		ret = tpsm83102_read_register(dev, tpsm83102_register_VOUT, &vout.value);
	}
	if (ret == 0) {
		ret = tpsm83102_apply_config(dev);
	}

	int release_ret = tpsm83102_bus_release(dev);
	if (ret == 0) {
		ret = release_ret;
	}
	if (ret != 0) {
		LOG_ERR("TPSM83102 initialization failed (%d)", ret);
		return ret;
	}

	data->output_uv = tpsm83102_code_to_uv(vout.bits.bOutputVoltage);
	data->state.bits.configured = 1;
	data->state.bits.initialized = 1;

	LOG_INF("TPSM83102 ready: %u uV, constraints %d..%d uV",
		data->output_uv, cfg->common.min_uv, cfg->common.max_uv);

	return regulator_common_init(dev, data->state.bits.enabled != 0U);
}

#define TPSM83102_CFG_INIT(node_id)                                                \
	static const struct tpsm83102_config tpsm83102_cfg_##node_id = {           \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                 \
		.device = SYS_I2C_DT_SPEC_GET(node_id),                             \
		.en_gpio = GPIO_DT_SPEC_GET_OR(node_id, enable_gpios, {0}),         \
		.has_en_gpio = DT_NODE_HAS_PROP(node_id, enable_gpios),             \
		.cfg_fpwm = DT_PROP_OR(node_id, ti_fpwm, 0),                        \
		.cfg_fast_ramp_en = DT_PROP_OR(node_id, ti_fast_ramp_enable, 0),    \
		.cfg_discharge_mode =                                               \
			TPSM83102_ENUM_IDX_OR(node_id, ti_discharge_vout, 0),       \
		.cfg_cl_ramp_min_high =                                             \
			DT_PROP_OR(node_id, ti_cl_ramp_min_high, 0),                \
		.cfg_td_ramp = DT_PROP_OR(node_id, ti_td_ramp, 5),                  \
		.cfg_enable_scp = DT_PROP_OR(node_id, ti_enable_scp, 0),            \
		.cfg_fast_dvs = DT_PROP_OR(node_id, ti_fast_dvs, 0),                \
		.strict_reset_defaults =                                            \
			DT_PROP_OR(node_id, ti_strict_reset_defaults, 0),           \
	};                                                                           \
	static struct tpsm83102_data tpsm83102_data_##node_id;                       \
	DEVICE_DT_DEFINE(node_id, tpsm83102_init, NULL,                               \
			 &tpsm83102_data_##node_id, &tpsm83102_cfg_##node_id,       \
			 POST_KERNEL, 61,                                           \
			 &tpsm83102_api);

DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, TPSM83102_CFG_INIT)
