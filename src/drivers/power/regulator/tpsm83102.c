/*
 * TPSM83102 I2C-programmable buck-boost module regulator driver
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tpsm83102
#include <zephyr/sys/util_macro.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tpsm83102, CONFIG_REGULATOR_LOG_LEVEL);

/* Registers from datasheet */
#define TPSM83102_REG_CONTROL1 0x02
#define TPSM83102_REG_VOUT     0x03
#define TPSM83102_REG_CONTROL2 0x05

#define TPSM83102_ENUM_IDX_OR(node_id, prop, default_val) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), \
		    (DT_ENUM_IDX(node_id, prop)), \
		    (default_val))

/* CONTROL1 bits */
#define TPSM83102_CONTROL1_CONVERTER_EN BIT(0)
#define TPSM83102_CONTROL1_EN_SCP       BIT(2)
#define TPSM83102_CONTROL1_EN_FAST_DVS  BIT(3)

/* CONTROL2 fields */
#define TPSM83102_CONTROL2_TD_RAMP_MASK  0x07 /* bits[2:0] */
#define TPSM83102_CONTROL2_CL_RAMP_MIN   BIT(3)
#define TPSM83102_CONTROL2_DISCH_MASK    (BIT(4) | BIT(5)) /* bits[5:4] */
#define TPSM83102_CONTROL2_FAST_RAMP_EN  BIT(6)
#define TPSM83102_CONTROL2_FPWM          BIT(7)

/* VOUT encoding */
#define TPSM83102_VOUT_CODE_MAX_LINEAR 0xB4
#define TPSM83102_VOUT_CODE_CLAMP_5V5  0xB5

#define TPSM83102_UV_MIN 1000000  /* 1.0V */
#define TPSM83102_UV_MAX 5500000  /* 5.5V */
#define TPSM83102_UV_STEP 25000   /* 25mV */

/* Add near the top with the other register defs */
#define TPSM83102_REG_UNDEF0   0x00
#define TPSM83102_REG_UNDEF1   0x01
#define TPSM83102_REG_UNDEF4   0x04

/* Datasheet reset defaults */
#define TPSM83102_RESET_CONTROL1 0x08
#define TPSM83102_RESET_VOUT     0x5C
#define TPSM83102_RESET_CONTROL2 0x45

struct tpsm83102_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec en_gpio;
	bool has_en_gpio;
	/* Optional policy defaults applied at init */
	bool cfg_fpwm;
	bool cfg_fast_ramp_en;
	uint8_t cfg_discharge_mode; /* 0..3 encoded into bits[5:4] */
	bool cfg_cl_ramp_min_high;
	uint8_t cfg_td_ramp; /* 0..7 */
	bool cfg_enable_scp;
	bool cfg_fast_dvs;
};

struct tpsm83102_data {
	struct regulator_common_data common;
	/* Cached state */
	bool enabled;
	uint32_t last_uV;
};

static int tpsm83102_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct tpsm83102_config *cfg = dev->config;
	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

static int tpsm83102_reg_update(const struct device *dev, uint8_t reg,
				uint8_t mask, uint8_t value)
{
	const struct tpsm83102_config *cfg = dev->config;
	uint8_t cur;
	int ret = tpsm83102_reg_read(dev, reg, &cur);
	if (ret) {
		return ret;
	}

	uint8_t next = (cur & ~mask) | (value & mask);
	if (next == cur) {
		return 0;
	}

	return i2c_reg_write_byte_dt(&cfg->i2c, reg, next);
}


static int tpsm83102_verify_device(const struct device *dev, bool strict_reset_defaults)
{
	uint8_t c1, vout, c2;
	uint8_t u0, u1, u4;
	int ret;

	/* Read the three defined registers */
	ret = tpsm83102_reg_read(dev, TPSM83102_REG_CONTROL1, &c1);
	if (ret) {
		return ret;
	}
	ret = tpsm83102_reg_read(dev, TPSM83102_REG_VOUT, &vout);
	if (ret) {
		return ret;
	}
	ret = tpsm83102_reg_read(dev, TPSM83102_REG_CONTROL2, &c2);
	if (ret) {
		return ret;
	}

	/*
	 * CONTROL1: bits [7:4] are NIL (read as 0), bit1 is NIL (read as 0).
	 * So mask 0xF2 must always read back as 0.
	 */
	if ((c1 & 0xF2u) != 0u) {
		LOG_ERR("Unexpected CONTROL1 reserved bits: 0x%02x", c1);
		return -ENODEV;
	}

	/* Optional strict reset-default check (POR-only) */
	if (strict_reset_defaults) {
		if ((c1 != TPSM83102_RESET_CONTROL1) ||
		    (vout != TPSM83102_RESET_VOUT) ||
		    (c2 != TPSM83102_RESET_CONTROL2)) {
			LOG_ERR("Reset defaults mismatch: C1=0x%02x VOUT=0x%02x C2=0x%02x",
				c1, vout, c2);
			return -ENODEV;
		}
	}

	LOG_DBG("TPSM83102 probe OK: C1=0x%02x VOUT=0x%02x C2=0x%02x", c1, vout, c2);
	return 0;
}


static int tpsm83102_hw_en_set(const struct device *dev, bool on)
{
	const struct tpsm83102_config *cfg = dev->config;

	if (!cfg->has_en_gpio) {
		return 0;
	}

	if (!device_is_ready(cfg->en_gpio.port)) {
		return -ENODEV;
	}

	return gpio_pin_set_dt(&cfg->en_gpio, on ? 1 : 0);
}

static int tpsm83102_set_vout_code(const struct device *dev, uint8_t code)
{
	const struct tpsm83102_config *cfg = dev->config;
	return i2c_reg_write_byte_dt(&cfg->i2c, TPSM83102_REG_VOUT, code);
}

static int tpsm83102_get_vout_code(const struct device *dev, uint8_t *code)
{
	return tpsm83102_reg_read(dev, TPSM83102_REG_VOUT, code);
}

static int tpsm83102_uV_to_code(uint32_t uV, uint8_t *code)
{
	if (uV < TPSM83102_UV_MIN) {
		return -EINVAL;
	}
	if (uV >= TPSM83102_UV_MAX) {
		*code = TPSM83102_VOUT_CODE_CLAMP_5V5;
		return 0;
	}

	/* code = round((uV - 1.0V)/25mV) */
	uint32_t delta = uV - TPSM83102_UV_MIN;
	uint32_t c = (delta + (TPSM83102_UV_STEP / 2U)) / TPSM83102_UV_STEP;

	if (c > TPSM83102_VOUT_CODE_MAX_LINEAR) {
		*code = TPSM83102_VOUT_CODE_CLAMP_5V5;
	} else {
		*code = (uint8_t)c;
	}
	return 0;
}

static uint32_t tpsm83102_code_to_uV(uint8_t code)
{
	if (code >= TPSM83102_VOUT_CODE_CLAMP_5V5) {
		return TPSM83102_UV_MAX;
	}

	return TPSM83102_UV_MIN + ((uint32_t)code * TPSM83102_UV_STEP);
}

/* regulator API */

static int tpsm83102_enable(const struct device *dev)
{
	int ret;

	/* Ensure EN pin high first (if used) */
	ret = tpsm83102_hw_en_set(dev, true);
	if (ret) {
		return ret;
	}

	/* Set internal CONVERTER_EN bit (AND'ed with EN pin) */
	ret = tpsm83102_reg_update(dev, TPSM83102_REG_CONTROL1,
				   TPSM83102_CONTROL1_CONVERTER_EN,
				   TPSM83102_CONTROL1_CONVERTER_EN);
	if (ret) {
		return ret;
	}

	((struct tpsm83102_data *)dev->data)->enabled = true;
	return 0;
}

static int tpsm83102_disable(const struct device *dev)
{
	int ret;

	ret = tpsm83102_reg_update(dev, TPSM83102_REG_CONTROL1,
				   TPSM83102_CONTROL1_CONVERTER_EN, 0);
	if (ret) {
		return ret;
	}

	ret = tpsm83102_hw_en_set(dev, false);
	if (ret) {
		return ret;
	}

	((struct tpsm83102_data *)dev->data)->enabled = false;
	return 0;
}

static int tpsm83102_set_voltage(const struct device *dev,
				 int32_t min_uV, int32_t max_uV)
{
	/* choose a value within the window; prefer min_uV */
	if (min_uV > max_uV) {
		return -EINVAL;
	}

	uint32_t target = (uint32_t)min_uV;

	/* clamp to device range; still error if below minimum */
	if (target < TPSM83102_UV_MIN) {
		return -EINVAL;
	}
	if (target > TPSM83102_UV_MAX) {
		target = TPSM83102_UV_MAX;
	}

	/* ensure chosen voltage is not above max_uV */
	if ((int32_t)target > max_uV) {
		return -EINVAL;
	}

	uint8_t code;
	int ret = tpsm83102_uV_to_code(target, &code);
	if (ret) {
		return ret;
	}

	ret = tpsm83102_set_vout_code(dev, code);
	if (ret) {
		return ret;
	}

	((struct tpsm83102_data *)dev->data)->last_uV = tpsm83102_code_to_uV(code);
	return 0;
}

static int tpsm83102_get_voltage(const struct device *dev, int32_t *uV)
{
	uint8_t code;
	int ret = tpsm83102_get_vout_code(dev, &code);
	if (ret) {
		return ret;
	}

	*uV = (int32_t)tpsm83102_code_to_uV(code);
	return 0;
}

static const struct regulator_driver_api tpsm83102_api = {
	.enable = tpsm83102_enable,
	.disable = tpsm83102_disable,
	.set_voltage = tpsm83102_set_voltage,
	.get_voltage = tpsm83102_get_voltage,
};

static int tpsm83102_apply_defaults(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	int ret;

	/* CONTROL1 options */
	uint8_t c1_mask = 0U;
	uint8_t c1_val  = 0U;

	if (cfg->cfg_enable_scp) {
		c1_mask |= TPSM83102_CONTROL1_EN_SCP;
		c1_val  |= TPSM83102_CONTROL1_EN_SCP;
	}
	if (cfg->cfg_fast_dvs) {
		c1_mask |= TPSM83102_CONTROL1_EN_FAST_DVS;
		c1_val  |= TPSM83102_CONTROL1_EN_FAST_DVS;
	}

	if (c1_mask) {
		ret = tpsm83102_reg_update(dev, TPSM83102_REG_CONTROL1, c1_mask, c1_val);
		if (ret) {
			return ret;
		}
	}

	/* CONTROL2 options */
	uint8_t c2_mask = 0U;
	uint8_t c2_val  = 0U;

	if (cfg->cfg_fpwm) {
		c2_mask |= TPSM83102_CONTROL2_FPWM;
		c2_val  |= TPSM83102_CONTROL2_FPWM;
	}
	if (cfg->cfg_fast_ramp_en) {
		c2_mask |= TPSM83102_CONTROL2_FAST_RAMP_EN;
		c2_val  |= TPSM83102_CONTROL2_FAST_RAMP_EN;
	}

	/* discharge mode bits[5:4] */
	c2_mask |= TPSM83102_CONTROL2_DISCH_MASK;
	c2_val  |= (uint8_t)((cfg->cfg_discharge_mode & 0x3U) << 4);

	/* CL_RAMP_MIN bit3 */
	c2_mask |= TPSM83102_CONTROL2_CL_RAMP_MIN;
	if (cfg->cfg_cl_ramp_min_high) {
		c2_val |= TPSM83102_CONTROL2_CL_RAMP_MIN;
	}

	/* TD_RAMP bits[2:0] */
	c2_mask |= TPSM83102_CONTROL2_TD_RAMP_MASK;
	c2_val  |= (cfg->cfg_td_ramp & 0x7U);

	ret = tpsm83102_reg_update(dev, TPSM83102_REG_CONTROL2, c2_mask, c2_val);
	if (ret) {
		return ret;
	}

	return 0;
}

static int tpsm83102_init(const struct device *dev)
{
	const struct tpsm83102_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (cfg->has_en_gpio && !device_is_ready(cfg->en_gpio.port)) {
		LOG_ERR("EN GPIO not ready");
		return -ENODEV;
	}

	/* If EN GPIO exists, configure it */
	if (cfg->has_en_gpio) {
		ret = gpio_pin_configure_dt(&cfg->en_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure EN GPIO: %d", ret);
			return ret;
		}
	}
	gpio_pin_set_dt(&cfg->en_gpio, 1);
	k_msleep(100);
	/* Verify the device answers like TPSM83102 */
	ret = tpsm83102_verify_device(dev, true);
	if (ret) {
		LOG_ERR("Device probe failed: %d", ret);
		return ret;
	}
	LOG_INF("Constraints: min=%u max=%u",
        cfg->common.min_uv, cfg->common.max_uv);
	/* Apply default CONTROL1/CONTROL2 settings (does not enable converter) */
	ret = tpsm83102_apply_defaults(dev);
	if (ret) {
		LOG_ERR("Failed to apply defaults: %d", ret);
		return ret;
	}
	return 0;
}

/* Devicetree config extraction */

#define TPSM83102_CFG_INIT(node_id)                                                     \
	static const struct tpsm83102_config tpsm83102_cfg_##node_id = {                 \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                          \
		.i2c = I2C_DT_SPEC_GET(node_id),                                             \
		.en_gpio = GPIO_DT_SPEC_GET_OR(node_id, enable_gpios, {0}),                  \
		.has_en_gpio = DT_NODE_HAS_PROP(node_id, enable_gpios),                      \
		.cfg_fpwm = DT_PROP_OR(node_id, ti_fpwm, 0),                                  \
		.cfg_fast_ramp_en = DT_PROP_OR(node_id, ti_fast_ramp_enable, 0),             \
		.cfg_discharge_mode = TPSM83102_ENUM_IDX_OR(node_id, ti_discharge_vout, 0),  \
		.cfg_cl_ramp_min_high = DT_PROP_OR(node_id, ti_cl_ramp_min_high, 0),         \
		.cfg_td_ramp = DT_PROP_OR(node_id, ti_td_ramp, 5),                            \
		.cfg_enable_scp = DT_PROP_OR(node_id, ti_enable_scp, 0),                      \
		.cfg_fast_dvs = DT_PROP_OR(node_id, ti_fast_dvs, 0),                          \
	};                                                                                \
	static struct tpsm83102_data tpsm83102_data_##node_id;                            \
	DEVICE_DT_DEFINE(node_id, tpsm83102_init, NULL,                                   \
			 &tpsm83102_data_##node_id, &tpsm83102_cfg_##node_id,            \
			 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &tpsm83102_api);

DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, TPSM83102_CFG_INIT)
