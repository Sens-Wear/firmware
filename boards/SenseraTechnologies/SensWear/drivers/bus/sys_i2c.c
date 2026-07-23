/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sys_i2c.c
 * @brief SensWear shared system I2C ownership-wrapper implementation.
 *
 * The implementation maintains one mutex and one current-owner pointer for each
 * `senswear,sys-i2c` instance. Acquiring ownership reconfigures the underlying
 * Zephyr I2C controller with the requesting consumer's speed and address mode.
 * Transfer functions reject calls from any spec other than the current owner.
 *
 * Public design principles, the locking contract, devicetree examples, and
 * typical usage are documented in @ref senswear_sys_i2c.
 */

#define DT_DRV_COMPAT senswear_sys_i2c

#include "sys_i2c.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(sys_i2c, CONFIG_LOG_DEFAULT_LEVEL);

/** Static configuration for one shared-bus wrapper instance. */
struct sys_i2c_dev_config {
	/** Physical Zephyr I2C controller wrapped by this device. */
	const struct device *controller;
};

/** Runtime ownership state for one shared-bus wrapper instance. */
struct sys_i2c_dev_data {
	/** Serializes ownership between different consumer specifications. */
	struct k_mutex lock;
	/** Current ownership token, or NULL while the bus is unowned. */
	const struct sys_i2c_config *owner;
	/** Thread that currently owns the bus. */
	k_tid_t owner_thread;
	/** Number of balanced locks held by @ref owner_thread. */
	uint32_t lock_depth;
};

static inline bool sys_i2c_is_owner(const struct sys_i2c_dev_data *data,
			     const struct sys_i2c_dt_spec *spec)
{
	return (data->owner == &spec->config) &&
	       (data->owner_thread == k_current_get()) &&
	       (data->lock_depth > 0U);
}

bool sys_i2c_is_ready(const struct sys_i2c_dt_spec *spec)
{
	if ((spec == NULL) || !device_is_ready(spec->bus)) {
		return false;
	}

	const struct sys_i2c_dev_config *cfg = spec->bus->config;

	return device_is_ready(cfg->controller);
}

int sys_i2c_lock(const struct sys_i2c_dt_spec *spec, k_timeout_t timeout)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;
	const struct sys_i2c_dev_config *cfg = spec->bus->config;
	k_tid_t current_thread = k_current_get();

	int ret = k_mutex_lock(&data->lock, timeout);

	if (ret != 0) {
		return ret;
	}

	/*
	 * Zephyr mutexes are recursive. If the current thread already owns this
	 * wrapper, accept only a balanced nested lock for the same consumer spec.
	 */
	if (data->owner != NULL) {
		if ((data->owner != &spec->config) ||
		    (data->owner_thread != current_thread)) {
			k_mutex_unlock(&data->lock);
			return -EALREADY;
		}

		if (data->lock_depth == UINT32_MAX) {
			k_mutex_unlock(&data->lock);
			return -EOVERFLOW;
		}

		data->lock_depth++;
		return 0;
	}

	data->owner = &spec->config;
	data->owner_thread = current_thread;
	data->lock_depth = 1U;

	/* Reconfigure the controller for the new owner. */
	uint32_t dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(spec->config.speed);

	if (spec->config.ten_bit) {
		dev_config |= I2C_ADDR_10_BITS;
	}

	ret = i2c_configure(cfg->controller, dev_config);
	if (ret != 0) {
		LOG_ERR("i2c_configure failed: %d", ret);
		data->owner = NULL;
		data->owner_thread = NULL;
		data->lock_depth = 0U;
		k_mutex_unlock(&data->lock);
	}

	return ret;
}

int sys_i2c_unlock(const struct sys_i2c_dt_spec *spec)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;

	if (!sys_i2c_is_owner(data, spec)) {
		return -EACCES;
	}

	data->lock_depth--;
	if (data->lock_depth == 0U) {
		data->owner = NULL;
		data->owner_thread = NULL;
	}

	return k_mutex_unlock(&data->lock);
}

int sys_i2c_release(const struct sys_i2c_dt_spec *spec)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;

	if (!sys_i2c_is_owner(data, spec)) {
		return -EACCES;
	}

	uint32_t depth = data->lock_depth;

	data->owner = NULL;
	data->owner_thread = NULL;
	data->lock_depth = 0U;

	for (uint32_t i = 0U; i < depth; i++) {
		int ret = k_mutex_unlock(&data->lock);

		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

int sys_i2c_write(const struct sys_i2c_dt_spec *spec, const uint8_t *buf, uint32_t len)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;
	const struct sys_i2c_dev_config *cfg = spec->bus->config;

	if (!sys_i2c_is_owner(data, spec)) {
		return -EACCES;
	}

	return i2c_write(cfg->controller, buf, len, spec->config.addr);
}

int sys_i2c_read(const struct sys_i2c_dt_spec *spec, uint8_t *buf, uint32_t len)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;
	const struct sys_i2c_dev_config *cfg = spec->bus->config;

	if (!sys_i2c_is_owner(data, spec)) {
		return -EACCES;
	}

	return i2c_read(cfg->controller, buf, len, spec->config.addr);
}

int sys_i2c_write_read(const struct sys_i2c_dt_spec *spec,
		       const void *wr, size_t wlen, void *rd, size_t rlen)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_i2c_dev_data *data = spec->bus->data;
	const struct sys_i2c_dev_config *cfg = spec->bus->config;

	if (!sys_i2c_is_owner(data, spec)) {
		return -EACCES;
	}

	return i2c_write_read(cfg->controller, spec->config.addr, wr, wlen, rd, rlen);
}

static int sys_i2c_init(const struct device *dev)
{
	struct sys_i2c_dev_data *data = dev->data;
	const struct sys_i2c_dev_config *cfg = dev->config;

	k_mutex_init(&data->lock);
	data->owner = NULL;
	data->owner_thread = NULL;
	data->lock_depth = 0U;

	if (!device_is_ready(cfg->controller)) {
		LOG_ERR("underlying I2C controller not ready");
		return -ENODEV;
	}

	return 0;
}

#define SYS_I2C_INIT(inst)                                                         \
	static struct sys_i2c_dev_data sys_i2c_data_##inst;                        \
	static const struct sys_i2c_dev_config sys_i2c_config_##inst = {           \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),    \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, sys_i2c_init, NULL,                            \
			      &sys_i2c_data_##inst, &sys_i2c_config_##inst,        \
			      POST_KERNEL, CONFIG_SENSWEAR_SYS_I2C_INIT_PRIORITY, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(SYS_I2C_INIT)
