/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sys_spi.c
 * @brief SensWear shared system SPI ownership-wrapper implementation.
 *
 * Each `senswear,sys-spi` instance maintains a recursive mutex, the current
 * consumer configuration pointer, the owning thread, and the balanced lock
 * depth. Transfer calls verify ownership and pass the consumer's complete
 * struct spi_config to the wrapped Zephyr SPI controller.
 *
 * Public design principles, the locking contract, devicetree examples, and
 * typical usage are documented in @ref senswear_sys_spi.
 */

#define DT_DRV_COMPAT senswear_sys_spi

#include "sys_spi.h"

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(sys_spi, CONFIG_LOG_DEFAULT_LEVEL);

/** Static configuration for one shared-bus wrapper instance. */
struct sys_spi_dev_config {
	/** Physical Zephyr SPI controller wrapped by this device. */
	const struct device *controller;
};

/** Runtime ownership state for one shared-bus wrapper instance. */
struct sys_spi_dev_data {
	/** Recursive mutex serializing access to the physical controller. */
	struct k_mutex lock;
	/** Current consumer ownership token, or NULL while unowned. */
	const struct sys_spi_config *owner;
	/** Thread that currently owns the bus. */
	k_tid_t owner_thread;
	/** Number of balanced locks held by @ref owner_thread. */
	uint32_t lock_depth;
};

static inline bool sys_spi_is_owner(const struct sys_spi_dev_data *data,
			     const struct sys_spi_dt_spec *spec)
{
	return (data->owner == &spec->config) &&
	       (data->owner_thread == k_current_get()) &&
	       (data->lock_depth > 0U);
}

bool sys_spi_is_ready(const struct sys_spi_dt_spec *spec)
{
	if ((spec == NULL) || !device_is_ready(spec->bus)) {
		return false;
	}

	const struct sys_spi_dev_config *cfg = spec->bus->config;

	return device_is_ready(cfg->controller);
}

int sys_spi_lock(const struct sys_spi_dt_spec *spec, k_timeout_t timeout)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;
	k_tid_t current_thread = k_current_get();

	int ret = k_mutex_lock(&data->lock, timeout);

	if (ret != 0) {
		return ret;
	}

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

	/* SPI needs no explicit reconfigure call: the owner's spi_config is applied
	 * on every transceive below.
	 */
	return 0;
}

int sys_spi_unlock(const struct sys_spi_dt_spec *spec)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;

	if (!sys_spi_is_owner(data, spec)) {
		return -EACCES;
	}

	data->lock_depth--;
	if (data->lock_depth == 0U) {
		data->owner = NULL;
		data->owner_thread = NULL;
	}

	return k_mutex_unlock(&data->lock);
}

int sys_spi_release(const struct sys_spi_dt_spec *spec)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;

	if (!sys_spi_is_owner(data, spec)) {
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

int sys_spi_write(const struct sys_spi_dt_spec *spec, const struct spi_buf_set *tx)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;
	const struct sys_spi_dev_config *cfg = spec->bus->config;

	if (!sys_spi_is_owner(data, spec)) {
		return -EACCES;
	}

	return spi_write(cfg->controller, &spec->config.spi, tx);
}

int sys_spi_read(const struct sys_spi_dt_spec *spec, const struct spi_buf_set *rx)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;
	const struct sys_spi_dev_config *cfg = spec->bus->config;

	if (!sys_spi_is_owner(data, spec)) {
		return -EACCES;
	}

	return spi_read(cfg->controller, &spec->config.spi, rx);
}

int sys_spi_transceive(const struct sys_spi_dt_spec *spec,
		       const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	if (spec == NULL) {
		return -EINVAL;
	}

	struct sys_spi_dev_data *data = spec->bus->data;
	const struct sys_spi_dev_config *cfg = spec->bus->config;

	if (!sys_spi_is_owner(data, spec)) {
		return -EACCES;
	}

	return spi_transceive(cfg->controller, &spec->config.spi, tx, rx);
}

static int sys_spi_init(const struct device *dev)
{
	struct sys_spi_dev_data *data = dev->data;
	const struct sys_spi_dev_config *cfg = dev->config;

	k_mutex_init(&data->lock);
	data->owner = NULL;
	data->owner_thread = NULL;
	data->lock_depth = 0U;

	if (!device_is_ready(cfg->controller)) {
		LOG_ERR("underlying SPI controller not ready");
		return -ENODEV;
	}

	return 0;
}

#define SYS_SPI_INIT(inst)                                                         \
	static struct sys_spi_dev_data sys_spi_data_##inst;                        \
	static const struct sys_spi_dev_config sys_spi_config_##inst = {           \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),    \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, sys_spi_init, NULL,                            \
			      &sys_spi_data_##inst, &sys_spi_config_##inst,        \
			      POST_KERNEL, CONFIG_SENSWEAR_SYS_SPI_INIT_PRIORITY, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(SYS_SPI_INIT)
