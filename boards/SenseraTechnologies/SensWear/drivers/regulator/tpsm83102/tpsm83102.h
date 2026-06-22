/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file tpsm83102.h
 * @brief SenseWear TPSM83102 regulator-driver declarations.
 *
 * @defgroup sensewear_tpsm83102 SenseWear TPSM83102 regulator
 * @ingroup io_interfaces
 * @{
 *
 * The board driver exposes the TPSM83102 through Zephyr's regulator API. Use
 * DEVICE_DT_GET(DT_NODELABEL(tpsm83102)) with regulator_enable(),
 * regulator_disable(), regulator_set_voltage(), and regulator_get_voltage().
 * Register transfers are serialized through the SenseWear shared-I2C wrapper.
 */
#ifndef SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_
#define SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_

#include "tpsm83102_registers.h"

/** Maximum time allowed for acquiring the shared I2C bus, in milliseconds. */
#ifndef TPSM83102_I2C_TIMEOUT
#define TPSM83102_I2C_TIMEOUT (100)
#endif

/** @} */

#endif /* SENSWEAR_DRIVERS_REGULATOR_TPSM83102_H_ */
