/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sys_i2c.h
 * @brief Ownership-locking access to the SenseWear shared system I2C bus.
 *
 * @defgroup sensewear_sys_i2c SenseWear shared system I2C bus
 * @ingroup io_interfaces
 * @{
 *
 * The SenseWear main board routes several devices through one physical I2C
 * controller. Device drivers may require different controller settings and may
 * perform multi-transfer operations that must not be interleaved with transfers
 * from another device.
 *
 * `sys_i2c` therefore sits between board device drivers and Zephyr's I2C API:
 *
 * @code{.text}
 * board device driver
 *        |
 *        | sys_i2c_lock(), transfer(s), sys_i2c_unlock()
 *        v
 * sys_i2c ownership wrapper
 *        |
 *        | i2c_configure(), i2c_write(), i2c_read()
 *        v
 * Zephyr I2C controller
 * @endcode
 *
 * Each consumer owns a persistent struct sys_i2c_dt_spec. The address and bus
 * configuration stored inside that object identify the consumer. In particular,
 * the address of `spec->config` is used as the ownership token.
 *
 * @section sensewear_sys_i2c_contract Locking contract
 *
 * A high-level driver operation must:
 *
 * 1. Call sys_i2c_lock() before its first transfer.
 * 2. Keep the lock for the complete atomic register sequence.
 * 3. Use only sys_i2c_write(), sys_i2c_read(), or sys_i2c_write_read().
 * 4. Call sys_i2c_unlock() exactly once on every path after a successful lock.
 *
 * Locking is recursive for the owning thread when every nested acquisition uses
 * the same spec. Each successful lock must have a matching unlock. Ownership is
 * released only by the final unlock:
 *
 * @code{.c}
 * sys_i2c_lock(&charger_i2c, K_FOREVER); // depth 1
 * sys_i2c_lock(&charger_i2c, K_FOREVER); // depth 2
 * sys_i2c_unlock(&charger_i2c);          // depth 1, still owned
 * sys_i2c_unlock(&charger_i2c);          // depth 0, released
 * @endcode
 *
 * A thread may not acquire a different consumer spec while it recursively owns
 * the wrapper. Other threads block on the mutex until the complete nested lock
 * chain is released.
 *
 * @section sensewear_sys_i2c_devicetree Devicetree representation
 *
 * The shared wrapper references the physical controller:
 *
 * @code{.dts}
 * sys_i2c: sys_i2c {
 *     compatible = "sensewear,sys-i2c";
 *     controller = <&i2c21>;
 *     status = "okay";
 * };
 * @endcode
 *
 * Consumers retain the standard Zephyr I2C hierarchy:
 *
 * @code{.dts}
 * &i2c21 {
 * charger: charger@6a {
 *     compatible = "vendor,charger";
 *     reg = <0x6a>;
 *     status = "okay";
 * };
 * };
 * @endcode
 *
 * The physical parent and `reg` remain available to standard Zephyr tooling.
 * Board drivers construct a sys_i2c spec to enforce ownership while performing
 * transfers through the wrapper.
 *
 * @section sensewear_sys_i2c_example Typical usage
 *
 * The DT spec must have static or otherwise stable lifetime because its embedded
 * configuration object is the ownership token.
 *
 * @code{.c}
 * #define CHARGER_NODE DT_NODELABEL(charger)
 *
 * static const struct sys_i2c_dt_spec charger_i2c =
 *     SYS_I2C_DT_SPEC_GET(CHARGER_NODE);
 *
 * static int charger_read_register(uint8_t reg, uint8_t *value)
 * {
 *     int ret;
 *
 *     ret = sys_i2c_lock(&charger_i2c, K_MSEC(100));
 *     if (ret != 0) {
 *         return ret;
 *     }
 *
 *     ret = sys_i2c_write_read(&charger_i2c, &reg, sizeof(reg),
 *                              value, sizeof(*value));
 *
 *     // Always release ownership after a successful lock.
 *     int unlock_ret = sys_i2c_unlock(&charger_i2c);
 *     return ret != 0 ? ret : unlock_ret;
 * }
 * @endcode
 *
 * A multi-register transaction uses one lock for the entire sequence:
 *
 * @code{.c}
 * static int charger_configure(void)
 * {
 *     const uint8_t first[] = {REG_A, VALUE_A};
 *     const uint8_t second[] = {REG_B, VALUE_B};
 *     int ret;
 *
 *     ret = sys_i2c_lock(&charger_i2c, K_MSEC(100));
 *     if (ret != 0) {
 *         return ret;
 *     }
 *
 *     ret = sys_i2c_write(&charger_i2c, first, sizeof(first));
 *     if (ret == 0) {
 *         ret = sys_i2c_write(&charger_i2c, second, sizeof(second));
 *     }
 *
 *     int unlock_ret = sys_i2c_unlock(&charger_i2c);
 *     return ret != 0 ? ret : unlock_ret;
 * }
 * @endcode
 *
 */

#ifndef SENSEWEAR_SYS_I2C_H_
#define SENSEWEAR_SYS_I2C_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** I2C settings and ownership token associated with one consumer device. */
struct sys_i2c_config {
	/** 7-bit or 10-bit target address. */
	uint16_t addr;
	/** Zephyr I2C speed selector, such as I2C_SPEED_STANDARD. */
	uint8_t speed : 3;
	/** Set to one when the target address uses 10-bit addressing. */
	uint8_t ten_bit : 1;
};

/** Devicetree-derived connection between a consumer and the shared bus. */
struct sys_i2c_dt_spec {
	/** The `sensewear,sys-i2c` wrapper device. */
	const struct device *bus;
	/** Consumer settings whose address also serves as the ownership token. */
	struct sys_i2c_config config;
};

/**
 * @brief Construct a shared-I2C specification from a consumer node.
 *
 * The node must be a child of the physical system I2C controller and provide
 * the standard `reg` property. All current SenseWear system-I2C consumers use
 * standard-speed, 7-bit addressing.
 *
 * @param node Devicetree node identifier for the consumer.
 */
#define SYS_I2C_DT_SPEC_GET(node)                                                  \
	{                                                                          \
		.bus = DEVICE_DT_GET(DT_NODELABEL(sys_i2c)),                       \
		.config = {                                                        \
			.addr    = DT_REG_ADDR(node),                              \
			.speed   = I2C_SPEED_STANDARD,                             \
			.ten_bit = 0,                                              \
		},                                                                 \
	}

/**
 * @brief Test whether the wrapper and underlying controller are ready.
 *
 * @param spec Consumer specification.
 * @retval true Both devices are ready.
 * @retval false The specification is NULL or either device is not ready.
 */
bool sys_i2c_is_ready(const struct sys_i2c_dt_spec *spec);

/**
 * @brief Acquire the bus and configure it for a consumer.
 *
 * @param spec Consumer specification and ownership token.
 * @param timeout Maximum time to wait while another consumer owns the bus.
 * @retval 0 Ownership was acquired, including a nested acquisition by the
 *         owning thread using the same spec.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EALREADY The owning thread attempted to nest a different spec.
 * @retval -EOVERFLOW The recursive lock depth reached its maximum value.
 * @return A negative errno returned by the mutex or I2C controller.
 */
int sys_i2c_lock(const struct sys_i2c_dt_spec *spec, k_timeout_t timeout);

/**
 * @brief Release bus ownership.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @retval 0 Ownership was released.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread is not the current owner.
 * @return A negative errno returned by the mutex.
 */
int sys_i2c_unlock(const struct sys_i2c_dt_spec *spec);

/**
 * @brief Completely release ownership held by the calling thread.
 *
 * This is intended for high-level operation cleanup when private helpers may
 * have acquired nested locks. It clears the ownership state and releases the
 * recursive mutex completely.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @retval 0 All ownership levels were released.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread is not the current owner.
 * @return A negative errno returned while releasing the mutex.
 */
int sys_i2c_release(const struct sys_i2c_dt_spec *spec);

/**
 * @brief Write bytes to the consumer's configured target address.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param buf Bytes to write.
 * @param len Number of bytes to write.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative I2C errno.
 */
int sys_i2c_write(const struct sys_i2c_dt_spec *spec, const uint8_t *buf, uint32_t len);

/**
 * @brief Read bytes from the consumer's configured target address.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param buf Destination buffer.
 * @param len Number of bytes to read.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative I2C errno.
 */
int sys_i2c_read(const struct sys_i2c_dt_spec *spec, uint8_t *buf, uint32_t len);

/**
 * @brief Perform a combined write followed by read without releasing the bus.
 *
 * This is typically used to write a register address and then read its value.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param wr Bytes to write.
 * @param wlen Number of bytes to write.
 * @param rd Destination buffer.
 * @param rlen Number of bytes to read.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative I2C errno.
 */
int sys_i2c_write_read(const struct sys_i2c_dt_spec *spec,
		       const void *wr, size_t wlen, void *rd, size_t rlen);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SENSEWEAR_SYS_I2C_H_ */
