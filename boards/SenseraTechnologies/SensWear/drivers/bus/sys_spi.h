/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sys_spi.h
 * @brief Ownership-locking access to the SensWear shared system SPI bus.
 *
 * @defgroup senswear_sys_spi SensWear shared system SPI bus
 * @ingroup io_interfaces
 * @{
 *
 * The SensWear main board connects multiple peripherals to one physical SPI
 * controller. Each peripheral may use a different chip-select, clock frequency,
 * polarity, phase, or word configuration. Multi-command operations such as
 * flash write-enable followed by program must not be interleaved with transfers
 * for another peripheral.
 *
 * `sys_spi` therefore sits between board device drivers and Zephyr's SPI API:
 *
 * @code{.text}
 * board device driver
 *        |
 *        | sys_spi_lock(), transfer(s), sys_spi_unlock()
 *        v
 * sys_spi ownership wrapper
 *        |
 *        | spi_write(), spi_read(), spi_transceive()
 *        v
 * Zephyr SPI controller
 * @endcode
 *
 * Each consumer owns a persistent struct sys_spi_dt_spec. Its embedded
 * struct sys_spi_config contains the Zephyr SPI frequency and operation
 * parameters. The address of that configuration object is used as the
 * ownership token.
 *
 * Chip-select is not managed by this wrapper or the SPI controller. Each
 * consumer driver owns its own CS GPIO (declared as a per-device `cs-gpios`
 * property) and asserts/deasserts it manually around its transfers. This gives
 * drivers full control over CS timing, including devices that require special
 * chip-select sequencing.
 *
 * Unlike I2C, SPI has no explicit target address in a transfer. The controller
 * applies `spec->config.spi` on every operation, selecting the bus parameters
 * for the consumer that currently holds the lock.
 *
 * @section senswear_sys_spi_contract Locking contract
 *
 * A high-level driver operation must:
 *
 * 1. Call sys_spi_lock() before its first transfer.
 * 2. Keep the lock for the complete atomic command sequence.
 * 3. Use only sys_spi_write(), sys_spi_read(), or sys_spi_transceive().
 * 4. Call sys_spi_unlock() exactly once for every successful lock.
 *
 * Locking is recursive for the owning thread when every nested acquisition uses
 * the same spec. Ownership is released only by the final matching unlock:
 *
 * @code{.c}
 * sys_spi_lock(&flash_spi, K_FOREVER); // depth 1
 * sys_spi_lock(&flash_spi, K_FOREVER); // depth 2
 * sys_spi_unlock(&flash_spi);          // depth 1, still owned
 * sys_spi_unlock(&flash_spi);          // depth 0, released
 * @endcode
 *
 * A thread may not acquire a different consumer spec while recursively owning
 * the wrapper. Other threads block until the complete nested lock chain has
 * been released. Transfer calls validate both the spec and owning thread.
 *
 * @section senswear_sys_spi_devicetree Devicetree representation
 *
 * The wrapper references the physical SPI controller:
 *
 * @code{.dts}
 * sys_spi: sys_spi {
 *     compatible = "senswear,sys-spi";
 *     controller = <&spi00>;
 *     status = "okay";
 * };
 * @endcode
 *
 * Peripherals retain the standard Zephyr SPI hierarchy. The controller declares
 * no `cs-gpios`; instead each peripheral declares its own `cs-gpios` so that the
 * owning driver can drive chip-select manually:
 *
 * @code{.dts}
 * sys_spi_peripheral: &spi00 {
 *     flash: eeprom@0 {
 *         compatible = "vendor,flash";
 *         reg = <0>;
 *         spi-max-frequency = <8000000>;
 *         cs-gpios = <&gpio2 3 GPIO_ACTIVE_LOW>;
 *         status = "okay";
 *     };
 *
 *     imu: imu@1 {
 *         compatible = "vendor,imu";
 *         reg = <1>;
 *         spi-max-frequency = <8000000>;
 *         cs-gpios = <&gpio2 5 GPIO_ACTIVE_LOW>;
 *         status = "okay";
 *     };
 * };
 * @endcode
 *
 * The physical parent and standard SPI properties remain available to Zephyr
 * tooling. Board drivers construct a sys_spi spec to enforce ownership and
 * configure/toggle their own CS GPIO from the per-device `cs-gpios` entry.
 *
 * @section senswear_sys_spi_example Typical usage
 *
 * The spec must have static or otherwise stable lifetime because its embedded
 * configuration object is the ownership token:
 *
 * @code{.c}
 * #define FLASH_NODE DT_NODELABEL(flash)
 *
 * static const struct sys_spi_dt_spec flash_spi =
 *     SYS_SPI_DT_SPEC_GET(FLASH_NODE,
 *                         SPI_OP_MODE_MASTER |
 *                         SPI_WORD_SET(8) |
 *                         SPI_TRANSFER_MSB);
 *
 * static int flash_read_id(uint8_t id[3])
 * {
 *     uint8_t command = 0x9f;
 *     const struct spi_buf tx_buf = {
 *         .buf = &command,
 *         .len = sizeof(command),
 *     };
 *     const struct spi_buf_set tx = {
 *         .buffers = &tx_buf,
 *         .count = 1,
 *     };
 *     struct spi_buf rx_bufs[] = {
 *         {.buf = NULL, .len = sizeof(command)},
 *         {.buf = id, .len = 3},
 *     };
 *     const struct spi_buf_set rx = {
 *         .buffers = rx_bufs,
 *         .count = ARRAY_SIZE(rx_bufs),
 *     };
 *     int ret = sys_spi_lock(&flash_spi, K_MSEC(100));
 *
 *     if (ret != 0) {
 *         return ret;
 *     }
 *
 *     ret = sys_spi_transceive(&flash_spi, &tx, &rx);
 *     int unlock_ret = sys_spi_unlock(&flash_spi);
 *     return ret != 0 ? ret : unlock_ret;
 * }
 * @endcode
 *
 * A multi-command flash operation uses one lock for the entire sequence:
 *
 * @code{.c}
 * static int flash_program(const uint8_t *program, size_t program_len)
 * {
 *     uint8_t write_enable = 0x06;
 *     const struct spi_buf we_buf = {
 *         .buf = &write_enable,
 *         .len = sizeof(write_enable),
 *     };
 *     const struct spi_buf_set we = {
 *         .buffers = &we_buf,
 *         .count = 1,
 *     };
 *     const struct spi_buf program_buf = {
 *         .buf = (void *)program,
 *         .len = program_len,
 *     };
 *     const struct spi_buf_set payload = {
 *         .buffers = &program_buf,
 *         .count = 1,
 *     };
 *     int ret = sys_spi_lock(&flash_spi, K_MSEC(100));
 *
 *     if (ret != 0) {
 *         return ret;
 *     }
 *
 *     ret = sys_spi_write(&flash_spi, &we);
 *     if (ret == 0) {
 *         ret = sys_spi_write(&flash_spi, &payload);
 *     }
 *
 *     int unlock_ret = sys_spi_unlock(&flash_spi);
 *     return ret != 0 ? ret : unlock_ret;
 * }
 * @endcode
 */

#ifndef SENSWEAR_SYS_SPI_H_
#define SENSWEAR_SYS_SPI_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** SPI settings and ownership token associated with one consumer device. */
struct sys_spi_config {
	/** Frequency and operation mode. Chip-select is driven by the consumer. */
	struct spi_config spi;
};

/** Devicetree-derived connection between a consumer and the shared bus. */
struct sys_spi_dt_spec {
	/** The `senswear,sys-spi` wrapper device. */
	const struct device *bus;
	/** Consumer settings whose object address serves as the ownership token. */
	struct sys_spi_config config;
};

/**
 * @brief Construct a shared-SPI specification from a consumer node.
 *
 * The node must be a standard child of the physical SPI controller and provide
 * `reg` and `spi-max-frequency`. Chip-select is not taken from the controller;
 * the consumer drives its own `cs-gpios` line, so the resulting spi_config
 * carries no GPIO chip-select.
 *
 * @param node Devicetree node identifier for the consumer.
 * @param op Zephyr SPI operation word, such as SPI_WORD_SET(8).
 */
#define SYS_SPI_DT_SPEC_GET(node, op)                                              \
	{                                                                          \
		.bus = DEVICE_DT_GET(DT_NODELABEL(sys_spi)),                       \
		.config = {                                                        \
			.spi = SPI_CONFIG_DT(node, op),                              \
		},                                                                 \
	}

/**
 * @brief Test whether the wrapper and underlying controller are ready.
 *
 * @param spec Consumer specification.
 * @retval true Both devices are ready.
 * @retval false The specification is NULL or either device is not ready.
 */
bool sys_spi_is_ready(const struct sys_spi_dt_spec *spec);

/**
 * @brief Acquire the shared SPI bus for a consumer.
 *
 * @param spec Consumer specification and ownership token.
 * @param timeout Maximum time to wait while another thread owns the bus.
 * @retval 0 Ownership was acquired, including a nested acquisition by the
 *         owning thread using the same spec.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EALREADY The owning thread attempted to nest a different spec.
 * @retval -EOVERFLOW The recursive lock depth reached its maximum value.
 * @return A negative errno returned by the mutex.
 */
int sys_spi_lock(const struct sys_spi_dt_spec *spec, k_timeout_t timeout);

/**
 * @brief Release one level of bus ownership.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @retval 0 One lock level was released.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread is not the current owner.
 * @return A negative errno returned by the mutex.
 */
int sys_spi_unlock(const struct sys_spi_dt_spec *spec);

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
int sys_spi_release(const struct sys_spi_dt_spec *spec);

/**
 * @brief Perform a write-only SPI transfer.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param tx Transmit buffers.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative SPI errno.
 */
int sys_spi_write(const struct sys_spi_dt_spec *spec, const struct spi_buf_set *tx);

/**
 * @brief Perform a read-only SPI transfer.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param rx Receive buffers.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative SPI errno.
 */
int sys_spi_read(const struct sys_spi_dt_spec *spec, const struct spi_buf_set *rx);

/**
 * @brief Perform a full-duplex SPI transfer.
 *
 * @param spec Consumer specification that currently owns the bus.
 * @param tx Transmit buffers, or NULL when no transmit data is required.
 * @param rx Receive buffers, or NULL when no receive data is required.
 * @retval -EINVAL @p spec is NULL.
 * @retval -EACCES @p spec or the calling thread does not own the bus.
 * @return Zero on success or a negative SPI errno.
 */
int sys_spi_transceive(const struct sys_spi_dt_spec *spec,
		       const struct spi_buf_set *tx, const struct spi_buf_set *rx);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SENSWEAR_SYS_SPI_H_ */
