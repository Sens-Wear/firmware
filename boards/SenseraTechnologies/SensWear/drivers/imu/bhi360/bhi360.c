/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bhi360.c
 * @brief SenseWear BHI360 smart IMU driver implementation.
 *
 * @section bhi360_impl_overview Implementation Overview
 *
 * This driver manages a BHI360 6-axis IMU with on-chip sensor fusion via the Bosch
 * Sensortec BHY2 host library. The driver operates entirely in interrupt-driven mode:
 * GPIO edges trigger bhi360_irq_callback() (ISR context), which posts bhi360_event_Irq
 * to the device-driver event queue. The application's event consumer thread calls
 * bhi360_process_irq() (thread context) to drain the FIFO, decode packets, and post
 * downstream sensor events.
 *
 * @section bhi360_impl_architecture Driver Architecture
 *
 * - **SPI Interface**: Singleton IMU struct holds SPI device spec, chip-select GPIO,
 *   reset GPIO, and two unconfigured GPIOs (gpio0, gpio1). SPI operations are guarded
 *   with sys_spi_lock() / sys_spi_unlock() to manage shared-bus arbitration.
 *
 * - **BHY2 Integration**: The driver wraps the Bosch BHY2 host library API, providing
 *   custom SPI read/write callbacks, delay function, and interrupt handling. The BHY2
 *   device struct (bhy2_dev) lives in the driver singleton; all higher-level operations
 *   delegate to BHY2 API calls.
 *
 * - **Sensor Configuration**: Three static tables enumerate enabled sensors:
 *   * **Base Sensors** (100 Hz): Rotation vector (quaternion), linear acceleration,
 *     gyroscope, and step counters (STC/STD variants at multiple power modes).
 *   * **Gesture Sensors** (1 Hz): Wake gesture, glance, pickup, wrist tilt, tilt
 *     detector, stationary/motion detectors, significant motion, and wrist-related
 *     detectors.
 *   * **Activity Sensors** (1 Hz): Activity recognition (AR) and wear activity
 *     recognition with wake-up variants.
 *
 * - **FIFO Parsing**: Each enabled sensor is registered with a callback function
 *   (parse_quaternion, parse_linear_acceleration, parse_gyro, parse_step_counter,
 *   parse_scalar_event, parse_activity, parse_meta_event) that decodes the packet
 *   payload and posts a corresponding device-driver event. Sensor data is cached in
 *   the driver singleton (quat_data, lacc_data, gyro_data, etc.) and passed to the
 *   event queue via p_param (driver-owned pointer).
 *
 * @section bhi360_impl_firmware Firmware
 *
 * The driver includes the firmware binary (BHI360_Aux_BMM150.fw.h or
 * BHI260AP-flash.fw.h) and uploads it to the device's RAM or flash at boot time.
 * The firmware contains the sensor fusion algorithm and virtual-sensor definitions;
 * upload must complete successfully before any sensors can be enabled.
 *
 * @section bhi360_impl_state_machine State Machine
 *
 * The driver maintains a state bitfield:
 * - initialized: SPI, GPIOs, and IRQ handlers ready; ready for configuration.
 * - configured: Firmware uploaded, sensors enabled; ready for IRQ processing.
 * - probed: Device communication successful (probed status after first product ID read).
 * - device_found: Product ID verified to be BHY2_PRODUCT_ID.
 * - irq_ready: GPIO interrupt callbacks registered and armed.
 *
 * bhi360_init() sets these bits; bhi360_configure() uploads firmware and enables
 * sensors. bhi360_stop() disables all sensors and clears the configured flag.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdio.h>
#include "bhi360.h"
#include "bhi360_api_error.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "bhi3_defs.h"
#include "sys_spi.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#define BHY2_RD_WR_LEN 256
#define WORK_BUFFER_SIZE 2048
#define BHI360_NODE DT_NODELABEL(bhi360)

BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, cs_gpios), "BHI360 is missing its chip-select GPIO");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, int_gpios), "BHI360 is missing its IRQ GPIO");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, gpio0_gpios), "BHI360 is missing GPIO0 wiring");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, gpio1_gpios), "BHI360 is missing GPIO1 wiring");

LOG_MODULE_REGISTER(bhi360, CONFIG_LOG_DEFAULT_LEVEL);

// TODO: Implement loading the firmware from external flash
/* Uncomment to upload firmware to flash instead of RAM */
/*#define UPLOAD_FIRMWARE_TO_FLASH*/

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "firmware/bhi360/BHI260AP-flash.fw.h"
#else
#include "firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#endif

#define BHI360_SPI_TIMEOUT K_MSEC(100)

/**
 * @brief Sensor enable configuration entry.
 *
 * Describes a single virtual sensor: its BHY2 sensor ID, human-readable name,
 * desired sample rate, latency tolerance, and the FIFO packet parser callback.
 * These entries populate static tables that drive sensor configuration during
 * bhi360_configure().
 */
struct bhi360_sensor_enable {
	/** @brief BHY2 sensor ID constant (e.g., BHY2_SENSOR_ID_RV, BHY2_SENSOR_ID_GYRO). */
	uint8_t sensor_id;
	/** @brief Human-readable sensor name for logging. */
	const char* name;
	/** @brief Desired output sample rate in Hz. */
	bhy2_float sample_rate_hz;
	/** @brief Acceptable latency in milliseconds; 0 for real-time. */
	uint32_t latency_ms;
	/** @brief FIFO parser callback function that decodes and posts events. */
	bhy2_fifo_parse_callback_t parser;
};

/**
 * @brief Driver state bitfield.
 *
 * Tracks initialization and configuration progression through the driver lifecycle.
 * Bits are set by bhi360_init() (initialized, irq_ready, probed, device_found),
 * bhi360_configure() (configured), and cleared by bhi360_stop() (configured).
 */
union bhi360_state_t {
	/** @brief Raw byte value. */
	uint8_t value;
	/** @brief Individual state flags. */
	struct {
		/** @brief SPI, GPIOs, and IRQ handlers ready; ready for configuration. */
		uint8_t initialized : 1;
		/** @brief Firmware uploaded and sensors enabled; ready for IRQ processing. */
		uint8_t configured : 1;
		/** @brief Device communication successful; used internally during probe. */
		uint8_t probed : 1;
		/** @brief Product ID verified to be BHY2_PRODUCT_ID. */
		uint8_t device_found : 1;
		/** @brief GPIO interrupt callbacks registered and armed. */
		uint8_t irq_ready : 1;
	} bits;
};

/**
 * @brief Driver singleton instance.
 *
 * Holds all device state, GPIO/SPI specs, BHY2 device handle, and cached
 * sensor data buffers. SPI operations serialize via sys_spi_lock/unlock().
 * Sensor data buffers (quat_data, lacc_data, etc.) are driver-owned; they are
 * referenced by p_param in posted events and remain valid until the next
 * event of the same type is posted.
 */
static struct bhi360_t {
	/** @brief SYS_SPI device specification (locked/unlocked by shared-bus manager). */
	struct sys_spi_dt_spec spi;
	/** @brief Chip-select GPIO (active high). */
	struct gpio_dt_spec cs_gpio;
	/** @brief Interrupt request GPIO (active low). */
	struct gpio_dt_spec irq_gpio;
	/** @brief Reset GPIO (active high). */
	struct gpio_dt_spec reset_gpio;
	/** @brief Unconfigured GPIO0 (ASDX pin); may be used by custom firmware. */
	struct gpio_dt_spec gpio0;
	/** @brief Unconfigured GPIO1 (ASCX pin); may be used by custom firmware. */
	struct gpio_dt_spec gpio1;
	/** @brief GPIO callback handle for IRQ line. */
	struct gpio_callback irq_cb;
	/** @brief BHY2 host library device handle and state. */
	struct bhy2_dev bhy2;
	/** @brief Lifecycle state bits. */
	union bhi360_state_t state;
	/** @brief Friendly name for logging ("BHI360"). */
	char name[32];
	/** @brief FIFO read buffer for BHY2 packet parsing (2 KiB). */
	uint8_t work_buffer[WORK_BUFFER_SIZE];
	/** @brief Cached quaternion data (posted via Quaternion event p_param). */
	struct bhi360_quat_data quat_data;
	/** @brief Cached linear acceleration data (posted via LinearAcceleration event p_param). */
	struct bhi360_lacc_data lacc_data;
	/** @brief Cached gyro data (posted via Gyro event p_param). */
	struct bhi360_gyro_data gyro_data;
	/** @brief Cached pedometer data (posted via Pedometer event p_param). */
	struct bhi360_pedometer_data pedometer_data;
	/** @brief Cached gesture data (posted via Gesture event p_param). */
	struct bhi360_gesture_data gesture_data;
	/** @brief Cached activity data (posted via Activity event p_param). */
	struct bhi360_activity_data activity_data;
} bhi360 = {
	.spi = SYS_SPI_DT_SPEC_GET(BHI360_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
	.cs_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, cs_gpios),
	.irq_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, int_gpios),
	.reset_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, reset_gpios),
	.gpio0 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio0_gpios),
	.gpio1 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio1_gpios),
	.name = "BHI360",
};

/** Assert (select) the BHI360 chip-select line. */
static inline void bhi360_cs_select(struct bhi360_t* imu) {
	gpio_pin_set_dt(&imu->cs_gpio, 1);
}

/** Deassert (release) the BHI360 chip-select line. */
static inline void bhi360_cs_deselect(struct bhi360_t* imu) {
	gpio_pin_set_dt(&imu->cs_gpio, 0);
}

/**
 * @brief Retrieve the GPIO specification for INT0.
 * @details Returns a pointer to the driver's cached GPIO spec for the unconfigured
 *          ASDX pin wiring. The application may configure this pin in custom firmware
 *          as a user-defined function output.
 * @return Pointer to the INT0 GPIO spec (driver-internal, do not modify).
 */
const struct gpio_dt_spec* bhi360_get_gpio0(void) {
	return &bhi360.gpio0;
}

/**
 * @brief Retrieve the GPIO specification for INT1.
 * @details Returns a pointer to the driver's cached GPIO spec for the unconfigured
 *          ASCX pin wiring. The application may configure this pin in custom firmware
 *          as a user-defined function output.
 * @return Pointer to the INT1 GPIO spec (driver-internal, do not modify).
 */
const struct gpio_dt_spec* bhi360_get_gpio1(void) {
	return &bhi360.gpio1;
}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref);
static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info* callback_info,
									  void* callback_ref);
static void parse_gyro(const struct bhy2_fifo_parse_data_info* callback_info, void* callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref);
static void parse_scalar_event(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref);
static void parse_step_counter(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref);
static void parse_activity(const struct bhy2_fifo_parse_data_info* callback_info,
						   void* callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev* dev);

/**
 * @brief Post a device-driver event to the central event queue.
 * @details Thread-safe wrapper that encodes the event, device ID, and optional
 *          pointer/value parameters. Called from FIFO parser callbacks to emit
 *          sensor and meta-events.
 * @param event Event type (bhi360_event_Quaternion, etc.).
 * @param v_param Value parameter (sensor ID, packed fields, or metadata).
 * @param p_param Optional pointer to driver-owned data buffer.
 */
static void bhi360_post_event(enum bhi360_event_type event, uint32_t v_param, void* p_param) {
	(void) device_driver_event_post(BHI360_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									(uintptr_t) p_param,
									K_NO_WAIT);
}

/**
 * @brief Post a device-driver event from ISR context.
 * @details ISR-safe variant that bypasses the queue's internal locking.
 *          Called from bhi360_irq_callback() to post the raw Irq event.
 * @param event Event type (typically bhi360_event_Irq).
 * @param v_param Value parameter (GPIO pin bits that triggered).
 */
static void bhi360_post_event_isr(enum bhi360_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(BHI360_DEVICE_DTS_ID, (uint32_t) event, v_param, 0U);
}

/**
 * @brief Enable all sensors in a static table.
 * @details Iterates through the sensor table, registers FIFO parse callbacks,
 *          and calls bhy2_set_virt_sensor_cfg() to enable each sensor at the
 *          specified sample rate and latency.
 * @param dev Driver instance pointer.
 * @param sensors Pointer to array of struct bhi360_sensor_enable entries.
 * @param count Number of entries in the array.
 * @pre Firmware has been uploaded and bhy2_update_virtual_sensor_list() called.
 */
static void bhi360_enable_sensor_table(struct bhi360_t* dev,
									   const struct bhi360_sensor_enable* sensors,
									   size_t count) {
	for (size_t i = 0; i < count; i++) {
		const struct bhi360_sensor_enable* sensor = &sensors[i];

		if (!bhy2_is_sensor_available(sensor->sensor_id, &dev->bhy2)) {
			LOG_WRN("%s: %s (id %u) is not available", dev->name, sensor->name, sensor->sensor_id);
			continue;
		}

		int8_t rslt =
			bhy2_register_fifo_parse_callback(sensor->sensor_id, sensor->parser, dev, &dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt != BHY2_OK) {
			continue;
		}

		rslt = bhy2_set_virt_sensor_cfg(sensor->sensor_id,
										sensor->sample_rate_hz,
										sensor->latency_ms,
										&dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt == BHY2_OK) {
			LOG_INF("%s: enabled %s (id %u) at %.2f Hz",
					dev->name,
					sensor->name,
					sensor->sensor_id,
					(double) sensor->sample_rate_hz);
		}
	}
}

/**
 * @brief Disable all sensors in a static table.
 * @details Clears the sample rate (disables output) and flushes pending data
 *          for each sensor in the table.
 * @param dev Driver instance pointer.
 * @param sensors Pointer to array of struct bhi360_sensor_enable entries.
 * @param count Number of entries in the array.
 */
static void bhi360_disable_sensor_table(struct bhi360_t* dev,
										const struct bhi360_sensor_enable* sensors,
										size_t count) {
	for (size_t i = 0; i < count; i++) {
		(void) bhy2_set_virt_sensor_cfg(sensors[i].sensor_id, 0.0f, 0, &dev->bhy2);
		(void) bhy2_flush_fifo(sensors[i].sensor_id, &dev->bhy2);
	}
}

/**
 * @brief Log BHY2 API errors with rich context.
 * @details If rslt indicates an error, logs the BHY2 error text and (if available)
 *          the interface error code. Clears the interface error flag after logging.
 * @param rslt BHY2 API return code.
 * @param dev BHY2 device handle for interface error extraction; may be NULL.
 */
static void print_api_error(int8_t rslt, struct bhy2_dev* dev) {
	if (rslt != BHY2_OK) {
		LOG_ERR("API error: %s (%d)", bhi360_api_get_error(rslt), rslt);
		if ((rslt == BHY2_E_IO) && (dev != NULL)) {
			LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
			dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
		}
	}
}

/**
 * @brief Upload the BHI360 firmware to RAM or flash.
 * @details Streams the firmware binary (included from bhi360/BHI360_Aux_BMM150.fw.h
 *          or BHI260AP-flash.fw.h) to the device in 256-byte chunks, padding to
 *          4-byte alignment as required by the BHY2 protocol.
 * @param dev BHY2 device handle.
 * @return BHY2 API return code; BHY2_OK on success.
 * @pre Device is in boot-loader-ready state (checked in bhi360_configure).
 */
static int8_t upload_firmware(struct bhy2_dev* dev) {
	uint32_t incr = 256; /* Max command packet size */
	uint32_t len = sizeof(bhy2_firmware_image);
	int8_t rslt = BHY2_OK;

	if ((incr % 4) != 0) /* Round off to higher 4 bytes */
	{
		incr = ((incr >> 2) + 1) << 2;
	}

	for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr) {
		if (incr > (len - i)) /* If last payload */
		{
			incr = len - i;
			if ((incr % 4) != 0) /* Round off to higher 4 bytes */
			{
				incr = ((incr >> 2) + 1) << 2;
			}
		}

#ifdef UPLOAD_FIRMWARE_TO_FLASH
		rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
		rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif

		LOG_INF("%.2f%% complete", (double) (i + incr) / (double) len * 100.0);
	}

	return rslt;
}

/**
 * @brief GPIO IRQ callback for the BHI360 interrupt line.
 * @details Fires in ISR context when a GPIO edge on irq_gpio transitions to active.
 *          Posts bhi360_event_Irq to the event queue; the event consumer thread
 *          then calls bhi360_process_irq() to drain the FIFO.
 * @param dev GPIO device (unused).
 * @param cb GPIO callback info (unused).
 * @param pins Bitmask of GPIO pins that fired; passed as v_param in the event.
 */
static void bhi360_irq_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	bhi360_post_event_isr(bhi360_event_Irq, pins);
}

/**
 * @brief Initialize the BHI360 IRQ GPIO and callback.
 * @details Configures the GPIO as input, enables interrupt-to-active detection,
 *          and registers the callback handler. Safe to call multiple times;
 *          subsequent calls return early if already initialized.
 * @param imu Driver instance pointer.
 * @return 0 on success, -ENODEV if GPIO not ready, or negative errno on failure.
 */
static int bhi360_irq_init(struct bhi360_t* imu) {
	if (imu->state.bits.irq_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&imu->irq_gpio)) {
		LOG_ERR("BHI360 IRQ GPIO not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&imu->irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&imu->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ interrupt (%d)", ret);
		return ret;
	}

	gpio_init_callback(&imu->irq_cb, bhi360_irq_callback, BIT(imu->irq_gpio.pin));
	ret = gpio_add_callback(imu->irq_gpio.port, &imu->irq_cb);
	if (ret != 0) {
		LOG_ERR("Failed to add BHI360 IRQ callback (%d)", ret);
		return ret;
	}

	imu->state.bits.irq_ready = 1U;
	return 0;
}

/**
 * @brief BHY2 SPI read callback.
 * @details Acquires the shared SPI bus, drives chip-select low, reads the register
 *          address with the read-bit set (0x80 | reg_addr), then reads the requested
 *          data. Releases the bus on return. Validates input and state.
 * @param reg_addr Register address to read.
 * @param reg_data Output buffer for read data.
 * @param length Number of bytes to read.
 * @param intf_ptr Opaque context (cast to struct bhi360_t*).
 * @return BHY2_INTF_RET_SUCCESS (0) on success, BHY2_E_IO on any failure.
 */
static int8_t bhi360_spi_read(uint8_t reg_addr,
							  uint8_t* reg_data,
							  uint32_t length,
							  void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || (reg_data == NULL) || (length == 0U)) {
		return BHY2_E_IO;
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = (uint8_t) (reg_addr | 0x80);

	const struct spi_buf tx_buf = {
		.buf = &tx_cmd,
		.len = sizeof(tx_cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	const struct spi_buf rx_buf = {
		.buf = reg_data,
		.len = length,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	if (ret == 0) {
		ret = sys_spi_read(&imu->spi, &rx);
	}
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO;
	}

	return BHY2_INTF_RET_SUCCESS;
}

/**
 * @brief BHY2 SPI write callback.
 * @details Acquires the shared SPI bus, drives chip-select low, writes the register
 *          address followed by the data bytes. Releases the bus on return.
 *          Validates input and state.
 * @param reg_addr Register address to write.
 * @param reg_data Pointer to data bytes to write.
 * @param length Number of bytes to write.
 * @param intf_ptr Opaque context (cast to struct bhi360_t*).
 * @return BHY2_INTF_RET_SUCCESS (0) on success, BHY2_E_IO on any failure.
 */
static int8_t bhi360_spi_write(uint8_t reg_addr,
							   const uint8_t* reg_data,
							   uint32_t length,
							   void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || ((reg_data == NULL) && (length > 0U))) {
		return BHY2_E_IO;
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = reg_addr;
	struct spi_buf tx_bufs[2] = {
		{
			.buf = &tx_cmd,
			.len = sizeof(tx_cmd),
		},
		{
			.buf = (void*) reg_data,
			.len = length,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = (length > 0U) ? 2U : 1U,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO;
	}
	return BHY2_INTF_RET_SUCCESS;
}

/**
 * @brief BHY2 delay callback.
 * @details Called by the BHY2 library when timed waits are needed (e.g., during
 *          chip initialization or register polling). Uses k_usleep() for
 *          microsecond-precision delays.
 * @param period_us Delay duration in microseconds.
 * @param intf_ptr Opaque context (unused by implementation).
 */
// Delay function for BHY2 driver
static void bhi360_delay_us(uint32_t period_us, void* intf_ptr) {
	k_usleep(period_us);
}
	{BHY2_SENSOR_ID_RV, "Rotation vector", 100.0f, 0, parse_quaternion},
	{BHY2_SENSOR_ID_LACC, "Linear acceleration", 100.0f, 0, parse_linear_acceleration},
	{BHY2_SENSOR_ID_GYRO, "Gyroscope", 100.0f, 0, parse_gyro},
	{BHY2_SENSOR_ID_STC, "Step counter", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_WU, "Step counter wake-up", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_LP, "Step counter low-power", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_LP_WU, "Step counter low-power wake-up", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STD, "Step detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_WU, "Step detector wake-up", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_LP, "Step detector low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_LP_WU, "Step detector low-power wake-up", 1.0f, 0, parse_scalar_event},
};

static const struct bhi360_sensor_enable bhi360_gesture_sensors[] = {
	{BHY2_SENSOR_ID_WAKE_GESTURE, "Wake gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_GLANCE_GESTURE, "Glance gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_PICKUP_GESTURE, "Pickup gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_WRIST_TILT_GESTURE, "Wrist tilt gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_TILT_DETECTOR, "Tilt detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STATIONARY_DET, "Stationary detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_MOTION_DET, "Motion detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG, "Significant motion", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG_LP, "Significant motion low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG_LP_WU, "Significant motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_ANY_MOTION_LP, "Any motion low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_ANY_MOTION_LP_WU, "Any motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHI3_SENSOR_ID_NO_MOTION_LP_WU, "No motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU,
	 "Wrist gesture detect low-power wake-up",
	 1.0f,
	 0,
	 parse_scalar_event},
	{BHI3_SENSOR_ID_WRIST_WEAR_LP_WU, "Wrist wear low-power wake-up", 1.0f, 0, parse_scalar_event},
};

static const struct bhi360_sensor_enable bhi360_activity_sensors[] = {
	{BHY2_SENSOR_ID_AR, "Activity recognition", 1.0f, 0, parse_activity},
	{BHI3_SENSOR_ID_AR_WEAR_WU, "Wear activity recognition wake-up", 1.0f, 0, parse_activity},
};

static void bhi360_post_event(enum bhi360_event_type event, uint32_t v_param, void* p_param) {
	(void) device_driver_event_post(BHI360_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									(uintptr_t) p_param,
									K_NO_WAIT);
}

static void bhi360_post_event_isr(enum bhi360_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(BHI360_DEVICE_DTS_ID, (uint32_t) event, v_param, 0U);
}

static void bhi360_enable_sensor_table(struct bhi360_t* dev,
									   const struct bhi360_sensor_enable* sensors,
									   size_t count) {
	for (size_t i = 0; i < count; i++) {
		const struct bhi360_sensor_enable* sensor = &sensors[i];

		if (!bhy2_is_sensor_available(sensor->sensor_id, &dev->bhy2)) {
			LOG_WRN("%s: %s (id %u) is not available", dev->name, sensor->name, sensor->sensor_id);
			continue;
		}

		int8_t rslt =
			bhy2_register_fifo_parse_callback(sensor->sensor_id, sensor->parser, dev, &dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt != BHY2_OK) {
			continue;
		}

		rslt = bhy2_set_virt_sensor_cfg(sensor->sensor_id,
										sensor->sample_rate_hz,
										sensor->latency_ms,
										&dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt == BHY2_OK) {
			LOG_INF("%s: enabled %s (id %u) at %.2f Hz",
					dev->name,
					sensor->name,
					sensor->sensor_id,
					(double) sensor->sample_rate_hz);
		}
	}
}

static void bhi360_disable_sensor_table(struct bhi360_t* dev,
										const struct bhi360_sensor_enable* sensors,
										size_t count) {
	for (size_t i = 0; i < count; i++) {
		(void) bhy2_set_virt_sensor_cfg(sensors[i].sensor_id, 0.0f, 0, &dev->bhy2);
		(void) bhy2_flush_fifo(sensors[i].sensor_id, &dev->bhy2);
	}
}

static void print_api_error(int8_t rslt, struct bhy2_dev* dev) {
	if (rslt != BHY2_OK) {
		LOG_ERR("API error: %s (%d)", bhi360_api_get_error(rslt), rslt);
		if ((rslt == BHY2_E_IO) && (dev != NULL)) {
			LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
			dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
		}
	}
}

static int8_t upload_firmware(struct bhy2_dev* dev) {
	uint32_t incr = 256; /* Max command packet size */
	uint32_t len = sizeof(bhy2_firmware_image);
	int8_t rslt = BHY2_OK;

	if ((incr % 4) != 0) /* Round off to higher 4 bytes */
	{
		incr = ((incr >> 2) + 1) << 2;
	}

	for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr) {
		if (incr > (len - i)) /* If last payload */
		{
			incr = len - i;
			if ((incr % 4) != 0) /* Round off to higher 4 bytes */
			{
				incr = ((incr >> 2) + 1) << 2;
			}
		}

#ifdef UPLOAD_FIRMWARE_TO_FLASH
		rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
		rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif

		LOG_INF("%.2f%% complete", (double) (i + incr) / (double) len * 100.0);
	}

	return rslt;
}

static void bhi360_irq_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	bhi360_post_event_isr(bhi360_event_Irq, pins);
}

static int bhi360_irq_init(struct bhi360_t* imu) {
	if (imu->state.bits.irq_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&imu->irq_gpio)) {
		LOG_ERR("BHI360 IRQ GPIO not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&imu->irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&imu->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ interrupt (%d)", ret);
		return ret;
	}

	gpio_init_callback(&imu->irq_cb, bhi360_irq_callback, BIT(imu->irq_gpio.pin));
	ret = gpio_add_callback(imu->irq_gpio.port, &imu->irq_cb);
	if (ret != 0) {
		LOG_ERR("Failed to add BHI360 IRQ callback (%d)", ret);
		return ret;
	}

	imu->state.bits.irq_ready = 1U;
	return 0;
}

static int8_t bhi360_spi_read(uint8_t reg_addr,
							  uint8_t* reg_data,
							  uint32_t length,
							  void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || (reg_data == NULL) || (length == 0U)) {
		return BHY2_E_IO; /* or your preferred error mapping */
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = (uint8_t) (reg_addr | 0x80);

	const struct spi_buf tx_buf = {
		.buf = &tx_cmd,
		.len = sizeof(tx_cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	const struct spi_buf rx_buf = {
		.buf = reg_data,
		.len = length,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	if (ret == 0) {
		ret = sys_spi_read(&imu->spi, &rx);
	}
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO; /* or map ret -> BHY2 error codes */
	}

	return BHY2_INTF_RET_SUCCESS;
}

static int8_t bhi360_spi_write(uint8_t reg_addr,
							   const uint8_t* reg_data,
							   uint32_t length,
							   void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || ((reg_data == NULL) && (length > 0U))) {
		return BHY2_E_IO; /* or your preferred mapping */
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = reg_addr;
	struct spi_buf tx_bufs[2] = {
		{
			.buf = &tx_cmd,
			.len = sizeof(tx_cmd),
		},
		{
			.buf = (void*) reg_data,
			.len = length,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = (length > 0U) ? 2U : 1U,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO;
	}
	return BHY2_INTF_RET_SUCCESS;
}

// Delay function for BHY2 driver
static void bhi360_delay_us(uint32_t period_us, void* intf_ptr) {
	k_usleep(period_us);
}

bool bhi360_init(void) {
	struct bhi360_t* imu = &bhi360;
	int ret;

	if (imu->state.bits.initialized) {
		return true;
	}

	LOG_INF("%s: Starting initialization", imu->name);

	if (!sys_spi_is_ready(&imu->spi)) {
		LOG_ERR("%s: SYS_SPI device not ready", imu->name);
		return false;
	}

	if (!gpio_is_ready_dt(&imu->cs_gpio)) {
		LOG_ERR("%s: chip-select GPIO not ready", imu->name);
		return false;
	}

	if (!gpio_is_ready_dt(&imu->reset_gpio)) {
		LOG_ERR("%s: reset GPIO not ready", imu->name);
		return false;
	}

	ret = gpio_pin_configure_dt(&imu->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: failed to configure chip-select GPIO (%d)", imu->name, ret);
		return false;
	}

	ret = gpio_pin_configure_dt(&imu->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: failed to configure reset GPIO (%d)", imu->name, ret);
		return false;
	}

	ret = bhi360_irq_init(imu);
	if (ret != 0) {
		return false;
	}

	imu->state.bits.initialized = 1U;
	return true;
}

bool bhi360_configure(void) {
	struct bhi360_t* imu = &bhi360;
	int8_t rslt;
	uint8_t product_id = 0;
	uint16_t version = 0;
	uint8_t hintr_ctrl, hif_ctrl, boot_status;

	if (imu->state.bits.configured) {
		return true;
	}

	if (!bhi360_init()) {
		return false;
	}

	LOG_INF("%s: Starting configuration", imu->name);

	k_sleep(K_USEC(1));

	// Initialize BHY2 device
	rslt = bhy2_init(BHY2_SPI_INTERFACE,
					 bhi360_spi_read,
					 bhi360_spi_write,
					 bhi360_delay_us,
					 BHY2_RD_WR_LEN,
					 imu, // Pass IMU struct as interface pointer
					 &imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Initialization failed", imu->name);
		return false;
	}

	// Soft reset
	rslt = bhy2_soft_reset(&imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Soft reset failed with code %d", imu->name, rslt);
		return false;
	}

	// Product ID check with retries
	bool id_read_success = false;
	imu->state.bits.probed = 1U;
	for (int retry = 0; retry < 20; retry++) {
		rslt = bhy2_get_product_id(&product_id, &imu->bhy2);
		if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID) {
			LOG_INF("%s: Product ID verified on attempt %d", imu->name, retry + 1);
			id_read_success = true;
			imu->state.bits.device_found = 1U;
			break;
		}
		k_msleep(10);
	}

	if (!id_read_success) {
		LOG_ERR("%s: Failed to verify product ID", imu->name);
		return false;
	}

	/*
	 * Enable all BHI360 interrupt sources. The board wiring uses active-low
	 * HIRQ (DTS: GPIO_ACTIVE_LOW), so configure the Bosch host interrupt as
	 * active-low level/open-drain. No DISABLE_* bits are set: FIFO, status,
	 * debug, reset and fault sources can all assert HIRQ.
	 */
	hintr_ctrl = BHY2_ICTL_ACTIVE_LOW | BHY2_ICTL_OPEN_DRAIN;
	rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	hif_ctrl = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	// Check boot status and upload firmware
	rslt = bhy2_get_boot_status(&boot_status, &imu->bhy2);
	if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
		LOG_INF("%s: Uploading firmware", imu->name);
		rslt = upload_firmware(&imu->bhy2);
		if (rslt != BHY2_OK) {
			LOG_ERR("%s: Firmware upload failed", imu->name);
			return false;
		}

		// Boot from RAM and verify
		rslt = bhy2_boot_from_ram(&imu->bhy2);
		rslt = bhy2_get_kernel_version(&version, &imu->bhy2);
		if (rslt != BHY2_OK || version == 0) {
			LOG_ERR("%s: Boot failed", imu->name);
			return false;
		}
		LOG_INF("%s: Boot successful, kernel version %u", imu->name, version);

		// Update virtual sensor list first
		LOG_INF("%s: Updating virtual sensor list", imu->name);
		rslt = bhy2_update_virtual_sensor_list(&imu->bhy2);
		print_api_error(rslt, &imu->bhy2);

		LOG_INF("%s: Registering meta-event callbacks", imu->name);
		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT,
												 parse_meta_event,
												 imu,
												 &imu->bhy2);
		print_api_error(rslt, &imu->bhy2);
		if (rslt != BHY2_OK) {
			return false;
		}

		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU,
												 parse_meta_event,
												 imu,
												 &imu->bhy2);
		print_api_error(rslt, &imu->bhy2);
		if (rslt != BHY2_OK) {
			return false;
		}

		bhi360_enable_sensor_table(imu, bhi360_base_sensors, ARRAY_SIZE(bhi360_base_sensors));
		bhi360_enable_sensor_table(imu, bhi360_gesture_sensors, ARRAY_SIZE(bhi360_gesture_sensors));
		bhi360_enable_sensor_table(imu,
								   bhi360_activity_sensors,
								   ARRAY_SIZE(bhi360_activity_sensors));

		imu->state.bits.configured = 1U;
		LOG_INF("%s: Configuration complete", imu->name);
		return true;
	}

	LOG_ERR("%s: Host interface not ready", imu->name);
	return false;
}

int bhi360_process_irq(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}

	uint8_t int_status = 0;
	int8_t rslt = bhy2_get_interrupt_status(&int_status, &bhi360.bhy2);
	print_api_error(rslt, &bhi360.bhy2);
	if (rslt != BHY2_OK) {
		return -EIO;
	}

	ARG_UNUSED(int_status);

	rslt = bhy2_get_and_process_fifo(bhi360.work_buffer, sizeof(bhi360.work_buffer), &bhi360.bhy2);
	print_api_error(rslt, &bhi360.bhy2);
	if (rslt != BHY2_OK) {
		return -EIO;
	}

	return 0;
}

void bhi360_stop(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return;
	}

	/**
	 * @brief Disable all sensors and put the BHI360 into standby.
	 * @details Disables all enabled sensor tables (base, gesture, activity), issues a
	 *          soft reset to clear firmware state, and clears the configured flag. Called
	 *          during system shutdown or when the sensor is not needed. Safe to call
	 *          multiple times; subsequent calls return early if not configured.
	 */
	void bhi360_stop(void) {
		if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
			return;
		}

	bhi360_disable_sensor_table(&bhi360, bhi360_base_sensors, ARRAY_SIZE(bhi360_base_sensors));
	bhi360_disable_sensor_table(&bhi360,
								bhi360_gesture_sensors,
								ARRAY_SIZE(bhi360_gesture_sensors));
	bhi360_disable_sensor_table(&bhi360,
								bhi360_activity_sensors,
								ARRAY_SIZE(bhi360_activity_sensors));
	(void) bhy2_soft_reset(&bhi360.bhy2);
	bhi360.state.bits.configured = 0U;
}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_quaternion data;
	if (callback_info->data_size != 11) { // Check for valid payload size
		LOG_ERR("Invalid data size: %d", callback_info->data_size);
		return;
	}

	bhy2_parse_quaternion(callback_info->data_ptr, &data);

	dev->quat_data.x = data.x;
	dev->quat_data.y = data.y;
	dev->quat_data.z = data.z;
	dev->quat_data.w = data.w;
	dev->quat_data.accuracy = data.accuracy;

	bhi360_post_event(bhi360_event_Quaternion, callback_info->sensor_id, &dev->quat_data);
}

static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info* callback_info,
									  void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;
	bhy2_parse_xyz(callback_info->data_ptr, &data);

	dev->lacc_data.x = data.x;
	dev->lacc_data.y = data.y;
	dev->lacc_data.z = data.z;

	bhi360_post_event(bhi360_event_LinearAcceleration,
					  callback_info->sensor_id,
					  &dev->lacc_data);
}

static void parse_gyro(const struct bhy2_fifo_parse_data_info* callback_info, void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;
	bhy2_parse_xyz(callback_info->data_ptr, &data);

	dev->gyro_data.x = data.x;
	dev->gyro_data.y = data.y;
	dev->gyro_data.z = data.z;

	bhi360_post_event(bhi360_event_Gyro, callback_info->sensor_id, &dev->gyro_data);
}

static void parse_scalar_event(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	uint8_t value = (callback_info->data_size > 0) ? callback_info->data_ptr[0] : 1U;
	uint32_t event_value = ((uint32_t) callback_info->sensor_id << 8) | value;

	if ((callback_info->sensor_id == BHY2_SENSOR_ID_STD) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_WU) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP_WU)) {
		dev->pedometer_data.sensor_id = callback_info->sensor_id;
		dev->pedometer_data.step_detected = true;
		bhi360_post_event(bhi360_event_Pedometer, callback_info->sensor_id, &dev->pedometer_data);
		return;
	}

	dev->gesture_data.sensor_id = callback_info->sensor_id;
	dev->gesture_data.value = value;
	bhi360_post_event(bhi360_event_Gesture, event_value, &dev->gesture_data);
	LOG_INF("Gesture/event sensor id %u value 0x%02x", callback_info->sensor_id, value);
}

static void parse_step_counter(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	uint32_t count = 0;

	if (callback_info->data_size >= 4) {
		count = BHY2_LE2U32(callback_info->data_ptr);
	} else if (callback_info->data_size >= 2) {
		count = BHY2_LE2U16(callback_info->data_ptr);
	} else if (callback_info->data_size == 1) {
		count = callback_info->data_ptr[0];
	}

	dev->pedometer_data.sensor_id = callback_info->sensor_id;
	dev->pedometer_data.step_count = count;
	dev->pedometer_data.step_detected = false;
	bhi360_post_event(bhi360_event_Pedometer, callback_info->sensor_id, &dev->pedometer_data);
	LOG_INF("Step counter sensor id %u count %u", callback_info->sensor_id, count);
}

static void parse_activity(const struct bhy2_fifo_parse_data_info* callback_info,
						   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	if (callback_info->data_size < 2) {
		LOG_WRN("Activity event sensor id %u has invalid size %u",
				callback_info->sensor_id,
				callback_info->data_size);
		return;
	}

	uint16_t activity = BHY2_LE2U16(callback_info->data_ptr);
	dev->activity_data.sensor_id = callback_info->sensor_id;
	dev->activity_data.activity = activity;
	bhi360_post_event(bhi360_event_Activity,
					  ((uint32_t) callback_info->sensor_id << 16) | activity,
					  &dev->activity_data);

	if (activity & BHY2_STILL_ACTIVITY_ENDED) {
		LOG_INF("Activity: still ended");
	}
	if (activity & BHY2_WALKING_ACTIVITY_ENDED) {
		LOG_INF("Activity: walking ended");
	}
	if (activity & BHY2_RUNNING_ACTIVITY_ENDED) {
		LOG_INF("Activity: running ended");
	}
	if (activity & BHY2_ON_BICYCLE_ACTIVITY_ENDED) {
		LOG_INF("Activity: bicycle ended");
	}
	if (activity & BHY2_IN_VEHICLE_ACTIVITY_ENDED) {
		LOG_INF("Activity: vehicle ended");
	}
	if (activity & BHY2_TILTING_ACTIVITY_ENDED) {
		LOG_INF("Activity: tilting ended");
	}
	if (activity & BHY2_STILL_ACTIVITY_STARTED) {
		LOG_INF("Activity: still started");
	}
	if (activity & BHY2_WALKING_ACTIVITY_STARTED) {
		LOG_INF("Activity: walking started");
	}
	if (activity & BHY2_RUNNING_ACTIVITY_STARTED) {
		LOG_INF("Activity: running started");
	}
	if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED) {
		LOG_INF("Activity: bicycle started");
	}
	if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED) {
		LOG_INF("Activity: vehicle started");
	}
	if (activity & BHY2_TILTING_ACTIVITY_STARTED) {
		LOG_INF("Activity: tilting started");
	}
}

static void bhi360_post_meta_event(uint8_t type, uint8_t byte1, uint8_t byte2) {
	bhi360_post_event(bhi360_event_MetaEvent,
					  ((uint32_t) type << 16) | ((uint32_t) byte1 << 8) | byte2,
					  NULL);
}

static void parse_meta_flush_complete(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Flush complete for sensor id %u", event_text, byte1);
}

static void parse_meta_sample_rate_changed(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Sample rate changed for sensor id %u", event_text, byte1);
}

static void parse_meta_power_mode_changed(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Power mode changed for sensor id %u", event_text, byte1);
}

static void parse_meta_algorithm_events(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s Algorithm event", event_text);
}

static void parse_meta_sensor_status(const char* event_text, uint8_t byte1, uint8_t byte2) {
	LOG_INF("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
}

static void parse_meta_bsx_do_steps_main(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s BSX event (do steps main)", event_text);
}

static void parse_meta_bsx_do_steps_calib(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s BSX event (do steps calib)", event_text);
}

static void parse_meta_bsx_get_output_signal(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s BSX event (get output signal)", event_text);
}

static void parse_meta_sensor_error(const char* event_text, uint8_t byte1, uint8_t byte2) {
	LOG_INF("%s %s (id %u) reported error: %s (0x%02X)",
			event_text,
			bhi360_api_get_sensor_name(byte1),
			byte1,
			bhi360_api_get_sensor_error_text(byte2),
			byte2);
}

static void parse_meta_fifo_overflow(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s FIFO overflow", event_text);
}

static void parse_meta_dynamic_range_changed(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Dynamic range changed for sensor id %u", event_text, byte1);
}

static void parse_meta_fifo_watermark(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s FIFO watermark reached", event_text);
}

static void parse_meta_initialized(const char* event_text, uint8_t byte1, uint8_t byte2) {
	LOG_INF("%s Firmware initialized. Firmware version %u",
			event_text,
			((uint16_t) byte2 << 8) | byte1);
}

static void parse_meta_transfer_cause(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Transfer cause for sensor id %u", event_text, byte1);
}

static void parse_meta_sensor_framework(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte2);
	LOG_INF("%s Sensor framework event for sensor id %u", event_text, byte1);
}

static void parse_meta_reset(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
	LOG_INF("%s Reset event", event_text);
}

static void parse_meta_spacer(const char* event_text, uint8_t byte1, uint8_t byte2) {
	ARG_UNUSED(event_text);
	ARG_UNUSED(byte1);
	ARG_UNUSED(byte2);
}

static void parse_meta_unknown(const char* event_text,
							   uint8_t meta_event_type,
							   uint8_t byte1,
							   uint8_t byte2) {
	LOG_INF("%s Unknown meta event with id %u, byte1 %u, byte2 %u",
			event_text,
			meta_event_type,
			byte1,
			byte2);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref) {
	(void) callback_ref;
	if (callback_info->data_size < 3) {
		LOG_WRN("Invalid meta event size %u", callback_info->data_size);
		return;
	}

	uint8_t meta_event_type = callback_info->data_ptr[0];
	uint8_t byte1 = callback_info->data_ptr[1];
	uint8_t byte2 = callback_info->data_ptr[2];
	const char* event_text;

	if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
		event_text = "[META EVENT]";
	} else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
		event_text = "[META EVENT WAKE UP]";
	} else {
		return;
	}

	bhi360_post_meta_event(meta_event_type, byte1, byte2);

	switch (meta_event_type) {
	case BHY2_META_EVENT_FLUSH_COMPLETE:
		parse_meta_flush_complete(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
		parse_meta_sample_rate_changed(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_POWER_MODE_CHANGED:
		parse_meta_power_mode_changed(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_ALGORITHM_EVENTS:
		parse_meta_algorithm_events(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_SENSOR_STATUS:
		parse_meta_sensor_status(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
		parse_meta_bsx_do_steps_main(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
		parse_meta_bsx_do_steps_calib(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
		parse_meta_bsx_get_output_signal(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_SENSOR_ERROR:
		parse_meta_sensor_error(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_FIFO_OVERFLOW:
		parse_meta_fifo_overflow(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
		parse_meta_dynamic_range_changed(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_FIFO_WATERMARK:
		parse_meta_fifo_watermark(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_INITIALIZED:
		parse_meta_initialized(event_text, byte1, byte2);
		break;
	case BHY2_META_TRANSFER_CAUSE:
		parse_meta_transfer_cause(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_SENSOR_FRAMEWORK:
		parse_meta_sensor_framework(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_RESET:
		parse_meta_reset(event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_SPACER:
		parse_meta_spacer(event_text, byte1, byte2);
		break;
	default:
		parse_meta_unknown(event_text, meta_event_type, byte1, byte2);
		break;
	}
}
