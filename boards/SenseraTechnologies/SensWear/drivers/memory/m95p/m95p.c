
/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file m95p.c
 * @brief SensWear M95P32 page EEPROM driver implementation.
 * @details Implements the board-level singleton declared by @ref senswear_m95p.
 *          Public operations own the shared system SPI bus for their complete
 *          command sequence. Private transfer helpers require that ownership
 *          to have already been acquired and drive chip select explicitly.
 */

#include "m95p.h"
#include "m95p_organization.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "sys_spi.h"

LOG_MODULE_REGISTER(m95p, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Devicetree node identifier for the board's M95P32 instance.
 * @details Used to construct the shared-SPI specification and software-managed
 *          chip-select GPIO specification stored in the singleton context.
 */
#define M95P_NODE DT_NODELABEL(m95p)
BUILD_ASSERT(DT_NODE_HAS_PROP(M95P_NODE, cs_gpios), "M95P is missing its chip-select GPIO");

/**
 * @brief Filesystem-facing sector size baseline.
 * @details Defaults to one physical 4-Kbyte sector. Decouples the size the
 *          filesystem layer treats as a sector from the device's physical
 *          sector geometry so the two can be tuned independently.
 */
#define FS_SUPPORTED_SECTOR_SIZE (M95P_SECTOR_SIZE)

/*
 * M95P32 physical geometry aliases.
 *
 * The device contains 4 MiB arranged as 8192 pages of 512 bytes. Eight pages
 * form a physical 4-Kbyte sector, and 16 physical sectors form a 64-Kbyte
 * protection/erase block.
 */
/**
 * @brief Total physical memory-array capacity in bytes.
 * @details Mirrors the part's full 4-MiB array (8192 pages of 512 bytes) and is
 *          the basis from which the whole-array page and sector totals derive.
 */
#define SYS_MEMORY_SIZE (M95P_SIZE)
/**
 * @brief Smallest physical page-program and page-erase unit in bytes.
 * @details One 512-byte page is the finest granularity the M95P32 can program
 *          or erase, so it also defines the driver's logical sector size.
 */
#define SYS_MEMORY_PAGE_SIZE (M95P_PAGE_SIZE)
/**
 * @brief Physical sector-erase unit in bytes.
 * @details Eight consecutive 512-byte pages form one physical 4-Kbyte sector.
 */
#define SYS_MEMORY_SECTOR_SIZE (M95P_SECTOR_SIZE)
/**
 * @brief Physical block-erase and block-protection unit in bytes.
 * @details Sixteen 4-Kbyte sectors form one 64-Kbyte block, which is also the
 *          granularity at which the array protection scheme operates.
 */
#define SYS_MEMORY_BLOCK_SIZE (M95P_BLOCK_SIZE)
/**
 * @brief Number of physical 4-Kbyte sectors in the complete array.
 * @details Counts every sector before any golden-section reservation is removed.
 */
#define SYS_MEMORY_TOTAL_SECTOR_COUNT (M95P_SECTOR_COUNT)
/**
 * @brief Number of 512-byte pages in the complete array.
 * @details Derived as the total array capacity divided by the page size.
 */
#define SYS_MEMORY_PAGE_COUNT (SYS_MEMORY_SIZE / SYS_MEMORY_PAGE_SIZE)
/**
 * @brief Number of 512-byte pages per physical 4-Kbyte sector.
 * @details Evaluates to eight and converts between page and sector indices.
 */
#define SYS_MEMORY_PAGES_PER_SECTOR (SYS_MEMORY_SECTOR_SIZE / SYS_MEMORY_PAGE_SIZE)
/**
 * @brief Number of physical 4-Kbyte sectors per 64-Kbyte block.
 * @details Evaluates to sixteen and converts between sector and block indices.
 */
#define SYS_MEMORY_SECTORS_PER_BLOCK (SYS_MEMORY_BLOCK_SIZE / SYS_MEMORY_SECTOR_SIZE)
/**
 * @brief Maximum number of blocks addressable by the protection scheme.
 * @details Upper bound enforced when reserving or protecting golden-section blocks.
 */
#define SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT M95P_MAX_PROTECTION_BLOCK_COUNT

/*
 * Driver logical-sector model.
 *
 * The public erase/write "sector" API operates on one 512-byte page because
 * M95P32 supports page erase and page write. The filesystem-facing sector size
 * remains separate and may span one or more driver-logical sectors.
 */
/**
 * @brief Filesystem-facing logical sector size in bytes.
 * @details Re-exports FS_SUPPORTED_SECTOR_SIZE under the SYS_MEMORY namespace
 *          shared by the rest of the geometry model.
 */
#define SYS_MEMORY_SUPPORTED_SECTOR_SIZE FS_SUPPORTED_SECTOR_SIZE
/**
 * @brief Number of filesystem sectors contained in one physical 4-Kbyte sector.
 * @details Ratio of the physical sector size to the filesystem-facing sector
 *          size; equals one when both sizes are identical.
 */
#define SYS_MEMORY_SECTOR_USAGE_RATIO (SYS_MEMORY_SECTOR_SIZE / SYS_MEMORY_SUPPORTED_SECTOR_SIZE)

/**
 * @brief Number of 64-Kbyte blocks reserved at the top of the array.
 * @details Defaults to zero. A nonzero value removes whole top-of-array blocks
 *          from the normal data area and makes them available through the
 *          golden-section API.
 */
#ifndef SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT
#define SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT (0)
#endif

#if SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT > 0
#if SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT > SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT
#error "Defined GOLDEN SECTION block count is larger than what can be protected"
#endif
#endif

/**
 * @brief Golden-section capacity in bytes.
 * @details Total size of the reserved top-of-array region, computed as the
 *          reserved block count times the 64-Kbyte block size.
 */
#define SYS_MEMORY_GOLDEN_SECTION_SIZE \
	(SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE)

/**
 * @brief Number of physical 4-Kbyte sectors reserved for the golden section.
 * @details Golden-section capacity expressed in whole physical sectors.
 */
#define SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT \
	(SYS_MEMORY_GOLDEN_SECTION_SIZE / SYS_MEMORY_SECTOR_SIZE)
#if SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT >= SYS_MEMORY_TOTAL_SECTOR_COUNT
#error "GOLDEN section configuration requires larger memory"
#endif

/**
 * @brief First physical 4-Kbyte sector reserved for the golden section.
 * @details Index of the lowest reserved sector, measured from the top of the
 *          array downward.
 */
#define SYS_MEMORY_GOLDEN_SECTION_SECTOR_START \
	(SYS_MEMORY_TOTAL_SECTOR_COUNT - SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT)

/**
 * @brief First 64-Kbyte block reserved for the golden section.
 * @details Block index that contains the first reserved sector.
 */
#define SYS_MEMORY_GOLDEN_SECTION_BLOCK_START \
	(SYS_MEMORY_GOLDEN_SECTION_SECTOR_START / SYS_MEMORY_SECTORS_PER_BLOCK)

/**
 * @brief Number of 512-byte pages reserved for the golden section.
 * @details Reserved sector count converted to pages.
 */
#define SYS_MEMORY_GOLDEN_SECTION_PAGE_COUNT \
	(SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT * SYS_MEMORY_PAGES_PER_SECTOR)
/**
 * @brief First 512-byte page reserved for the golden section.
 * @details Page index of the lowest reserved page.
 */
#define SYS_MEMORY_GOLDEN_SECTION_PAGE_START \
	(SYS_MEMORY_GOLDEN_SECTION_SECTOR_START * SYS_MEMORY_PAGES_PER_SECTOR)

/**
 * @brief First byte address reserved for the golden section.
 * @details Base byte offset added to golden-section-relative addresses to reach
 *          the physical array.
 */
#define SYS_MEMORY_GOLDEN_SECTION_ADDRESS_START \
	(SYS_MEMORY_GOLDEN_SECTION_PAGE_START * SYS_MEMORY_PAGE_SIZE)

/**
 * @brief Number of physical 4-Kbyte sectors remaining in the normal data area.
 * @details Total sector count minus the sectors reserved for the golden section.
 */
#define SYS_MEMORY_SECTOR_COUNT \
	(SYS_MEMORY_TOTAL_SECTOR_COUNT - SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT)

/**
 * @brief Internal singleton M95P driver context.
 * @details Collects all devicetree-derived hardware resources and cached
 *          runtime state. The SPI specification is also the ownership token
 *          used by the shared system SPI wrapper.
 */
static struct m95p_t {
	/** Shared-SPI connection and recursive ownership token from devicetree. */
	struct sys_spi_dt_spec device;
	/** Software-managed active-low chip-select GPIO from devicetree. */
	struct gpio_dt_spec cs_gpio;
	/** True after reset, JEDEC identification, and initial cleanup succeed. */
	bool initialized;
	/** Cached status-register image reserved for future state reporting. */
	union m95p_status_register_t status_register;
	/** Cached configuration/safety image reserved for future state reporting. */
	struct m95p_configuration_safety_registers_t config_safety_registers;
	/** Cached volatile-register image reserved for future state reporting. */
	union m95p_volatile_register_t volatile_register;
	/** JEDEC identification bytes captured and validated during initialization. */
	union m95p_jedec_id_t jedec_id;
} m95p = {
	.device =
		SYS_SPI_DT_SPEC_GET(M95P_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
	.cs_gpio = GPIO_DT_SPEC_GET(M95P_NODE, cs_gpios),
};

/**
 * @brief Assert the software-managed M95P chip-select signal.
 * @details The devicetree GPIO is active-low, so logical value 1 selects the
 *          device through Zephyr's active-level translation.
 * @retval 0 Chip select was asserted.
 * @return A negative errno value returned by the GPIO driver on failure.
 */
static inline int m95p_cs_select(void) {
	int ret = gpio_pin_set_dt(&m95p.cs_gpio, 1);

	if (ret != 0) {
		LOG_ERR("Failed to assert M95P chip-select (%d)", ret);
	}
	return ret;
}

/**
 * @brief Deassert the software-managed M95P chip-select signal.
 * @details Logical value 0 releases the active-low line through Zephyr's
 *          active-level translation.
 * @retval 0 Chip select was deasserted.
 * @return A negative errno value returned by the GPIO driver on failure.
 */
static inline int m95p_cs_deselect(void) {
	int ret = gpio_pin_set_dt(&m95p.cs_gpio, 0);

	if (ret != 0) {
		LOG_ERR("Failed to deassert M95P chip-select (%d)", ret);
	}
	return ret;
}

/**
 * @brief Acquire shared-SPI ownership for one high-level M95P operation.
 * @details Uses the singleton's SPI specification as the ownership token and
 *          waits for at most M95P_SPI_TIMEOUT milliseconds.
 * @retval true Ownership was acquired.
 * @retval false The shared bus could not be acquired before the timeout.
 */
static inline bool m95p_bus_lock(void) {
	int ret = sys_spi_lock(&m95p.device, K_MSEC(M95P_SPI_TIMEOUT));

	if (ret != 0) {
		LOG_ERR("Failed to lock SYS_SPI (%d)", ret);
		return false;
	}

	return true;
}

/**
 * @brief Fully release shared-SPI ownership after a high-level operation.
 * @details Calls sys_spi_release() so any recursive acquisitions made by
 *          private helpers are completely unwound.
 * @retval true Ownership was fully released.
 * @retval false The shared-bus wrapper reported a release failure.
 */
static inline bool m95p_bus_unlock(void) {
	int ret = sys_spi_release(&m95p.device);

	if (ret != 0) {
		LOG_ERR("Failed to release SYS_SPI ownership (%d)", ret);
		return false;
	}
	// enforce chip select deassertion after bus release to avoid leaving the device selected
	m95p_cs_deselect();
	return true;
}

/**
 * @brief Encode an M95P instruction and 24-bit byte address.
 * @details Produces the four-byte command prefix required by addressed array
 *          operations: instruction, address[23:16], address[15:8], address[7:0].
 * @param buffer Destination with space for four command bytes.
 * @param command M95P instruction code.
 * @param address Zero-based byte address in the M95P array.
 */
static inline void m95p_command_with_address(uint8_t* buffer, uint8_t command, uint32_t address) {
	buffer[0] = command;
	buffer[1] = (uint8_t) (address >> 16);
	buffer[2] = (uint8_t) (address >> 8);
	buffer[3] = (uint8_t) address;
}

/**
 * @brief Execute one command followed by an SPI read payload.
 * @details The caller must own the shared SPI bus. Chip select remains asserted
 *          across the command and receive phases, and is deasserted on every
 *          path after a successful assertion.
 * @param command Command bytes to transmit.
 * @param commandSize Number of command bytes.
 * @param buffer Destination for received payload bytes.
 * @param readSize Number of payload bytes to receive.
 * @retval 0 Transfer and chip-select release succeeded.
 * @return A negative errno value from GPIO or SPI on failure.
 */
static inline int mp95p_spi_read(uint8_t* command,
								 size_t commandSize,
								 uint8_t* buffer,
								 size_t readSize) {
	struct spi_buf tx_buf = {.buf = command, .len = commandSize};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

	struct spi_buf rx_buf[2] = {{
									.buf = NULL,
									.len = commandSize // Skip bytes while sending command
								},
								{
									.buf = buffer,
									.len = readSize // Actual data to read
								}};
	struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 2};

	int ret = m95p_cs_select();
	if (ret != 0) {
		return ret;
	}

	ret = sys_spi_transceive(&m95p.device, &tx_bufs, &rx_bufs);
	int cs_ret = m95p_cs_deselect();

	if (ret != 0) {
		LOG_ERR("M95P SPI read failed (%d)", ret);
		return ret;
	}
	return cs_ret;
}

/**
 * @brief Execute one command with an optional SPI write payload.
 * @details The caller must own the shared SPI bus. The command and payload are
 *          emitted as one transaction while chip select remains asserted.
 * @param command Command bytes to transmit.
 * @param commandSize Number of command bytes.
 * @param buffer Optional payload bytes, or NULL when no payload is required.
 * @param writeSize Number of payload bytes.
 * @retval 0 Transfer and chip-select release succeeded.
 * @return A negative errno value from GPIO or SPI on failure.
 */
static inline int mp95p_spi_write(uint8_t* command,
								  size_t commandSize,
								  const uint8_t* buffer,
								  size_t writeSize) {
	// We can have up to 2 buffers: the command and the data payload
	struct spi_buf tx_bufs_array[2];
	uint8_t buf_count = 0;

	// 1. Add the command buffer
	tx_bufs_array[buf_count].buf = command;
	tx_bufs_array[buf_count].len = commandSize;
	buf_count++;

	// 2. Add the data buffer if it exists and has size
	if (writeSize > 0 && buffer != NULL) {
		tx_bufs_array[buf_count].buf = (void*) buffer;
		tx_bufs_array[buf_count].len = writeSize;
		buf_count++;
	}

	// Wrap the array in a buffer set
	struct spi_buf_set tx_bufs = {.buffers = tx_bufs_array, .count = buf_count};

	// 3. Execute the write.
	// Passing NULL for rx_bufs indicates a write-only operation.
	int ret = m95p_cs_select();
	if (ret != 0) {
		return ret;
	}

	ret = sys_spi_write(&m95p.device, &tx_bufs);
	int cs_ret = m95p_cs_deselect();

	if (ret != 0) {
		LOG_ERR("M95P SPI write failed (%d)", ret);
		return ret;
	}
	return cs_ret;
}

/**
 * @brief Set the M95P write-enable latch.
 * @details Sends WREN. The caller must own the shared SPI bus.
 * @retval 0 WREN was transmitted successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_spi_write_enable(void) {
	uint8_t command = m95p_instruction_WREN;
	return mp95p_spi_write(&command, 1, NULL, 0);
}

/**
 * @brief Read the M95P status register.
 * @details Sends RDSR and stores the returned raw byte in @p status. The caller
 *          must own the shared SPI bus.
 * @param status Destination for the status-register image.
 * @retval 0 The register was read successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_spi_read_status_register(union m95p_status_register_t* status) {
	uint8_t command = m95p_instruction_RDSR;
	uint8_t value;
	int ret = mp95p_spi_read(&command, 1, &value, 1);

	if (ret != 0) {
		return ret;
	}
	status->value = value;
	return 0;
}

/**
 * @brief Read the configuration and safety registers.
 * @details Sends RDCR and maps its first response byte to the configuration
 *          register and second response byte to the safety register. The caller
 *          must own the shared SPI bus.
 * @param config Destination for both register images.
 * @retval 0 Both bytes were read successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_spi_read_configuration_safety_register(
	struct m95p_configuration_safety_registers_t* config) {
	uint8_t buffer[2];
	uint8_t command = m95p_instruction_RDCR;
	int ret = mp95p_spi_read(&command, 1, buffer, 2);

	if (ret != 0) {
		return ret;
	}
	config->configuration_register.value = buffer[0];
	config->safety_register.value = buffer[1];
	return 0;
}

/**
 * @brief Clear volatile M95P safety flags.
 * @details Sends CLRSF. The caller must own the shared SPI bus.
 * @retval 0 The command was transmitted successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_clear_safety_flags(void) {
	uint8_t command = m95p_instruction_CLRSF;
	return mp95p_spi_write(&command, 1, NULL, 0);
}

/**
 * @brief Write the status register and optionally the configuration register.
 * @details Sends WRSR. When @p bStatusOnly is false, the configuration byte is
 *          included after the status byte. The caller must issue WREN first and
 *          must own the shared SPI bus.
 * @param statusRegister Raw status-register image to write.
 * @param configurationRegister Raw configuration-register image to write.
 * @param bStatusOnly true to write only status; false to include configuration.
 * @retval 0 The command was transmitted successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_write_status_and_configuration_register(
	union m95p_status_register_t statusRegister,
	union m95p_configuration_register_t configurationRegister,
	bool bStatusOnly) {
	uint8_t buffer[4];
	size_t size = 1;
	uint8_t command = m95p_instruction_WRSR;
	buffer[0] = statusRegister.value;
	if (bStatusOnly == false) {
		buffer[1] = configurationRegister.value;
		buffer[2] = 0x00;
		buffer[3] = 0x00;
		size += 1;
	}
	return mp95p_spi_write(&command, 1, buffer, size);
}

/**
 * @brief Read the three-byte JEDEC device identifier.
 * @details Sends JEDID and preserves the device's receive-byte order. The caller
 *          must own the shared SPI bus.
 * @param id Destination for manufacturer, family, and density bytes.
 * @retval 0 All identification bytes were read successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_read_jedec_id(union m95p_jedec_id_t* id) {
	uint8_t command = m95p_instruction_JEDID;
	return mp95p_spi_read(&command, 1, id->data, sizeof(id->data));
}

/**
 * @brief Execute the M95P software-reset command sequence.
 * @details Sends RSTEN followed by RESET with the required instruction spacing.
 *          The caller must own the shared SPI bus.
 * @retval 0 Both reset commands were transmitted successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_reset(void) {
	uint8_t command = m95p_instruction_RSTEN;
	int ret = mp95p_spi_write(&command, 1, NULL, 0);

	if (ret != 0) {
		return ret;
	}
	command = m95p_instruction_RESET;
	for (int i = 10; i > 0; i--) {
		__NOP();
	}
	return mp95p_spi_write(&command, 1, NULL, 0);
}

/**
 * @brief Read the write-in-progress state from status-register WIP.
 * @details The caller must own the shared SPI bus.
 * @param busy Receives true while a modify or power-up operation is active.
 * @retval 0 The status register was read and @p busy was updated.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_is_busy(bool* busy) {
	union m95p_status_register_t status;
	int ret = m95p_spi_read_status_register(&status);

	if (ret == 0) {
		*busy = status.bits.WIP != 0;
	}
	return ret;
}

/**
 * @brief Check the safety register for program or erase failures.
 * @details Reports the logical OR of PRF and ERF. The caller must own the
 *          shared SPI bus.
 * @param error Receives true when either failure flag is asserted.
 * @retval 0 The safety register was read and @p error was updated.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_is_error(bool* error) {
	struct m95p_configuration_safety_registers_t config;
	int ret = m95p_spi_read_configuration_safety_register(&config);

	if (ret == 0) {
		*error = (config.safety_register.bits.ERF != 0) || (config.safety_register.bits.PRF != 0);
	}
	return ret;
}

/**
 * @brief Clear program and erase failure state after it has been observed.
 * @details Delegates to the CLRSF helper. The caller must own the shared bus.
 * @retval 0 Safety flags were cleared.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_clear_error(void) {
	return m95p_device_clear_safety_flags();
}

/**
 * @brief Poll status-register WIP until the device becomes idle.
 * @details This helper does not inspect or clear safety flags and is therefore
 *          suitable immediately after reset. The caller must own the bus.
 * @retval 0 The device became idle.
 * @return A negative errno value when status polling fails.
 */
static int m95p_device_wait_while_busy(void) {
	bool busy;
	int ret = m95p_device_is_busy(&busy);

	while ((ret == 0) && busy) {
		__NOP();
		ret = m95p_device_is_busy(&busy);
	}
	return ret;
}

/**
 * @brief Wait for idle and validate the preceding modify operation.
 * @details After WIP clears, reads PRF and ERF. Failure flags are cleared to
 *          leave the device usable, but the helper returns -EIO so the failed
 *          operation is not reported as successful.
 * @retval 0 The device became idle without a program or erase failure.
 * @retval -EIO The device reported PRF or ERF.
 * @return Another negative errno value when polling or flag clearing fails.
 */
static int m95p_device_wait_until_not_busy(void) {
	int ret = m95p_device_wait_while_busy();

	if (ret != 0) {
		return ret;
	}

	bool error;
	ret = m95p_device_is_error(&error);
	if ((ret == 0) && error) {
		int clear_ret = m95p_device_clear_error();

		if (clear_ret != 0) {
			return clear_ret;
		}
		LOG_ERR("M95P reported a failed program or erase operation");
		return -EIO;
	}
	return ret;
}

/**
 * @brief Read status-register write-disable state.
 * @details Maps status-register SRWD to a boolean. This does not report BP
 *          array protection. The caller must own the shared SPI bus.
 * @param write_protected Receives true when SRWD is asserted.
 * @retval 0 The status register was read successfully.
 * @return A negative errno value from the transfer path on failure.
 */
static int m95p_device_is_write_protected(bool* write_protected) {
	union m95p_status_register_t status;
	int ret = m95p_spi_read_status_register(&status);

	if (ret == 0) {
		*write_protected = status.bits.SRWD != 0;
	}
	return ret;
}

bool m95p_init(void* arg) {
	(void) arg;
	if (m95p.initialized) {
		return true;
	}

	if (!sys_spi_is_ready(&m95p.device)) {
		LOG_ERR("SYS_SPI device not ready");
		return false;
	}
	if (!gpio_is_ready_dt(&m95p.cs_gpio)) {
		LOG_ERR("M95P chip-select GPIO not ready");
		return false;
	}

	int ret = gpio_pin_configure_dt(&m95p.cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure M95P chip-select GPIO (%d)", ret);
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	ret = m95p_device_reset();
	if (ret == 0) {
		for (int i = 0; i < 10000; i++) {
			__NOP();
		}
		ret = m95p_device_wait_while_busy();
	}
	if (ret == 0) {
		ret = m95p_read_jedec_id(&m95p.jedec_id);
	}
	if ((ret == 0) && ((m95p.jedec_id.fields.manufacturer_id != M95P_MANUFACTURER_ID) ||
					   (m95p.jedec_id.fields.memory_type != M95P_FAMILY_CODE) ||
					   (m95p.jedec_id.fields.capacity != M95P_MEMORY_DENSITY))) {
		LOG_ERR("Invalid M95P JEDEC ID: %02x %02x %02x (expected %02x %02x %02x)",
				m95p.jedec_id.data[0],
				m95p.jedec_id.data[1],
				m95p.jedec_id.data[2],
				M95P_MANUFACTURER_ID,
				M95P_FAMILY_CODE,
				M95P_MEMORY_DENSITY);
		ret = -ENODEV;
	}
	if (ret == 0) {
		ret = m95p_device_clear_error();
	}

	if (ret != 0) {
		LOG_ERR("M95P initialization failed (%d)", ret);
	}
	bool success = (ret == 0);
	if (!m95p_bus_unlock()) {
		success = false;
	}
	m95p.initialized = success;
	return success;
}

bool m95p_is_ready(void) {
	return (m95p.initialized && m95p.jedec_id.fields.manufacturer_id == M95P_MANUFACTURER_ID);
}

union m95p_jedec_id_t m95p_get_jedec_id(void) {
	assert(m95p.initialized);
	return m95p.jedec_id;
}

/**
 * @brief Erase the complete M95P memory array.
 * @details Waits for idle, sends WREN and CHER, then waits for completion and
 *          validates the safety flags. The caller must own the shared SPI bus.
 * @retval 0 Chip erase completed without a device-reported failure.
 * @return A negative errno value from a command, poll, or safety check.
 */
static int m95p_device_chip_erase(void) {
	uint8_t command = m95p_instruction_CHER;
	int ret = m95p_device_wait_until_not_busy();

	if (ret == 0) {
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = mp95p_spi_write(&command, 1, NULL, 0);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	return ret;
}

/**
 * @brief Erase one physical 64-Kbyte block at a byte address.
 * @details Sends BKER after WREN and validates completion. The caller must own
 *          the shared SPI bus and provide an aligned, validated address.
 * @param address Byte address within the target block.
 * @retval 0 Block erase completed without a device-reported failure.
 * @return A negative errno value from a command, poll, or safety check.
 */
static int m95p_device_block_erase(uint32_t address) {
	uint8_t command[4];
	m95p_command_with_address(command, m95p_instruction_BKER, address);
	int ret = m95p_device_wait_until_not_busy();

	if (ret == 0) {
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = mp95p_spi_write(command, sizeof(command), NULL, 0);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	return ret;
}

/**
 * @brief Erase one 512-byte page at a byte address.
 * @details Sends PGER after WREN and validates completion. The caller must own
 *          the shared SPI bus and provide an aligned, validated address.
 * @param address Byte address within the target page.
 * @retval 0 Page erase completed without a device-reported failure.
 * @return A negative errno value from a command, poll, or safety check.
 */
static int m95p_device_page_erase(uint32_t address) {
	uint8_t command[4];
	m95p_command_with_address(command, m95p_instruction_PGER, address);
	int ret = m95p_device_wait_until_not_busy();

	if (ret == 0) {
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = mp95p_spi_write(command, sizeof(command), NULL, 0);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	return ret;
}

/**
 * @brief Program erased bytes within one 512-byte page.
 * @details Sends PGPR after WREN and validates completion. PGPR does not erase
 *          existing contents. The caller must own the shared SPI bus.
 * @param address Starting byte address in the target page.
 * @param data Bytes to program.
 * @param size Number of bytes, already validated to fit one page.
 * @retval 0 Programming completed without a device-reported failure.
 * @return A negative errno value from a command, poll, or safety check.
 */
static int m95p_device_program_page(uint32_t address, const uint8_t* data, size_t size) {
	uint8_t command[4];
	m95p_command_with_address(command, m95p_instruction_PGPR, address);
	int ret = m95p_device_wait_until_not_busy();

	if (ret == 0) {
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = mp95p_spi_write(command, sizeof(command), data, size);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	return ret;
}

/**
 * @brief Erase and write bytes within one 512-byte page.
 * @details Sends PGWR after WREN and validates completion. The caller must own
 *          the shared SPI bus.
 * @param address Starting byte address in the target page.
 * @param data Bytes to write.
 * @param size Number of bytes, already validated to fit one page.
 * @retval 0 Page write completed without a device-reported failure.
 * @return A negative errno value from a command, poll, or safety check.
 */
static int m95p_device_page_write(uint32_t address, const uint8_t* data, size_t size) {
	uint8_t command[4];
	m95p_command_with_address(command, m95p_instruction_PGWR, address);
	int ret = m95p_device_wait_until_not_busy();

	if (ret == 0) {
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = mp95p_spi_write(command, sizeof(command), data, size);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	return ret;
}

/**
 * @brief Read bytes from the memory array.
 * @details Waits for the device to become idle, sends READ with a 24-bit byte
 *          address, and receives @p size bytes. The caller must own the bus.
 * @param address Starting byte address.
 * @param buffer Destination buffer.
 * @param size Number of bytes to read.
 * @retval 0 The complete payload was received.
 * @return A negative errno value from polling or the transfer path.
 */
static int m95p_device_read(uint32_t address, uint8_t* buffer, size_t size) {
	uint8_t command[4];
	int ret = m95p_device_wait_until_not_busy();

	if (ret != 0) {
		return ret;
	}
	m95p_command_with_address(command, m95p_instruction_READ, address);
	return mp95p_spi_read(command, sizeof(command), buffer, size);
}

/**
 * @brief Validate a byte range against the complete M95P array.
 * @details Uses subtraction-based bounds checking to avoid address-plus-size
 *          integer overflow.
 * @param address Starting byte address.
 * @param size Number of bytes in the range.
 * @retval true The complete range lies within the array.
 * @retval false The size or end address exceeds the array.
 */
static bool m95p_range_valid(uint32_t address, size_t size) {
	return (size <= M95P_SIZE) && (address <= (M95P_SIZE - size));
}

bool m95p_program_page(uint32_t page, const void* data, size_t size) {
	if (!m95p.initialized || (data == NULL) || (size == 0) || (size > SYS_MEMORY_PAGE_SIZE) ||
		(page >= SYS_MEMORY_PAGE_COUNT)) {
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_program_page(page * SYS_MEMORY_PAGE_SIZE, data, size);
	if (ret != 0) {
		LOG_ERR("M95P page program failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

size_t m95p_get_size(void) {
	return SYS_MEMORY_SECTOR_COUNT * SYS_MEMORY_PAGES_PER_SECTOR * SYS_MEMORY_PAGE_SIZE;
}

size_t m95p_get_sector_size(void) {
	return SYS_MEMORY_PAGE_SIZE;
}

size_t m95p_get_supported_sector_size(void) {
	return SYS_MEMORY_SUPPORTED_SECTOR_SIZE;
}

size_t m95p_get_page_size(void) {
	return SYS_MEMORY_PAGE_SIZE;
}

size_t m95p_get_block_size(void) {
	return SYS_MEMORY_BLOCK_SIZE;
}

size_t m95p_get_sector_count(void) {
	// we can perform page erase and program --> SECTOR == PAGE for this device
	return SYS_MEMORY_SECTOR_COUNT * SYS_MEMORY_PAGES_PER_SECTOR;
}

size_t m95p_get_page_count(void) {
	return SYS_MEMORY_SECTOR_COUNT * SYS_MEMORY_PAGES_PER_SECTOR;
}

size_t m95p_get_block_count(void) {
	return SYS_MEMORY_SECTOR_COUNT / SYS_MEMORY_SECTORS_PER_BLOCK;
}

size_t m95p_get_max_protected_block_count(void) {
	return SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT;
}

size_t m95p_get_golden_section_size(void) {
	return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE;
}

size_t m95p_get_golden_section_page_count(void) {
	return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE / SYS_MEMORY_PAGE_SIZE;
}

size_t m95p_get_golden_section_sector_count(void) {
	return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE / SYS_MEMORY_PAGE_SIZE;
}

bool m95p_write_sector(uint32_t sector, const void* data, size_t size) {
	if (!m95p.initialized || (data == NULL) || (size == 0) || (size > SYS_MEMORY_PAGE_SIZE) ||
		(sector >= m95p_get_sector_count())) {
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_page_write(sector * SYS_MEMORY_PAGE_SIZE, data, size);
	if (ret != 0) {
		LOG_ERR("M95P sector write failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_read(uint32_t address, void* data, size_t size) {
	if (!m95p.initialized || (data == NULL) || (size == 0) || !m95p_range_valid(address, size)) {
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_read(address, data, size);
	if (ret != 0) {
		LOG_ERR("M95P read failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_erase_sector(uint32_t sector) {
	if (!m95p.initialized || (sector >= m95p_get_sector_count())) {
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_page_erase(sector * SYS_MEMORY_PAGE_SIZE);
	if (ret != 0) {
		LOG_ERR("M95P sector erase failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_erase_block(uint32_t block) {
	if (!m95p.initialized || (block >= m95p_get_block_count())) {
		return false;
	}
	if (!m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_block_erase(block * SYS_MEMORY_BLOCK_SIZE);
	if (ret != 0) {
		LOG_ERR("M95P block erase failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_reset_to_factory_defaults(void) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	union m95p_status_register_t status = {.value = 0};
	struct m95p_configuration_safety_registers_t config = {0};
	int ret = m95p_spi_read_status_register(&status);

	if (ret == 0) {
		status.bits.SRWD = 0;
		status.bits.BP = 0;
		ret = m95p_spi_read_configuration_safety_register(&config);
	}
	if (ret == 0) {
		config.configuration_register.bits.LID = 0;
		ret = m95p_spi_write_enable();
	}
	if (ret == 0) {
		ret = m95p_device_write_status_and_configuration_register(status,
																  config.configuration_register,
																  false);
	}
	if (ret == 0) {
		ret = m95p_device_wait_until_not_busy();
	}
	if (ret == 0) {
		ret = m95p_device_chip_erase();
	}
	if (ret != 0) {
		LOG_ERR("M95P factory reset failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_reset(void) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	int ret = m95p_device_reset();
	if (ret != 0) {
		LOG_ERR("M95P reset failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_set_write_protection_state(bool bWriteProtect) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	union m95p_status_register_t status = {.value = 0};
	int ret = m95p_spi_read_status_register(&status);

	if ((ret == 0) && ((status.bits.SRWD != 0) != bWriteProtect)) {
		status.bits.SRWD = bWriteProtect;
		ret = m95p_device_wait_until_not_busy();
		if (ret == 0) {
			ret = m95p_spi_write_enable();
		}
		if (ret == 0) {
			ret = m95p_device_write_status_and_configuration_register(
				status,
				(union m95p_configuration_register_t) {.value = 0},
				true);
		}
		if (ret == 0) {
			ret = m95p_device_wait_until_not_busy();
		}
	}
	if (ret != 0) {
		LOG_ERR("M95P write-protection update failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

bool m95p_is_write_protected(void) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	bool write_protected = false;
	int ret = m95p_device_is_write_protected(&write_protected);
	if (ret != 0) {
		LOG_ERR("Failed to read M95P write protection (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return (ret == 0) && write_protected;
}

uint32_t m95p_write_protected_blocks_count(void) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return 0;
	}

	union m95p_status_register_t status = {.value = 0};
	int ret = m95p_spi_read_status_register(&status);
	uint32_t count = (status.bits.BP == 0) ? 0U : BIT(status.bits.BP - 1U);

	if (ret != 0) {
		LOG_ERR("Failed to read M95P protected block count (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return 0U;
	}
	return (ret == 0) ? count : 0U;
}

/**
 * @brief Encode and apply top-of-array block protection.
 * @details Converts a supported power-of-two block count to BP[2:0], forces
 *          TB=0 for top protection, writes the status register, and validates
 *          completion. The public legacy API discards this result while golden
 *          section helpers propagate it.
 * @param count Number of 64-Kbyte blocks to protect, or zero to unprotect.
 * @param permanent Reserved for a future permanent-lock policy; currently ignored.
 * @retval true The requested protection state was applied or already active.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 */
static bool m95p_set_protected_blocks(uint32_t count, bool permanent) {
	(void) permanent;
	uint8_t bp = 0;

	if (count > 0) {
		for (uint8_t candidate = 1; candidate <= 7; candidate++) {
			if (BIT(candidate - 1U) == count) {
				bp = candidate;
				break;
			}
		}
		if ((bp == 0) || (count > SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT)) {
			LOG_ERR("Unsupported M95P protected block count: %u", count);
			return false;
		}
	}
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	union m95p_status_register_t status = {.value = 0};
	int ret = m95p_spi_read_status_register(&status);

	if ((ret == 0) && (status.bits.BP != bp)) {
		status.bits.BP = bp;
		status.bits.TB = 0;
		ret = m95p_device_wait_until_not_busy();
		if (ret == 0) {
			ret = m95p_spi_write_enable();
		}
		if (ret == 0) {
			ret = m95p_device_write_status_and_configuration_register(
				status,
				(union m95p_configuration_register_t) {.value = 0},
				true);
		}
		if (ret == 0) {
			ret = m95p_device_wait_until_not_busy();
		}
	}
	if (ret != 0) {
		LOG_ERR("M95P block-protection update failed (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return ret == 0;
}

void m95p_write_protect_blocks(uint32_t count, bool permanent) {
	(void) m95p_set_protected_blocks(count, permanent);
}

bool m95p_is_busy(void) {
	if (!m95p.initialized || !m95p_bus_lock()) {
		return false;
	}

	bool busy = false;
	int ret = m95p_device_is_busy(&busy);
	if (ret != 0) {
		LOG_ERR("Failed to read M95P busy state (%d)", ret);
	}
	if (!m95p_bus_unlock()) {
		return false;
	}
	return (ret == 0) && busy;
}

bool m95p_golden_section_read(uint32_t address, void* data, size_t size) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
	return false;
#else
	if ((size > SYS_MEMORY_GOLDEN_SECTION_SIZE) ||
		(address > (SYS_MEMORY_GOLDEN_SECTION_SIZE - size))) {
		return false;
	}
	address += SYS_MEMORY_GOLDEN_SECTION_ADDRESS_START;
	return m95p_read(address, data, size);
#endif
}

bool m95p_golden_section_program_page(uint32_t page, const void* data, size_t size) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
	return true;
#else
	if (page >= SYS_MEMORY_GOLDEN_SECTION_PAGE_COUNT) {
		return false;
	}
	page += SYS_MEMORY_GOLDEN_SECTION_PAGE_START;
	return m95p_write_sector(page, data, size);
#endif
}

bool m95p_golden_section_erase_sector(uint32_t sector) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
	return true;
#else
	if (sector >= SYS_MEMORY_GOLDEN_SECTION_PAGE_COUNT) {
		return false;
	}
	sector += SYS_MEMORY_GOLDEN_SECTION_PAGE_START;
	return m95p_erase_sector(sector);
#endif
}

bool m95p_golden_section_erase(void) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
	return true;
#else
	for (uint32_t page = SYS_MEMORY_GOLDEN_SECTION_PAGE_START; page < SYS_MEMORY_PAGE_COUNT;
		 page++) {
		if (!m95p_erase_sector(page)) {
			return false;
		}
	}
	return true;
#endif
}

bool m95p_golden_section_is_page_in(uint32_t page) {
	return (page >= SYS_MEMORY_GOLDEN_SECTION_PAGE_START) && (page < SYS_MEMORY_PAGE_COUNT);
}

bool m95p_golden_section_lock(void) {
	return m95p_set_protected_blocks(SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT, true);
}

bool m95p_golden_section_unlock(void) {
	return m95p_set_protected_blocks(0, false);
}
