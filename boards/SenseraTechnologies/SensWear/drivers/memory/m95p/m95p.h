/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file m95p.h
 * @brief Public interface for the board's M95P32 page EEPROM.
 *
 * @defgroup sensewear_m95p SenseWear M95P32 page EEPROM
 * @ingroup io_interfaces
 * @{
 *
 * The driver provides synchronous access to the board-mounted M95P32 over the
 * shared SPI bus. Public operations acquire and completely release shared-bus
 * ownership around each transaction sequence.
 *
 * The API uses a 512-byte page as its logical erase/write sector because the
 * M95P32 supports page-level erase and page-write operations. This differs
 * from the device's physical 4-Kbyte sector terminology.
 */
#ifndef M95P_H_
#define M95P_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "m95p_registers.h"

/**
 * @brief Initialize and identify the M95P32.
 * @details Verifies that the shared SPI device is ready, resets the memory,
 *          reads the three identification bytes, validates them against the
 *          expected M95P32 values, and clears pending safety flags. Repeated
 *          calls after successful initialization return immediately.
 * @param arg Reserved; pass NULL.
 * @retval true Initialization completed or the driver was already initialized.
 * @retval false The shared SPI device was unavailable or could not be acquired.
 */
bool m95p_init(void* arg);

/**
 * @brief Return the identification bytes cached during initialization.
 * @return Cached manufacturer, family, and density identification.
 * @pre m95p_init() has completed successfully.
 */
union m95p_jedec_id_t m95p_get_jedec_id(void);

/**
 * @brief Check whether the driver has accepted an M95P32 device.
 * @details Checks the initialization flag and cached manufacturer identifier.
 *          This function does not read WIP and therefore does not report
 *          whether a program or erase operation is currently active.
 * @retval true The driver is initialized and the cached manufacturer ID is valid.
 * @retval false The driver is not initialized or its cached ID is invalid.
 */
bool m95p_is_ready(void);

/**
 * @brief Read the device's write-in-progress state.
 * @details Acquires the shared SPI bus and reads status-register WIP.
 * @retval true A modify cycle or power-up operation is in progress.
 * @retval false The device is idle.
 * @pre m95p_init() has completed successfully.
 */
bool m95p_is_busy(void);

/**
 * @name Memory geometry
 * @details These functions return constants derived from the M95P32 geometry
 *          and the optional top-of-array golden-section reservation.
 * @{
 */

/**
 * @brief Return the data-area capacity in bytes.
 * @details Excludes the configured golden section.
 */
size_t m95p_get_size(void);

/**
 * @brief Return the driver's logical erase-sector size.
 * @details The driver exposes one 512-byte M95P32 page as a sector because
 *          m95p_erase_sector() performs a page erase.
 */
size_t m95p_get_sector_size(void);

/**
 * @brief Return the filesystem-facing logical sector size.
 * @details Equals FF_MAX_SS when FatFS is enabled; otherwise it equals the
 *          physical 4-Kbyte M95P32 sector size.
 */
size_t m95p_get_supported_sector_size(void);

/** @brief Return the M95P32 page size in bytes (512 bytes). */
size_t m95p_get_page_size(void);

/** @brief Return the M95P32 erase-block size in bytes (64 Kbytes). */
size_t m95p_get_block_size(void);

/**
 * @brief Return the number of logical 512-byte sectors in the data area.
 * @details This value equals m95p_get_page_count().
 */
size_t m95p_get_sector_count(void);

/** @brief Return the number of 512-byte pages in the data area. */
size_t m95p_get_page_count(void);

/** @brief Return the number of 64-Kbyte blocks in the data area. */
size_t m95p_get_block_count(void);

/** @brief Return the maximum number of blocks representable by BP[2:0]. */
size_t m95p_get_max_protected_block_count(void);

/** @brief Return the reserved golden-section capacity in bytes. */
size_t m95p_get_golden_section_size(void);

/** @brief Return the number of 512-byte pages in the golden section. */
size_t m95p_get_golden_section_page_count(void);

/**
 * @brief Return the golden-section count in driver-logical sectors.
 * @details Driver-logical sectors are 512-byte pages, so this currently
 *          returns the same value as m95p_get_golden_section_page_count().
 */
size_t m95p_get_golden_section_sector_count(void);

/** @} */

/**
 * @brief Read bytes from the memory array.
 * @param address Zero-based byte address in the complete memory array.
 * @param data Destination buffer.
 * @param size Number of bytes to read.
 * @retval true The shared SPI bus was acquired and the read completed.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 * @pre m95p_init() has completed successfully.
 */
bool m95p_read(uint32_t address, void* data, size_t size);

/**
 * @brief Program erased bytes using the M95P32 page-program instruction.
 * @param page Zero-based 512-byte page index in the complete memory array.
 * @param data Source buffer.
 * @param size Number of bytes to program.
 * @retval true The shared SPI bus was acquired and the operation completed.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 * @pre m95p_init() has completed successfully.
 * @warning The caller must provide bytes in the erased state.
 */
bool m95p_program_page(uint32_t page, const void* data, size_t size);

/**
 * @brief Erase and write one logical 512-byte sector.
 * @details Uses the M95P32 PGWR instruction at `sector * 512`. Despite the
 *          function name, this operation targets one page rather than a
 *          physical 4-Kbyte sector.
 * @param sector Zero-based logical 512-byte sector index.
 * @param data Source buffer.
 * @param size Number of bytes to write.
 * @retval true The shared SPI bus was acquired and the operation completed.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 */
bool m95p_write_sector(uint32_t sector, const void* data, size_t size);

/**
 * @brief Erase one logical 512-byte sector.
 * @details Sends the M95P32 page-erase instruction at `sector * 512`; it does
 *          not issue the physical 4-Kbyte sector-erase instruction.
 * @param sector Zero-based logical 512-byte sector index.
 * @retval true The shared SPI bus was acquired and the operation completed.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 */
bool m95p_erase_sector(uint32_t sector);

/**
 * @brief Erase one physical 64-Kbyte block.
 * @param block Zero-based block index in the data area.
 * @retval true The erase and shared-bus release completed.
 * @retval false Validation, transfer, or shared-bus ownership failed.
 */
bool m95p_erase_block(uint32_t block);

/**
 * @brief Restore writable defaults and erase the complete memory array.
 * @details Clears SRWD and BP, attempts to clear LID, writes the status and
 *          configuration registers, and executes chip erase.
 * @retval true The shared SPI bus was acquired and the sequence completed.
 * @retval false The shared SPI bus could not be acquired.
 * @warning LID is one-time lock state on the device; writing zero cannot
 *          unlock an identification page that has already been permanently
 *          locked.
 */
bool m95p_reset_to_factory_defaults(void);

/**
 * @brief Issue the M95P software-reset sequence.
 * @retval true Both reset commands and shared-bus release completed.
 * @retval false The driver was unavailable or an operation failed.
 */
bool m95p_reset(void);

/**
 * @name Golden-section access
 * @details The golden section is an optional whole-block reservation at the
 *          top of the memory array. Its size is controlled by
 *          SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT in m95p.c and defaults to
 *          zero blocks.
 * @{
 */

/**
 * @brief Read from an address relative to the golden-section start.
 * @param address Byte offset within the golden section.
 * @param data Destination buffer.
 * @param size Number of bytes to read.
 * @retval true The read completed.
 * @retval false No golden section is configured or SPI acquisition failed.
 */
bool m95p_golden_section_read(uint32_t address, void* data, size_t size);

/**
 * @brief Erase and write a page relative to the golden-section start.
 * @param page Zero-based page index within the golden section.
 * @param data Source buffer.
 * @param size Number of bytes to write.
 * @retval true The operation completed, or no golden section is configured.
 * @retval false SPI acquisition failed.
 */
bool m95p_golden_section_program_page(uint32_t page, const void* data, size_t size);

/**
 * @brief Erase a logical sector relative to the golden-section start.
 * @param sector Zero-based logical sector index within the golden section.
 * @retval true The operation completed, or no golden section is configured.
 */
bool m95p_golden_section_erase_sector(uint32_t sector);

/**
 * @brief Erase the complete golden section.
 * @retval true No golden section is configured or all reserved pages erased.
 * @retval false A page erase failed.
 */
bool m95p_golden_section_erase(void);

/**
 * @brief Check whether an absolute page index lies in the golden section.
 * @param page Zero-based page index in the complete memory array.
 * @retval true The page is at or above the golden-section start and below the
 *              end of the memory array.
 * @retval false The page is outside that range.
 */
bool m95p_golden_section_is_page_in(uint32_t page);

/**
 * @brief Protect the configured top-of-array golden-section blocks.
 * @retval true The protection request was issued.
 */
bool m95p_golden_section_lock(void);

/**
 * @brief Clear block protection for the golden section.
 * @retval true The protection request was issued.
 */
bool m95p_golden_section_unlock(void);

/** @} */

/**
 * @name Write protection
 * @{
 */

/**
 * @brief Read the status-register write-disable state.
 * @pre m95p_init() has completed successfully.
 * @note This reports SRWD, not whether BP currently protects array blocks.
 */
bool m95p_is_write_protected(void);

/**
 * @brief Set or clear status-register write disable.
 * @param bWriteProtect true to set SRWD; false to clear it.
 * @retval true The shared SPI bus was acquired and the update completed or
 *              the requested state was already active.
 * @retval false The shared SPI bus could not be acquired.
 * @note Array protection is controlled separately through BP[2:0].
 */
bool m95p_set_write_protection_state(bool bWriteProtect);

/**
 * @brief Decode BP[2:0] as the number of protected 64-Kbyte blocks.
 * @return Number of protected top-of-array blocks.
 */
uint32_t m95p_write_protected_blocks_count(void);

/**
 * @brief Configure top-of-array block protection.
 * @param count Number of 64-Kbyte blocks to protect. Supported nonzero values
 *              are powers of two representable by BP[2:0].
 * @param permanent Reserved by the current implementation and ignored.
 * @pre m95p_init() has completed successfully.
 */
void m95p_write_protect_blocks(uint32_t count, bool permanent);

/** @} */

/** @} */

#endif /* M95P_H_ */
