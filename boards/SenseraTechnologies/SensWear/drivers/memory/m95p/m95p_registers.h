/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file m95p_registers.h
 * @brief Instruction, register, and identification declarations for M95P32.
 *
 * @defgroup sensewear_m95p SenseWear M95P32 page EEPROM
 * @ingroup io_interfaces
 * @{
 *
 * This header describes the SPI instruction codes and byte-level register
 * representations used by the M95P32 memory driver. Hardware-facing types
 * mirror bytes transferred on the SPI bus unless explicitly documented as a
 * software-only aggregate.
 */
#ifndef M95P_REGISTERS_H
#define M95P_REGISTERS_H

#include <stdint.h>

/**
 * @brief Expected ST manufacturer identifier.
 * @details Compared against byte 0 returned by reading identification-page
 *          address 0 with the RDID instruction.
 */
#define M95P_MANUFACTURER_ID (0x20)

/**
 * @brief Expected M95P32 memory-family code.
 * @details Compared against byte 1 returned by reading identification-page
 *          address 0 with the RDID instruction.
 */
#define M95P_FAMILY_CODE (0x00)

/**
 * @brief Expected M95P32 density code.
 * @details Compared against byte 2 returned by reading identification-page
 *          address 0 with the RDID instruction. Code 0x16 identifies the
 *          32-Mbit device used by this board.
 */
#define M95P_MEMORY_DENSITY (0x16)

/**
 * @brief Shared-SPI acquisition timeout in milliseconds.
 * @details Used by the M95P driver when acquiring ownership of the board's
 *          shared SPI bus.
 */
#define M95P_SPI_TIMEOUT (100)

/**
 * @brief M95P32 SPI instruction-code map.
 * @details Each enumerator is the first byte transmitted in an M95P32 SPI
 *          command. Instructions operating on the memory array or an
 *          identification page require a 24-bit address unless noted
 *          otherwise.
 */
enum m95_instruction_type {
	m95p_instruction_WRSR = 0x01,  /**< Write status and optionally configuration register. */
	m95p_instruction_PGWR = 0x02,  /**< Erase and write 1 to 512 bytes within one page. */
	m95p_instruction_READ = 0x03,  /**< Read array data using single-output SPI. */
	m95p_instruction_WRDI = 0x04,  /**< Clear the status-register WEL bit. */
	m95p_instruction_RDSR = 0x05,  /**< Read the status register, with rollover. */
	m95p_instruction_WREN = 0x06,  /**< Set WEL before a modifying instruction. */
	m95p_instruction_PGPR = 0x0A,  /**< Program 1 to 512 erased bytes within one page. */
	m95p_instruction_FREAD = 0x0B, /**< Fast single-output read with one dummy byte. */
	m95p_instruction_RDCR = 0x15,  /**< Read configuration then safety register, with rollover. */
	m95p_instruction_SCER = 0x20,  /**< Erase one addressed 4-Kbyte sector. */
	m95p_instruction_CLRSF = 0x50, /**< Clear all volatile safety-register flags. */
	m95p_instruction_RDSFDP = 0x5A,/**< Read SFDP data using a 24-bit address and dummy byte. */
	m95p_instruction_RSTEN = 0x66, /**< Arm acceptance of the RESET instruction. */
	m95p_instruction_FQREAD = 0x6B,/**< Fast quad-output read with one dummy byte. */
	m95p_instruction_WRVR = 0x81,  /**< Write the volatile register; only BUFEN is writable. */
	m95p_instruction_WRID = 0x82,  /**< Write 1 to 512 bytes in the user identification page. */
	m95p_instruction_RDID = 0x83,  /**< Read either identification page using a 24-bit address. */
	m95p_instruction_RDVR = 0x85,  /**< Read the volatile register, with rollover. */
	m95p_instruction_FRDID = 0x8B, /**< Fast identification-page read with one dummy byte. */
	m95p_instruction_RESET = 0x99, /**< Perform software reset after RSTEN. */
	m95p_instruction_JEDID = 0x9F, /**< Read the three-byte JEDEC identification value. */
	m95p_instruction_RDPD = 0xAB,  /**< Release the device from deep power-down. */
	m95p_instruction_DPD = 0xB9,   /**< Enter deep power-down mode. */
	m95p_instruction_CHER = 0xC7,  /**< Erase the complete memory array. */
	m95p_instruction_BKER = 0xD8,  /**< Erase one addressed 64-Kbyte block. */
	m95p_instruction_PGER = 0xDB   /**< Erase one addressed 512-byte page. */
};

/**
 * @brief M95P32 status-register representation.
 * @details Maps the byte read by RDSR and written by WRSR. WIP and WEL are
 *          volatile read-only status bits. BP, TB, and SRWD are nonvolatile
 *          protection controls.
 */
union m95p_status_register_t {
	unsigned int value; /**< Complete raw status-register value. */
	/**
	 * @brief Status-register bit-field mapping.
	 * @details Maps status-register bits 0 through 7.
	 */
	struct m95p_status_register_bits {
		unsigned int WIP : 1;  /**< Bit 0: modify cycle or power-up is in progress. */
		unsigned int WEL : 1;  /**< Bit 1: write-enable latch is set. */
		unsigned int BP : 3;   /**< Bits 4:2: protected-array size selection. */
		unsigned int : 1;      /**< Bit 5: don't-care bit. */
		unsigned int TB : 1;   /**< Bit 6: protect top (0) or bottom (1) of array. */
		unsigned int SRWD : 1; /**< Bit 7: enable status-register hardware protection. */
	} bits;
};

/**
 * @brief M95P32 configuration-register representation.
 * @details Maps the first byte returned by RDCR and the optional second byte
 *          written by WRSR. LID and DRV are nonvolatile configuration fields.
 */
union m95p_configuration_register_t {
	unsigned int value; /**< Complete raw configuration-register value. */
	/**
	 * @brief Configuration-register bit-field mapping.
	 * @details Maps configuration-register bits 0 through 7.
	 */
	struct m95p_configuration_register_bits {
		unsigned int LID : 1; /**< Bit 0: permanently lock the user identification page. */
		unsigned int : 4;     /**< Bits 4:1: don't-care bits. */
		unsigned int DRV : 2; /**< Bits 6:5: output-driver strength selection. */
		unsigned int : 1;     /**< Bit 7: don't-care bit. */
	} bits;
};

/**
 * @brief M95P32 safety-register representation.
 * @details Maps the second byte returned by RDCR. All fields are volatile,
 *          read-only operation-status flags. CLRSF clears the flags; some
 *          fields are also refreshed by their associated device operation.
 */
union m95p_safety_register_t {
	unsigned int value; /**< Complete raw safety-register value. */
	/**
	 * @brief Safety-register bit-field mapping.
	 * @details Maps safety-register bits 0 through 7.
	 */
	struct m95p_safety_register_bits {
		unsigned int ECC3DS : 1; /**< Bit 0: sticky triple-bit ECC detection flag. */
		unsigned int ECC3D : 1;  /**< Bit 1: triple-bit ECC detection flag. */
		unsigned int ECC2C : 1;  /**< Bit 2: double-bit ECC correction occurred. */
		unsigned int ECC1C : 1;  /**< Bit 3: single-bit ECC correction occurred. */
		unsigned int PRF : 1;    /**< Bit 4: previous program/write operation failed. */
		unsigned int ERF : 1;    /**< Bit 5: previous erase/write operation failed. */
		unsigned int PUF : 1;    /**< Bit 6: power-up operation failed. */
		unsigned int PAMAF : 1;  /**< Bit 7: protected-array modification was attempted. */
	} bits;
};

/**
 * @brief M95P32 volatile-register representation.
 * @details Maps the byte read by RDVR and written by WRVR. BUFEN is the only
 *          writable field; BUFLD reports whether another page-program command
 *          can be loaded while buffer mode is active.
 */
union m95p_volatile_register_t {
	uint8_t value; /**< Complete raw volatile-register value. */
	/**
	 * @brief Volatile-register bit-field mapping.
	 * @details Maps volatile-register bits 0 through 7.
	 */
	struct m95p_volatile_register_bits {
		uint8_t BUFLD : 1;    /**< Bit 0: buffer is free (0) or full/unavailable (1). */
		uint8_t BUFEN : 1;    /**< Bit 1: page-program buffer mode enable. */
		uint8_t reserved : 6; /**< Bits 7:2: don't-care bits. */
	} bits;
};

/**
 * @brief Software aggregate for one RDCR response pair.
 * @details This is not a hardware register image. RDCR returns the
 *          configuration byte first and the safety byte second; the driver
 *          stores those bytes in the corresponding union members.
 */
struct m95p_configuration_safety_registers_t {
	union m95p_configuration_register_t configuration_register; /**< First RDCR byte. */
	union m95p_safety_register_t safety_register; /**< Second RDCR byte. */
};

/**
 * @brief Three-byte M95P32 identification representation used by the driver.
 * @details This software overlay represents the first three bytes read from
 *          identification-page address 0 with RDID. It is not a device
 *          register. The byte array preserves SPI receive order while the
 *          fields provide named access for device validation.
 */
union m95p_jedec_id_t {
	uint8_t data[3]; /**< Identification bytes in SPI receive order. */
	/**
	 * @brief Named identification-byte mapping.
	 * @details Maps bytes 0 through 2 returned by the driver's RDID transfer.
	 */
	struct m95p_jedec_id_fields_t {
		uint8_t manufacturer_id; /**< Byte 0; expected M95P_MANUFACTURER_ID. */
		uint8_t memory_type;     /**< Byte 1; expected M95P_FAMILY_CODE. */
		uint8_t capacity;        /**< Byte 2; expected M95P_MEMORY_DENSITY. */
	} fields;
};

/** @} */

#endif /* M95P_REGISTERS_H */
