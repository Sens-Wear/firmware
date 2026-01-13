/*
 * m95p.h
 *
 *  Created on: Feb 17, 2025
 *      Author: husey
 */

#ifndef M95P_H_
#define M95P_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "m95p_organization.h"
#include <zephyr/drivers/gpio.h>

#define M95P_CS_GPIO_NODE DT_NODELABEL(gpio2)
#define M95P_CS_PIN (7)

#define M95P_MANUFACTURER_ID (0x20)
#define M95P_FAMILY_CODE (0x00)
#define M95P_MEMORY_DENSITY (0x16)
#define M95P_SPI_TIMEOUT (100)

enum m95_instruction_type {
	m95p_instruction_WRSR = 0x01, // Write status register
	m95p_instruction_PGWR = 0x02, // Page write (erase and program)
	m95p_instruction_READ = 0x03, // Read data
	m95p_instruction_WRDI = 0x04, // Write disable
	m95p_instruction_RDSR = 0x05, // Read status register
	m95p_instruction_WREN = 0x06, // Write enable
	m95p_instruction_FREAD =
		0x0B,					  // Fast read single output with one dummy byte
	m95p_instruction_PGPR = 0x0A, // Page program
	m95p_instruction_RDCR = 0x15, // Read configuration and safety register
	m95p_instruction_SCER = 0x20, // Sector erase (4 Kbytes)
	m95p_instruction_CLRSF = 0x50,	// Clear safety sticky flags
	m95p_instruction_RDSFDP = 0x5A, // Read SFDP
	m95p_instruction_RSTEN = 0x66,	// Enable reset
	m95p_instruction_FQREAD = 0x6B, // Fast read quad output with one dummy byte
	m95p_instruction_RESET = 0x99,	// Software reset
	m95p_instruction_RDPD = 0xAB,	// Deep power-down release
	m95p_instruction_DPD = 0xB9,	// Deep power-down enter
	m95p_instruction_CHER = 0xC7,	// Chip erase
	m95p_instruction_RDVR = 0x85,	// Read volatile register
	m95p_instruction_WRVR = 0x81,	// Write volatile register
	m95p_instruction_WRID = 0x82,	// Write identification page (EE)
	m95p_instruction_RDID = 0x83,	// Read identification (EE)
	m95p_instruction_FRDID = 0x8B,	// Fast read identification (EE)
	m95p_instruction_JEDID = 0x9F,	// JEDEC identification (SF)
	m95p_instruction_BKER = 0xD8,	// Block erase (64 Kbytes)
	m95p_instruction_PGER = 0xDB	// Page erase (512 bytes)
};

union m95p_status_register_t {
	unsigned int value;
	struct m95p_status_register_bits {
		unsigned int WIP : 1;  // Write in progress
		unsigned int WEL : 1;  // Write enable latch
		unsigned int BP : 3;  // Block protect bits
        unsigned int : 1; // Don't care bit
        unsigned int TB : 1; // Top/Bottom protection bit
		unsigned int SRWD : 1; // Status register write disable
	} bits;
};

union m95p_configuration_register_t {
	unsigned int value;
	struct m95p_configuration_register_bits {
		unsigned int LID : 1; // Lock identification page
		unsigned int  : 4;
		unsigned int DRV : 2; // Output driver strength bit 0
		unsigned int : 1;
	} bits;
};

union m95p_safety_register_t {
	unsigned int value;
	struct m95p_safety_register_bits {
		unsigned int ECC3DS : 1; // ECC3DS
		unsigned int ECC3D : 1;	// ECC3D
		unsigned int ECC2C : 1;	// ECC2C
		unsigned int ECC1C : 1;	// ECC1C
		unsigned int PRF : 1;	// PRF
		unsigned int ERF : 1;	// ERF
		unsigned int PUF : 1;	// PUF
		unsigned int PAMAF : 1;	// Protected array modify attempt flag
	} bits;
};

union m95p_volatile_register_t {
	uint8_t value;
	struct m95p_volatile_register_bits {
		uint8_t BUFLD : 1;	  // Buffer loading status
		uint8_t BUFEN : 1;	  // Buffer loading activation
		uint8_t reserved : 6; // Don't care bits
	} bits;
};

struct m95p_configuration_safety_registers_t {
	union m95p_configuration_register_t configuration_register;
	union m95p_safety_register_t safety_register;
};

union m95p_jedec_id_t {
	uint8_t data[3];
	struct m95p_jedec_id_fields_t {
		uint8_t manufacturer_id;
		uint8_t memory_type;
		uint8_t capacity;
	} fields;
};

union m95p_jedec_id_t m95p_get_jedec_id(void);



#endif /* M95P_H_ */
