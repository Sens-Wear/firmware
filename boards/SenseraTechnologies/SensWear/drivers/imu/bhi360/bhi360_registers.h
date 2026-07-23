/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bhi360_registers.h
 * @brief Host-interface register and status-bit declarations for the BHI360.
 *
 * @defgroup senswear_bhi360 SensWear BHI360 smart IMU
 * @ingroup io_interfaces
 * @{
 *
 * This header provides SensWear-side, byte-level representations of the BHI360
 * host-interface status registers. The canonical register addresses and bit
 * masks are owned by the Bosch BHY2 sensor API (@c bhy2_defs.h); the unions
 * below mirror that hardware layout in a form convenient for decoding and
 * event dispatch. They are decode aids only and perform no I/O.
 */
#ifndef BHI360_REGISTERS_H_
#define BHI360_REGISTERS_H_

#include <stdint.h>

#include "bhy2_defs.h"

/** Host-interrupt status register address (BHY2 reg 0x2D). */
#define BHI360_REG_INT_STATUS (BHY2_REG_INT_STATUS)
/** Boot status register address (BHY2 reg 0x25). */
#define BHI360_REG_BOOT_STATUS (BHY2_REG_BOOT_STATUS)

/**
 * @brief FIFO host-interrupt cause encoded in the 2-bit FIFO_W / FIFO_NW fields.
 * @details Both the wake-up and non-wake-up FIFO fields of INT_STATUS use this
 *          same 2-bit encoding.
 */
enum bhi360_fifo_int_cause {
	bhi360_fifo_int_None = 0,       /**< No FIFO interrupt for this channel. */
	bhi360_fifo_int_DataReady = 1,  /**< New data is ready (immediate). */
	bhi360_fifo_int_Latency = 2,    /**< Report latency / max-latency elapsed. */
	bhi360_fifo_int_Watermark = 3,  /**< FIFO watermark level reached. */
};

/**
 * @brief BHI360 host-interrupt status register (INT_STATUS, 0x2D).
 * @details Mirrors the byte read by bhy2_get_interrupt_status(). The host reads
 *          this register when the MPU_IRQ line asserts to learn why.
 */
union bhi360_int_status_register_t {
	uint8_t value; /**< Complete raw INT_STATUS byte. */
	/** @brief INT_STATUS bit-field mapping. */
	struct bhi360_int_status_register_bits {
		uint8_t bAsserted : 1;      /**< Bit 0: host interrupt asserted. */
		uint8_t bWakeupFifo : 2;    /**< Bits 2:1: wake-up FIFO cause (::bhi360_fifo_int_cause). */
		uint8_t bNonWakeupFifo : 2; /**< Bits 4:3: non-wake-up FIFO cause (::bhi360_fifo_int_cause). */
		uint8_t bStatus : 1;        /**< Bit 5: status/parameter data available. */
		uint8_t bDebug : 1;         /**< Bit 6: async debug data available. */
		uint8_t bResetOrFault : 1;  /**< Bit 7: device reset or fault occurred. */
	} bits;
};

/**
 * @brief BHI360 boot status register (BOOT_STATUS, 0x25).
 * @details Mirrors the byte read by bhy2_get_boot_status(); reported during
 *          firmware bring-up. Retained here for diagnostics and completeness.
 */
union bhi360_boot_status_register_t {
	uint8_t value; /**< Complete raw BOOT_STATUS byte. */
	/** @brief BOOT_STATUS bit-field mapping. */
	struct bhi360_boot_status_register_bits {
		uint8_t bFlashDetected : 1;      /**< Bit 0: external flash detected. */
		uint8_t bFlashVerifyDone : 1;    /**< Bit 1: flash firmware verification done. */
		uint8_t bFlashVerifyError : 1;   /**< Bit 2: flash firmware verification error. */
		uint8_t bNoFlash : 1;            /**< Bit 3: no external flash present. */
		uint8_t bHostInterfaceReady : 1; /**< Bit 4: host interface ready for upload. */
		uint8_t bHostFwVerifyDone : 1;   /**< Bit 5: host-supplied firmware verified. */
		uint8_t bHostFwVerifyError : 1;  /**< Bit 6: host firmware verification error. */
		uint8_t bHostFwIdle : 1;         /**< Bit 7: firmware halted / idle. */
	} bits;
};

/** @} */

#endif /* BHI360_REGISTERS_H_ */
