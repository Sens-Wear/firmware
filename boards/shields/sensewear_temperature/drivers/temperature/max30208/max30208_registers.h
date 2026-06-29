/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30208_registers.h
 * @brief MAX30208 register map, field encodings, and bit-field overlays.
 *
 * @defgroup sensewear_max30208_registers SenseWear MAX30208 register map
 * @ingroup sensewear_max30208
 * @{
 *
 * This header is a pure hardware description of the Maxim MAX30208 digital
 * temperature sensor. It exposes the register addresses, the relevant field
 * values, and the register bit-field overlays the driver writes through. It
 * deliberately contains no driver state and no I/O: the higher-level @ref
 * sensewear_max30208 API maps engineering-unit configuration onto these
 * encodings.
 */

#ifndef MAX30208_REGISTERS_H_
#define MAX30208_REGISTERS_H_

#include <zephyr/sys/util.h>

/**
 * @brief Expected MAX30208 PART_ID (0xFF) value.
 * @details Read back during probing to confirm a MAX30208 is on the bus.
 */
#define MAX30208_PART_ID (0x30)

/** @brief MAX30208 7-bit I2C target address (A0/A1 strapped, see overlay). */
#define MAX30208_I2C_ADDRESS (0x50)

/**
 * @brief MAX30208 register map.
 */
enum max30208_register_type {
	/* Interrupt and status */
	max30208_register_Status = 0x00,		  /**< Status register. */
	max30208_register_InterruptEnable = 0x01, /**< Interrupt enable register. */

	/* FIFO */
	max30208_register_FifoWritePointer = 0x04, /**< FIFO write pointer. */
	max30208_register_FifoReadPointer = 0x05,  /**< FIFO read pointer. */
	max30208_register_FifoOverflow = 0x06,	   /**< FIFO overflow counter. */
	max30208_register_FifoDataCount = 0x07,	   /**< FIFO data counter. */
	max30208_register_FifoData = 0x08,		   /**< FIFO data register. */
	max30208_register_FifoConfig1 = 0x09,	   /**< FIFO almost-full threshold. */
	max30208_register_FifoConfig2 = 0x0A,	   /**< FIFO roll-over / status clear / flush. */

	/* System */
	max30208_register_SystemControl = 0x0C, /**< System control register. */

	/* Temperature alarm and conversion */
	max30208_register_AlarmHighMsb = 0x10, /**< Alarm high threshold MSB. */
	max30208_register_AlarmHighLsb = 0x11, /**< Alarm high threshold LSB. */
	max30208_register_AlarmLowMsb = 0x12,  /**< Alarm low threshold MSB. */
	max30208_register_AlarmLowLsb = 0x13,  /**< Alarm low threshold LSB. */
	max30208_register_TempSetup = 0x14,	   /**< Temperature sensor setup. */

	/* GPIO */
	max30208_register_GpioSetup = 0x20,	  /**< GPIO mode configuration. */
	max30208_register_GpioControl = 0x21, /**< GPIO logic-level control/status. */

	/* Identifiers */
	max30208_register_PartId1 = 0x31, /**< Factory unique ID byte 1. */
	max30208_register_PartId2 = 0x32, /**< Factory unique ID byte 2. */
	max30208_register_PartId3 = 0x33, /**< Factory unique ID byte 3. */
	max30208_register_PartId4 = 0x34, /**< Factory unique ID byte 4. */
	max30208_register_PartId5 = 0x35, /**< Factory unique ID byte 5. */
	max30208_register_PartId6 = 0x36, /**< Factory unique ID byte 6. */

	max30208_register_PartID = 0xFF /**< Part identifier; expected value 0x30. */
};

/** @brief FIFO depth in 16-bit temperature words. */
#define MAX30208_FIFO_DEPTH 32U

/** @brief FIFO sample size in bytes. */
#define MAX30208_FIFO_SAMPLE_BYTES 2U

/** @brief FIFO pointer and overflow counter mask, bits [4:0]. */
#define MAX30208_FIFO_PTR_MASK GENMASK(4, 0)

/** @brief FIFO data-count mask, bits [5:0]. */
#define MAX30208_FIFO_DATA_COUNT_MASK GENMASK(5, 0)

/** @brief FIFO almost-full threshold mask, bits [4:0]. */
#define MAX30208_FIFO_A_FULL_MASK GENMASK(4, 0)

/* STATUS / INTERRUPT_ENABLE common bit positions */
#define MAX30208_STATUS_TEMP_RDY BIT(0)
#define MAX30208_STATUS_TEMP_HI BIT(1)
#define MAX30208_STATUS_TEMP_LO BIT(2)
#define MAX30208_STATUS_A_FULL BIT(7)

#define MAX30208_INTERRUPT_TEMP_RDY_EN BIT(0)
#define MAX30208_INTERRUPT_TEMP_HI_EN BIT(1)
#define MAX30208_INTERRUPT_TEMP_LO_EN BIT(2)
#define MAX30208_INTERRUPT_A_FULL_EN BIT(7)

/* FIFO_CONFIG2 bits */
#define MAX30208_FIFO_CONFIG2_FIFO_RO BIT(1)
#define MAX30208_FIFO_CONFIG2_A_FULL_TYPE BIT(2)
#define MAX30208_FIFO_CONFIG2_FIFO_STAT_CLR BIT(3)
#define MAX30208_FIFO_CONFIG2_FLUSH BIT(4)

/* SYSTEM_CONTROL bits */
#define MAX30208_SYSTEM_CONTROL_RESET BIT(0)

/*
 * TEMP_SENSOR_SETUP:
 * Bits [7:6] are RFU and must be written as 1.
 * Bit 0 starts a conversion.
 */
#define MAX30208_TEMP_SETUP_RFU GENMASK(7, 6)
#define MAX30208_TEMP_SETUP_DEFAULT 0xC0
#define MAX30208_TEMP_SETUP_CONVERT_T BIT(0)
#define MAX30208_TEMP_SETUP_CONVERT (MAX30208_TEMP_SETUP_DEFAULT | MAX30208_TEMP_SETUP_CONVERT_T)

/* Alarm reset/default thresholds */
#define MAX30208_ALARM_HIGH_DEFAULT 0x7FFF
#define MAX30208_ALARM_LOW_DEFAULT 0x8000

/* GPIO_SETUP fields */
#define MAX30208_GPIO0_MODE_SHIFT 0U
#define MAX30208_GPIO0_MODE_MASK GENMASK(1, 0)

#define MAX30208_GPIO1_MODE_SHIFT 6U
#define MAX30208_GPIO1_MODE_MASK GENMASK(7, 6)

#define MAX30208_GPIO_MODE_INPUT_HIZ 0x0
#define MAX30208_GPIO_MODE_OUTPUT_OD 0x1
#define MAX30208_GPIO_MODE_INPUT_PULLDOWN 0x2

/** @brief GPIO0 special mode: active-low open-drain interrupt output. */
#define MAX30208_GPIO0_MODE_INTB 0x3

/** @brief GPIO1 special mode: active-low convert-temperature input. */
#define MAX30208_GPIO1_MODE_CONVERT_T 0x3

/** @brief GPIO_SETUP reset value: GPIO1_MODE = 0b10, GPIO0_MODE = 0b10. */
#define MAX30208_GPIO_SETUP_DEFAULT                                     \
	((MAX30208_GPIO_MODE_INPUT_PULLDOWN << MAX30208_GPIO1_MODE_SHIFT) | \
	 (MAX30208_GPIO_MODE_INPUT_PULLDOWN << MAX30208_GPIO0_MODE_SHIFT))

/* GPIO_CONTROL bits */
#define MAX30208_GPIO_CONTROL_GPIO0_LL BIT(0)
#define MAX30208_GPIO_CONTROL_GPIO1_LL BIT(3)

/* Unique factory ID */
#define MAX30208_UNIQUE_ID_START_REGISTER max30208_register_PartId1
#define MAX30208_UNIQUE_ID_NUM_BYTES 6U

/** @brief Temperature LSB weight in milli-degrees Celsius (0.005 degC). */
#define MAX30208_TEMP_LSB_MDEG_C (5)

/**
 * @brief Status register overlay (max30208_register_Status).
 *
 * @note Datasheet bit order:
 *       bit 0 TEMP_RDY, bit 1 TEMP_HI, bit 2 TEMP_LO, bit 7 A_FULL.
 */
union max30208_status_register_t {
	uint8_t value;
	struct max30208_status_register_bits {
		unsigned int temp_ready : 1; /**< Bit 0: temperature conversion ready. */
		unsigned int temp_hi : 1;	 /**< Bit 1: high threshold crossed. */
		unsigned int temp_lo : 1;	 /**< Bit 2: low threshold crossed. */
		unsigned int reserved : 4;	 /**< Bits 3-6: reserved. */
		unsigned int a_full : 1;	 /**< Bit 7: FIFO almost full. */
	} bits;
};

/**
 * @brief Interrupt enable register overlay (0x01).
 */
union max30208_interrupt_enable_register_t {
	uint8_t value;
	struct max30208_interrupt_enable_register_bits {
		unsigned int temp_ready_en : 1; /**< Bit 0: enable TEMP_RDY interrupt. */
		unsigned int temp_hi_en : 1;	/**< Bit 1: enable TEMP_HI interrupt. */
		unsigned int temp_lo_en : 1;	/**< Bit 2: enable TEMP_LO interrupt. */
		unsigned int reserved : 4;		/**< Bits 3-6: reserved. */
		unsigned int a_full_en : 1;		/**< Bit 7: enable A_FULL interrupt. */
	} bits;
};

/**
 * @brief FIFO pointer register overlay for FIFO_WR_PTR and FIFO_RD_PTR.
 */
union max30208_fifo_pointer_register_t {
	uint8_t value;
	struct max30208_fifo_pointer_register_bits {
		unsigned int ptr : 5;	   /**< Bits 0-4: FIFO pointer. */
		unsigned int reserved : 3; /**< Bits 5-7: reserved. */
	} bits;
};

/**
 * @brief FIFO overflow counter register overlay (0x06).
 */
union max30208_fifo_overflow_register_t {
	uint8_t value;
	struct max30208_fifo_overflow_register_bits {
		unsigned int count : 5;	   /**< Bits 0-4: overflow counter. */
		unsigned int reserved : 3; /**< Bits 5-7: reserved. */
	} bits;
};

/**
 * @brief FIFO data-count register overlay (0x07).
 */
union max30208_fifo_data_count_register_t {
	uint8_t value;
	struct max30208_fifo_data_count_register_bits {
		unsigned int count : 6;	   /**< Bits 0-5: number of FIFO words available. */
		unsigned int reserved : 2; /**< Bits 6-7: reserved. */
	} bits;
};

/**
 * @brief FIFO configuration 1 register overlay (0x09).
 */
union max30208_fifo_config1_register_t {
	uint8_t value;
	struct max30208_fifo_config1_register_bits {
		unsigned int a_full : 5;   /**< Bits 0-4: almost-full threshold. */
		unsigned int reserved : 3; /**< Bits 5-7: reserved. */
	} bits;
};

/**
 * @brief FIFO configuration 2 register overlay (0x0A).
 */
union max30208_fifo_config2_register_t {
	uint8_t value;
	struct max30208_fifo_config2_register_bits {
		unsigned int reserved0 : 1;		/**< Bit 0: reserved. */
		unsigned int fifo_ro : 1;		/**< Bit 1: FIFO rollover enable. */
		unsigned int a_full_type : 1;	/**< Bit 2: A_FULL behavior. */
		unsigned int fifo_stat_clr : 1; /**< Bit 3: clear status on FIFO read. */
		unsigned int flush_fifo : 1;	/**< Bit 4: flush FIFO, self-clearing. */
		unsigned int reserved1 : 3;		/**< Bits 5-7: reserved. */
	} bits;
};

/**
 * @brief System control register overlay (0x0C).
 */
union max30208_system_control_register_t {
	uint8_t value;
	struct max30208_system_control_register_bits {
		unsigned int reset : 1;	   /**< Bit 0: reset device registers, self-clearing. */
		unsigned int reserved : 7; /**< Bits 1-7: reserved. */
	} bits;
};

/**
 * @brief Temperature sensor setup register overlay (0x14).
 */
union max30208_temp_setup_register_t {
	uint8_t value;
	struct max30208_temp_setup_register_bits {
		unsigned int convert_t : 1; /**< Bit 0: start one temperature conversion. */
		unsigned int reserved : 5;	/**< Bits 1-5: reserved. */
		unsigned int rfu : 2;		/**< Bits 6-7: write as 1. */
	} bits;
};

/**
 * @brief GPIO setup register overlay (0x20).
 */
union max30208_gpio_setup_register_t {
	uint8_t value;
	struct max30208_gpio_setup_register_bits {
		unsigned int gpio0_mode : 2; /**< Bits 0-1: GPIO0 mode. */
		unsigned int reserved : 4;	 /**< Bits 2-5: reserved. */
		unsigned int gpio1_mode : 2; /**< Bits 6-7: GPIO1 mode. */
	} bits;
};

/**
 * @brief GPIO control register overlay (0x21).
 */
union max30208_gpio_control_register_t {
	uint8_t value;
	struct max30208_gpio_control_register_bits {
		unsigned int gpio0_ll : 1;	/**< Bit 0: GPIO0 logic level. */
		unsigned int reserved0 : 2; /**< Bits 1-2: reserved. */
		unsigned int gpio1_ll : 1;	/**< Bit 3: GPIO1 logic level. */
		unsigned int reserved1 : 4; /**< Bits 4-7: reserved. */
	} bits;
};

/** @} */

#endif /* MAX30208_REGISTERS_H_ */
