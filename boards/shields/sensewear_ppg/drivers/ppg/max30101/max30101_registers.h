/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30101_registers.h
 * @brief MAX30101 register map, field encodings, and bit-field overlays.
 *
 * @defgroup sensewear_max30101_registers SenseWear MAX30101 register map
 * @ingroup sensewear_max30101
 * @{
 *
 * This header is a pure hardware description of the Maxim MAX30101 pulse-oximeter
 * and heart-rate optical sensor. It exposes the register addresses, the
 * enumerated field selectors (sample rate, ADC range, pulse width, ...), and the
 * register bit-field overlays the driver writes through. It deliberately contains
 * no driver state and no I/O: the higher-level @ref sensewear_max30101 API maps
 * engineering-unit configuration onto these encodings.
 */

#ifndef MAX30101_REGISTERS_H_
#define MAX30101_REGISTERS_H_

/**
 * @brief Expected MAX30101 PART_ID (0xFF) value.
 * @details Read back during probing to confirm a MAX30101 is on the bus.
 */
#define MAX30101_PART_ID (0x15)

/** @brief MAX30101 7-bit I2C target address. */
#define MAX30101_I2C_ADDRESS (0x57)

/**
 * @brief MAX30101 register map.
 */
enum max30101_register_type {
	max30101_register_InterruptStatus1 = 0x00,	 /**< Interrupt status register 1. */
	max30101_register_InterruptStatus2 = 0x01,	 /**< Interrupt status register 2. */
	max30101_register_InterruptEnable1 = 0x02,	 /**< Interrupt enable register 1. */
	max30101_register_InterruptEnable2 = 0x03,	 /**< Interrupt enable register 2. */
	max30101_register_FIFO_WritePointer = 0x04,	 /**< FIFO write pointer. */
	max30101_register_OverflowCounter = 0x05,	 /**< FIFO overflow counter. */
	max30101_register_FIFO_ReadPointer = 0x06,	 /**< FIFO read pointer. */
	max30101_register_FIFO_DataRegister = 0x07,	 /**< FIFO data register. */
	max30101_register_FIFO_Configuration = 0x08, /**< FIFO configuration. */
	max30101_register_ModeConfiguration = 0x09,	 /**< Mode configuration. */
	max30101_register_SpO2Configuration = 0x0A,	 /**< SpO2 configuration. */
	max30101_register_LED1_PA = 0x0C,			 /**< LED1 (Red) pulse amplitude. */
	max30101_register_LED2_PA = 0x0D,			 /**< LED2 (IR) pulse amplitude. */
	max30101_register_LED3_PA = 0x0E,			 /**< LED3 (Green) pulse amplitude. */
	max30101_register_ProxModeLED_PA = 0x10,	 /**< Proximity-mode LED pulse amplitude. */
	max30101_register_ModeControlReg1 = 0x11,	 /**< Multi-LED mode control (slots 1-2). */
	max30101_register_ModeControlReg2 = 0x12,	 /**< Multi-LED mode control (slots 3-4). */
	max30101_register_DieTempInt = 0x1F,		 /**< Die temperature integer part. */
	max30101_register_DieTempFrac = 0x20,		 /**< Die temperature fractional part. */
	max30101_register_DieTempConfig = 0x21,		 /**< Die temperature configuration. */
	max30101_register_ProxIntThreshold = 0x30,	 /**< Proximity interrupt threshold. */
	max30101_register_RevID = 0xFE,				 /**< Revision ID. */
	max30101_register_PartID = 0xFF				 /**< Part ID (see ::MAX30101_PART_ID). */
};

/**
 * @brief MAX30101 operational modes (ModeConfiguration.mode).
 */
enum max30101_operation_mode_type {
	max30101_mode_HeartRate = 2, /**< Heart-rate mode (Red only, 1 channel). */
	max30101_mode_SpO2 = 3,		 /**< SpO2 mode (Red + IR, 2 channels). */
	max30101_mode_MultiLed = 7	 /**< Multi-LED mode (programmable slots). */
};

/**
 * @brief Number of samples averaged per FIFO sample (FIFO_Configuration.sample_average).
 */
enum max30101_averaged_samples_type {
	max30101_averaged_samples_1 = 0,  /**< No averaging. */
	max30101_averaged_samples_2 = 1,  /**< Average 2 samples. */
	max30101_averaged_samples_4 = 2,  /**< Average 4 samples. */
	max30101_averaged_samples_8 = 3,  /**< Average 8 samples. */
	max30101_averaged_samples_16 = 4, /**< Average 16 samples. */
	max30101_averaged_samples_32 = 5  /**< Average 32 samples. */
};

/**
 * @brief ADC full-scale range (SpO2Configuration.spo2_adc_range).
 */
enum max30101_adc_range_type {
	max30101_adc_range_2048 = 0,  /**< 2048 nA full scale. */
	max30101_adc_range_4096 = 1,  /**< 4096 nA full scale. */
	max30101_adc_range_8192 = 2,  /**< 8192 nA full scale. */
	max30101_adc_range_16384 = 3  /**< 16384 nA full scale. */
};

/**
 * @brief LED pulse width / ADC resolution (SpO2Configuration.led_pw).
 */
enum max30101_led_pulsewidth_type {
	max30101_led_pulsewidth_69 = 0,	 /**< 68.95 us, 15-bit resolution. */
	max30101_led_pulsewidth_118 = 1, /**< 117.78 us, 16-bit resolution. */
	max30101_led_pulsewidth_215 = 2, /**< 215.44 us, 17-bit resolution. */
	max30101_led_pulsewidth_411 = 3	 /**< 410.75 us, 18-bit resolution. */
};

/**
 * @brief Sample rate (SpO2Configuration.spo2_sr).
 */
enum max30101_sample_rate_type {
	max30101_sample_rate_50Hz = 0,	 /**< 50 samples per second. */
	max30101_sample_rate_100Hz = 1,	 /**< 100 samples per second. */
	max30101_sample_rate_200Hz = 2,	 /**< 200 samples per second. */
	max30101_sample_rate_400Hz = 3,	 /**< 400 samples per second. */
	max30101_sample_rate_800Hz = 4,	 /**< 800 samples per second. */
	max30101_sample_rate_1000Hz = 5, /**< 1000 samples per second. */
	max30101_sample_rate_1600Hz = 6, /**< 1600 samples per second. */
	max30101_sample_rate_3200Hz = 7	 /**< 3200 samples per second. */
};

/**
 * @brief Interrupt source masks across the two interrupt status/enable registers.
 * @details The low byte corresponds to InterruptStatus1 / InterruptEnable1 and
 *          the high byte to InterruptStatus2 / InterruptEnable2.
 */
enum max30101_interrupt_type {
	max30101_interrupt_PowerReady = 0x0001,				 /**< Power-ready (PWR_RDY). */
	max30101_interrupt_Proximity = 0x0010,				 /**< Proximity threshold (PROX_INT). */
	max30101_interrupt_AmbientLightCancelOverflow = 0x0020, /**< ALC overflow (ALC_OVF). */
	max30101_interrupt_FifoDataReady = 0x0040,			 /**< New FIFO sample (PPG_RDY). */
	max30101_interrupt_FifoAlmostFull = 0x0080,			 /**< FIFO almost full (A_FULL). */
	max30101_interrupt_DieTempReady = 0x0200			 /**< Die temperature ready (DIE_TEMP_RDY). */
};

/**
 * @brief MAX30101 LED channel identifiers used in multi-LED slot programming.
 */
enum max30101_led_type {
	max30101_led_Red = 1,	/**< Red LED (LED1). */
	max30101_led_IR = 2,	/**< IR LED (LED2). */
	max30101_led_Green = 3	/**< Green LED (LED3). */
};

/**
 * @brief Interrupt status overlay (InterruptStatus1 + InterruptStatus2).
 */
union max30101_interrupt_status_t {
	int value;
	struct max30101_interrupt_status_bits {
		unsigned int pwr_rdy : 1;		 /**< Bit 0: power ready. */
		unsigned int reserved_1 : 3;	 /**< Bits 1-3: reserved. */
		unsigned int prox_int : 1;		 /**< Bit 4: proximity. */
		unsigned int alc_ovf : 1;		 /**< Bit 5: ambient-light-cancel overflow. */
		unsigned int ppg_rdy : 1;		 /**< Bit 6: new FIFO data ready. */
		unsigned int a_full : 1;		 /**< Bit 7: FIFO almost full. */
		unsigned int reserved_2 : 1;	 /**< Bit 8: reserved. */
		unsigned int die_tamp_ready : 1; /**< Bit 9: die temperature ready. */
		unsigned int reserved_3 : 6;	 /**< Bits 10-15: reserved. */
	} bits;
};

/**
 * @brief Interrupt enable overlay (InterruptEnable1 + InterruptEnable2).
 */
union max30101_interrupt_enable_t {
	int value;
	struct max30101_interrupt_enable_bits {
		unsigned int reserved_1 : 4;	 /**< Bits 0-3: reserved. */
		unsigned int prox_int : 1;		 /**< Bit 4: enable proximity interrupt. */
		unsigned int alc_ovf : 1;		 /**< Bit 5: enable ALC overflow interrupt. */
		unsigned int ppg_rdy : 1;		 /**< Bit 6: enable new-data interrupt. */
		unsigned int a_full : 1;		 /**< Bit 7: enable FIFO-almost-full interrupt. */
		unsigned int reserved_2 : 1;	 /**< Bit 8: reserved. */
		unsigned int die_temp_ready : 1; /**< Bit 9: enable die-temperature-ready interrupt. */
		unsigned int reserved_3 : 6;	 /**< Bits 10-15: reserved. */
	} bits;
};

/**
 * @brief FIFO configuration overlay (FIFO_Configuration).
 */
union max30101_fifo_configuration_t {
	int value;
	struct max30101_fifo_configuration_bits {
		unsigned int fifo_a_full : 4;	   /**< Almost-full threshold (empty slots). */
		unsigned int fifo_roll_over_en : 1; /**< FIFO roll-over enable. */
		unsigned int sample_average : 3;   /**< ::max30101_averaged_samples_type. */
	} bits;
};

/**
 * @brief Mode configuration overlay (ModeConfiguration).
 */
union max30101_mode_configuration_t {
	int value;
	struct max30101_mode_configuration_bits {
		unsigned int mode : 3;		/**< ::max30101_operation_mode_type. */
		unsigned int reserved : 3;	/**< Reserved. */
		unsigned int reset : 1;		/**< Software reset request. */
		unsigned int shdn : 1;		/**< Shutdown control. */
	} bits;
};

/**
 * @brief SpO2 configuration overlay (SpO2Configuration).
 */
union max30101_spo2_configuration_t {
	int value;
	struct max30101_spO2_configuration_bits {
		unsigned int led_pw : 2;		/**< ::max30101_led_pulsewidth_type. */
		unsigned int spo2_sr : 3;		/**< ::max30101_sample_rate_type. */
		unsigned int spo2_adc_range : 2; /**< ::max30101_adc_range_type. */
		unsigned int reserved : 1;		/**< Reserved. */
	} bits;
};

/**
 * @brief Multi-LED mode control overlay (ModeControlReg1 + ModeControlReg2).
 */
union max30101_multi_led_mode_control_t {
	int value;
	struct max30101_mode_control_bits {
		unsigned int slot1 : 3;		/**< Time slot 1 LED (::max30101_led_type). */
		unsigned int reserved1 : 1; /**< Reserved. */
		unsigned int slot2 : 3;		/**< Time slot 2 LED. */
		unsigned int reserved2 : 1; /**< Reserved. */
		unsigned int slot3 : 3;		/**< Time slot 3 LED. */
		unsigned int reserved3 : 1; /**< Reserved. */
		unsigned int slot4 : 3;		/**< Time slot 4 LED. */
		unsigned int reserved4 : 1; /**< Reserved. */
	} bits;
};

/**
 * @brief Die-temperature configuration overlay (DieTempConfig).
 */
union max30101_die_temperature_config_t {
	int value;
	struct max30101_die_temperature_config_bits {
		unsigned int temp_en : 1;	/**< Start a single die-temperature conversion. */
		unsigned int reserved : 7;	/**< Reserved. */
	} bits;
};

/** @} */

#endif /* MAX30101_REGISTERS_H_ */
