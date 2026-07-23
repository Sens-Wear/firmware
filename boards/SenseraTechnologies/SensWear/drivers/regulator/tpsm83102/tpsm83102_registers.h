/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file tpsm83102_registers.h
 * @brief Register declarations for the TPSM83102 register map.
 */
#ifndef TPSM83102_REGISTERS_H_
#define TPSM83102_REGISTERS_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

/** TPSM83102 register-address map. */
enum tpsm83102_register_type {
	tpsm83102_register_CONTROL1 = 0x02, /**< Converter and protection controls. */
	tpsm83102_register_VOUT = 0x03,     /**< Output-voltage selection. */
	tpsm83102_register_CONTROL2 = 0x05, /**< Ramp, discharge, and PWM controls. */
};

/** CONTROL1.CONVERTER_EN mask, register 0x02 bit 0. */
#define TPSM83102_CONTROL1_CONVERTER_EN BIT(0)
/** CONTROL1.EN_SCP mask, register 0x02 bit 2. */
#define TPSM83102_CONTROL1_EN_SCP BIT(2)
/** CONTROL1.EN_FAST_DVS mask, register 0x02 bit 3. */
#define TPSM83102_CONTROL1_EN_FAST_DVS BIT(3)
/** CONTROL1 bits that must read as zero. */
#define TPSM83102_CONTROL1_RESERVED_MASK (0xF2u)

/** CONTROL2.TD_RAMP mask, register 0x05 bits 2:0. */
#define TPSM83102_CONTROL2_TD_RAMP_MASK (0x07u)
/** CONTROL2.CL_RAMP_MIN mask, register 0x05 bit 3. */
#define TPSM83102_CONTROL2_CL_RAMP_MIN BIT(3)
/** CONTROL2.EN_DISCH_VOUT mask, register 0x05 bits 5:4. */
#define TPSM83102_CONTROL2_DISCH_MASK (BIT(4) | BIT(5))
/** CONTROL2.EN_DISCH_VOUT field position. */
#define TPSM83102_CONTROL2_DISCH_POS (4u)
/** CONTROL2.FAST_RAMP_EN mask, register 0x05 bit 6. */
#define TPSM83102_CONTROL2_FAST_RAMP_EN BIT(6)
/** CONTROL2.FPWM mask, register 0x05 bit 7. */
#define TPSM83102_CONTROL2_FPWM BIT(7)

/**
 * @brief TPSM83102 CONTROL1 register representation.
 * @details Maps register 0x02. The bit fields mirror the hardware register;
 *          @ref value provides access to the complete byte transferred over
 *          I2C.
 */
union tpsm83102_CONTROL1_register_t {
	uint8_t value; /**< Complete raw CONTROL1 register value. */
	/**
	 * @brief CONTROL1 bit-field mapping.
	 * @details Maps fields in register 0x02 from bit 0 through bit 7.
	 */
	struct tpsm83102_CONTROL1_register_bits {
		uint8_t bConverterEnable : 1; /**< Bit 0: software converter enable. */
		uint8_t _reserved1 : 1;       /**< Bit 1: reserved; reads as zero. */
		uint8_t bEnableSCP : 1;       /**< Bit 2: short-circuit hiccup protection. */
		uint8_t bEnableFastDVS : 1;   /**< Bit 3: fast dynamic-voltage scaling. */
		uint8_t _reserved4_7 : 4;     /**< Bits 7:4: reserved; read as zero. */
	} bits;
};

/**
 * @brief TPSM83102 VOUT register representation.
 * @details Maps register 0x03. Codes 0x00 through 0xB4 select 1.0 V through
 *          5.5 V in 25-mV steps. Codes 0xB5 through 0xFF clamp the output
 *          selection to 5.5 V.
 */
union tpsm83102_VOUT_register_t {
	uint8_t value; /**< Complete raw VOUT code. */
	/**
	 * @brief VOUT bit-field mapping.
	 * @details Maps the output-voltage field in register 0x03.
	 */
	struct tpsm83102_VOUT_register_bits {
		uint8_t bOutputVoltage : 8; /**< Bits 7:0: output-voltage code. */
	} bits;
};

/**
 * @brief TPSM83102 CONTROL2 register representation.
 * @details Maps register 0x05 and controls startup ramp timing, output
 *          discharge strength, fast ramp behavior, and forced-PWM mode.
 */
union tpsm83102_CONTROL2_register_t {
	uint8_t value; /**< Complete raw CONTROL2 register value. */
	/**
	 * @brief CONTROL2 bit-field mapping.
	 * @details Maps fields in register 0x05 from bit 0 through bit 7.
	 */
	struct tpsm83102_CONTROL2_register_bits {
		uint8_t bRampTime : 3;          /**< Bits 2:0: soft-start ramp selector. */
		uint8_t bCurrentLimitHigh : 1;  /**< Bit 3: high minimum ramp current limit. */
		uint8_t bDischargeMode : 2;     /**< Bits 5:4: output-discharge strength. */
		uint8_t bFastRampEnable : 1;    /**< Bit 6: fast startup ramp enable. */
		uint8_t bForcedPWM : 1;         /**< Bit 7: forced-PWM operation enable. */
	} bits;
};

/** Lowest programmable output voltage in microvolts. */
#define TPSM83102_VOUT_MIN_UV (1000000u)
/** Highest programmable output voltage in microvolts. */
#define TPSM83102_VOUT_MAX_UV (5500000u)
/** Output-voltage step for linear VOUT codes in microvolts. */
#define TPSM83102_VOUT_STEP_UV (25000u)
/** Highest VOUT code in the 25-mV linear range. */
#define TPSM83102_VOUT_CODE_MAX_LINEAR (0xB4u)
/** First VOUT code clamped to 5.5 V. */
#define TPSM83102_VOUT_CODE_CLAMP_5V5 (0xB5u)

/** CONTROL1 power-on reset value. */
#define TPSM83102_CONTROL1_RESET (0x08u)
/** VOUT power-on reset value. */
#define TPSM83102_VOUT_RESET (0x5Cu)
/** CONTROL2 power-on reset value. */
#define TPSM83102_CONTROL2_RESET (0x45u)

#endif /* TPSM83102_REGISTERS_H_ */
