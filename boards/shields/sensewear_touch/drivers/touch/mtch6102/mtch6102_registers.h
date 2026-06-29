/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mtch6102_registers.h
 * @brief MTCH6102 register map and register overlays.
 *
 * @defgroup sensewear_mtch6102_registers SenseWear MTCH6102 register map
 * @ingroup sensewear_mtch6102
 * @{
 *
 * This header is a pure hardware description of the Microchip MTCH6102 touch
 * controller. It exposes the core, touch, compensation, acquisition, and
 * configuration register maps together with the bit-field overlays used by the
 * SenseWear touch driver.
 */

#ifndef SENSWEAR_MTCH6102_REGISTERS_H_
#define SENSWEAR_MTCH6102_REGISTERS_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Decoded touch position and raw status snapshot.
 */
struct mtch6102_position {
	bool touched;		/**< true when a touch is currently present. */
	uint16_t x;		/**< Reconstructed X coordinate. */
	uint16_t y;		/**< Reconstructed Y coordinate. */
	uint8_t touch_state;	/**< Raw TOUCHSTATE register value. */
};

/**
 * @brief MTCH6102 core RAM register map.
 */
enum mtch6102_core_register_type {
	mtch6102_core_FWMajor = 0x00, /**< Firmware major version. */
	mtch6102_core_FWMinor = 0x01, /**< Firmware minor version. */
	mtch6102_core_APPIDH = 0x02,  /**< Application ID high byte. */
	mtch6102_core_APPIDL = 0x03,  /**< Application ID low byte. */
	mtch6102_core_CMD = 0x04,     /**< Command register. */
	mtch6102_core_MODE = 0x05,    /**< Touch decode mode register. */
	mtch6102_core_MODECON = 0x06, /**< Raw-ADC mode control register. */
};
typedef enum mtch6102_core_register_type MTCH6102_Core_Ram_Memory;

/**
 * @brief MTCH6102 CMD register overlay.
 * @details Bits are written with a value of 1 to request an action. The device
 *          clears them after the command completes.
 */
union mtch6102_cmd_register_t {
	unsigned int value; /**< Complete raw register value. */
	struct mtch6102_cmd_register_bits {
		unsigned int bs : 1;		/**< Force baseline recalibration. */
		unsigned int reserved_1 : 2;	/**< Reserved; read as zero. */
		unsigned int mfg : 1;		/**< Execute manufacturing test. */
		unsigned int reserved_2 : 1;	/**< Reserved; read as zero. */
		unsigned int cfg : 1;		/**< Apply configuration changes. */
		unsigned int def : 1;		/**< Restore default configuration values. */
		unsigned int nv : 1;		/**< Write configuration to NVRAM. */
	} bits;
};

/**
 * @brief MTCH6102 touch decode mode register overlay.
 */
union mtch6102_mode_register_t {
	unsigned int value; /**< Complete raw register value. */
	struct mtch6102_mode_register_bits {
		unsigned int mode : 4;	   /**< Touch decode mode selection. */
		unsigned int reserved : 4; /**< Reserved; read as zero. */
	} bits;
};

/**
 * @brief MTCH6102 raw-ADC mode control register overlay.
 */
union mtch6102_modecon_register_t {
	unsigned int value; /**< Complete raw register value. */
	struct mtch6102_modecon_register_bits {
		unsigned int channel : 4; /**< RX channel selection. */
		unsigned int type : 4;	   /**< Raw conversion type selection. */
	} bits;
};

/**
 * @brief MTCH6102 touch state register overlay.
 */
union mtch6102_touchstate_register_t {
	unsigned int value; /**< Complete raw register value. */
	struct mtch6102_touchstate_register_bits {
		unsigned int tch : 1;		/**< Touch is present. */
		unsigned int ges : 1;		/**< Gesture is present. */
		unsigned int lrg : 1;		/**< Large activation is present. */
		unsigned int reserved : 1;	/**< Reserved; read as zero. */
		unsigned int frame : 4;	/**< Frame counter. */
	} bits;
};

/**
 * @brief MTCH6102 touch RAM register map.
 */
enum mtch6102_touch_register_type {
	mtch6102_touch_TOUCHSTATE = 0x10,  /**< Current touch state. */
	mtch6102_touch_TOUCHX = 0x11,      /**< Touch X MSB byte. */
	mtch6102_touch_TOUCHY = 0x12,      /**< Touch Y MSB byte. */
	mtch6102_touch_TOUCHLSB = 0x13,    /**< Packed X/Y low nibble byte. */
	mtch6102_touch_GESTURESTATE = 0x14, /**< Current gesture code. */
	mtch6102_touch_GESTUREDIAG = 0x15,  /**< Gesture diagnostics. */
};
typedef enum mtch6102_touch_register_type MTCH6102_Touch_Ram_Memory;

/**
 * @brief MTCH6102 compensation RAM register map.
 */
enum mtch6102_compensation_register_type {
	mtch6102_compensation_SENSORCOMP_RX0 = 0x50,
	mtch6102_compensation_SENSORCOMP_RX1,
	mtch6102_compensation_SENSORCOMP_RX2,
	mtch6102_compensation_SENSORCOMP_RX3,
	mtch6102_compensation_SENSORCOMP_RX4,
	mtch6102_compensation_SENSORCOMP_RX5,
	mtch6102_compensation_SENSORCOMP_RX6,
	mtch6102_compensation_SENSORCOMP_RX7,
	mtch6102_compensation_SENSORCOMP_RX8,
	mtch6102_compensation_SENSORCOMP_RX9,
	mtch6102_compensation_SENSORCOMP_RX10,
	mtch6102_compensation_SENSORCOMP_RX11,
	mtch6102_compensation_SENSORCOMP_RX12,
	mtch6102_compensation_SENSORCOMP_RX13,
	mtch6102_compensation_SENSORCOMP_RX14,
};
typedef enum mtch6102_compensation_register_type MTCH6102_Compensation_Ram_Memory;

/**
 * @brief MTCH6102 acquisition RAM register map.
 */
enum mtch6102_acquisition_register_type {
	mtch6102_acquisition_SENSORVALUES_RX0 = 0x80,
	mtch6102_acquisition_SENSORVALUES_RX1,
	mtch6102_acquisition_SENSORVALUES_RX2,
	mtch6102_acquisition_SENSORVALUES_RX3,
	mtch6102_acquisition_SENSORVALUES_RX4,
	mtch6102_acquisition_SENSORVALUES_RX5,
	mtch6102_acquisition_SENSORVALUES_RX6,
	mtch6102_acquisition_SENSORVALUES_RX7,
	mtch6102_acquisition_SENSORVALUES_RX8,
	mtch6102_acquisition_SENSORVALUES_RX9,
	mtch6102_acquisition_SENSORVALUES_RX10,
	mtch6102_acquisition_SENSORVALUES_RX11,
	mtch6102_acquisition_SENSORVALUES_RX12,
	mtch6102_acquisition_SENSORVALUES_RX13,
	mtch6102_acquisition_SENSORVALUES_RX14,

	mtch6102_acquisition_RAWVALUES_RX0_L = 0x90,
	mtch6102_acquisition_RAWVALUES_RX0_H,
	mtch6102_acquisition_RAWVALUES_RX1_L,
	mtch6102_acquisition_RAWVALUES_RX1_H,
	mtch6102_acquisition_RAWVALUES_RX2_L,
	mtch6102_acquisition_RAWVALUES_RX2_H,
	mtch6102_acquisition_RAWVALUES_RX3_L,
	mtch6102_acquisition_RAWVALUES_RX3_H,
	mtch6102_acquisition_RAWVALUES_RX4_L,
	mtch6102_acquisition_RAWVALUES_RX4_H,
	mtch6102_acquisition_RAWVALUES_RX5_L,
	mtch6102_acquisition_RAWVALUES_RX5_H,
	mtch6102_acquisition_RAWVALUES_RX6_L,
	mtch6102_acquisition_RAWVALUES_RX6_H,
	mtch6102_acquisition_RAWVALUES_RX7_L,
	mtch6102_acquisition_RAWVALUES_RX7_H,
	mtch6102_acquisition_RAWVALUES_RX8_L,
	mtch6102_acquisition_RAWVALUES_RX8_H,
	mtch6102_acquisition_RAWVALUES_RX9_L,
	mtch6102_acquisition_RAWVALUES_RX9_H,
	mtch6102_acquisition_RAWVALUES_RX10_L,
	mtch6102_acquisition_RAWVALUES_RX10_H,
	mtch6102_acquisition_RAWVALUES_RX11_L,
	mtch6102_acquisition_RAWVALUES_RX11_H,
	mtch6102_acquisition_RAWVALUES_RX12_L,
	mtch6102_acquisition_RAWVALUES_RX12_H,
	mtch6102_acquisition_RAWVALUES_RX13_L,
	mtch6102_acquisition_RAWVALUES_RX13_H,
	mtch6102_acquisition_RAWVALUES_RX14_L,
	mtch6102_acquisition_RAWVALUES_RX14_H,

	mtch6102_acquisition_BASEVALUES_RX0_L = 0xB0,
	mtch6102_acquisition_BASEVALUES_RX0_H,
	mtch6102_acquisition_BASEVALUES_RX1_L,
	mtch6102_acquisition_BASEVALUES_RX1_H,
	mtch6102_acquisition_BASEVALUES_RX2_L,
	mtch6102_acquisition_BASEVALUES_RX2_H,
	mtch6102_acquisition_BASEVALUES_RX3_L,
	mtch6102_acquisition_BASEVALUES_RX3_H,
	mtch6102_acquisition_BASEVALUES_RX4_L,
	mtch6102_acquisition_BASEVALUES_RX4_H,
	mtch6102_acquisition_BASEVALUES_RX5_L,
	mtch6102_acquisition_BASEVALUES_RX5_H,
	mtch6102_acquisition_BASEVALUES_RX6_L,
	mtch6102_acquisition_BASEVALUES_RX6_H,
	mtch6102_acquisition_BASEVALUES_RX7_L,
	mtch6102_acquisition_BASEVALUES_RX7_H,
	mtch6102_acquisition_BASEVALUES_RX8_L,
	mtch6102_acquisition_BASEVALUES_RX8_H,
	mtch6102_acquisition_BASEVALUES_RX9_L,
	mtch6102_acquisition_BASEVALUES_RX9_H,
	mtch6102_acquisition_BASEVALUES_RX10_L,
	mtch6102_acquisition_BASEVALUES_RX10_H,
	mtch6102_acquisition_BASEVALUES_RX11_L,
	mtch6102_acquisition_BASEVALUES_RX11_H,
	mtch6102_acquisition_BASEVALUES_RX12_L,
	mtch6102_acquisition_BASEVALUES_RX12_H,
	mtch6102_acquisition_BASEVALUES_RX13_L,
	mtch6102_acquisition_BASEVALUES_RX13_H,
	mtch6102_acquisition_BASEVALUES_RX14_L,
	mtch6102_acquisition_BASEVALUES_RX14_H,

	mtch6102_acquisition_RAWADC_00 = 0xD0,
	mtch6102_acquisition_RAWADC_01,
	mtch6102_acquisition_RAWADC_02,
	mtch6102_acquisition_RAWADC_03,
	mtch6102_acquisition_RAWADC_04,
	mtch6102_acquisition_RAWADC_05,
	mtch6102_acquisition_RAWADC_06,
	mtch6102_acquisition_RAWADC_07,
	mtch6102_acquisition_RAWADC_08,
	mtch6102_acquisition_RAWADC_09,
	mtch6102_acquisition_RAWADC_10,
	mtch6102_acquisition_RAWADC_11,
	mtch6102_acquisition_RAWADC_12,
	mtch6102_acquisition_RAWADC_13,
	mtch6102_acquisition_RAWADC_14,
	mtch6102_acquisition_RAWADC_15,
	mtch6102_acquisition_RAWADC_16,
	mtch6102_acquisition_RAWADC_17,
	mtch6102_acquisition_RAWADC_18,
	mtch6102_acquisition_RAWADC_19,
	mtch6102_acquisition_RAWADC_20,
	mtch6102_acquisition_RAWADC_21,
	mtch6102_acquisition_RAWADC_22,
	mtch6102_acquisition_RAWADC_23,
	mtch6102_acquisition_RAWADC_24,
	mtch6102_acquisition_RAWADC_25,
	mtch6102_acquisition_RAWADC_26,
	mtch6102_acquisition_RAWADC_27,
	mtch6102_acquisition_RAWADC_28,
	mtch6102_acquisition_RAWADC_29,
	mtch6102_acquisition_RAWADC_30,
	mtch6102_acquisition_RAWADC_31,
};
typedef enum mtch6102_acquisition_register_type MTCH6102_Acquisition_Ram_Memory;

/**
 * @brief MTCH6102 configuration RAM register map.
 */
enum mtch6102_configuration_register_type {
	mtch6102_config_NumberOfXChannels = 0x20, /**< Number of X channels. */
	mtch6102_config_NumberOfYChannels,         /**< Number of Y channels. */
	mtch6102_config_ScanCount,                 /**< Scan count. */
	mtch6102_config_TouchThreshX,              /**< X touch threshold. */
	mtch6102_config_TouchThreshY,              /**< Y touch threshold. */
	mtch6102_config_ActivePeriodL,             /**< Active-period low byte. */
	mtch6102_config_ActivePeriodH,             /**< Active-period high byte. */
	mtch6102_config_IdlePeriodL,               /**< Idle-period low byte. */
	mtch6102_config_IdlePeriodH,               /**< Idle-period high byte. */
	mtch6102_config_IdleTimeout,               /**< Idle timeout. */
	mtch6102_config_Hysteresis,                /**< Touch hysteresis. */
	mtch6102_config_DebounceUp,                /**< Up debounce. */
	mtch6102_config_DebounceDown,              /**< Down debounce. */
	mtch6102_config_BaseIntervalL,             /**< Baseline interval low byte. */
	mtch6102_config_BaseIntervalH,             /**< Baseline interval high byte. */
	mtch6102_config_BasePosFilter,             /**< Positive baseline filter. */
	mtch6102_config_BaseNegFilter,             /**< Negative baseline filter. */
	mtch6102_config_FilterType,                /**< Touch filter type. */
	mtch6102_config_FilterStrength,            /**< Touch filter strength. */
	mtch6102_config_BaseFilterType,            /**< Baseline filter type. */
	mtch6102_config_BaseFilterStrength,        /**< Baseline filter strength. */
	mtch6102_config_LargeActivationThreshL,    /**< Large-activation threshold low byte. */
	mtch6102_config_LargeActivationThreshH,    /**< Large-activation threshold high byte. */
	mtch6102_config_HorizontalSwipeDistance,    /**< Horizontal swipe distance. */
	mtch6102_config_VerticalSwipeDistance,      /**< Vertical swipe distance. */
	mtch6102_config_SwipeHoldBoundary,          /**< Swipe-hold boundary. */
	mtch6102_config_TapDistance,               /**< Tap distance. */
	mtch6102_config_DistanceBetweenTaps,       /**< Double-tap distance. */
	mtch6102_config_TapHoldTimeL,              /**< Tap-hold time low byte. */
	mtch6102_config_TapHoldTimeH,              /**< Tap-hold time high byte. */
	mtch6102_config_GestureClickTime,          /**< Gesture click time. */
	mtch6102_config_SwipeHoldThresh,           /**< Swipe-hold threshold. */
	mtch6102_config_MinSwipeVelocity,          /**< Minimum swipe velocity. */
	mtch6102_config_HorizontalGestureAngle,    /**< Horizontal gesture angle. */
	mtch6102_config_VerticalGestureAngle,      /**< Vertical gesture angle. */
	mtch6102_config_I2CAddr,                   /**< I2C address register. */
};
typedef enum mtch6102_configuration_register_type MTCH6102_Configuration_Ram_Memory;

/** @} */

#endif /* SENSWEAR_MTCH6102_REGISTERS_H_ */
