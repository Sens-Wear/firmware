/*
 * max30101.h
 *
 *  Created on: Oct 15, 2021
 *      Author: husey
 */

#ifndef MAX30101_H_
#define MAX30101_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>

#define MAX30101_PART_ID (0x15)
#define MAX30101_BYTES_PER_CHANNEL (3)

/*
#include "max30101/max30101.h"
#include "max30205/max30205.h"
#include <assert.h>
 */
/**
 * \brief MAX30101 Register Map
 */
enum max30101_register_type {
	max30101_register_InterruptStatus1 = 0x00,	 /**< max30101_register_InterruptStatus1 */
	max30101_register_InterruptStatus2 = 0x01,	 /**< max30101_register_InterruptStatus2 */
	max30101_register_InterruptEnable1 = 0x02,	 /**< max30101_register_InterruptEnable1 */
	max30101_register_InterruptEnable2 = 0x03,	 /**< max30101_register_InterruptEnable2 */
	max30101_register_FIFO_WritePointer = 0x04,	 /**< max30101_register_FIFO_WritePointer */
	max30101_register_OverflowCounter = 0x05,	 /**< max30101_register_OverflowCounter */
	max30101_register_FIFO_ReadPointer = 0x06,	 /**< max30101_register_FIFO_ReadPointer */
	max30101_register_FIFO_DataRegister = 0x07,	 /**< max30101_register_FIFO_DataRegister */
	max30101_register_FIFO_Configuration = 0x08, /**< max30101_register_FIFO_Configuration */
	max30101_register_ModeConfiguration = 0x09,	 /**< max30101_register_ModeConfiguration */
	max30101_register_SpO2Configuration = 0x0A,	 /**< max30101_register_SpO2Configuration */
	max30101_register_LED1_PA = 0x0C,			 /**< max30101_register_LED1_PA */
	max30101_register_LED2_PA = 0x0D,			 /**< max30101_register_LED2_PA */
	max30101_register_LED3_PA = 0x0E,			 /**< max30101_register_LED3_PA */
	max30101_register_ProxModeLED_PA = 0x10,	 /**< max30101_register_ProxModeLED_PA */
	max30101_register_ModeControlReg1 = 0x11,	 /**< max30101_register_ModeControlReg1 */
	max30101_register_ModeControlReg2 = 0x12,	 /**< max30101_register_ModeControlReg2 */
	max30101_register_DieTempInt = 0x1F,		 /**< max30101_register_DieTempInt */
	max30101_register_DieTempFrac = 0x20,		 /**< max30101_register_DieTempFrac */
	max30101_register_DieTempConfig = 0x21,		 /**< max30101_register_DieTempConfig */
	max30101_register_ProxIntThreshold = 0x30,	 /**< max30101_register_ProxIntThreshold */
	max30101_register_RevID = 0xFE,				 /**< max30101_register_RevID */
	max30101_register_PartID = 0xFF				 /**< max30101_register_PartID */
};

/**
 * \brief MAX30101 Operational Modes
 */
enum max30101_operation_mode_type {
	max30101_mode_HeartRate = 2, /**< max30101_mode_HeartRateMode */
	max30101_mode_SpO2 = 3,		 /**< max30101_mode_SpO2Mode */
	max30101_mode_MultiLed = 7	 /**< max30101_mode_MultiLedMode */
};

/**
 * \brief Number of samples averaged per FIFO sample, set in FIFO config
 */
enum max30101_averaged_samples_type {
	max30101_averaged_samples_1 = 0,  ///< AveragedSamples_0
	max30101_averaged_samples_2 = 1,  ///< AveragedSamples_2
	max30101_averaged_samples_4 = 2,  ///< AveragedSamples_4
	max30101_averaged_samples_8 = 3,  ///< AveragedSamples_8
	max30101_averaged_samples_16 = 4, ///< AveragedSamples_16
	max30101_averaged_samples_32 = 5  ///< AveragedSamples_32
};

/**
 * \brief ADC Range, set in SpO2 config
 */
enum max30101_adc_range_type {
	max30101_adc_range_2048 = 0, ///< ADC_Range_0
	max30101_adc_range_4096 = 1, ///< ADC_Range_1
	max30101_adc_range_8192 = 2, ///< ADC_Range_2
	max30101_adc_range_16384 = 3 ///< ADC_Range_3
};

//
/**
 * \brief LED PulseWidth, set in SpO2 config
 */
enum max30101_led_pulsewidth_type {
	max30101_led_pulsewidth_69 = 0,	 /**< PW_0 */
	max30101_led_pulsewidth_118 = 1, /**< PW_1 */
	max30101_led_pulsewidth_215 = 2, /**< PW_2 */
	max30101_led_pulsewidth_411 = 3	 /**< PW_3 */
};

/**
 * \brief Sample rate, set in SpO2 config
 */
enum max30101_sample_rate_type {
	max30101_sample_rate_50Hz = 0,	 ///< SR_50_Hz
	max30101_sample_rate_100Hz = 1,	 ///< SR_100_Hz
	max30101_sample_rate_200Hz = 2,	 ///< SR_200_Hz
	max30101_sample_rate_400Hz = 3,	 ///< SR_400_Hz
	max30101_sample_rate_800Hz = 4,	 ///< SR_800_Hz
	max30101_sample_rate_1000Hz = 5, ///< SR_1000_Hz
	max30101_sample_rate_1600Hz = 6, ///< SR_1600_Hz
	max30101_sample_rate_3200Hz = 7	 ///< SR_3200_Hz
};

/**
 * \brief supported interrupt types
 */
enum max30101_interrupt_type {
	max30101_interrupt_PowerReady = 0x0001,					/**< max30101_interrupt_PowerReady */
	max30101_interrupt_Proximity = 0x0010,					/**< max30101_interrupt_Proximity */
	max30101_interrupt_AmbientLightCancelOverflow = 0x0020, /**< max30101_interrupt_AmbientLightCancelOverflow */
	max30101_interrupt_FifoDataReady = 0x0040,				/**< max30101_interrupt_FifoDataReady */
	max30101_interrupt_FifoAlmostFull = 0x0080,				/**< max30101_interrupt_FifoAlmostFull */
	max30101_interrupt_DieTempReady = 0x0200				/**< max30101_interrupt_DieTempReady */
};

/**
 * \brief MAX30101 LED types
 */
enum max30101_led_type {
	max30101_led_Red = 1,  /**< max30101_led_Red */
	max30101_led_IR = 2,   /**< max30101_led_IR */
	max30101_led_Green = 3 /**< max30101_led_Green */
};

/**
 * \brief Interrupt Status BitField
 */
union max30101_interrupt_status_t {
	int value;
	struct max30101_interrupt_status_bits {
		unsigned int pwr_rdy : 1;		 ///< Bit0
		unsigned int reserved_1 : 3;	 ///< Bit3:1
		unsigned int prox_int : 1;		 ///< Bit4
		unsigned int alc_ovf : 1;		 ///< Bit5
		unsigned int ppg_rdy : 1;		 ///< Bit6
		unsigned int a_full : 1;		 ///< Bit7
		unsigned int reserved_2 : 1;	 ///< Bit0
		unsigned int die_tamp_ready : 1; ///< Bit 1
		unsigned int reserved_3 : 6;	 ///< Bits 2-7
	} bits;
};

/**
 * \brief Interrupt Enable BitField
 */
union max30101_interrupt_enable_t {
	int value;
	struct max30101_interrupt_enable_bits {
		int reserved_1 : 4;		///< Bit3:0
		int prox_int : 1;		///< Bit4
		int alc_ovf : 1;		///< Bit5
		int ppg_rdy : 1;		///< Bit6
		int a_full : 1;			///< Bit7
		int reserved_2 : 1;		///< Bit0
		int die_temp_ready : 1; ///< Bit 1
		int reserved_3 : 6;		///< Bits 2-7
	} bits;
};

/**
 * \brief FIFO Configuration BitField
 */
union max30101_fifo_configuration_t {
	int value;
	struct max30101_fifo_configuration_bits {
		unsigned int fifo_a_full : 4;
		unsigned int fifo_roll_over_en : 1;
		unsigned int sample_average : 3;
	} bits;
};

///Mode Configuration BitField
union max30101_mode_configuration_t {
	int value;
	struct max30101_mode_configuration_bits {
		unsigned int mode : 3;
		unsigned int reserved : 3;
		unsigned int reset : 1;
		unsigned int shdn : 1;
	} bits;
};

/**
 * \brief SpO2 Configuration BitField
 */
union max30101_spo2_configuration_t {
	int value;
	struct max30101_spO2_configuration_bits {
		unsigned int led_pw : 2;
		unsigned int spo2_sr : 3;
		unsigned int spo2_adc_range : 2;
		unsigned int reserved : 1;
	} bits;
};

/**
 * \brief Multi-LED Mode Control Register BitField
 */
union max30101_multi_led_mode_control_t {
	int value;
	struct max30101_mode_control_bits {
		unsigned int slot1 : 3;
		unsigned int reserved1 : 1;
		unsigned int slot2 : 3;
		unsigned int reserved2 : 1;
		unsigned int slot3 : 3;
		unsigned int reserved3 : 1;
		unsigned int slot4 : 3;
		unsigned int reserved4 : 1;
	} bits;
};

union max30101_die_temperature_config_t {
	int value;
	struct max30101_die_temperature_config_bits {
		unsigned int temp_en : 1;
		unsigned int reserved : 7;
	} bits;
};

union max30101_state_t {
	int value;
	struct max30101_state_bits {
		int bInitialized : 1;
		int bConfigured : 1;
		int bDetectingProximity : 1;
		int bSampling : 1;
	} bits;
};

struct max30101_configuration_t {
	union max30101_interrupt_enable_t interrupts;
	union max30101_fifo_configuration_t fifo_config;
	union max30101_mode_configuration_t mode_config;
	union max30101_spo2_configuration_t spo2_config;
	union max30101_multi_led_mode_control_t multi_led_config;
	uint8_t red_led_pulse_amplitude_config;
	uint8_t ir_led_pulse_amplitude_config;
	uint8_t green_led_pulse_amplitude_config;
	uint8_t proximity_led_pulse_amplitude_config;
	uint8_t proximity_int_threshold;
};

struct max30101_t {
	struct i2c_dt_spec *device;
	union max30101_state_t state;

	int led_count;
	float sampling_rate;

	uint32_t proximity_led_value_sum;
	int proximity_led_read_count;

	uint32_t start_time;
	uint32_t total_on_duration;
};

/**
 * \brief Initializes MAX30101
 *
 * \param i2c The I2C device MAX30101 connected to
 */
void ppg_sensor_init();

/**
 * \brief Enable or disable PPG streaming.
 *
 * \param enabled True to enable streaming, false to disable.
 */
void ppg_set_streaming_enabled(bool enabled);

/**
 * \brief Configures MAX30101
 */
void max30101_config(void);

/**
 * \brief Returns number of activated LEDs
 *
 * \return The led count
 */
int max30101_get_led_count(void);
/**
 * \brief Calculates and returns the sampling rate of MAX30101
 *
 * \return the sampling rate
 */
float max30101_get_sampling_rate(void);

/**
 * \brief Reads FIFO buffer of MAX30101
 *
 * \param buffer The buffer to hold the samples
 * \param bufferSize The size of the buffer
 * \return Number of bytes read
 */
size_t max30101_read_fifo(void* buffer, size_t bufferSize);

/**
 * \brief Enables proximity detection
 *
 * \return True if the proximity mode can be enabled
 */
bool max30101_enable_proximity(void);

/**
 * \brief Enables wrist HR sampling -- multi-led mode of 3 leds
 *
 * \return True if the multi-mode sampling can be enabled
 */
bool max30101_enable_wrist_hr_sampling(void);

/**
 * \brief Enables sampling operation mode
 *
 * \param mode The mode to be enabled
 * \return True if the mode can be enabled
 */
bool max30101_enable_sampling(enum max30101_operation_mode_type mode);

/**
 * \brief Puts MAX30101 into shutdown mode.
 */
void max30101_shutdown(void);
/**
 * \brief Handles the MAX30101 interrupts
 *
 */
void max30101_irq_handler(void);

#endif /* MAX30101_H_ */
