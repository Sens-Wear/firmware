/*
 * max30101.h
 *
 *  Created on: Oct 15, 2021
 *      Author: husey
 */

#ifndef MAX30101_H_
#define MAX30101_H_

#include "max30101_registers.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

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

struct max30101_configuration_t* max30101_get_default_config(void);
/**
 * \brief Configures MAX30101
 */
void max30101_config(struct max30101_configuration_t*);

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
