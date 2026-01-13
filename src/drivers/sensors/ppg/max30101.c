/*
 * max30101.c
 *
 *  Created on: Oct 15, 2021
 *      Author: husey
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "max30101.h"
#include "max30101_config.h"

LOG_MODULE_REGISTER(SENS_WEAR_PPG_SENSOR_LOGGER);


#define MAX30101_NODE DT_NODELABEL(max30101)
#define MAX_SENSOR_READING_SIZE 32
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(MAX30101_NODE);
static struct max30101_t max30101 = {0};

__STATIC_INLINE void
max30101_i2c_write_registers(enum max30101_register_type startAddress,
                             uint8_t *value,
                             size_t count)
{
  // Allocate a buffer to hold the register address and data
  uint8_t buffer[count + 1];
  buffer[0] = (uint8_t)startAddress; // First byte is the register address

  // Copy the data to the buffer
  memcpy(&buffer[1], value, count);

  // Perform the I2C write operation
  int ret = i2c_write_dt(max30101.device, buffer, count + 1);
  if (ret != 0)
  {
    LOG_ERR("I2C bus %s: Error while writing registers", max30101.device->bus->name);
    return;
  }
}

__STATIC_INLINE void
max30101_i2c_read_registers(enum max30101_register_type startAddress,
                            uint8_t *value,
                            size_t count)
{
  // Perform the I2C read operation
  uint8_t addressRegister = (uint8_t)startAddress;
  int ret = i2c_write_read_dt(max30101.device, &addressRegister, 1, value, count);
  if (ret != 0)
  {
    LOG_ERR("I2C bus %s: Error while reading registers", max30101.device->bus->name);
    return;
  }
}

// Sensor thread function
static void sensor_data_work_handler()
{
  /* Read the FIFO data registers */
  uint8_t buffer[MAX_SENSOR_READING_SIZE * 9] = {0};
  size_t actualSize = max30101_read_fifo(buffer, MAX_SENSOR_READING_SIZE * max30101.led_count * 3);
  size_t sampleCount = actualSize / 9; // 3 bytes per LED x 3 LEDs = 9 bytes per sample set
  LOG_INF("FIFO data: ");
  LOG_HEXDUMP_INF(buffer, actualSize, "");
}

// Function to update BLE reporting frequency dynamically
void set_ppg_operation_mode(uint16_t operation_mode)
{
  if (operation_mode != max30101_mode_MultiLed && operation_mode != max30101_mode_HeartRate && operation_mode != max30101_mode_SpO2)
  {
    LOG_ERR("Invalid operation mode: %d\n", operation_mode);
    return;
  }
  max30101_enable_sampling(operation_mode);
}

void ppg_sensor_init()
{
  if (!device_is_ready(dev_i2c.bus))
  {
    LOG_ERR("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
    return;
  }

  max30101.device = &dev_i2c;
  max30101.state.bits.bInitialized = 1;
  max30101_config();
  max30101_enable_wrist_hr_sampling();
  while(1) {
    sensor_data_work_handler();
    k_msleep(100);
  } 
}

void max30101_config(void)
{
  if (max30101.state.bits.bInitialized == 0)
  {
    LOG_ERR("MAX30101 not initialized\n\r");
    return;
  }
  int value = 0;
  max30101_i2c_read_registers(max30101_register_PartID, &value, 1);
  if (value != MAX30101_PART_ID)
  {
    LOG_ERR("MAX30101 not connected\n\r");
    return;
  }
  /* Reset the sensor */
  {
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 1;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);

    /* Wait for reset to be cleared */
    do
    {
      max30101_i2c_read_registers(max30101_register_ModeConfiguration,
                                  (void *)&value,
                                  1);
    } while (modeConfig->bits.reset != 0);
  }
  /* Write the FIFO configuration register */
  {
    union max30101_fifo_configuration_t *fifoConfig = (void *)&value;
    fifoConfig->value = 0;
    fifoConfig->bits.fifo_a_full = 32 - MAX30101_FIFO_ALMOST_FULL_THRESHOLD;
    fifoConfig->bits.fifo_roll_over_en = MAX30101_FIFO_ROLLOVER;
    fifoConfig->bits.sample_average = MAX30101_AVERAGED_SAMPLES;

    max30101_i2c_write_registers(max30101_register_FIFO_Configuration,
                                 (void *)&value,
                                 1);
  }

  /* Write the SpO2 configuration register */
  {
    union max30101_spo2_configuration_t *spo2Config = (void *)&value;
    spo2Config->value = 0;
    spo2Config->bits.led_pw = MAX30101_LED_PULSEWIDHT;
    spo2Config->bits.spo2_adc_range = MAX30101_ADC_RANGE;
    spo2Config->bits.spo2_sr = MAX30101_SAMPLE_RATE;
    max30101_i2c_write_registers(max30101_register_SpO2Configuration,
                                 (void *)&value,
                                 1);

    max30101_i2c_read_registers(max30101_register_SpO2Configuration,
                                (void *)&value,
                                1);
    if (spo2Config->bits.spo2_sr == max30101_sample_rate_50Hz)
    {
      max30101.sampling_rate = 50.0f;
    }
    else
    {
      max30101.sampling_rate = 100.0f * ((float)spo2Config->bits.spo2_sr);
    }
  }

  /* Write the LED pulse amplitude registers */
  {
    uint8_t *ledPulseAmplitudes = (void *)&value;
    value = 0;
    ledPulseAmplitudes[(int)max30101_led_Red] =
        MAX30101_RED_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[(int)max30101_led_IR] =
        MAX30101_IR_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[(int)max30101_led_Red] =
        MAX30101_GREEN_LED_PULSE_AMPLITUDE;
    max30101_i2c_write_registers(max30101_register_LED1_PA,
                                 (void *)&value,
                                 3);

    value = 0;
    ledPulseAmplitudes[0] = MAX30101_PROXIMITY_MODE_LED_PULSE_AMPLITUDE;
    max30101_i2c_write_registers(max30101_register_ProxModeLED_PA,
                                 (void *)&value,
                                 1);
  }
  /* Write the Multi-LED mode register */
  {
    union max30101_multi_led_mode_control_t *multiLedModeConfig =
        (void *)&value;
    multiLedModeConfig->value = 0;
    // first slot is for IR LED that is used for proximity detection
    multiLedModeConfig->bits.slot1 = ((int)max30101_led_IR) | 0x04;
    // second slot is for RED LED
    multiLedModeConfig->bits.slot2 = (int)max30101_led_Red;
    // third one is for Green LED
    multiLedModeConfig->bits.slot3 = (int)max30101_led_Green;

    max30101_i2c_write_registers(max30101_register_ModeControlReg1,
                                 (void *)&value,
                                 2);
  }

  /* Write the mode configuration register */
  {
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 0;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    modeConfig->bits.shdn = 1;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);
  }

  /* Read the configuration registers to calculate sampling rate */
  {
    union max30101_spo2_configuration_t *spo2Config = (void *)&value;
    union max30101_fifo_configuration_t *fifoConfig = (void *)&value;
    spo2Config->value = 0;
    max30101_i2c_read_registers(max30101_register_SpO2Configuration,
                                (void *)&value,
                                1);
    if (spo2Config->bits.spo2_sr == max30101_sample_rate_50Hz)
    {
      max30101.sampling_rate = 50.0f;
    }
    else
    {
      max30101.sampling_rate = 100.0f * ((float)spo2Config->bits.spo2_sr);
    }
    max30101_i2c_read_registers(max30101_register_FIFO_Configuration,
                                (void *)&value,
                                1);
    max30101.sampling_rate /= (float)(1 << fifoConfig->bits.sample_average);
  }

  /* Clear the FIFO registers */
  {
    value = 0;
    max30101_i2c_write_registers(max30101_register_FIFO_WritePointer,
                                 (void *)&value,
                                 3);
  }

  /* Write Interrupt Enable registers to zero */
  {
    union max30101_interrupt_enable_t *intEnable = (void *)&value;
    intEnable->value = 0;
    max30101_i2c_write_registers(max30101_register_InterruptEnable1,
                                 (void *)&value,
                                 2);
  }

  // indicate that the part is not functioning
  max30101.state.bits.bDetectingProximity = 0;
  max30101.state.bits.bSampling = 0;
  max30101.state.bits.bConfigured = 1;
}

/*
 * \brief Returns number of activated LEDs
 *
 * \return The led count
 */
int max30101_get_led_count(void)
{
  if (max30101.state.bits.bConfigured == 0)
  {
    LOG_ERR("MAX30101 not configured\n\r");
    return -1;
  }
  return max30101.led_count;
}

/*
 * \brief Calculates and returns the sampling rate of MAX30101
 *
 * \return the sampling rate
 */
float max30101_get_sampling_rate(void)
{
  if (max30101.state.bits.bConfigured == 0)
  {
    LOG_ERR("MAX30101 not configured\n\r");
    return -1.0f;
  }
  return max30101.sampling_rate;
}

/*
 * \brief Reads FIFO buffer of MAX30101
 *
 * \param buffer The buffer to hold the samples
 * \param bufferSize The size of the buffer
 * \return Number of bytes read
 */
size_t max30101_read_fifo(void *buffer, size_t bufferSize)
{
  if (max30101.state.bits.bConfigured == 0)
  {
    LOG_ERR("MAX30101 not configured\n\r");
    return -1;
  }
  if (buffer == NULL)
  {
    LOG_ERR("Buffer is NULL\n\r");
    return -1;
  }
  if (bufferSize < 4 * MAX30101_BYTES_PER_CHANNEL)
  {
    LOG_ERR("Buffer size is too small\n\r");
    return -1;
  }
  int value;
  size_t byteCount = 0;

  // calculate number of bytes to read
  {
    max30101_i2c_read_registers(max30101_register_FIFO_WritePointer,
                                (void *)&value,
                                3);
    uint8_t *bytes = (void *)&value;
    uint8_t writePtr = bytes[0];
    uint8_t readPtr = bytes[2];

    if (writePtr > readPtr)
    {
      byteCount = (writePtr - readPtr) * MAX30101_BYTES_PER_CHANNEL *
                  max30101.led_count;
    }
    else
    {
      byteCount = (32 + writePtr - readPtr) * MAX30101_BYTES_PER_CHANNEL *
                  max30101.led_count;
    }
  }

  // adjust what we can read
  while (bufferSize < byteCount)
  {
    byteCount -= MAX30101_BYTES_PER_CHANNEL * max30101.led_count;
  }

  max30101_i2c_read_registers(max30101_register_FIFO_DataRegister,
                              buffer,
                              byteCount);

  return byteCount;
}

/*
 * \brief Enables wrist HR sampling -- multi-led mode of 3 leds
 *
 * \return True if the multi-mode sampling can be enabled
 */
bool max30101_enable_wrist_hr_sampling(void)
{
  int value;
  if (max30101.state.bits.bConfigured == 0)
  {
    LOG_ERR("MAX30101 not configured\n\r");
    return false;
  }
  if (max30101.state.bits.bDetectingProximity != 0)
  {
    return false;
  }
  max30101.state.bits.bDetectingProximity = 0;
  max30101.state.bits.bSampling = 0;
  /* Write the mode configuration register -- shutdown*/
  {
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 0;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    modeConfig->bits.shdn = 1;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);
  }
  /* Write the SpO2 configuration register */
  {
    union max30101_spo2_configuration_t *spo2Config = (void *)&value;
    spo2Config->value = 0;
    spo2Config->bits.led_pw = MAX30101_LED_PULSEWIDHT;
    spo2Config->bits.spo2_adc_range = MAX30101_ADC_RANGE;
    spo2Config->bits.spo2_sr = MAX30101_SAMPLE_RATE;
    max30101_i2c_write_registers(max30101_register_SpO2Configuration,
                                 (void *)&value,
                                 1);
  }
  /* Write the FIFO configuration register */
  {
    union max30101_fifo_configuration_t *fifoConfig = (void *)&value;
    fifoConfig->value = 0;
    fifoConfig->bits.fifo_a_full = 32 - MAX30101_FIFO_ALMOST_FULL_THRESHOLD;
    fifoConfig->bits.fifo_roll_over_en = MAX30101_FIFO_ROLLOVER;
    fifoConfig->bits.sample_average = MAX30101_AVERAGED_SAMPLES;

    max30101_i2c_write_registers(max30101_register_FIFO_Configuration,
                                 (void *)&value,
                                 1);
  }
  /* Write the Multi-LED mode register */
  {
    union max30101_multi_led_mode_control_t *multiLedModeConfig =
        (void *)&value;
    multiLedModeConfig->value = 0;
    // first slot is for IR LED that is used for proximity detection
    multiLedModeConfig->bits.slot1 = (int)max30101_led_IR;
    multiLedModeConfig->bits.slot2 = (int)max30101_led_Red;
    multiLedModeConfig->bits.slot3 = (int)max30101_led_Green;

    max30101_i2c_write_registers(max30101_register_ModeControlReg1,
                                 (void *)&value,
                                 2);

    max30101.led_count = 3;
  }
  /* Clear the FIFO registers */
  {
    value = 0;
    max30101_i2c_write_registers(max30101_register_FIFO_WritePointer,
                                 (void *)&value,
                                 3);
  }

  /* Write Interrupt Enable registers */
  {
    union max30101_interrupt_enable_t *intEnable = (void *)&value;
    intEnable->value = 0;
    intEnable->bits.ppg_rdy = 0;
    intEnable->bits.prox_int = 0;
    intEnable->bits.a_full = 1;
    max30101_i2c_write_registers(max30101_register_InterruptEnable1,
                                 (void *)&value,
                                 2);
  }

  /* Write the mode configuration register */
  {
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 0;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    modeConfig->bits.shdn = 0;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);
  }
  max30101.state.bits.bSampling = 1;

  return true;
}

/*
 * \brief Enables sampling operation mode
 *
 * \param mode The mode to be enabled
 * \return True if the mode can be enabled
 */
bool max30101_enable_sampling(enum max30101_operation_mode_type mode)
{

  // TODO: complete the implementation
  /* Set number of leds */
  {
    switch (mode)
    {
    case max30101_mode_HeartRate:
      max30101.led_count = 1;
      break;
    case max30101_mode_SpO2:
      max30101.led_count = 2;
      break;
    case max30101_mode_MultiLed:
      max30101.led_count = 3;
      break;
    default:
      LOG_ERR("Invalid mode %d\n\r", mode);
      break;
    }
  }
  return false;
}

/*
 * \brief Puts MAX30101 into shutdown mode.
 */
void max30101_shutdown(void)
{
  int value;
  if (max30101.state.bits.bConfigured == 0)
  {
    LOG_ERR("MAX30101 not configured\n\r");
    return;
  }
  /* Write the mode configuration register */
  {
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 0;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    modeConfig->bits.shdn = 1;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);
  }

  max30101.state.bits.bDetectingProximity = 0;
  max30101.state.bits.bSampling = 0;
}
/*
 * \brief Handles the MAX30101 interrupts
 *
 */
void max30101_irq_handler(void)
{
  int value;
  union max30101_interrupt_status_t *intStatus = (void *)&value;
  // check if the device has been configured
  if (max30101.state.bits.bConfigured == 0)
  {
    return;
  }

  max30101_i2c_read_registers(max30101_register_InterruptStatus1,
                              (void *)&value,
                              2);

  if (intStatus->bits.a_full != 0)
  {
    // TODO handle almost full
    extern void max30101_handle_fifo_full(void);
    max30101_handle_fifo_full();
  }

  if (intStatus->bits.ppg_rdy != 0)
  {
    uint8_t buffer[16];
    size_t ret = max30101_read_fifo(buffer, sizeof(buffer));
    if (max30101.state.bits.bDetectingProximity != 0)
    {
      // we are reading just one channel
      unsigned int value =
          ((buffer[0] << 16) + (buffer[1] << 8) + buffer[2]) & 0x03FFFFF;
      printf("PPG %u IR 0x%08x\r\n", ret, value);
      max30101.proximity_led_value_sum += value;
      max30101.proximity_led_read_count++;

      if (max30101.proximity_led_read_count >=
          MAX30101_PROXIMITY_READCOUNT_FOR_DECISION)
      {
        uint32_t irLedValue = max30101.proximity_led_value_sum /
                              max30101.proximity_led_read_count;
        irLedValue >>= 10;
        if (irLedValue > MAX30101_PROXIMITY_THRESHOLD)
        {
          // TODO: start sampling
          printf("Starting sampling 0x%02X\r\n", (uint8_t)irLedValue);
          max30101.state.bits.bDetectingProximity = 0;
          max30101_enable_wrist_hr_sampling();
        }
        max30101.proximity_led_value_sum = 0;
        max30101.proximity_led_read_count = 0;
      }
    }
  }

  if (intStatus->bits.prox_int != 0)
  {

    printf("Proximity Interrupt\r\n");
    /* Write the mode configuration register -- to retrigger */
    union max30101_mode_configuration_t *modeConfig = (void *)&value;
    modeConfig->value = 0;
    modeConfig->bits.reset = 0;
    modeConfig->bits.mode = max30101_mode_MultiLed;
    modeConfig->bits.shdn = 0;
    max30101_i2c_write_registers(max30101_register_ModeConfiguration,
                                 (void *)&value,
                                 1);
  }
}
