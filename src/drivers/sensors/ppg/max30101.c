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
#include <zephyr/sys/atomic.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "max30101.h"
#include "max30101_config.h"
#include "bluetooth/services/ppg/ppg_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_PPG_SENSOR_LOGGER);


#define MAX30101_NODE DT_NODELABEL(max30101)
#define MAX_SENSOR_READING_SIZE 32
#define PPG_STREAM_SLEEP_MS 1
#define PPG_IDLE_SLEEP_MS 250
#define PPG_BATCH_MIN_SAMPLES 10
#define PPG_BATCH_TARGET_SAMPLES 12
#define PPG_BATCH_MAX_SAMPLES 15
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(MAX30101_NODE);
static struct max30101_t max30101 = {0};
static atomic_t ppg_streaming_enabled;
static bool ppg_thread_started;

struct ppg_channel_batch_t {
  struct ppg_sample_notification_t samples[PPG_BATCH_MAX_SAMPLES];
  size_t count;
};

static struct ppg_channel_batch_t red_batch;
static struct ppg_channel_batch_t ir_batch;
static struct ppg_channel_batch_t green_batch;

K_THREAD_STACK_DEFINE(ppg_sensor_stack, 4096);
static struct k_thread ppg_sensor_thread;

static uint32_t max30101_unpack_sample(const uint8_t *sample)
{
  return ((uint32_t)sample[0] << 16 | (uint32_t)sample[1] << 8 | (uint32_t)sample[2]) & 0x03FFFF;
}

static uint64_t ppg_get_unix_ms(void)
{
  time_t now_sec = time(NULL);

  if ((int64_t)now_sec > 1700000000LL) {
    return ((uint64_t)now_sec * 1000ULL) + ((uint64_t)k_uptime_get() % 1000ULL);
  }

  return (uint64_t)k_uptime_get();
}

static void ppg_batch_reset(struct ppg_channel_batch_t *batch)
{
  batch->count = 0;
}

static void ppg_batch_push(struct ppg_channel_batch_t *batch, uint64_t unix_ms, uint32_t value)
{
  if (batch->count >= PPG_BATCH_MAX_SAMPLES) {
    return;
  }

  batch->samples[batch->count].unix_ms = unix_ms;
  batch->samples[batch->count].value = value;
  batch->count++;
}

static void ppg_batch_flush_channel(struct ppg_channel_batch_t *batch, int (*notify_fn)(const struct ppg_sample_notification_t *, size_t))
{
  if (batch->count == 0U) {
    return;
  }

  (void)notify_fn(batch->samples, batch->count);
  ppg_batch_reset(batch);
}

static void ppg_batch_flush_all(void)
{
  ppg_batch_flush_channel(&red_batch, ppg_lbs_notify_red_batch);
  ppg_batch_flush_channel(&ir_batch, ppg_lbs_notify_ir_batch);
  ppg_batch_flush_channel(&green_batch, ppg_lbs_notify_green_batch);
}

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
  // LOG_INF("actualSize: %d", actualSize);
  // LOG_INF("FIFO data: ");
  // LOG_HEXDUMP_INF(buffer, actualSize, "");
  if (actualSize >= (size_t)(max30101.led_count * MAX30101_BYTES_PER_CHANNEL)) {
    size_t sample_size = (size_t)max30101.led_count * MAX30101_BYTES_PER_CHANNEL;
    size_t offset = 0;

    // Drain all complete samples from FIFO in order (oldest to newest).
    while (offset + sample_size <= actualSize) {
      const uint8_t *sample = &buffer[offset];
      uint64_t unix_ms = ppg_get_unix_ms();

      if (max30101.led_count >= 1) {
        uint32_t ir = max30101_unpack_sample(sample);
        ppg_batch_push(&ir_batch, unix_ms, ir);
        LOG_INF("Timestamp: %llu, IR: %u", unix_ms, ir);
        sample += MAX30101_BYTES_PER_CHANNEL;
      }

      if (max30101.led_count >= 2) {
        uint32_t red = max30101_unpack_sample(sample);
        ppg_batch_push(&red_batch, unix_ms, red);
        sample += MAX30101_BYTES_PER_CHANNEL;
      }

      if (max30101.led_count >= 3) {
        uint32_t green = max30101_unpack_sample(sample);
        ppg_batch_push(&green_batch, unix_ms, green);
      }

      offset += sample_size;
    }
  }
  if ((red_batch.count >= PPG_BATCH_TARGET_SAMPLES && red_batch.count >= PPG_BATCH_MIN_SAMPLES) ||
      (ir_batch.count >= PPG_BATCH_TARGET_SAMPLES && ir_batch.count >= PPG_BATCH_MIN_SAMPLES) ||
      (green_batch.count >= PPG_BATCH_TARGET_SAMPLES && green_batch.count >= PPG_BATCH_MIN_SAMPLES) ||
      red_batch.count >= PPG_BATCH_MAX_SAMPLES ||
      ir_batch.count >= PPG_BATCH_MAX_SAMPLES ||
      green_batch.count >= PPG_BATCH_MAX_SAMPLES) {
    ppg_batch_flush_all();
  }
}

static void ppg_sensor_thread_fn(void *p1, void *p2, void *p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (atomic_get(&ppg_streaming_enabled)) {
    // Wait for FIFO almost-full interrupt instead of polling
    // OR poll at a rate matching FIFO fill rate (not fixed 5ms)
    k_msleep(PPG_STREAM_SLEEP_MS);
    sensor_data_work_handler();
  }
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
}

void ppg_set_streaming_enabled(bool enabled)
{
  if (enabled) {
    if (atomic_get(&ppg_streaming_enabled)) {
      return;
    }
    max30101_config();
    max30101_enable_wrist_hr_sampling();
    ppg_batch_reset(&red_batch);
    ppg_batch_reset(&ir_batch);
    ppg_batch_reset(&green_batch);

    atomic_set(&ppg_streaming_enabled, 1);

    if (!ppg_thread_started) {
      ppg_thread_started = true;
      k_thread_create(&ppg_sensor_thread, ppg_sensor_stack,
                      K_THREAD_STACK_SIZEOF(ppg_sensor_stack),
                      ppg_sensor_thread_fn, NULL, NULL, NULL,
                      K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    }
  } else {
    if (!atomic_get(&ppg_streaming_enabled)) {
      return;
    }

    atomic_set(&ppg_streaming_enabled, 0);
    ppg_batch_flush_all();
    if (ppg_thread_started) {
      k_thread_abort(&ppg_sensor_thread);
      ppg_thread_started = false;
    }

    if (max30101.state.bits.bSampling) {
      max30101_shutdown();
    }
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
    uint8_t ledPulseAmplitudes[3] = {0};
    ledPulseAmplitudes[0] = MAX30101_IR_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[1] = MAX30101_RED_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[2] = MAX30101_GREEN_LED_PULSE_AMPLITUDE;
    max30101_i2c_write_registers(max30101_register_LED1_PA,
                                 ledPulseAmplitudes,
                                 sizeof(ledPulseAmplitudes));

    uint8_t proximityAmplitude = MAX30101_PROXIMITY_MODE_LED_PULSE_AMPLITUDE;
    max30101_i2c_write_registers(max30101_register_ProxModeLED_PA,
                                 &proximityAmplitude,
                                 1);
  }
  /* Write the Multi-LED mode register */
  {
    union max30101_multi_led_mode_control_t *multiLedModeConfig =
        (void *)&value;
    multiLedModeConfig->value = 0;
    // Enable all three LEDs in FIFO sample order: IR, Red, Green.
    multiLedModeConfig->bits.slot1 = (int)max30101_led_IR;
    multiLedModeConfig->bits.slot2 = (int)max30101_led_Red;
    multiLedModeConfig->bits.slot3 = (int)max30101_led_Green;
    multiLedModeConfig->bits.slot4 = 0;

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
    modeConfig->bits.shdn = 0;
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
  uint8_t value[4] = {0};
  size_t byteCount = 0;

  // calculate number of bytes to read
  max30101_i2c_read_registers(max30101_register_FIFO_WritePointer,
                              value,
                              3);
  uint8_t writePtr = value[0] & 0x1F;
  uint8_t overFlowCounter = value[1] & 0x1F;
  uint8_t readPtr = value[2] & 0x1F;
  uint8_t availableSamples = (writePtr - readPtr) & 0x1F;

  if (overFlowCounter > 0) {
    LOG_WRN("FIFO overflow detected: overFlowCounter=%d, readPtr=%d, writePtr=%d\n\r", overFlowCounter, readPtr, writePtr);
    availableSamples = 32; // FIFO is full
  }

  if (availableSamples == 0) {
    return 0;
  }

  byteCount = (size_t)availableSamples * MAX30101_BYTES_PER_CHANNEL *
              max30101.led_count;

  // adjust what we can read
  while (bufferSize < byteCount)
  {
    byteCount -= MAX30101_BYTES_PER_CHANNEL * max30101.led_count;
  }

  max30101_i2c_read_registers(max30101_register_FIFO_DataRegister,
                              buffer,
                              byteCount);

  if (overFlowCounter > 0) {
    byteCount = 0; // discard data if overflow occurred, as it may be corrupted
  }

  *(uint32_t*)value = 0;
  //max30101_i2c_write_registers(max30101_register_FIFO_WritePointer,
  //                             value,
  //                             3); // reset read pointer to clear overflow condition
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
    modeConfig->bits.shdn = 0;
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
  /* Configure LED pulse amplitude for all three channels */
  {
    uint8_t ledPulseAmplitudes[3] = {0};
    ledPulseAmplitudes[0] = MAX30101_IR_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[1] = MAX30101_RED_LED_PULSE_AMPLITUDE;
    ledPulseAmplitudes[2] = MAX30101_GREEN_LED_PULSE_AMPLITUDE;
    max30101_i2c_write_registers(max30101_register_LED1_PA,
                                 ledPulseAmplitudes,
                                 sizeof(ledPulseAmplitudes));
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
    // Enable all three LEDs in FIFO sample order: IR, Red, Green.
    multiLedModeConfig->bits.slot1 = (int)max30101_led_IR;
    multiLedModeConfig->bits.slot2 = (int)max30101_led_Red;
    multiLedModeConfig->bits.slot3 = (int)max30101_led_Green;
    multiLedModeConfig->bits.slot4 = 0;

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
