#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fdc1004.h"

LOG_MODULE_REGISTER(SENSE_WEAR_PRESSURE_SENSOR_LOGGER);

static uint8_t *raw_data_buffer = NULL;
static size_t base_buffer_size = 0;
static int buffer_index = 0;

// Frequency settings (default values)
static uint16_t sampling_rate = 10;   // Default: 10 Hz (100 ms interval)
static uint16_t transfer_interval = 1;       // Default: 1 Hz (every second)

static struct k_timer sensor_timer;
static bool subscription_active = false;

// Function to update the buffer
void update_sensor_buffer()
{
  size_t new_base_buffer_size = (size_t)(sampling_rate * transfer_interval);

  uint8_t *new_raw_data_buffer = k_malloc(new_base_buffer_size);

  if (new_raw_data_buffer == NULL)
  {
    LOG_ERR("Memory allocation failed!\n");
    return;
  }

  // Copy existing data (if needed)
  if (raw_data_buffer != NULL)
  {
    size_t copy_size = (base_buffer_size < new_base_buffer_size) ? base_buffer_size : new_base_buffer_size;
    memcpy(new_raw_data_buffer, raw_data_buffer, copy_size);
    k_free(raw_data_buffer); // Free the old buffer
  }

  raw_data_buffer = new_raw_data_buffer;
  base_buffer_size = new_base_buffer_size + 10;
  buffer_index = 0;
  LOG_INF("Buffer updated. New size (%d): %zu bytes\n", base_buffer_size, base_buffer_size);
}

void set_pressure_sensor_sampling_rate(uint16_t new_sampling_rate)
{
    if (new_sampling_rate > 0) {
        sampling_rate = new_sampling_rate;
    }
    update_sensor_buffer();
}

void set_pressure_sensor_transfer_interval(uint16_t new_transfer_interval)
{
    if (transfer_interval > 0) {
        transfer_interval = new_transfer_interval;
    }
    update_sensor_buffer();
}

// Sensor thread function
static void sensor_data_work_handler(struct k_work *work) {
    uint16_t measurement = 100;
    if (buffer_index >= base_buffer_size) {
        LOG_ERR("Buffer overflow happened in sensor data.");
        buffer_index = 0;
    }
    raw_data_buffer[buffer_index] = measurement;
    buffer_index++;
    LOG_INF("FDCA1004 pressure measurment: %d", measurement);
}

K_WORK_DEFINE(pressure_sensor_data_work, sensor_data_work_handler);
// Timer callback function
static void sensor_timer_callback(struct k_timer *timer) {
    if (subscription_active) {
      k_work_submit(&pressure_sensor_data_work);
    }
  }

static void pressure_status_handler(bool enabled) {
    if (enabled) {
        subscription_active = true;
        k_timer_start(&sensor_timer, K_MSEC(0), K_MSEC(1000 / sampling_rate)); // Start timer immediately, and every sample interval
    } else {
        subscription_active = false;
        k_timer_stop(&sensor_timer);
    }
}

// Initialize the sensor thread
void pressure_sensor_init(void)
{
    register_pressure_status_callback(pressure_status_handler);
    register_pressure_sampling_rate_callback(set_pressure_sensor_sampling_rate);
    register_pressure_transfer_interval_callback(set_pressure_sensor_transfer_interval);
    k_timer_init(&sensor_timer, sensor_timer_callback, NULL);
}
