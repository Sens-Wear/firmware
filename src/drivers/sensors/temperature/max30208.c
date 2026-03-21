#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "max30208.h"

LOG_MODULE_REGISTER(SENS_WEAR_TEMPERATURE_SENSOR_LOGGER);

#define MAX30208_NODE DT_NODELABEL(max30208)
#define MAX30208_CONVERSION_POLL_MS 10U
#define MAX30208_CONVERSION_TIMEOUT_MS 200U

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(MAX30208_NODE);

static struct k_work_delayable temperature_sample_work;
static temperature_sample_cb_t temperature_callback;
static void *temperature_callback_user_data;
static atomic_t temperature_streaming_enabled;

static bool temperature_initialized;
static bool temperature_work_started;
static uint16_t temperature_sampling_rate_hz = 1U;

static void max30208_i2c_write_registers(uint8_t start_address, const uint8_t *value, size_t count)
{
	uint8_t buffer[3];

	if ((value == NULL) || (count == 0U) || (count > (sizeof(buffer) - 1U))) {
		return;
	}

	buffer[0] = start_address;
	memcpy(&buffer[1], value, count);

	if (i2c_write_dt(&dev_i2c, buffer, count + 1U) != 0) {
		LOG_ERR("I2C bus %s: error writing registers", dev_i2c.bus->name);
	}
}

static void max30208_i2c_read_registers(uint8_t start_address, uint8_t *value, size_t count)
{
	if ((value == NULL) || (count == 0U)) {
		return;
	}

	if (i2c_write_read_dt(&dev_i2c, &start_address, sizeof(start_address), value, count) != 0) {
		LOG_ERR("I2C bus %s: error reading registers", dev_i2c.bus->name);
	}
}

static uint32_t temperature_sample_period_ms(void)
{
	uint32_t hz = temperature_sampling_rate_hz;

	if (hz == 0U) {
		hz = 1U;
	}

	return MAX(1U, 1000U / hz);
}

static int temperature_sensor_read_sample(struct temperature_sample *sample)
{
	write_SetupRegister();
	int value = 0;
	union max30208_status_register_t *statusRegister = (void *)&value;
	/* Wait for reset to be cleared */
	do
	{
		k_msleep(100);
		read_StatusRegister((void *)&value);
	} while (statusRegister->bits.temp_ready != 0);
	uint16_t FiFo_data[1];
	read_FiFoDataRegister(FiFo_data);
	sample->temperature_mdeg_c = (int32_t)FiFo_data[0] * 5;
	return 0;
}

static void temperature_sample_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!atomic_get(&temperature_streaming_enabled)) {
		return;
	}

	struct temperature_sample sample;
	if (temperature_sensor_read_sample(&sample) == 0) {
		if (temperature_callback != NULL) {
			temperature_callback(&sample, temperature_callback_user_data);
		}
	}

	k_work_schedule(&temperature_sample_work, K_MSEC(temperature_sample_period_ms()));
}

int temperature_sensor_register_callback(temperature_sample_cb_t cb, void *user_data)
{
	temperature_callback = cb;
	temperature_callback_user_data = user_data;
	return 0;
}

void temperature_sensor_set_streaming_enabled(bool enabled)
{
	atomic_set(&temperature_streaming_enabled, enabled ? 1 : 0);
}

void temperature_sensor_set_sampling_rate(uint16_t new_sampling_rate)
{
	if (new_sampling_rate == 0U) {
		return;
	}

	temperature_sampling_rate_hz = new_sampling_rate;

	if (temperature_work_started && atomic_get(&temperature_streaming_enabled)) {
		k_work_reschedule(&temperature_sample_work, K_NO_WAIT);
	}
}

void temperature_sensor_set_transfer_interval(uint16_t new_transfer_interval)
{
	ARG_UNUSED(new_transfer_interval);
}

int temperature_sensor_start(void)
{
	if (!temperature_initialized) {
		int err = temperature_sensor_init();

		if (err != 0) {
			return err;
		}
	}

	temperature_work_started = true;
	k_work_reschedule(&temperature_sample_work, K_NO_WAIT);
	return 0;
}

void temperature_sensor_stop(void)
{
	k_work_cancel_delayable(&temperature_sample_work);
	temperature_work_started = false;
}

void temperature_sensor_deinit(void)
{
	temperature_sensor_stop();
	atomic_set(&temperature_streaming_enabled, 0);
	temperature_initialized = false;
}

int temperature_sensor_init(void)
{
	uint8_t id_reg = 0U;

	if (temperature_initialized) {
		return 0;
	}

	if (!device_is_ready(dev_i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready", dev_i2c.bus->name);
		return -ENODEV;
	}

	read_PartID(&id_reg);
	if (id_reg != MAX30208_PART_ID_VALUE) {
		LOG_ERR("MAX30208 not connected");
		return -ENODEV;
	}

	if (write_FiFoConfig1() != 0) {
		return -EIO;
	}

	k_work_init_delayable(&temperature_sample_work, temperature_sample_work_fn);
	atomic_set(&temperature_streaming_enabled, 0);
	temperature_initialized = true;
	return 0;
}

int read_PartID(uint8_t *data)
{
	uint8_t id = 0U;

	if (data == NULL) {
		return -EINVAL;
	}

	max30208_i2c_read_registers(MAX30208_ID_ADDR, &id, 1U);
	*data = id;
	return 0;
}

void read_OverFlow(uint8_t *data)
{
	if (data == NULL) {
		return;
	}

	max30208_i2c_read_registers(MAX30208_FIFO_OVERFLOW_ADDR, data, 1U);
}

void read_StatusRegister(uint8_t *data)
{
	if (data == NULL) {
		return;
	}

	max30208_i2c_read_registers(MAX30208_STATUS_REGISTER_ADDR, data, 1U);
}

void write_SetupRegister(void)
{
	const uint8_t setup_value = MAX30208_SETUP_VALUE;

	max30208_i2c_write_registers(MAX30208_SETUP_ADDR, &setup_value, 1U);
}

int write_FiFoConfig1(void)
{
	const uint8_t fifo_config = MAX30208_FIFO_CONFIG1_VALUE;

	max30208_i2c_write_registers(MAX30208_FIFO_CONFIG1_ADDR, &fifo_config, 1U);
	return 0;
}

int flush_FiFo(void)
{
	const uint8_t fifo_config2 = MAX30208_FIFO_CONFIG2_FLUSH_FIFO;

	max30208_i2c_write_registers(MAX30208_FIFO_CONFIG2_ADDR, &fifo_config2, 1U);
	return 0;
}

int read_FiFoDataRegister(uint16_t *data)
{
	uint8_t temp[2];

	if (data == NULL) {
		return -EINVAL;
	}

	max30208_i2c_read_registers(MAX30208_TEMP_ADDR, temp, sizeof(temp));
	*data = ((uint16_t)temp[0] << 8) | temp[1];
	return 0;
}

double read_Temperature(double data_fifo)
{
	return data_fifo * 0.005;
}
