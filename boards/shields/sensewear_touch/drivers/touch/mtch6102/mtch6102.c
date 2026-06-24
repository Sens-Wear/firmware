#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "mtch6102.h"

LOG_MODULE_REGISTER(SENS_WEAR_TOUCH_SENSOR_LOGGER);

#define MTCH6102_NODE DT_ALIAS(sensewear_touch)
#define MAX_CHANNEL_NUMBER 15
#define TOUCH_DEFAULT_SAMPLING_RATE_HZ 10U
BUILD_ASSERT(DT_NODE_HAS_STATUS(MTCH6102_NODE, okay),
	     "Touch firmware requires the sensewear_touch shield");
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(MTCH6102_NODE);
static uint8_t touch_sensor_buf[3];
static struct k_work_delayable touch_sample_work;
static touch_sensor_sample_cb_t touch_callback;
static void *touch_callback_user_data;
static atomic_t touch_streaming_enabled;
static bool touch_initialized;
static bool touch_work_started;
static uint16_t touch_sampling_rate_hz = TOUCH_DEFAULT_SAMPLING_RATE_HZ;

int mtch6102_get_position(const struct i2c_dt_spec *i2c, struct mtch6102_position *pos)
{
	if (i2c == NULL || pos == NULL) {
		return -EINVAL;
	}

	uint8_t start_reg = MTCH6102__TOUCH_STATE;
	uint8_t rx[4] = {0}; /* state, x, y, lsb */

	int ret = i2c_write_read_dt(i2c, &start_reg, 1, rx, sizeof(rx));
	if (ret != 0) {
		LOG_ERR("I2C bus %s: failed reading touch position (%d)",
			i2c->bus->name, ret);
		return ret;
	}

	pos->touch_state = rx[0];
	pos->touched = (rx[0] & 0x01U) != 0U;

	if (!pos->touched) {
		pos->x = 0;
		pos->y = 0;
		return -ENODATA;
	}

	const uint8_t x_msb = rx[1];
	const uint8_t y_msb = rx[2];
	const uint8_t lsb   = rx[3];

	pos->x = ((uint16_t)x_msb << 4) | (uint16_t)(lsb & 0x0FU);
	pos->y = ((uint16_t)y_msb << 4) | (uint16_t)((lsb >> 4) & 0x0FU);

	return 0;
}

/*
   @Brief         Set Core Register
   @Description   Set Core Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Core_Ram_Memory           ->  MTCH6102_Core
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Core(const struct i2c_dt_spec *i2c, MTCH6102_Core_Ram_Memory MTCH6102_Core, uint8_t value)
{
  touch_sensor_buf[0] = MTCH6102_Core;
  touch_sensor_buf[1] = value;
  i2c_write_dt(i2c, touch_sensor_buf, 2);
}

/*
   @Brief         Set Touch Register
   @Description   Set Touch Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Touch_Ram_Memory          ->  MTCH6102_Touch
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Touch(const struct i2c_dt_spec *i2c, MTCH6102_Touch_Ram_Memory MTCH6102_Touch, uint8_t value)
{
  touch_sensor_buf[0] = MTCH6102_Touch;
  touch_sensor_buf[1] = value;
  i2c_write_dt(i2c, touch_sensor_buf, 2);
}

/*
   @Brief         Get Touch Register
   @Description   Get Touch Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Touch_Ram_Memory          ->  MTCH6102_Touch
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Touch(const struct i2c_dt_spec *i2c, MTCH6102_Touch_Ram_Memory MTCH6102_Touch)
{
  touch_sensor_buf[0] = MTCH6102_Touch;
  uint8_t status[1] = {0};
  int ret = i2c_write_read_dt(i2c, touch_sensor_buf, 1, status, 1);
  if (ret != 0)
	{
		LOG_ERR("I2C bus %s: Error while reading registers", i2c->bus->name);
		return 0U;
	}
  return status[0];
}

/*
   @Brief         Get Compensation Value
   @Description   Get Compensation Register Value
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Compensation_Ram_Memory   ->  MTCH6102_Compensation
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Compensation(const struct i2c_dt_spec *i2c, MTCH6102_Compensation_Ram_Memory MTCH6102_Compensation)
{
  touch_sensor_buf[0] = MTCH6102_Compensation;
  uint8_t status[1] = {0};
  int ret = i2c_write_read_dt(i2c, touch_sensor_buf, 1, status, 1);
  if (ret != 0)
	{
		LOG_ERR("I2C bus %s: Error while reading registers", i2c->bus->name);
		return 0U;
	}
  return status[0];
}

/*
   @Brief         Get Acquisition Value
   @Description   Get Acquisition Register Value
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Acquisition_Ram_Memory    ->  MTCH6102_Acquisition
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Acquisition(const struct i2c_dt_spec *i2c, MTCH6102_Acquisition_Ram_Memory MTCH6102_Acquisition)
{
  touch_sensor_buf[0] = MTCH6102_Acquisition;
  int ret = i2c_write_read_dt(i2c, touch_sensor_buf, 1, touch_sensor_buf, 1);
  if (ret != 0)
	{
		LOG_ERR("I2C bus %s: Error while reading registers", i2c->bus->name);
		return 0U;
	}
  return touch_sensor_buf[0];
}

/*
   @Brief         Set Configuration Registers
   @Description   Get Configuration Register Value
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Configuration_Ram_Memory  ->  MTCH6102_Configuration
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Configuration(const struct i2c_dt_spec *i2c, MTCH6102_Configuration_Ram_Memory MTCH6102_Configuration, uint8_t value)
{
  touch_sensor_buf[0] = MTCH6102_Configuration;
  touch_sensor_buf[1] = value;
  i2c_write_dt(i2c, touch_sensor_buf, 2);
}

/*
   @Brief         Set Default Settings
   @Description   Set Default Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
   @Return value  None
 */
void MTCH6102_InitializeDEFAULT(const struct i2c_dt_spec *i2c)
{
  MTCH6102_set_Configuration(i2c, MTCH6102__NUMBER_OF_X_CHANNELS, 0x0C);
  MTCH6102_set_Configuration(i2c, MTCH6102__NUMBER_OF_Y_CHANNELS, 0x03);
  MTCH6102_set_Configuration(i2c, MTCH6102__SCAN_COUNT, 0x06);
  MTCH6102_set_Configuration(i2c, MTCH6102__TOUCH_THRESH_X, 0x37);
  MTCH6102_set_Configuration(i2c, MTCH6102__TOUCH_THRESH_Y, 0x28);
  MTCH6102_set_Configuration(i2c, MTCH6102__ACTIVE_PERIOD_L, 0x85);
  MTCH6102_set_Configuration(i2c, MTCH6102__ACTIVE_PERIOD_H, 0x02);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_PERIOD_L, 0x4C);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_PERIOD_H, 0x06);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_TIMEOUT, 0x10);
  MTCH6102_set_Configuration(i2c, MTCH6102__HYSTERESIS, 0x04);
  MTCH6102_set_Configuration(i2c, MTCH6102__DEBOUNCE_UP, 0x01);
  MTCH6102_set_Configuration(i2c, MTCH6102__DEBOUNCE_DOWN, 0x01);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_INTERVAL_L, 0x0A);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_INTERVAL_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_POS_FILTER, 0x14);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_NEG_FILTER, 0x14);
  MTCH6102_set_Configuration(i2c, MTCH6102__FILTER_TYPE, 0x02);
  MTCH6102_set_Configuration(i2c, MTCH6102__FILTER_STRENGTH, 0x01);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_FILTER_TYPE, 0x01);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_FILTER_STRENGTH, 0x05);
  MTCH6102_set_Configuration(i2c, MTCH6102__LARGE_ACTIVATION_THRESH_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__LARGE_ACTIVATION_THRESH_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__HORIZONTAL_SWIPE_DISTANCE, 0x40);
  MTCH6102_set_Configuration(i2c, MTCH6102__VERTICAL_SWIPE_DISTANCE, 0x40);
  MTCH6102_set_Configuration(i2c, MTCH6102__SWIPE_HOLD_BOUNDARY, 0x19);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_DISTANCE, 0x19);
  MTCH6102_set_Configuration(i2c, MTCH6102__DISTANCE_BETWEEN_TAPS, 0x40);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_HOLD_TIME_L, 0x32);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_HOLD_TIME_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__GESTURE_CLICK_TIME, 0x0C);
  MTCH6102_set_Configuration(i2c, MTCH6102__SWIPE_HOLD_THRESH, 0x20);
  MTCH6102_set_Configuration(i2c, MTCH6102__MIN_SWIPE_VELOCITY, 0x04);
  MTCH6102_set_Configuration(i2c, MTCH6102__HORIZONTAL_GESTURE_ANGLE, 0x2D);
  MTCH6102_set_Configuration(i2c, MTCH6102__VERTICAL_GESTURE_ANGLE, 0x2D);
  MTCH6102_set_Configuration(i2c, MTCH6102__I2CADDR, 0x25);

  MTCH6102_set_Core(i2c, MTCH6102__FW_MAJOR, 0x02);
  MTCH6102_set_Core(i2c, MTCH6102__FW_MINOR, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__APP_ID_H, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__APP_ID_L, 0x12);
  MTCH6102_set_Core(i2c, MTCH6102__CMD, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__MODE, 0x03);
  MTCH6102_set_Core(i2c, MTCH6102__MODE_CON, 0x00);

  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_STATE, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_X, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_Y, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_LSB, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__GESTURE_STATE, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__GESTURE_DIAG, 0x00);
}

/*
   @Brief         Set Custom Settings
   @Description   Set Custom Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
   @Return value  None
 */
void MTCH6102_Initialize(const struct i2c_dt_spec *i2c)
{
  MTCH6102_set_Configuration(i2c, MTCH6102__NUMBER_OF_X_CHANNELS, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__NUMBER_OF_Y_CHANNELS, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__SCAN_COUNT, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__TOUCH_THRESH_X, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__TOUCH_THRESH_Y, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__ACTIVE_PERIOD_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__ACTIVE_PERIOD_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_PERIOD_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_PERIOD_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__IDLE_TIMEOUT, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__HYSTERESIS, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__DEBOUNCE_UP, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__DEBOUNCE_DOWN, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_INTERVAL_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_INTERVAL_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_POS_FILTER, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_NEG_FILTER, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__FILTER_TYPE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__FILTER_STRENGTH, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_FILTER_TYPE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__BASE_FILTER_STRENGTH, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__LARGE_ACTIVATION_THRESH_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__LARGE_ACTIVATION_THRESH_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__HORIZONTAL_SWIPE_DISTANCE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__VERTICAL_SWIPE_DISTANCE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__SWIPE_HOLD_BOUNDARY, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_DISTANCE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__DISTANCE_BETWEEN_TAPS, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_HOLD_TIME_L, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__TAP_HOLD_TIME_H, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__GESTURE_CLICK_TIME, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__SWIPE_HOLD_THRESH, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__MIN_SWIPE_VELOCITY, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__HORIZONTAL_GESTURE_ANGLE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__VERTICAL_GESTURE_ANGLE, 0x00);
  MTCH6102_set_Configuration(i2c, MTCH6102__I2CADDR, 0x00);

  MTCH6102_set_Core(i2c, MTCH6102__FW_MAJOR, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__FW_MINOR, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__APP_ID_H, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__APP_ID_L, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__CMD, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__MODE, 0x00);
  MTCH6102_set_Core(i2c, MTCH6102__MODE_CON, 0x00);

  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_STATE, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_X, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_Y, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__TOUCH_LSB, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__GESTURE_STATE, 0x00);
  MTCH6102_set_Touch(i2c, MTCH6102__GESTURE_DIAG, 0x00);
}

static uint32_t touch_sample_period_ms(void)
{
	uint32_t hz = touch_sampling_rate_hz;

	if (hz == 0U) {
		hz = TOUCH_DEFAULT_SAMPLING_RATE_HZ;
	}

	return MAX(1U, 1000U / hz);
}

static int touch_sensor_read_sample(struct touch_sensor_sample *sample)
{
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	sample->position.touch_state = MTCH6102_get_Touch(&dev_i2c, MTCH6102__TOUCH_STATE);
	sample->gesture_state = MTCH6102_get_Touch(&dev_i2c, MTCH6102__GESTURE_STATE);
	ret = mtch6102_get_position(&dev_i2c, &sample->position);
	if (ret == -ENODATA) {
		return 0;
	}

	return ret;
}

static void touch_sample_work_fn(struct k_work *work)
{
	struct touch_sensor_sample sample;
	int ret;

	ARG_UNUSED(work);

	if (!atomic_get(&touch_streaming_enabled)) {
		return;
	}

	ret = touch_sensor_read_sample(&sample);
	if (ret == 0) {
		if (touch_callback != NULL) {
			touch_callback(&sample, touch_callback_user_data);
		}
	} else {
		LOG_ERR("Touch read failed: %d", ret);
	}

	k_work_schedule(&touch_sample_work, K_MSEC(touch_sample_period_ms()));
}

int touch_sensor_register_callback(touch_sensor_sample_cb_t cb, void *user_data)
{
	touch_callback = cb;
	touch_callback_user_data = user_data;
	return 0;
}

void touch_sensor_set_streaming_enabled(bool enabled)
{
	atomic_set(&touch_streaming_enabled, enabled ? 1 : 0);
}

void touch_sensor_set_sampling_rate(uint16_t new_sampling_rate)
{
	if (new_sampling_rate == 0U) {
		return;
	}

	touch_sampling_rate_hz = new_sampling_rate;

	if (touch_work_started && atomic_get(&touch_streaming_enabled)) {
		k_work_reschedule(&touch_sample_work, K_NO_WAIT);
	}
}

void touch_sensor_set_transfer_interval(uint16_t new_transfer_interval)
{
	ARG_UNUSED(new_transfer_interval);
}

int touch_sensor_start(void)
{
	if (!touch_initialized) {
		int err = touch_sensor_init();

		if (err != 0) {
			return err;
		}
	}

	touch_work_started = true;
	k_work_reschedule(&touch_sample_work, K_NO_WAIT);
	return 0;
}

void touch_sensor_stop(void)
{
	k_work_cancel_delayable(&touch_sample_work);
	touch_work_started = false;
}

void touch_sensor_deinit(void)
{
	touch_sensor_stop();
	atomic_set(&touch_streaming_enabled, 0);
	touch_initialized = false;
}

int touch_sensor_init(void)
{
	if (touch_initialized) {
		return 0;
	}

	if (!device_is_ready(dev_i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready!", dev_i2c.bus->name);
		return -ENODEV;
	}

	MTCH6102_InitializeDEFAULT(&dev_i2c);
	k_work_init_delayable(&touch_sample_work, touch_sample_work_fn);
	atomic_set(&touch_streaming_enabled, 0);
	touch_initialized = true;
	return 0;
}
