#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app/daughter_board_manager.h"
#include "drivers/actuators/haptic/drv2605.h"
#include "drivers/sensors/ppg/max30101.h"
#include "drivers/sensors/temperature/max30208.h"
#include "drivers/sensors/touch/mtch6102.h"

LOG_MODULE_REGISTER(SENSE_WEAR_DAUGHTER_BOARD_MANAGER);

#define TPSM83102_NODE DT_NODELABEL(tpsm83102)
#define MAX30101_NODE DT_NODELABEL(max30101)
#define MAX30208_NODE DT_NODELABEL(max30208)
#define MTCH6102_NODE DT_NODELABEL(mtch6102)
#define DRV2605_NODE DT_NODELABEL(drv2605)

#define DAUGHTER_BOARD_SCAN_INTERVAL_MS 1500
#define DAUGHTER_BOARD_RAMP_DELAY_MS 20
#define DAUGHTER_BOARD_LOW_UV 3700000
#define DAUGHTER_BOARD_HIGH_UV 5000000
#define DRV2605_REG_MODE 0x01
#define DAUGHTER_BOARD_STATUS_CB_MAX 8

static const struct device *const daughter_regulator = DEVICE_DT_GET(TPSM83102_NODE);
static const struct i2c_dt_spec ppg_i2c = I2C_DT_SPEC_GET(MAX30101_NODE);
static const struct i2c_dt_spec temperature_i2c = I2C_DT_SPEC_GET(MAX30208_NODE);
static const struct i2c_dt_spec touch_i2c = I2C_DT_SPEC_GET(MTCH6102_NODE);
static const struct i2c_dt_spec haptic_i2c = I2C_DT_SPEC_GET(DRV2605_NODE);

static struct k_work_delayable daughter_board_scan_work;
static K_MUTEX_DEFINE(daughter_board_lock);

static struct power_lbs_daughter_state daughter_board_state;
static bool daughter_board_initialized;
static bool ppg_active_requested;
static daughter_board_manager_status_cb_t daughter_board_status_cbs[DAUGHTER_BOARD_STATUS_CB_MAX];
static void *daughter_board_status_user_data[DAUGHTER_BOARD_STATUS_CB_MAX];

static bool probe_register_value(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t expected)
{
	uint8_t value = 0U;

	if ((spec == NULL) || !i2c_is_ready_dt(spec)) {
		return false;
	}

	if (i2c_write_read_dt(spec, &reg, sizeof(reg), &value, sizeof(value)) != 0) {
		return false;
	}

	return value == expected;
}

static bool probe_register_access(const struct i2c_dt_spec *spec, uint8_t reg)
{
	uint8_t value = 0U;

	if ((spec == NULL) || !i2c_is_ready_dt(spec)) {
		return false;
	}

	return i2c_write_read_dt(spec, &reg, sizeof(reg), &value, sizeof(value)) == 0;
}

static bool probe_ppg(void)
{
	return probe_register_value(&ppg_i2c, max30101_register_PartID, MAX30101_PART_ID);
}

static bool probe_temperature(void)
{
	return probe_register_value(&temperature_i2c, MAX30208_ID_ADDR, MAX30208_PART_ID_VALUE);
}

static bool probe_touch(void)
{
	return probe_register_value(&touch_i2c, MTCH6102__FW_MAJOR, 0x02U);
}

static bool probe_haptic(void)
{
	return probe_register_access(&haptic_i2c, DRV2605_REG_MODE);
}

static enum power_lbs_daughter_board_type choose_low_voltage_board(uint8_t connected_mask)
{
	if ((connected_mask & POWER_LBS_DAUGHTER_MASK_TEMPERATURE) != 0U) {
		return POWER_LBS_DAUGHTER_BOARD_TEMPERATURE;
	}

	if ((connected_mask & POWER_LBS_DAUGHTER_MASK_TOUCH) != 0U) {
		return POWER_LBS_DAUGHTER_BOARD_TOUCH;
	}

	if ((connected_mask & POWER_LBS_DAUGHTER_MASK_HAPTIC) != 0U) {
		return POWER_LBS_DAUGHTER_BOARD_HAPTIC;
	}

	if ((connected_mask & POWER_LBS_DAUGHTER_MASK_PPG) != 0U) {
		return POWER_LBS_DAUGHTER_BOARD_PPG;
	}

	return POWER_LBS_DAUGHTER_BOARD_NONE;
}

static int set_regulator_voltage_uv(int32_t uv)
{
	int err;

	err = regulator_set_voltage(daughter_regulator, uv, uv);
	if (err != 0) {
		LOG_ERR("Failed to set daughter regulator to %d uV: %d", uv, err);
		return err;
	}

	k_msleep(DAUGHTER_BOARD_RAMP_DELAY_MS);
	return 0;
}

static void publish_state_if_changed(const struct power_lbs_daughter_state *next_state)
{
	bool changed;
	struct power_lbs_daughter_state old_state;

	old_state = daughter_board_state;
	changed = memcmp(&old_state, next_state, sizeof(*next_state)) != 0;
	daughter_board_state = *next_state;

	if (changed) {
		LOG_INF("Daughter board status changed: mask=0x%02x active=%u regulator=%umV flags=0x%02x",
			next_state->connected_mask, next_state->active_board,
			next_state->regulator_mv, next_state->flags);
	}

	if (changed) {
		for (size_t i = 0; i < ARRAY_SIZE(daughter_board_status_cbs); i++) {
			if (daughter_board_status_cbs[i] != NULL) {
				daughter_board_status_cbs[i](next_state,
							 daughter_board_status_user_data[i]);
			}
		}
	}
}

static void refresh_daughter_board_state_locked(void)
{
	struct power_lbs_daughter_state next_state = {
		.flags = POWER_LBS_DAUGHTER_FLAG_REGULATOR_ENABLED,
		.regulator_mv = DAUGHTER_BOARD_LOW_UV / 1000U,
	};
	int err;
	uint8_t low_voltage_mask = 0U;

	if (ppg_active_requested) {
		next_state.flags |= POWER_LBS_DAUGHTER_FLAG_PPG_REQUESTED;
	}

	if ((daughter_board_state.active_board == POWER_LBS_DAUGHTER_BOARD_PPG) && ppg_active_requested) {
		err = set_regulator_voltage_uv(DAUGHTER_BOARD_HIGH_UV);
		if ((err == 0) && probe_ppg()) {
			next_state.connected_mask = POWER_LBS_DAUGHTER_MASK_PPG;
			next_state.active_board = POWER_LBS_DAUGHTER_BOARD_PPG;
			next_state.regulator_mv = DAUGHTER_BOARD_HIGH_UV / 1000U;
			publish_state_if_changed(&next_state);
			return;
		}
	}

	err = set_regulator_voltage_uv(DAUGHTER_BOARD_LOW_UV);
	if (err != 0) {
		next_state.flags = 0U;
		next_state.regulator_mv = 0U;
		publish_state_if_changed(&next_state);
		return;
	}

	if (probe_temperature()) {
		low_voltage_mask |= POWER_LBS_DAUGHTER_MASK_TEMPERATURE;
	}

	if (probe_touch()) {
		low_voltage_mask |= POWER_LBS_DAUGHTER_MASK_TOUCH;
	}

	if (probe_haptic()) {
		low_voltage_mask |= POWER_LBS_DAUGHTER_MASK_HAPTIC;
	}

	if (probe_ppg()) {
		low_voltage_mask |= POWER_LBS_DAUGHTER_MASK_PPG;
	}

	next_state.connected_mask = low_voltage_mask;
	next_state.active_board = choose_low_voltage_board(low_voltage_mask);

	if (ppg_active_requested && (low_voltage_mask & (POWER_LBS_DAUGHTER_MASK_TEMPERATURE |
						       POWER_LBS_DAUGHTER_MASK_TOUCH |
						       POWER_LBS_DAUGHTER_MASK_HAPTIC)) == 0U) {
		err = set_regulator_voltage_uv(DAUGHTER_BOARD_HIGH_UV);
		if ((err == 0) && probe_ppg()) {
			next_state.connected_mask |= POWER_LBS_DAUGHTER_MASK_PPG;
			next_state.active_board = POWER_LBS_DAUGHTER_BOARD_PPG;
			next_state.regulator_mv = DAUGHTER_BOARD_HIGH_UV / 1000U;
		} else {
			(void)set_regulator_voltage_uv(DAUGHTER_BOARD_LOW_UV);
			next_state.regulator_mv = DAUGHTER_BOARD_LOW_UV / 1000U;
		}
	}

	publish_state_if_changed(&next_state);
}

static void daughter_board_scan_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	refresh_daughter_board_state_locked();
	k_mutex_unlock(&daughter_board_lock);

	(void)k_work_reschedule(&daughter_board_scan_work,
			       K_MSEC(DAUGHTER_BOARD_SCAN_INTERVAL_MS));
}

int daughter_board_manager_init(void)
{
	int err;

	if (daughter_board_initialized) {
		return 0;
	}

	if (!device_is_ready(daughter_regulator)) {
		LOG_ERR("Daughter regulator is not ready");
		return -ENODEV;
	}

	err = regulator_enable(daughter_regulator);
	if (err != 0) {
		LOG_ERR("Failed to enable daughter regulator: %d", err);
		return err;
	}

	k_work_init_delayable(&daughter_board_scan_work, daughter_board_scan_work_fn);

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	refresh_daughter_board_state_locked();
	k_mutex_unlock(&daughter_board_lock);

	(void)k_work_reschedule(&daughter_board_scan_work,
			       K_MSEC(DAUGHTER_BOARD_SCAN_INTERVAL_MS));
	daughter_board_initialized = true;
	return 0;
}

int daughter_board_manager_get_status(struct power_lbs_daughter_state *state)
{
	if (state == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	*state = daughter_board_state;
	k_mutex_unlock(&daughter_board_lock);
	return 0;
}

int daughter_board_manager_set_ppg_active(bool active)
{
	if (!daughter_board_initialized) {
		return -EACCES;
	}

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	ppg_active_requested = active;
	refresh_daughter_board_state_locked();
	k_mutex_unlock(&daughter_board_lock);

	(void)k_work_reschedule(&daughter_board_scan_work,
			       K_MSEC(DAUGHTER_BOARD_SCAN_INTERVAL_MS));
	return 0;
}

void daughter_board_manager_register_status_cb(daughter_board_manager_status_cb_t cb,
					       void *user_data)
{
	k_mutex_lock(&daughter_board_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(daughter_board_status_cbs); i++) {
		if (daughter_board_status_cbs[i] == cb &&
		    daughter_board_status_user_data[i] == user_data) {
			k_mutex_unlock(&daughter_board_lock);
			return;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(daughter_board_status_cbs); i++) {
		if (daughter_board_status_cbs[i] == NULL) {
			daughter_board_status_cbs[i] = cb;
			daughter_board_status_user_data[i] = user_data;
			k_mutex_unlock(&daughter_board_lock);
			return;
		}
	}

	k_mutex_unlock(&daughter_board_lock);
	LOG_ERR("No free daughter board status callback slots");
}
