#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app/daughter_board_manager.h"

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
#include "drivers/actuators/haptic/drv2605.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
#include "drivers/sensors/ppg/max30101.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
#include "drivers/sensors/temperature/max30208.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
#include "drivers/sensors/touch/mtch6102.h"
#endif

LOG_MODULE_REGISTER(SENSE_WEAR_DAUGHTER_BOARD_MANAGER);

#define TPSM83102_NODE DT_NODELABEL(tpsm83102)
#define SENSEWEAR_DAUGHTER_NODE DT_ALIAS(sensewear_daughter)

#define DAUGHTER_BOARD_SCAN_INTERVAL_MS 1500
#define DAUGHTER_BOARD_RAMP_DELAY_MS 20
#define DAUGHTER_BOARD_TOUCH_UV 3700000
#define DAUGHTER_BOARD_PPG_UV 5000000
#define DAUGHTER_BOARD_TEMPERATURE_UV 3700000
#define DAUGHTER_BOARD_HAPTIC_UV 3700000
#define DRV2605_REG_MODE 0x01
#define DAUGHTER_BOARD_STATUS_CB_MAX 8

#define DAUGHTER_BOARD_SELECTION_COUNT							\
	(IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE) +				\
	 IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH) +				\
	 IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG) +				\
	 IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE) +			\
	 IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC))

BUILD_ASSERT(DAUGHTER_BOARD_SELECTION_COUNT == 1,
	     "Select exactly one SenseWear daughter board option");
BUILD_ASSERT(DT_NODE_HAS_STATUS(TPSM83102_NODE, okay),
	     "SensWear board DTS must enable the tpsm83102 daughter regulator");

#if !IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE)
BUILD_ASSERT(DT_NODE_HAS_STATUS(SENSEWEAR_DAUGHTER_NODE, okay),
	     "Selected daughter board must provide an okay sensewear-daughter alias");
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
#define SELECTED_DAUGHTER_MASK POWER_LBS_DAUGHTER_MASK_TOUCH
#define SELECTED_DAUGHTER_TYPE POWER_LBS_DAUGHTER_BOARD_TOUCH
#define SELECTED_DAUGHTER_NAME "touch"
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
#define SELECTED_DAUGHTER_MASK POWER_LBS_DAUGHTER_MASK_PPG
#define SELECTED_DAUGHTER_TYPE POWER_LBS_DAUGHTER_BOARD_PPG
#define SELECTED_DAUGHTER_NAME "ppg"
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
#define SELECTED_DAUGHTER_MASK POWER_LBS_DAUGHTER_MASK_TEMPERATURE
#define SELECTED_DAUGHTER_TYPE POWER_LBS_DAUGHTER_BOARD_TEMPERATURE
#define SELECTED_DAUGHTER_NAME "temperature"
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
#define SELECTED_DAUGHTER_MASK POWER_LBS_DAUGHTER_MASK_HAPTIC
#define SELECTED_DAUGHTER_TYPE POWER_LBS_DAUGHTER_BOARD_HAPTIC
#define SELECTED_DAUGHTER_NAME "haptic"
#else
#define SELECTED_DAUGHTER_MASK 0U
#define SELECTED_DAUGHTER_TYPE POWER_LBS_DAUGHTER_BOARD_NONE
#define SELECTED_DAUGHTER_NAME "none"
#endif

static const struct device *const daughter_regulator = DEVICE_DT_GET(TPSM83102_NODE);
#if !IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE)
static const struct i2c_dt_spec daughter_i2c = I2C_DT_SPEC_GET(SENSEWEAR_DAUGHTER_NODE);
#endif
static struct k_work_delayable daughter_board_scan_work;
static K_MUTEX_DEFINE(daughter_board_lock);

static struct power_lbs_daughter_state daughter_board_state;
static bool daughter_board_initialized;
static bool daughter_board_present;
static bool daughter_regulator_ready;
static bool ppg_active_requested;
static int32_t daughter_regulator_uv;
static daughter_board_manager_status_cb_t daughter_board_status_cbs[DAUGHTER_BOARD_STATUS_CB_MAX];
static void *daughter_board_status_user_data[DAUGHTER_BOARD_STATUS_CB_MAX];

static int32_t selected_regulator_voltage_uv(void)
{
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
	return DAUGHTER_BOARD_TOUCH_UV;
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	return DAUGHTER_BOARD_PPG_UV;
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
	return DAUGHTER_BOARD_TEMPERATURE_UV;
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
	return DAUGHTER_BOARD_HAPTIC_UV;
#else
	return 0;
#endif
}

static uint8_t selected_state_flags(void)
{
	uint8_t flags = 0U;

	if (daughter_regulator_ready) {
		flags |= POWER_LBS_DAUGHTER_FLAG_REGULATOR_ENABLED;
	}

	if (IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG) && ppg_active_requested) {
		flags |= POWER_LBS_DAUGHTER_FLAG_PPG_REQUESTED;
	}

	return flags;
}

static bool refresh_daughter_board_state_locked(void)
{
	struct power_lbs_daughter_state next_state = {
		.connected_mask = daughter_board_present ? SELECTED_DAUGHTER_MASK : 0U,
		.active_board = daughter_board_present ? SELECTED_DAUGHTER_TYPE :
							  POWER_LBS_DAUGHTER_BOARD_NONE,
		.regulator_mv = (uint16_t)(daughter_regulator_uv / 1000),
		.flags = selected_state_flags(),
	};
	bool changed = memcmp(&daughter_board_state, &next_state, sizeof(next_state)) != 0;

	daughter_board_state = next_state;
	return changed;
}

static void notify_status_cbs(const struct power_lbs_daughter_state *state)
{
	for (size_t i = 0; i < ARRAY_SIZE(daughter_board_status_cbs); i++) {
		if (daughter_board_status_cbs[i] != NULL) {
			daughter_board_status_cbs[i](state, daughter_board_status_user_data[i]);
		}
	}
}

static bool probe_register_value(uint8_t reg, uint8_t expected)
{
#if !IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE)
	uint8_t value = 0U;

	if (!i2c_is_ready_dt(&daughter_i2c)) {
		return false;
	}

	if (i2c_write_read_dt(&daughter_i2c, &reg, sizeof(reg), &value, sizeof(value)) != 0) {
		return false;
	}

	return value == expected;
#else
	ARG_UNUSED(reg);
	ARG_UNUSED(expected);
	return false;
#endif
}

static bool probe_register_access(uint8_t reg)
{
#if !IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE)
	uint8_t value = 0U;

	if (!i2c_is_ready_dt(&daughter_i2c)) {
		return false;
	}

	return i2c_write_read_dt(&daughter_i2c, &reg, sizeof(reg), &value, sizeof(value)) == 0;
#else
	ARG_UNUSED(reg);
	return false;
#endif
}

static bool probe_selected_daughter_board(void)
{
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
	return probe_register_value(MTCH6102__FW_MAJOR, 0x02U);
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	return probe_register_value(max30101_register_PartID, MAX30101_PART_ID);
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
	return probe_register_value(MAX30208_ID_ADDR, MAX30208_PART_ID_VALUE);
#elif IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
	return probe_register_access(DRV2605_REG_MODE);
#else
	return false;
#endif
}

static void update_presence_and_notify(bool present)
{
	struct power_lbs_daughter_state state;
	bool changed;

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	daughter_board_present = present;
	changed = refresh_daughter_board_state_locked();
	state = daughter_board_state;
	k_mutex_unlock(&daughter_board_lock);

	if (!changed) {
		return;
	}

	LOG_INF("Daughter board %s: connected=%u regulator=%umV",
		SELECTED_DAUGHTER_NAME, present ? 1U : 0U, state.regulator_mv);
	notify_status_cbs(&state);
}

static void daughter_board_scan_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	update_presence_and_notify(probe_selected_daughter_board());
	(void)k_work_schedule(&daughter_board_scan_work,
			       K_MSEC(DAUGHTER_BOARD_SCAN_INTERVAL_MS));
}

static int set_regulator_voltage_uv(int32_t uv)
{
	int err;

	if (uv == 0) {
		daughter_regulator_uv = 0;
		return 0;
	}

	err = regulator_set_voltage(daughter_regulator, uv, uv);
	if (err != 0) {
		LOG_ERR("Failed to set daughter regulator to %d uV: %d", uv, err);
		return err;
	}

	k_msleep(DAUGHTER_BOARD_RAMP_DELAY_MS);
	daughter_regulator_uv = uv;
	return 0;
}

static int init_daughter_regulator(void)
{
	int err;

	daughter_regulator_ready = false;
	daughter_regulator_uv = 0;

	if (IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_NONE)) {
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

	err = set_regulator_voltage_uv(selected_regulator_voltage_uv());
	if (err != 0) {
		return err;
	}

	daughter_regulator_ready = true;
	return 0;
}

int daughter_board_manager_init(void)
{
	struct power_lbs_daughter_state state;
	bool changed;
	int err;

	if (daughter_board_initialized) {
		return 0;
	}

	err = init_daughter_regulator();
	if (err != 0) {
		LOG_ERR("Daughter regulator unavailable; reporting %s daughter as disconnected: %d",
			SELECTED_DAUGHTER_NAME, err);
	}

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	daughter_board_initialized = true;
	daughter_board_present = daughter_regulator_ready ? probe_selected_daughter_board() : false;
	changed = refresh_daughter_board_state_locked();
	state = daughter_board_state;
	k_mutex_unlock(&daughter_board_lock);

	LOG_INF("Daughter board selection: %s connected=%u regulator=%umV",
		SELECTED_DAUGHTER_NAME, daughter_board_present ? 1U : 0U,
		state.regulator_mv);

	if (changed) {
		notify_status_cbs(&state);
	}

	if (daughter_regulator_ready) {
		k_work_init_delayable(&daughter_board_scan_work, daughter_board_scan_work_fn);
		(void)k_work_schedule(&daughter_board_scan_work,
				       K_MSEC(DAUGHTER_BOARD_SCAN_INTERVAL_MS));
	}

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
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	struct power_lbs_daughter_state state;
	bool changed;

	if (!daughter_board_initialized) {
		return -EACCES;
	}

	if (!daughter_regulator_ready) {
		return -ENODEV;
	}

	k_mutex_lock(&daughter_board_lock, K_FOREVER);
	ppg_active_requested = active;
	changed = refresh_daughter_board_state_locked();
	state = daughter_board_state;
	k_mutex_unlock(&daughter_board_lock);

	if (changed) {
		LOG_INF("PPG daughter board %s", active ? "active" : "idle");
		notify_status_cbs(&state);
	}

	return 0;
#else
	ARG_UNUSED(active);
	return -ENOTSUP;
#endif
}

void daughter_board_manager_register_status_cb(daughter_board_manager_status_cb_t cb,
					       void *user_data)
{
	if (cb == NULL) {
		return;
	}

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
