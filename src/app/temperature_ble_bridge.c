#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/daughter_board_manager.h"
#include "app/temperature_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bluetooth/services/temperature/temperature_lbs.h"
#include "max30208.h"

static bool temperature_bridge_started;
static bool temperature_board_connected;
static atomic_t temperature_notify_enabled;
static atomic_t temperature_streaming_enabled;
static struct k_work temperature_board_status_work;

static void temperature_update_streaming_state(void)
{
	bool enabled = atomic_get(&temperature_notify_enabled);

	if (enabled && temperature_board_connected) {
		atomic_set(&temperature_streaming_enabled, 1);
		temperature_sensor_set_streaming_enabled(true);
		(void)temperature_sensor_start();
	} else {
		atomic_set(&temperature_streaming_enabled, 0);
		temperature_sensor_set_streaming_enabled(false);
		temperature_sensor_stop();
	}
}

static void temperature_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&temperature_notify_enabled, enabled ? 1 : 0);
	temperature_update_streaming_state();
}

static void temperature_board_status_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!temperature_board_connected) {
		atomic_set(&temperature_streaming_enabled, 0);
		temperature_sensor_set_streaming_enabled(false);
		temperature_sensor_deinit();
		return;
	}

	(void)temperature_sensor_init();

	if (atomic_get(&temperature_notify_enabled)) {
		temperature_update_streaming_state();
	}
}

static void temperature_board_status_changed(const struct power_lbs_daughter_state *state,
					     void *user_data)
{
	ARG_UNUSED(user_data);

	if (state == NULL) {
		return;
	}

	temperature_board_connected =
		(state->connected_mask & POWER_LBS_DAUGHTER_MASK_TEMPERATURE) != 0U;
	k_work_submit(&temperature_board_status_work);
}

static void temperature_sample_cb(const struct temperature_sample *sample, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!atomic_get(&temperature_streaming_enabled) || (sample == NULL)) {
		return;
	}

	struct temperature_lbs_sample ble_sample = {
		.temperature_mdeg_c = sample->temperature_mdeg_c,
	};

	(void)temperature_lbs_notify(&ble_sample);
}

int temperature_ble_bridge_init(void)
{
	struct power_lbs_daughter_state daughter_state = {0};

	if (temperature_bridge_started) {
		return 0;
	}

	k_work_init(&temperature_board_status_work, temperature_board_status_work_fn);
	temperature_lbs_register_notify_cb(temperature_notify_state_cb, NULL);
	temperature_lbs_register_sampling_rate_cb(temperature_sensor_set_sampling_rate);
	temperature_lbs_register_transfer_interval_cb(temperature_sensor_set_transfer_interval);
	temperature_sensor_register_callback(temperature_sample_cb, NULL);
	daughter_board_manager_register_status_cb(temperature_board_status_changed, NULL);
	if (daughter_board_manager_get_status(&daughter_state) == 0) {
		temperature_board_connected =
			(daughter_state.connected_mask & POWER_LBS_DAUGHTER_MASK_TEMPERATURE) != 0U;
	}
	atomic_set(&temperature_notify_enabled, 0);
	atomic_set(&temperature_streaming_enabled, 0);
	temperature_sensor_set_streaming_enabled(false);
	if (temperature_board_connected) {
		k_work_submit(&temperature_board_status_work);
	}
	temperature_bridge_started = true;
	return 0;
}
