#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/daughter_board_manager.h"
#include "app/ppg_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bluetooth/services/ppg/ppg_lbs.h"
#include "max30101.h"

static bool ppg_bridge_started;
static bool ppg_sensor_initialized;
static bool ppg_board_connected;
static atomic_t ppg_red_notify_enabled;
static atomic_t ppg_ir_notify_enabled;
static atomic_t ppg_green_notify_enabled;
static struct k_work ppg_board_status_work;

static void ppg_ensure_sensor_initialized(void)
{
	if (ppg_sensor_initialized) {
		return;
	}

	ppg_sensor_init();
	ppg_sensor_initialized = true;
}

static void ppg_update_streaming_state(void)
{
	bool enabled = atomic_get(&ppg_red_notify_enabled) ||
		       atomic_get(&ppg_ir_notify_enabled) ||
		       atomic_get(&ppg_green_notify_enabled);

	if (enabled) {
		(void)daughter_board_manager_set_ppg_active(true);

		if (ppg_board_connected) {
			ppg_ensure_sensor_initialized();
			ppg_set_streaming_enabled(true);
		} else {
			ppg_set_streaming_enabled(false);
		}
	} else {
		ppg_set_streaming_enabled(false);
		(void)daughter_board_manager_set_ppg_active(false);
	}
}

static void ppg_board_status_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!ppg_board_connected) {
		ppg_set_streaming_enabled(false);
		ppg_sensor_initialized = false;
		return;
	}

	ppg_ensure_sensor_initialized();

	if (atomic_get(&ppg_red_notify_enabled) ||
	    atomic_get(&ppg_ir_notify_enabled) ||
	    atomic_get(&ppg_green_notify_enabled)) {
		ppg_set_streaming_enabled(true);
	}
}

static void ppg_board_status_changed(const struct power_lbs_daughter_state *state,
				     void *user_data)
{
	ARG_UNUSED(user_data);

	if (state == NULL) {
		return;
	}

	ppg_board_connected = (state->connected_mask & POWER_LBS_DAUGHTER_MASK_PPG) != 0U;
	k_work_submit(&ppg_board_status_work);
}

static void ppg_red_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&ppg_red_notify_enabled, enabled ? 1 : 0);
	ppg_update_streaming_state();
}

static void ppg_ir_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&ppg_ir_notify_enabled, enabled ? 1 : 0);
	ppg_update_streaming_state();
}

static void ppg_green_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&ppg_green_notify_enabled, enabled ? 1 : 0);
	ppg_update_streaming_state();
}

int ppg_ble_bridge_init(void)
{
	struct power_lbs_daughter_state daughter_state = {0};

	if (ppg_bridge_started) {
		return 0;
	}

	k_work_init(&ppg_board_status_work, ppg_board_status_work_fn);
	ppg_lbs_register_red_notify_cb(ppg_red_notify_state_cb, NULL);
	ppg_lbs_register_ir_notify_cb(ppg_ir_notify_state_cb, NULL);
	ppg_lbs_register_green_notify_cb(ppg_green_notify_state_cb, NULL);
	daughter_board_manager_register_status_cb(ppg_board_status_changed, NULL);
	if (daughter_board_manager_get_status(&daughter_state) == 0) {
		ppg_board_connected =
			(daughter_state.connected_mask & POWER_LBS_DAUGHTER_MASK_PPG) != 0U;
	}
	ppg_sensor_initialized = false;
	ppg_set_streaming_enabled(false);
	if (ppg_board_connected) {
		k_work_submit(&ppg_board_status_work);
	}
	ppg_bridge_started = true;
	return 0;
}
