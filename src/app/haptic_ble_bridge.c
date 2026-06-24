#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "app/daughter_board_manager.h"
#include "app/haptic_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bluetooth/services/haptic/haptic_lbs.h"
#include "drv2605.h"

BUILD_ASSERT(HAPTIC_ACTUATOR_MAX_FRAMES >= HAPTIC_LBS_MAX_FRAMES,
	     "Actuator frame capacity must cover BLE haptic payloads");

static bool haptic_bridge_started;
static bool haptic_board_connected;
static bool haptic_actuator_initialized;
static struct k_work haptic_board_status_work;

static int haptic_ble_run_pattern(const struct haptic_lbs_frame *frames, size_t frame_count)
{
	struct haptic_actuator_frame actuator_frames[HAPTIC_LBS_MAX_FRAMES];
	int ret;

	if (frames == NULL || frame_count == 0U || frame_count > HAPTIC_LBS_MAX_FRAMES) {
		return -EINVAL;
	}

	if (!haptic_board_connected) {
		return -ENODEV;
	}

	ret = haptic_actuator_init();
	if (ret != 0) {
		haptic_actuator_initialized = false;
		return ret;
	}

	if (!haptic_actuator_initialized) {
		haptic_actuator_initialized = true;
	}

	for (size_t i = 0; i < frame_count; i++) {
		actuator_frames[i].duration_ms = frames[i].duration_ms;
		actuator_frames[i].intensity = frames[i].intensity;
	}

	return haptic_actuator_play_pattern(actuator_frames, frame_count);
}

static const struct haptic_lbs_ops haptic_ble_ops = {
	.run_pattern = haptic_ble_run_pattern,
};

static void haptic_board_status_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!haptic_board_connected) {
		haptic_actuator_deinit();
		haptic_actuator_initialized = false;
	}
}

static void haptic_board_status_changed(const struct power_lbs_daughter_state *state,
					void *user_data)
{
	ARG_UNUSED(user_data);

	if (state == NULL) {
		return;
	}

	haptic_board_connected = (state->connected_mask & POWER_LBS_DAUGHTER_MASK_HAPTIC) != 0U;
	k_work_submit(&haptic_board_status_work);
}

int haptic_ble_bridge_init(void)
{
	struct power_lbs_daughter_state daughter_state = {0};

	if (haptic_bridge_started) {
		return 0;
	}

	k_work_init(&haptic_board_status_work, haptic_board_status_work_fn);
	haptic_lbs_register_ops(&haptic_ble_ops);
	daughter_board_manager_register_status_cb(haptic_board_status_changed, NULL);
	if (daughter_board_manager_get_status(&daughter_state) == 0) {
		haptic_board_connected =
			(daughter_state.connected_mask & POWER_LBS_DAUGHTER_MASK_HAPTIC) != 0U;
	}
	if (haptic_board_connected) {
		k_work_submit(&haptic_board_status_work);
	}
	haptic_bridge_started = true;
	return 0;
}
