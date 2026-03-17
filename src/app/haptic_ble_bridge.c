#include <errno.h>

#include <zephyr/sys/util.h>

#include "app/haptic_ble_bridge.h"
#include "bluetooth/services/haptic/haptic_lbs.h"
#include "drivers/actuators/haptic/drv2605.h"

BUILD_ASSERT(HAPTIC_ACTUATOR_MAX_FRAMES >= HAPTIC_LBS_MAX_FRAMES,
	     "Actuator frame capacity must cover BLE haptic payloads");

static int haptic_ble_run_pattern(const struct haptic_lbs_frame *frames, size_t frame_count)
{
	struct haptic_actuator_frame actuator_frames[HAPTIC_LBS_MAX_FRAMES];

	if (frames == NULL || frame_count == 0U || frame_count > HAPTIC_LBS_MAX_FRAMES) {
		return -EINVAL;
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

int haptic_ble_bridge_init(void)
{
	int ret;

	ret = haptic_actuator_init();
	if (ret != 0) {
		return ret;
	}

	haptic_lbs_register_ops(&haptic_ble_ops);
	return 0;
}
