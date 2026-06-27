#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "app/daughter_board_manager.h"
#include "app/haptic_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bluetooth/services/haptic/haptic_lbs.h"
#include "drv2605.h"

#define HAPTIC_US_PER_MS 1000U

static const struct device *const haptic_dev = DEVICE_DT_GET(DT_ALIAS(sensewear_haptic));

static bool haptic_bridge_started;
static bool haptic_board_connected;
static struct k_work haptic_board_status_work;

/*
 * RTP playback on the DRV2605 driver runs asynchronously on a work queue and
 * streams directly from the buffers handed to drv2605_haptic_config(), so they
 * must outlive the call. A single static set is safe because we only refill it
 * while no stream is active (guarded by drv2605_rtp_is_active() under the mutex),
 * and the driver clears that flag only after the worker has finished reading.
 */
static K_MUTEX_DEFINE(haptic_rtp_mutex);
static uint32_t haptic_rtp_hold_us[HAPTIC_LBS_MAX_FRAMES];
static uint8_t haptic_rtp_input[HAPTIC_LBS_MAX_FRAMES];
static struct drv2605_rtp_data haptic_rtp_data = {
	.rtp_hold_us = haptic_rtp_hold_us,
	.rtp_input = haptic_rtp_input,
};

static int haptic_ble_run_pattern(const struct haptic_lbs_frame *frames, size_t frame_count)
{
	const union drv2605_config_data config_data = {.rtp_data = &haptic_rtp_data};
	int ret;

	if (frames == NULL || frame_count == 0U || frame_count > HAPTIC_LBS_MAX_FRAMES) {
		return -EINVAL;
	}

	if (!haptic_board_connected) {
		return -ENODEV;
	}

	if (!device_is_ready(haptic_dev)) {
		return -ENODEV;
	}

	k_mutex_lock(&haptic_rtp_mutex, K_FOREVER);

	if (drv2605_rtp_is_active(haptic_dev)) {
		k_mutex_unlock(&haptic_rtp_mutex);
		return -EBUSY;
	}

	for (size_t i = 0; i < frame_count; i++) {
		haptic_rtp_hold_us[i] = (uint32_t)frames[i].duration_ms * HAPTIC_US_PER_MS;
		haptic_rtp_input[i] = frames[i].intensity;
	}
	haptic_rtp_data.size = frame_count;

	ret = drv2605_haptic_config(haptic_dev, DRV2605_HAPTICS_SOURCE_RTP, &config_data);
	if (ret == 0) {
		ret = haptics_start_output(haptic_dev);
	}

	k_mutex_unlock(&haptic_rtp_mutex);
	return ret;
}

static const struct haptic_lbs_ops haptic_ble_ops = {
	.run_pattern = haptic_ble_run_pattern,
};

static void haptic_board_status_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!haptic_board_connected && device_is_ready(haptic_dev)) {
		(void)haptics_stop_output(haptic_dev);
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
