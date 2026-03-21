#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "app/daughter_board_manager.h"
#include "app/touch_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bluetooth/services/touch/touch_lbs.h"
#include "drivers/sensors/touch/mtch6102.h"

static bool touch_bridge_started;
static bool touch_board_connected;
static atomic_t touch_state_notify_enabled;
static atomic_t touch_gesture_notify_enabled;
static atomic_t touch_raw_notify_enabled;
static atomic_t touch_streaming_enabled;
static struct k_work touch_board_status_work;

static bool touch_notifications_enabled(void)
{
	return atomic_get(&touch_state_notify_enabled) ||
	       atomic_get(&touch_gesture_notify_enabled) ||
	       atomic_get(&touch_raw_notify_enabled);
}

static void touch_update_streaming_state(void)
{
	if (touch_notifications_enabled() && touch_board_connected) {
		atomic_set(&touch_streaming_enabled, 1);
		touch_sensor_set_streaming_enabled(true);
		(void)touch_sensor_start();
	} else {
		atomic_set(&touch_streaming_enabled, 0);
		touch_sensor_set_streaming_enabled(false);
		touch_sensor_stop();
	}
}

static void touch_state_notify_state_cb(bool enabled)
{
	atomic_set(&touch_state_notify_enabled, enabled ? 1 : 0);
	touch_update_streaming_state();
}

static void touch_gesture_notify_state_cb(bool enabled)
{
	atomic_set(&touch_gesture_notify_enabled, enabled ? 1 : 0);
	touch_update_streaming_state();
}

static void touch_raw_notify_state_cb(bool enabled)
{
	atomic_set(&touch_raw_notify_enabled, enabled ? 1 : 0);
	touch_update_streaming_state();
}

static void touch_sample_cb(const struct touch_sensor_sample *sample, void *user_data)
{
	uint8_t touch_state;
	uint8_t gesture_state;
	uint8_t raw_data[6];

	ARG_UNUSED(user_data);

	if (!atomic_get(&touch_streaming_enabled) || sample == NULL) {
		return;
	}

	touch_state = sample->position.touch_state;
	gesture_state = sample->gesture_state;

	if (atomic_get(&touch_state_notify_enabled)) {
		(void)touch_lbs_send_touch_state_notify(&touch_state, sizeof(touch_state));
	}

	if (atomic_get(&touch_gesture_notify_enabled)) {
		(void)touch_lbs_send_gesture_state_notify(&gesture_state, sizeof(gesture_state));
	}

	if (atomic_get(&touch_raw_notify_enabled)) {
		raw_data[0] = sample->position.touch_state;
		raw_data[1] = sample->gesture_state;
		sys_put_le16(sample->position.x, &raw_data[2]);
		sys_put_le16(sample->position.y, &raw_data[4]);
		(void)touch_lbs_send_raw_data_notify(raw_data, sizeof(raw_data));
	}
}

static void touch_board_status_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!touch_board_connected) {
		atomic_set(&touch_streaming_enabled, 0);
		touch_sensor_set_streaming_enabled(false);
		touch_sensor_deinit();
		return;
	}

	(void)touch_sensor_init();

	if (touch_notifications_enabled()) {
		touch_update_streaming_state();
	}
}

static void touch_board_status_changed(const struct power_lbs_daughter_state *state,
				       void *user_data)
{
	ARG_UNUSED(user_data);

	if (state == NULL) {
		return;
	}

	touch_board_connected = (state->connected_mask & POWER_LBS_DAUGHTER_MASK_TOUCH) != 0U;
	k_work_submit(&touch_board_status_work);
}

int touch_ble_bridge_init(void)
{
	struct power_lbs_daughter_state daughter_state = {0};

	if (touch_bridge_started) {
		return 0;
	}

	k_work_init(&touch_board_status_work, touch_board_status_work_fn);
	register_touch_touch_state_callback(touch_state_notify_state_cb);
	register_touch_gesture_state_callback(touch_gesture_notify_state_cb);
	register_touch_raw_data_callback(touch_raw_notify_state_cb);
	register_touch_sampling_rate_callback(touch_sensor_set_sampling_rate);
	register_touch_transfer_interval_callback(touch_sensor_set_transfer_interval);
	touch_sensor_register_callback(touch_sample_cb, NULL);
	daughter_board_manager_register_status_cb(touch_board_status_changed, NULL);
	if (daughter_board_manager_get_status(&daughter_state) == 0) {
		touch_board_connected =
			(daughter_state.connected_mask & POWER_LBS_DAUGHTER_MASK_TOUCH) != 0U;
	}
	atomic_set(&touch_state_notify_enabled, 0);
	atomic_set(&touch_gesture_notify_enabled, 0);
	atomic_set(&touch_raw_notify_enabled, 0);
	atomic_set(&touch_streaming_enabled, 0);
	touch_sensor_set_streaming_enabled(false);
	if (touch_board_connected) {
		k_work_submit(&touch_board_status_work);
	}
	touch_bridge_started = true;
	return 0;
}
