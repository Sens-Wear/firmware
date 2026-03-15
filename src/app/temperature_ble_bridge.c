#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/temperature_ble_bridge.h"
#include "bluetooth/services/temperature/temperature_lbs.h"
#include "drivers/sensors/temperature/max30208.h"

static bool temperature_bridge_started;
static atomic_t temperature_notify_enabled;
static atomic_t temperature_streaming_enabled;

static void temperature_update_streaming_state(void)
{
	bool enabled = atomic_get(&temperature_notify_enabled);

	if (enabled) {
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
	if (temperature_bridge_started) {
		return 0;
	}

	temperature_lbs_register_notify_cb(temperature_notify_state_cb, NULL);
	temperature_lbs_register_sampling_rate_cb(temperature_sensor_set_sampling_rate);
	temperature_lbs_register_transfer_interval_cb(temperature_sensor_set_transfer_interval);
	temperature_sensor_register_callback(temperature_sample_cb, NULL);
	atomic_set(&temperature_notify_enabled, 0);
	atomic_set(&temperature_streaming_enabled, 0);
	temperature_sensor_set_streaming_enabled(false);
	temperature_bridge_started = true;
	return 0;
}
