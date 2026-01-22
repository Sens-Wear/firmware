#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/ppg_ble_bridge.h"
#include "bluetooth/services/ppg/ppg_lbs.h"
#include "drivers/sensors/ppg/max30101.h"

static bool ppg_bridge_started;
static atomic_t ppg_red_notify_enabled;
static atomic_t ppg_ir_notify_enabled;
static atomic_t ppg_green_notify_enabled;

static void ppg_update_streaming_state(void)
{
	bool enabled = atomic_get(&ppg_red_notify_enabled) ||
		       atomic_get(&ppg_ir_notify_enabled) ||
		       atomic_get(&ppg_green_notify_enabled);

	ppg_set_streaming_enabled(enabled);
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
	if (ppg_bridge_started) {
		return 0;
	}

	ppg_sensor_init();
	ppg_lbs_register_red_notify_cb(ppg_red_notify_state_cb, NULL);
	ppg_lbs_register_ir_notify_cb(ppg_ir_notify_state_cb, NULL);
	ppg_lbs_register_green_notify_cb(ppg_green_notify_state_cb, NULL);
	ppg_set_streaming_enabled(false);
	ppg_bridge_started = true;
	return 0;
}
