#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/daughter_board_manager.h"
#include "app/power_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "bq25180.h"
#include "bq27427.h"
#include "device_driver_dts_ids.h"

#define POWER_BLE_POLL_INTERVAL_MS 2000

static bool power_bridge_started;
static bool charger_checked;
static bool charger_available;
static bool gauge_checked;
static bool gauge_available;
static atomic_t charger_notify_enabled;
static atomic_t gauge_notify_enabled;
static atomic_t daughter_notify_enabled;
static struct k_work_delayable power_poll_work;
static struct k_work daughter_notify_work;

static struct power_lbs_charger_state last_charger_state;
static struct power_lbs_gauge_state last_gauge_state;
static struct power_lbs_daughter_state last_daughter_state;
static bool have_last_charger_state;
static bool have_last_gauge_state;
static bool have_last_daughter_state;

static void power_ble_stub_charger_state(struct power_lbs_charger_state* state) {
	memset(state, 0, sizeof(*state));
}

static void power_ble_stub_gauge_state(struct power_lbs_gauge_state* state) {
	memset(state, 0, sizeof(*state));
}

static void power_ble_init_optional_charger(void) {
	struct bq25180_config_t charger_config;

	if (charger_checked) {
		return;
	}

	charger_checked = true;
	charger_available = false;

	if (bq25180_init() && bq25180_is_ready()) {
		bq25180_get_default_lipo_usb_charger_config(&charger_config);
		charger_config.battery_uvlo = bq25180_battery_UVLO_threshold_2V8;
		charger_available = bq25180_config(&charger_config);
	}
}

static void power_ble_init_optional_gauge(void) {
	struct bq27427_config_t gauge_config;

	if (gauge_checked) {
		return;
	}

	gauge_checked = true;
	gauge_available = false;

	if (bq27427_init() && bq27427_is_ready()) {
		bq27427_get_default_config(&gauge_config);
		gauge_config.battery_capacity = 450;
		gauge_available = bq27427_config(&gauge_config);
	}
}

static int power_ble_get_charger_state(struct power_lbs_charger_state* state) {
	union bq25180_charger_state_t charger;

	if (state == NULL) {
		return -EINVAL;
	}

	power_ble_init_optional_charger();

	if (!charger_available || !bq25180_update_state(&charger)) {
		power_ble_stub_charger_state(state);
		return 0;
	}

	state->flags = (uint32_t) charger.value;
	return 0;
}

static int power_ble_get_gauge_state(struct power_lbs_gauge_state* state) {
	struct bq27427_battery_state_t gauge;

	if (state == NULL) {
		return -EINVAL;
	}

	power_ble_init_optional_gauge();

	if (!gauge_available || !bq27427_update_state(&gauge)) {
		power_ble_stub_gauge_state(state);
		return 0;
	}

	state->temperature_cdec = (int16_t) gauge.temperature;
	state->voltage_mv = (uint16_t) gauge.voltage;
	state->average_current_ma = (int16_t) gauge.average_current;
	state->average_power_mw = (int16_t) gauge.average_power;
	state->state_of_charge_cdec = (uint16_t) gauge.state_of_charge;
	state->nominal_available_capacity_mah = (uint16_t) gauge.nominal_available_capacity;
	state->full_battery_capacity_mah = (uint16_t) gauge.full_battery_capacity;
	state->remaining_capacity_mah = (uint16_t) gauge.remaining_capacity;
	return 0;
}

static int power_ble_get_daughter_state(struct power_lbs_daughter_state* state) {
	return daughter_board_manager_get_status(state);
}

static void power_ble_daughter_notify_work_fn(struct k_work* work) {
	struct power_lbs_daughter_state current;

	ARG_UNUSED(work);

	if (!atomic_get(&daughter_notify_enabled)) {
		return;
	}

	if (power_ble_get_daughter_state(&current) != 0) {
		return;
	}

	last_daughter_state = current;
	have_last_daughter_state = true;
	(void) power_lbs_notify_daughter_state(&current);
}

static void power_ble_poll_work_fn(struct k_work* work) {
	ARG_UNUSED(work);

	bool charger_enabled = atomic_get(&charger_notify_enabled);
	bool gauge_enabled = atomic_get(&gauge_notify_enabled);

	if (!charger_enabled && !gauge_enabled) {
		return;
	}

	if (charger_enabled) {
		struct power_lbs_charger_state current;
		if (power_ble_get_charger_state(&current) == 0) {
			if (!have_last_charger_state || current.flags != last_charger_state.flags) {
				last_charger_state = current;
				have_last_charger_state = true;
				(void) power_lbs_notify_charger_state(&current);
			}
		}
	}

	if (gauge_enabled) {
		struct power_lbs_gauge_state current;
		if (power_ble_get_gauge_state(&current) == 0) {
			if (!have_last_gauge_state ||
				memcmp(&current, &last_gauge_state, sizeof(current)) != 0) {
				last_gauge_state = current;
				have_last_gauge_state = true;
				(void) power_lbs_notify_gauge_state(&current);
			}
		}
	}

	k_work_schedule(&power_poll_work, K_MSEC(POWER_BLE_POLL_INTERVAL_MS));
}

static void power_ble_update_polling_state(void) {
	bool enabled = atomic_get(&charger_notify_enabled) || atomic_get(&gauge_notify_enabled);

	if (enabled) {
		k_work_reschedule(&power_poll_work, K_NO_WAIT);
	} else {
		k_work_cancel_delayable(&power_poll_work);
	}
}

static void power_ble_charger_notify_state_cb(bool enabled, void* user_data) {
	ARG_UNUSED(user_data);
	atomic_set(&charger_notify_enabled, enabled ? 1 : 0);
	power_ble_update_polling_state();
}

static void power_ble_gauge_notify_state_cb(bool enabled, void* user_data) {
	ARG_UNUSED(user_data);
	atomic_set(&gauge_notify_enabled, enabled ? 1 : 0);
	power_ble_update_polling_state();
}

static void power_ble_daughter_status_changed(const struct power_lbs_daughter_state* state,
											  void* user_data) {
	ARG_UNUSED(user_data);

	if ((state == NULL) || !atomic_get(&daughter_notify_enabled)) {
		return;
	}

	last_daughter_state = *state;
	have_last_daughter_state = true;
	k_work_submit(&daughter_notify_work);
}

static void power_ble_daughter_notify_state_cb(bool enabled, void* user_data) {
	struct power_lbs_daughter_state current;

	ARG_UNUSED(user_data);
	atomic_set(&daughter_notify_enabled, enabled ? 1 : 0);

	if (!enabled) {
		return;
	}

	if (power_ble_get_daughter_state(&current) == 0) {
		last_daughter_state = current;
		have_last_daughter_state = true;
	}

	k_work_submit(&daughter_notify_work);
}

static const struct power_lbs_ops power_ble_ops = {
	.get_charger_state = power_ble_get_charger_state,
	.get_gauge_state = power_ble_get_gauge_state,
	.get_daughter_state = power_ble_get_daughter_state,
};

int power_ble_bridge_init(void) {
	if (power_bridge_started) {
		return 0;
	}

	k_work_init_delayable(&power_poll_work, power_ble_poll_work_fn);
	k_work_init(&daughter_notify_work, power_ble_daughter_notify_work_fn);
	charger_checked = false;
	charger_available = false;
	gauge_checked = false;
	gauge_available = false;
	atomic_set(&charger_notify_enabled, 0);
	atomic_set(&gauge_notify_enabled, 0);
	atomic_set(&daughter_notify_enabled, 0);
	have_last_charger_state = false;
	have_last_gauge_state = false;
	have_last_daughter_state = false;

	power_lbs_register_ops(&power_ble_ops);
	power_lbs_register_charger_notify_cb(power_ble_charger_notify_state_cb, NULL);
	power_lbs_register_gauge_notify_cb(power_ble_gauge_notify_state_cb, NULL);
	power_lbs_register_daughter_notify_cb(power_ble_daughter_notify_state_cb, NULL);
	daughter_board_manager_register_status_cb(power_ble_daughter_status_changed, NULL);

	power_bridge_started = true;
	return 0;
}
