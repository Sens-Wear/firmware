#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/power_ble_bridge.h"
#include "bluetooth/services/power/power_lbs.h"
#include "drivers/power/charger/bq25180.h"
#include "drivers/power/gauge/bq27427.h"

#define POWER_BLE_POLL_INTERVAL_MS 2000

static bool power_bridge_started;
static atomic_t charger_notify_enabled;
static atomic_t gauge_notify_enabled;
static struct k_work_delayable power_poll_work;

static struct power_lbs_charger_state last_charger_state;
static struct power_lbs_gauge_state last_gauge_state;
static bool have_last_charger_state;
static bool have_last_gauge_state;

static int power_ble_get_charger_state(struct power_lbs_charger_state *state)
{
	union bq25180_charger_state_t charger;

	if (state == NULL) {
		return -EINVAL;
	}

	if (!bq25180_update_state(&charger)) {
		return -EIO;
	}

	state->flags = (uint32_t)charger.value;
	return 0;
}

static int power_ble_get_gauge_state(struct power_lbs_gauge_state *state)
{
	struct bq27427_battery_state_t gauge;

	if (state == NULL) {
		return -EINVAL;
	}

	if (!bq27427_update_state(&gauge)) {
		return -EIO;
	}

	state->temperature_cdec = (int16_t)gauge.temperature;
	state->voltage_mv = (uint16_t)gauge.voltage;
	state->average_current_ma = (int16_t)gauge.average_current;
	state->average_power_mw = (int16_t)gauge.average_power;
	state->state_of_charge_cdec = (uint16_t)gauge.state_of_charge;
	state->nominal_available_capacity_mah = (uint16_t)gauge.nominal_available_capacity;
	state->full_battery_capacity_mah = (uint16_t)gauge.full_battery_capacity;
	state->remaining_capacity_mah = (uint16_t)gauge.remaining_capacity;
	return 0;
}

static void power_ble_poll_work_fn(struct k_work *work)
{
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
				(void)power_lbs_notify_charger_state(&current);
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
				(void)power_lbs_notify_gauge_state(&current);
			}
		}
	}

	k_work_schedule(&power_poll_work, K_MSEC(POWER_BLE_POLL_INTERVAL_MS));
}

static void power_ble_update_polling_state(void)
{
	bool enabled = atomic_get(&charger_notify_enabled) || atomic_get(&gauge_notify_enabled);

	if (enabled) {
		k_work_reschedule(&power_poll_work, K_NO_WAIT);
	} else {
		k_work_cancel_delayable(&power_poll_work);
	}
}

static void power_ble_charger_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&charger_notify_enabled, enabled ? 1 : 0);
	power_ble_update_polling_state();
}

static void power_ble_gauge_notify_state_cb(bool enabled, void *user_data)
{
	ARG_UNUSED(user_data);
	atomic_set(&gauge_notify_enabled, enabled ? 1 : 0);
	power_ble_update_polling_state();
}

static const struct power_lbs_ops power_ble_ops = {
	.get_charger_state = power_ble_get_charger_state,
	.get_gauge_state = power_ble_get_gauge_state,
};

int power_ble_bridge_init(void)
{
	if (power_bridge_started) {
		return 0;
	}

	k_work_init_delayable(&power_poll_work, power_ble_poll_work_fn);
	atomic_set(&charger_notify_enabled, 0);
	atomic_set(&gauge_notify_enabled, 0);
	have_last_charger_state = false;
	have_last_gauge_state = false;

	power_lbs_register_ops(&power_ble_ops);
	power_lbs_register_charger_notify_cb(power_ble_charger_notify_state_cb, NULL);
	power_lbs_register_gauge_notify_cb(power_ble_gauge_notify_state_cb, NULL);

	power_bridge_started = true;
	return 0;
}
