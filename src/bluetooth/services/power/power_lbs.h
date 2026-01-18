#ifndef POWER_LBS_H_
#define POWER_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>

#define BT_UUID_LBS_POWER_SERVICE_VAL BT_UUID_128_ENCODE(0x8f9a1c20, 0x84e7, 0x4a73, 0x9a57, 0x6f6d5d0ab1c2)
#define BT_UUID_LBS_POWER_CHARGER_STATE_VAL BT_UUID_128_ENCODE(0x8f9a1c21, 0x84e7, 0x4a73, 0x9a57, 0x6f6d5d0ab1c2)
#define BT_UUID_LBS_POWER_GAUGE_STATE_VAL BT_UUID_128_ENCODE(0x8f9a1c22, 0x84e7, 0x4a73, 0x9a57, 0x6f6d5d0ab1c2)

#define BT_UUID_LBS_POWER_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_POWER_SERVICE_VAL)
#define BT_UUID_LBS_POWER_CHARGER_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_POWER_CHARGER_STATE_VAL)
#define BT_UUID_LBS_POWER_GAUGE_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_POWER_GAUGE_STATE_VAL)

struct power_lbs_charger_state {
	uint32_t flags;
} __packed;

struct power_lbs_gauge_state {
	int16_t temperature_cdec;
	uint16_t voltage_mv;
	int16_t average_current_ma;
	int16_t average_power_mw;
	uint16_t state_of_charge_cdec;
	uint16_t nominal_available_capacity_mah;
	uint16_t full_battery_capacity_mah;
	uint16_t remaining_capacity_mah;
} __packed;

struct power_lbs_ops {
	int (*get_charger_state)(struct power_lbs_charger_state *state);
	int (*get_gauge_state)(struct power_lbs_gauge_state *state);
};

typedef void (*power_lbs_notify_state_cb_t)(bool enabled, void *user_data);

void power_lbs_register_ops(const struct power_lbs_ops *ops);
void power_lbs_register_charger_notify_cb(power_lbs_notify_state_cb_t cb, void *user_data);
void power_lbs_register_gauge_notify_cb(power_lbs_notify_state_cb_t cb, void *user_data);
void power_lbs_set_conn(struct bt_conn *conn);
void power_lbs_clear_conn(void);

int power_lbs_notify_charger_state(const struct power_lbs_charger_state *state);
int power_lbs_notify_gauge_state(const struct power_lbs_gauge_state *state);

#ifdef __cplusplus
}
#endif

#endif
