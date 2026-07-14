#ifndef POWER_LBS_H_
#define POWER_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_LBS_POWER_SERVICE BT_UUID_BAS
#define BT_UUID_LBS_POWER_BATTERY_LEVEL BT_UUID_BAS_BATTERY_LEVEL
#define BT_UUID_LBS_POWER_BATTERY_LEVEL_STATUS BT_UUID_BAS_BATTERY_LEVEL_STATUS

void power_lbs_set_conn(struct bt_conn* conn);
void power_lbs_clear_conn(void);
bool power_lbs_streams_ready(void);
int power_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

#endif
