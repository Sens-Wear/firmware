#ifndef BODY_TEMPERATURE_LBS_H_
#define BODY_TEMPERATURE_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_LBS_BODY_TEMPERATURE_SERVICE BT_UUID_HTS
#define BT_UUID_LBS_BODY_TEMPERATURE_MEASUREMENT BT_UUID_HTS_MEASUREMENT
#define BT_UUID_LBS_BODY_TEMPERATURE_TYPE BT_UUID_HTS_TEMP_TYP
#define BT_UUID_LBS_BODY_TEMPERATURE_MEASUREMENT_INTERVAL BT_UUID_HTS_INTERVAL

void body_temperature_lbs_set_conn(struct bt_conn* conn);
void body_temperature_lbs_clear_conn(void);
bool body_temperature_lbs_stream_ready(void);
int body_temperature_lbs_register_stream(void);

#ifdef __cplusplus
}
#endif

#endif
