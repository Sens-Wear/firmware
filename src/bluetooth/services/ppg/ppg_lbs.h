#ifndef PPG_LBS_H_
#define PPG_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#define BT_UUID_LBS_PPG_SERVICE_VAL BT_UUID_128_ENCODE(0x029ca54e, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x029ca54f, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_OPERATION_MODE_CONF_VAL BT_UUID_128_ENCODE(0x029ca550, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_PPG_RAW_DATA_VAL BT_UUID_128_ENCODE(0x029ca551, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

#define BT_UUID_LBS_PPG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_SERVICE_VAL)
#define BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_PPG_OPERATION_MODE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_OPERATION_MODE_CONF_VAL)
#define BT_UUID_LBS_PPG_RAW_DATA BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_RAW_DATA_VAL)

void register_ppg_raw_data_callback(void (*callback)(bool));
void register_ppg_transfer_interval_callback(void (*callback)(uint16_t));
void register_ppg_operation_mode_callback(void (*callback)(uint16_t));

int ppg_lbs_send_raw_data_notify(uint8_t *sensor_value, size_t size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif