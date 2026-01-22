#ifndef PPG_LBS_H_
#define PPG_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <stdbool.h>
#define BT_UUID_LBS_PPG_SERVICE_VAL BT_UUID_128_ENCODE(0x029ca54e, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x029ca54f, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_OPERATION_MODE_CONF_VAL BT_UUID_128_ENCODE(0x029ca550, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_RED_VAL BT_UUID_128_ENCODE(0x029ca551, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_IR_VAL BT_UUID_128_ENCODE(0x029ca552, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)
#define BT_UUID_LBS_PPG_GREEN_VAL BT_UUID_128_ENCODE(0x029ca553, 0xd022, 0x4583, 0xb483, 0x91e9ea77034a)

#define BT_UUID_LBS_PPG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_SERVICE_VAL)
#define BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_PPG_OPERATION_MODE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_OPERATION_MODE_CONF_VAL)
#define BT_UUID_LBS_PPG_RED BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_RED_VAL)
#define BT_UUID_LBS_PPG_IR BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_IR_VAL)
#define BT_UUID_LBS_PPG_GREEN BT_UUID_DECLARE_128(BT_UUID_LBS_PPG_GREEN_VAL)

typedef void (*ppg_lbs_notify_state_cb_t)(bool enabled, void *user_data);

void ppg_lbs_register_red_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data);
void ppg_lbs_register_ir_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data);
void ppg_lbs_register_green_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data);
void register_ppg_transfer_interval_callback(void (*callback)(uint16_t));
void register_ppg_operation_mode_callback(void (*callback)(uint16_t));

int ppg_lbs_notify_red(uint32_t value);
int ppg_lbs_notify_ir(uint32_t value);
int ppg_lbs_notify_green(uint32_t value);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
