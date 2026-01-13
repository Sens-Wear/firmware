#ifndef TOUCH_LBS_H_
#define TOUCH_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#define BT_UUID_LBS_TOUCH_SERVICE_VAL BT_UUID_128_ENCODE(0x33a5eb3f, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_SAMPLING_RATE_CONF_VAL BT_UUID_128_ENCODE(0x33a5eb40, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x33a5eb41, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL BT_UUID_128_ENCODE(0x33a5eb42, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL BT_UUID_128_ENCODE(0x33a5eb43, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_RAW_DATA_VAL BT_UUID_128_ENCODE(0x33a5eb44, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

#define BT_UUID_LBS_TOUCH_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_SERVICE_VAL)
#define BT_UUID_LBS_TOUCH_SAMPLING_RATE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_SAMPLING_RATE_CONF_VAL)
#define BT_UUID_LBS_TOUCH_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_TOUCH_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL)
#define BT_UUID_LBS_GESTURE_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL)
#define BT_UUID_LBS_RAW_DATA BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_RAW_DATA_VAL)

void register_touch_touch_state_callback(void (*callback)(bool));
void register_touch_gesture_state_callback(void (*callback)(bool));
void register_touch_raw_data_callback(void (*callback)(bool));

void register_touch_sampling_rate_callback(void (*callback)(uint16_t));
void register_touch_transfer_interval_callback(void (*callback)(uint16_t));

int touch_lbs_send_touch_state_notify(uint8_t* sensor_value, size_t size);
int touch_lbs_send_gesture_state_notify(uint8_t* sensor_value, size_t size);
int touch_lbs_send_raw_data_notify(uint8_t* sensor_value, size_t size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif