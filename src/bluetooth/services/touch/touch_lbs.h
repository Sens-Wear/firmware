#ifndef TOUCH_LBS_H_
#define TOUCH_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

#define BT_UUID_LBS_TOUCH_SERVICE_VAL BT_UUID_128_ENCODE(0x33a5eb3f, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL BT_UUID_128_ENCODE(0x33a5eb42, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL BT_UUID_128_ENCODE(0x33a5eb43, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_RAW_DATA_VAL BT_UUID_128_ENCODE(0x33a5eb44, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

#define BT_UUID_LBS_TOUCH_CONFIG_SERVICE_VAL BT_UUID_128_ENCODE(0x33a5eb50, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)
#define BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE_VAL BT_UUID_128_ENCODE(0x33a5eb51, 0x0e13, 0x424f, 0x8b7a, 0x942be0ee5cfc)

#define BT_UUID_LBS_TOUCH_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_SERVICE_VAL)
#define BT_UUID_LBS_TOUCH_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_TOUCH_STATE_VAL)
#define BT_UUID_LBS_GESTURE_STATE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_GESTURE_STATE_VAL)
#define BT_UUID_LBS_RAW_DATA BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_RAW_DATA_VAL)

#define BT_UUID_LBS_TOUCH_CONFIG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_CONFIG_SERVICE_VAL)
#define BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE BT_UUID_DECLARE_128(BT_UUID_LBS_TOUCH_CONFIG_SAMPLING_ENABLE_VAL)

struct touch_lbs_touch_state {
	time_t timestamp;
	bool touched;
	uint16_t x;
	uint16_t y;
} __packed;

struct touch_lbs_gesture_state {
	time_t timestamp;
	uint8_t gesture;
	uint8_t gesture_state;
} __packed;

void touch_lbs_set_conn(struct bt_conn* conn);
void touch_lbs_clear_conn(void);
bool touch_lbs_streams_ready(void);
int touch_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

#endif
