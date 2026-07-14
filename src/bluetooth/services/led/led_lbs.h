#ifndef LED_LBS_H_
#define LED_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_LBS_LED_SERVICE_VAL BT_UUID_128_ENCODE(0x3c688942, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
#define BT_UUID_LBS_LED_COLOR_VAL BT_UUID_128_ENCODE(0x3c688943, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
#define BT_UUID_LBS_LED_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_LED_SERVICE_VAL)
#define BT_UUID_LBS_LED_COLOR_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_LED_COLOR_VAL)

void led_lbs_set_conn(struct bt_conn* conn);
void led_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

#endif
