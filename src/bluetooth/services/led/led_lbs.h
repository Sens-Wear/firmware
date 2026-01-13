#ifndef LED_LBS_H_
#define LED_LBS_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <zephyr/types.h>
#define BT_UUID_LBS_LED_SERVICE_VAL BT_UUID_128_ENCODE(0x3c688942, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
#define BT_UUID_LBS_LED_COLOR_VAL BT_UUID_128_ENCODE(0x3c688943, 0x4143, 0x470d, 0xa798, 0x4629803a1983)
#define BT_UUID_LBS_LED_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_LED_SERVICE_VAL)
#define BT_UUID_LBS_LED_COLOR_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_LED_COLOR_VAL)
struct led_lbs_ops {
    int (*set_color)(uint32_t color);
    int (*get_color)(uint32_t *color);
};
void led_lbs_register_ops(const struct led_lbs_ops *ops);
#ifdef __cplusplus
}
#endif
#endif
