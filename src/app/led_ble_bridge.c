#include <errno.h>
#include <zephyr/types.h>

#include "app/led_ble_bridge.h"
#include "bluetooth/services/led/led_lbs.h"
#include "drivers/actuators/led/led_controller.h"

static uint32_t led_ble_color;

static int led_ble_set_color(uint32_t color)
{
    union led_color_t led_color = {.color = color};

    if (color == 0U) {
        if (!led_controller_turn_off_leds(0)) {
            return -EIO;
        }
    } else {
        if (!led_controller_turn_on_leds(0, led_color)) {
            return -EIO;
        }
    }

    led_ble_color = color;
    return 0;
}

static int led_ble_get_color(uint32_t *color)
{
    if (color == NULL) {
        return -EINVAL;
    }

    *color = led_ble_color;
    return 0;
}

static const struct led_lbs_ops led_ble_ops = {
    .set_color = led_ble_set_color,
    .get_color = led_ble_get_color,
};

int led_ble_bridge_init(void)
{
    if (!led_controller_init() || !led_controller_configure()) {
        return -EIO;
    }

    led_lbs_register_ops(&led_ble_ops);
    return 0;
}
