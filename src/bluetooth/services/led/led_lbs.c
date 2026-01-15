#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "led_lbs.h"
LOG_MODULE_REGISTER(SENSE_WEAR_LED_SENSOR_BLUETOOTH_LOGGER);
static uint32_t led_color_state;
static const struct led_lbs_ops *led_ops;
static struct bt_conn *led_lbs_conn;
static ssize_t update_color(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);
    if (len != sizeof(uint32_t)) {
        LOG_ERR("LED color: Incorrect data length");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    if (offset != 0U) {
        LOG_ERR("LED color: Incorrect data offset");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint32_t val = sys_get_le32(buf);
    int rc = 0;
    if (led_ops && led_ops->set_color) {
        rc = led_ops->set_color(val);
    }
    if (rc != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    led_color_state = val;
    return len;
}
static ssize_t read_color(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    if (led_ops && led_ops->get_color) {
        uint32_t current;
        if (led_ops->get_color(&current) == 0) {
            led_color_state = current;
        }
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &led_color_state, sizeof(led_color_state));
}
BT_GATT_SERVICE_DEFINE(
    led_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_LED_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED_COLOR_CONF,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_color, update_color, &led_color_state));
                           
void led_lbs_register_ops(const struct led_lbs_ops *ops)
{
    led_ops = ops;
}

void led_lbs_set_conn(struct bt_conn *conn)
{
    if (conn == NULL) {
        return;
    }

    if (led_lbs_conn != NULL) {
        bt_conn_unref(led_lbs_conn);
    }

    led_lbs_conn = bt_conn_ref(conn);
}

void led_lbs_clear_conn(void)
{
    if (led_lbs_conn != NULL) {
        bt_conn_unref(led_lbs_conn);
        led_lbs_conn = NULL;
    }
}
