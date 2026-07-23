#include <errno.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "device_manager.h"
#include "led_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_LED_SENSOR_BLUETOOTH_LOGGER);

static uint32_t led_color_state;
static struct bt_conn* led_lbs_conn;

static ssize_t update_color(struct bt_conn* conn,
			    const struct bt_gatt_attr* attr,
			    const void* buf,
			    uint16_t len,
			    uint16_t offset,
			    uint8_t flags) {
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint32_t requested_color = sys_get_le32(buf);
	struct device_manager_led_color_t color = {
		.red = (uint8_t) (requested_color >> 16),
		.green = (uint8_t) (requested_color >> 8),
		.blue = (uint8_t) requested_color,
	};

#if defined(CONFIG_SENSWEAR_LP5562_DRIVER)
	int ret = device_manager_set_led_color(color);
#else
	int ret = -ENOTSUP;
#endif
	if (ret != 0) {
		LOG_WRN("LED color update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	led_color_state = requested_color;
	LOG_INF("LED color set to 0x%08x", led_color_state);
	return len;
}

static ssize_t read_color(struct bt_conn* conn,
			  const struct bt_gatt_attr* attr,
			  void* buf,
			  uint16_t len,
			  uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &led_color_state, sizeof(led_color_state));
}

BT_GATT_SERVICE_DEFINE(
	led_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_LED_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED_COLOR_CONF,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_color,
			       update_color,
			       &led_color_state),
	BT_GATT_CUD("LED Color", BT_GATT_PERM_READ));

void led_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (led_lbs_conn != NULL) {
		bt_conn_unref(led_lbs_conn);
	}

	led_lbs_conn = bt_conn_ref(conn);
	LOG_INF("LED BLE connection attached");
}

void led_lbs_clear_conn(void) {
	if (led_lbs_conn != NULL) {
		bt_conn_unref(led_lbs_conn);
		led_lbs_conn = NULL;
		LOG_INF("LED BLE connection cleared");
	}
}
