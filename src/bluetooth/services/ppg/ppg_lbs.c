#include <errno.h>
#include <stdint.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"
#include "ppg_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_PPG_SENSOR_BLUETOOTH_LOGGER);

static struct bt_conn* ppg_lbs_conn;

static bool notify_red_enabled;
static bool notify_ir_enabled;
static bool notify_green_enabled;
static bool ppg_listener_registered;

static struct ppg_sample_notification_t ppg_red_state;
static struct ppg_sample_notification_t ppg_ir_state;
static struct ppg_sample_notification_t ppg_green_state;

static void red_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_red_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("PPG red notifications %s", notify_red_enabled ? "enabled" : "disabled");
}

static void ir_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_ir_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("PPG IR notifications %s", notify_ir_enabled ? "enabled" : "disabled");
}

static void green_notification_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value) {
	ARG_UNUSED(attr);
	notify_green_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("PPG green notifications %s", notify_green_enabled ? "enabled" : "disabled");
}

static ssize_t read_red(struct bt_conn* conn,
						const struct bt_gatt_attr* attr,
						void* buf,
						uint16_t len,
						uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppg_red_state, sizeof(ppg_red_state));
}

static ssize_t read_ir(struct bt_conn* conn,
					   const struct bt_gatt_attr* attr,
					   void* buf,
					   uint16_t len,
					   uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppg_ir_state, sizeof(ppg_ir_state));
}

static ssize_t read_green(struct bt_conn* conn,
						  const struct bt_gatt_attr* attr,
						  void* buf,
						  uint16_t len,
						  uint16_t offset) {
	return bt_gatt_attr_read(conn,
							 attr,
							 buf,
							 len,
							 offset,
							 &ppg_green_state,
							 sizeof(ppg_green_state));
}

static ssize_t read_sampling_enable(struct bt_conn* conn,
									const struct bt_gatt_attr* attr,
									void* buf,
									uint16_t len,
									uint16_t offset) {
	uint8_t enabled = device_manager_is_ppg_sampling_enabled() ? 1U : 0U;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &enabled, sizeof(enabled));
}

static ssize_t write_sampling_enable(struct bt_conn* conn,
									 const struct bt_gatt_attr* attr,
									 const void* buf,
									 uint16_t len,
									 uint16_t offset,
									 uint8_t flags) {
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t enabled = *(const uint8_t*) buf;

	if (enabled > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	bool requested_enabled = (enabled != 0U);

	int ret = device_manager_set_ppg_sampling_enabled(requested_enabled);

	if (ret != 0) {
		LOG_WRN("PPG sampling update failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	LOG_INF("PPG sampling %s, IRQ cadence=%s",
			device_manager_is_ppg_sampling_enabled() ? "enabled" : "disabled",
			device_manager_is_ppg_per_sample_irq() ? "per-sample" : "batch");
	return len;
}

static ssize_t read_per_sample_irq(struct bt_conn* conn,
								   const struct bt_gatt_attr* attr,
								   void* buf,
								   uint16_t len,
								   uint16_t offset) {
	uint8_t enabled = device_manager_is_ppg_per_sample_irq() ? 1U : 0U;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &enabled, sizeof(enabled));
}

static ssize_t write_per_sample_irq(struct bt_conn* conn,
									const struct bt_gatt_attr* attr,
									const void* buf,
									uint16_t len,
									uint16_t offset,
									uint8_t flags) {
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t enabled = *(const uint8_t*) buf;

	if (enabled > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	int ret = device_manager_set_ppg_per_sample_irq(enabled != 0U);

	if (ret != 0) {
		return BT_GATT_ERR(ret == -EBUSY ? BT_ATT_ERR_VALUE_NOT_ALLOWED : BT_ATT_ERR_UNLIKELY);
	}

	LOG_INF("PPG IRQ cadence set to %s",
			device_manager_is_ppg_per_sample_irq() ? "per-sample" : "batch");
	return len;
}

BT_GATT_SERVICE_DEFINE(
	ppg_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_PPG_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_RED,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_red,
						   NULL,
						   &ppg_red_state),
	BT_GATT_CUD("PPG Red", BT_GATT_PERM_READ),
	BT_GATT_CCC(red_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_IR,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_ir,
						   NULL,
						   &ppg_ir_state),
	BT_GATT_CUD("PPG Infrared", BT_GATT_PERM_READ),
	BT_GATT_CCC(ir_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_GREEN,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_green,
						   NULL,
						   &ppg_green_state),
	BT_GATT_CUD("PPG Green", BT_GATT_PERM_READ),
	BT_GATT_CCC(green_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

BT_GATT_SERVICE_DEFINE(ppg_lbs_config_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_PPG_CONFIG_SERVICE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_CONFIG_SAMPLING_ENABLE,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
											  read_sampling_enable,
											  write_sampling_enable,
											  NULL),
					   BT_GATT_CUD("PPG Sampling Enable", BT_GATT_PERM_READ),
					   BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_CONFIG_PER_SAMPLE_IRQ,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
											  read_per_sample_irq,
											  write_per_sample_irq,
											  NULL),
					   BT_GATT_CUD("PPG Per-Sample IRQ", BT_GATT_PERM_READ));

void ppg_lbs_set_conn(struct bt_conn* conn) {
	if (conn == NULL) {
		return;
	}

	if (ppg_lbs_conn != NULL) {
		bt_conn_unref(ppg_lbs_conn);
	}

	ppg_lbs_conn = bt_conn_ref(conn);
	LOG_INF("PPG BLE connection attached");
}

void ppg_lbs_clear_conn(void) {
	if (ppg_lbs_conn != NULL) {
		bt_conn_unref(ppg_lbs_conn);
		ppg_lbs_conn = NULL;
		LOG_INF("PPG BLE connection cleared");
	}
}

static void ppg_lbs_update_cache(const struct ppg_msg_t* msg) {
	uint64_t unix_ms = (uint64_t) (msg->timestamp / 1000);

	ppg_red_state.unix_ms = unix_ms;
	ppg_red_state.value = msg->red;
	ppg_ir_state.unix_ms = unix_ms;
	ppg_ir_state.value = msg->ir;
	ppg_green_state.unix_ms = unix_ms;
	ppg_green_state.value = msg->green;
}

static void ppg_lbs_notify_cache(void) {
	if (ppg_lbs_conn == NULL) {
		return;
	}

	if (notify_red_enabled) {
		(void) bt_gatt_notify(ppg_lbs_conn,
							  &ppg_lbs_svc.attrs[2],
							  &ppg_red_state,
							  sizeof(ppg_red_state));
	}
	if (notify_ir_enabled) {
		(void) bt_gatt_notify(ppg_lbs_conn,
							  &ppg_lbs_svc.attrs[6],
							  &ppg_ir_state,
							  sizeof(ppg_ir_state));
	}
	if (notify_green_enabled) {
		(void) bt_gatt_notify(ppg_lbs_conn,
							  &ppg_lbs_svc.attrs[10],
							  &ppg_green_state,
							  sizeof(ppg_green_state));
	}
}

static void ppg_lbs_listener_cb(const struct zbus_channel* chan) {
	if (device_manager_stream_from_channel(chan) != device_manager_stream_Ppg) {
		return;
	}

	const struct ppg_msg_t* msg = zbus_chan_const_msg(chan);

	if (msg == NULL) {
		return;
	}

	ppg_lbs_update_cache(msg);
	ppg_lbs_notify_cache();
}

ZBUS_LISTENER_DEFINE(ppg_lbs_listener, ppg_lbs_listener_cb);

bool ppg_lbs_stream_ready(void) {
	return device_manager_stream_ready(device_manager_stream_Ppg);
}

int ppg_lbs_register_stream(void) {
	if (ppg_listener_registered) {
		LOG_INF("PPG stream already registered");
		return 0;
	}

	int ret =
		device_manager_stream_register(device_manager_stream_Ppg, &ppg_lbs_listener, K_MSEC(100));

	if (ret == 0) {
		ppg_listener_registered = true;
		LOG_INF("PPG stream registered");
	} else {
		LOG_INF("PPG stream registration failed: %d", ret);
	}

	return ret;
}
