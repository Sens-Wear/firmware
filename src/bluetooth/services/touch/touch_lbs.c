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

#include "touch_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_TOUCH_SENSOR_BLUETOOTH_LOGGER);

static bool notify_touch_state_enabled;
static bool notify_gesture_state_enabled;
static bool notify_raw_data_enabled;
static struct bt_conn *touch_lbs_conn;

static void (*touch_state_callback)(bool enabled) = NULL;
static void (*gesture_state_callback)(bool enabled) = NULL;
static void (*raw_data_callback)(bool enabled) = NULL;
static void (*update_sampling_rate_callback)(uint16_t new_sampling_rate) = NULL;
static void (*update_transfer_interval_callback)(uint16_t transfer_interval) = NULL;

static void touch_state_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_touch_state_enabled = (value == BT_GATT_CCC_NOTIFY);
  // Notify registered callback
  if (touch_state_callback)
  {
    touch_state_callback(notify_touch_state_enabled);
  }
}

static void gesture_state_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_gesture_state_enabled = (value == BT_GATT_CCC_NOTIFY);
  // Notify registered callback
  if (gesture_state_callback)
  {
    gesture_state_callback(notify_gesture_state_enabled);
  }
}

static void raw_data_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_raw_data_enabled = (value == BT_GATT_CCC_NOTIFY);
  // Notify registered callback
  if (raw_data_callback)
  {
    raw_data_callback(notify_raw_data_enabled);
  }
}

static ssize_t update_sampling_rate(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

  if (len != 1U)
  {
    LOG_ERR("Sampling frequency: Incorrect data length");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (offset != 0)
  {
    LOG_ERR("Sampling frequency: Incorrect data offset");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  if (update_sampling_rate_callback)
  {
    uint16_t val = *((uint16_t *)buf);
    if (val > 0x00)
    {
      update_sampling_rate_callback(val);
    }
    else
    {
      LOG_ERR("The sampling frequency is wrong.");
      return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
  }
  return len;
}

static ssize_t update_transfer_interval(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

  if (len != 1U)
  {
    LOG_ERR("Transfer_interval: Incorrect data length");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (offset != 0)
  {
    LOG_ERR("Transfer_interval: Incorrect data offset");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  if (update_transfer_interval_callback)
  {
    uint16_t val = *((uint16_t *)buf);
    if (val > 0x00)
    {
      update_transfer_interval_callback(val);
    }
    else
    {
      LOG_ERR("The transfer_interval is wrong.");
      return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
  }
  return len;
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
    touch_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_TOUCH_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TOUCH_SAMPLING_RATE_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_sampling_rate, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TOUCH_TRANSFER_INTERVAL_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_transfer_interval, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TOUCH_STATE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(touch_state_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_GESTURE_STATE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(gesture_state_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_RAW_DATA, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(raw_data_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

int touch_lbs_send_touch_state_notify(uint8_t *sensor_value, size_t size)
{
  if (!notify_touch_state_enabled)
  {
    return -EACCES;
  }
  if (touch_lbs_conn == NULL)
  {
    return -ENOTCONN;
  }
  LOG_INF("Touch state sent over BLE!.");
  return bt_gatt_notify(touch_lbs_conn, &touch_lbs_svc.attrs[5], sensor_value, size);
}

int touch_lbs_send_gesture_state_notify(uint8_t *sensor_value, size_t size)
{
  if (!notify_gesture_state_enabled)
  {
    return -EACCES;
  }
  if (touch_lbs_conn == NULL)
  {
    return -ENOTCONN;
  }
  LOG_INF("Gesture state sent over BLE!.");
  return bt_gatt_notify(touch_lbs_conn, &touch_lbs_svc.attrs[8], sensor_value, size);
}

int touch_lbs_send_raw_data_notify(uint8_t *sensor_value, size_t size)
{
  if (!notify_raw_data_enabled)
  {
    return -EACCES;
  }
  if (touch_lbs_conn == NULL)
  {
    return -ENOTCONN;
  }
  LOG_INF("Touch raw data sent over BLE!.");
  return bt_gatt_notify(touch_lbs_conn, &touch_lbs_svc.attrs[11], sensor_value, size);
}

void register_touch_touch_state_callback(void (*callback)(bool))
{
  touch_state_callback = callback;
}

void register_touch_gesture_state_callback(void (*callback)(bool))
{
  gesture_state_callback = callback;
}

void register_touch_raw_data_callback(void (*callback)(bool))
{
  raw_data_callback = callback;
}

void register_touch_sampling_rate_callback(void (*callback)(uint16_t))
{
  update_sampling_rate_callback = callback;
}

void register_touch_transfer_interval_callback(void (*callback)(uint16_t))
{
  update_transfer_interval_callback = callback;
}

void touch_lbs_set_conn(struct bt_conn *conn)
{
  if (conn == NULL) {
    return;
  }

  if (touch_lbs_conn != NULL) {
    bt_conn_unref(touch_lbs_conn);
  }

  touch_lbs_conn = bt_conn_ref(conn);
}

void touch_lbs_clear_conn(void)
{
  if (touch_lbs_conn != NULL) {
    bt_conn_unref(touch_lbs_conn);
    touch_lbs_conn = NULL;
  }
}
