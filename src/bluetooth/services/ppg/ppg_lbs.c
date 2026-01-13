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

#include "ppg_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_PPG_SENSOR_BLUETOOTH_LOGGER);

static bool notify_raw_data_enabled;

static void (*raw_data_callback)(bool enabled) = NULL;
static void (*update_transfer_interval_callback)(uint16_t transfer_interval) = NULL;
static void (*update_operation_mode_callback)(uint16_t operation_mode) = NULL;


static void raw_data_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_raw_data_enabled = (value == BT_GATT_CCC_NOTIFY);
  // Notify registered callback
  if (raw_data_callback)
  {
    raw_data_callback(notify_raw_data_enabled);
  }
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

static ssize_t update_operation_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

  if (len != 1U)
  {
    LOG_ERR("Operation Mode: Incorrect data length");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (offset != 0)
  {
    LOG_ERR("Operation Mode: Incorrect data offset");
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  if (update_operation_mode_callback)
  {
    uint16_t val = *((uint16_t *)buf);
    update_operation_mode_callback(val);
  }
  return len;
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
    ppg_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_PPG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_transfer_interval, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_OPERATION_MODE_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_operation_mode, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_RAW_DATA, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(raw_data_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));


int ppg_lbs_send_raw_data_notify(uint8_t *sensor_value, size_t size)
{
  if (!notify_raw_data_enabled)
  {
    return -EACCES;
  }
  LOG_INF("PPG raw data sent over BLE!.");
  return bt_gatt_notify(NULL, &ppg_lbs_svc.attrs[5], sensor_value, size);
}

void register_ppg_raw_data_callback(void (*callback)(bool))
{
  raw_data_callback = callback;
}

void register_ppg_transfer_interval_callback(void (*callback)(uint16_t))
{
  update_transfer_interval_callback = callback;
}

void register_ppg_operation_mode_callback(void (*callback)(uint16_t))
{
  update_operation_mode_callback = callback;
}