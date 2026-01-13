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

#include "temperature_lbs.h"

LOG_MODULE_REGISTER(SENS_WEAR_TEMPERATURE_SENSOR_BLUETOOTH_LOGGER);

static bool notify_temperature_enabled;

static void (*temperature_status_callback)(bool enabled) = NULL;
static void (*update_sampling_rate_callback)(uint16_t new_sampling_rate) = NULL;
static void (*update_transfer_interval_callback)(uint16_t transfer_interval) = NULL;

static void temperature_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_temperature_enabled = (value == BT_GATT_CCC_NOTIFY);
  // Notify registered callback
  if (temperature_status_callback)
  {
    temperature_status_callback(notify_temperature_enabled);
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

BT_GATT_SERVICE_DEFINE(
    temperature_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_TEMPERATURE_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_sampling_rate, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_transfer_interval, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_TEMPERATURE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(temperature_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

int temperature_lbs_send_sensor_notify(uint8_t *sensor_value, size_t size)
{
  if (!notify_temperature_enabled)
  {
    return -EACCES;
  }
  LOG_INF("Pressure data sent over BLE!.");
  return bt_gatt_notify(NULL, &temperature_lbs_svc.attrs[5], sensor_value, size);
}

void register_temperature_status_callback(void (*callback)(bool))
{
  temperature_status_callback = callback;
}

void register_temperature_sampling_rate_callback(void (*callback)(uint16_t))
{
  update_sampling_rate_callback = callback;
}

void register_temperature_transfer_interval_callback(void (*callback)(uint16_t))
{
  update_transfer_interval_callback = callback;
}