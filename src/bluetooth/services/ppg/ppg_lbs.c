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

static bool notify_red_enabled;
static bool notify_ir_enabled;
static bool notify_green_enabled;

static uint32_t ppg_red_state;
static uint32_t ppg_ir_state;
static uint32_t ppg_green_state;

static ppg_lbs_notify_state_cb_t red_notify_cb;
static void *red_notify_user_data;
static ppg_lbs_notify_state_cb_t ir_notify_cb;
static void *ir_notify_user_data;
static ppg_lbs_notify_state_cb_t green_notify_cb;
static void *green_notify_user_data;

static void (*update_transfer_interval_callback)(uint16_t transfer_interval) = NULL;
static void (*update_operation_mode_callback)(uint16_t operation_mode) = NULL;


static void red_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_red_enabled = (value == BT_GATT_CCC_NOTIFY);
  if (red_notify_cb) {
    red_notify_cb(notify_red_enabled, red_notify_user_data);
  }
}

static void ir_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_ir_enabled = (value == BT_GATT_CCC_NOTIFY);
  if (ir_notify_cb) {
    ir_notify_cb(notify_ir_enabled, ir_notify_user_data);
  }
}

static void green_notification_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_green_enabled = (value == BT_GATT_CCC_NOTIFY);
  if (green_notify_cb) {
    green_notify_cb(notify_green_enabled, green_notify_user_data);
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

static ssize_t read_red(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppg_red_state, sizeof(ppg_red_state));
}

static ssize_t read_ir(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppg_ir_state, sizeof(ppg_ir_state));
}

static ssize_t read_green(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppg_green_state, sizeof(ppg_green_state));
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
    ppg_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_PPG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_TRANSFER_INTERVAL_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_transfer_interval, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_OPERATION_MODE_CONF, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, update_operation_mode, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_RED, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_red, NULL, &ppg_red_state),
    BT_GATT_CCC(red_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_IR, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_ir, NULL, &ppg_ir_state),
    BT_GATT_CCC(ir_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LBS_PPG_GREEN, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_green, NULL, &ppg_green_state),
    BT_GATT_CCC(green_notification_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));


int ppg_lbs_notify_red(uint32_t value)
{
  ppg_red_state = value;
  if (!notify_red_enabled) {
    return -EACCES;
  }
  return bt_gatt_notify(NULL, &ppg_lbs_svc.attrs[6], &ppg_red_state, sizeof(ppg_red_state));
}

int ppg_lbs_notify_ir(uint32_t value)
{
  ppg_ir_state = value;
  if (!notify_ir_enabled) {
    return -EACCES;
  }
  return bt_gatt_notify(NULL, &ppg_lbs_svc.attrs[9], &ppg_ir_state, sizeof(ppg_ir_state));
}

int ppg_lbs_notify_green(uint32_t value)
{
  ppg_green_state = value;
  if (!notify_green_enabled) {
    return -EACCES;
  }
  return bt_gatt_notify(NULL, &ppg_lbs_svc.attrs[12], &ppg_green_state, sizeof(ppg_green_state));
}

void ppg_lbs_register_red_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data)
{
  red_notify_cb = cb;
  red_notify_user_data = user_data;
}

void ppg_lbs_register_ir_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data)
{
  ir_notify_cb = cb;
  ir_notify_user_data = user_data;
}

void ppg_lbs_register_green_notify_cb(ppg_lbs_notify_state_cb_t cb, void *user_data)
{
  green_notify_cb = cb;
  green_notify_user_data = user_data;
}

void register_ppg_transfer_interval_callback(void (*callback)(uint16_t))
{
  update_transfer_interval_callback = callback;
}

void register_ppg_operation_mode_callback(void (*callback)(uint16_t))
{
  update_operation_mode_callback = callback;
}
