#ifndef HAPTIC_LBS_H_
#define HAPTIC_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#include "device_manager.h"

#define BT_UUID_LBS_HAPTIC_SERVICE_VAL BT_UUID_128_ENCODE(0xdaa05e91, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
#define BT_UUID_LBS_HAPTIC_PATTERN_VAL BT_UUID_128_ENCODE(0xdaa05e92, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
#define BT_UUID_LBS_HAPTIC_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_SERVICE_VAL)
#define BT_UUID_LBS_HAPTIC_PATTERN_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_PATTERN_VAL)

#define HAPTIC_LBS_PATTERN_VERSION 1U

#if defined(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
#define HAPTIC_LBS_MAX_FRAMES DEVICE_MANAGER_HAPTIC_RTP_MAX
#else
#define HAPTIC_LBS_MAX_FRAMES 32U
#endif

void haptic_lbs_set_conn(struct bt_conn* conn);
void haptic_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

#endif
