#ifndef HAPTIC_LBS_H_
#define HAPTIC_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_LBS_HAPTIC_SERVICE_VAL BT_UUID_128_ENCODE(0xdaa05e91, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
#define BT_UUID_LBS_HAPTIC_PATTERN_VAL BT_UUID_128_ENCODE(0xdaa05e92, 0xf514, 0x4a4e, 0x8fc5, 0xd1b80f25f24d)
#define BT_UUID_LBS_HAPTIC_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_SERVICE_VAL)
#define BT_UUID_LBS_HAPTIC_PATTERN_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_HAPTIC_PATTERN_VAL)

#define HAPTIC_LBS_PATTERN_VERSION 1U
#define HAPTIC_LBS_MAX_FRAMES 64U

/*
 * BLE payload layout:
 * byte 0: protocol version (currently 1)
 * byte 1: flags (reserved, send 0)
 * byte 2-3: frame count, little-endian
 * repeated frame payload:
 *   byte 0-1: frame duration in milliseconds, little-endian
 *   byte 2: intensity from 0-255
 */
struct haptic_lbs_frame {
	uint16_t duration_ms;
	uint8_t intensity;
};

struct haptic_lbs_ops {
	int (*run_pattern)(const struct haptic_lbs_frame *frames, size_t frame_count);
};

void haptic_lbs_register_ops(const struct haptic_lbs_ops *ops);

#ifdef __cplusplus
}
#endif

#endif
