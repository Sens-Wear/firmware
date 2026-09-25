/* SPDX-License-Identifier: Apache-2.0 */

#ifndef SENSWEAR_DEVICE_INFO_LBS_H_
#define SENSWEAR_DEVICE_INFO_LBS_H_

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>

#define BT_UUID_SENSWEAR_DEVICE_INFO_VAL \
	BT_UUID_128_ENCODE(0x9b8e0001, 0x6b7d, 0x4e9f, 0x9b0d, 0x2d7f6e5a4c30)
#define BT_UUID_SENSWEAR_CAPABILITIES_VAL \
	BT_UUID_128_ENCODE(0x9b8e0002, 0x6b7d, 0x4e9f, 0x9b0d, 0x2d7f6e5a4c30)
#define BT_UUID_SENSWEAR_DEVICE_INFO BT_UUID_DECLARE_128(BT_UUID_SENSWEAR_DEVICE_INFO_VAL)
#define BT_UUID_SENSWEAR_CAPABILITIES BT_UUID_DECLARE_128(BT_UUID_SENSWEAR_CAPABILITIES_VAL)

/* READ-only capabilities: u8 schema, LE u32 shields, LE u32 features.
 * These describe compiled support, not physical presence or runtime readiness.
 * Unassigned bits are reserved; clients ignore them. Unsupported schema versions
 * must not be interpreted using this layout. No existing service UUID is reused.
 */
#define SENSWEAR_CAPABILITIES_SCHEMA_VERSION 1U
#define SENSWEAR_CAPABILITIES_SIZE 9U
#define SENSWEAR_CAPABILITIES_SHIELD_OFFSET 1U
#define SENSWEAR_CAPABILITIES_FEATURE_OFFSET 5U

enum senswear_shield {
	SENSWEAR_SHIELD_HAPTIC = BIT(0),
	SENSWEAR_SHIELD_PPG = BIT(1),
	SENSWEAR_SHIELD_TEMPERATURE = BIT(2),
	SENSWEAR_SHIELD_TOUCH = BIT(3),
};

enum senswear_feature {
	SENSWEAR_FEATURE_IMU = BIT(0),
	SENSWEAR_FEATURE_LED = BIT(1),
	SENSWEAR_FEATURE_HAPTIC = BIT(2),
	SENSWEAR_FEATURE_PPG = BIT(3),
	SENSWEAR_FEATURE_TEMPERATURE = BIT(4),
	SENSWEAR_FEATURE_TOUCH = BIT(5),
	SENSWEAR_FEATURE_BATTERY = BIT(6),
	SENSWEAR_FEATURE_TIME = BIT(7),
};

#endif /* SENSWEAR_DEVICE_INFO_LBS_H_ */
