/* SPDX-License-Identifier: Apache-2.0 */

#include <stdint.h>

#include <zephyr/app_version.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include "device_info_lbs.h"

/* This application owns DIS so the revision cannot be overridden by persistent
 * settings or mistaken for the Zephyr kernel version. VERSION is authoritative.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_DIS), "SensWear already provides Device Information Service");
static const char firmware_revision[] = APP_VERSION_EXTENDED_STRING;

static const uint32_t shield_mask =
	(IS_ENABLED(CONFIG_SHIELD_SENSWEAR_HAPTIC) ? SENSWEAR_SHIELD_HAPTIC : 0U) |
	(IS_ENABLED(CONFIG_SHIELD_SENSWEAR_PPG) ? SENSWEAR_SHIELD_PPG : 0U) |
	(IS_ENABLED(CONFIG_SHIELD_SENSWEAR_TEMPERATURE) ? SENSWEAR_SHIELD_TEMPERATURE : 0U) |
	(IS_ENABLED(CONFIG_SHIELD_SENSWEAR_TOUCH) ? SENSWEAR_SHIELD_TOUCH : 0U);

/* Match services.cmake AND the device-manager producer/actuator gates. Core
 * services remain discoverable when their drivers are disabled, so UUID
 * discovery alone is not an accurate indication of supported functionality.
 * Temperature's periodic producer also needs the RTC minute event.
 */
static const uint32_t feature_mask =
	IS_ENABLED(CONFIG_SENSWEAR_DEVICE_MANAGER)
		? (IS_ENABLED(CONFIG_SENSWEAR_BHI360_DRIVER) ? SENSWEAR_FEATURE_IMU : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_LP5562_DRIVER) ? SENSWEAR_FEATURE_LED : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_DAUGHTER_HAPTIC) &&
					   IS_ENABLED(CONFIG_SHIELD_SENSWEAR_HAPTIC) &&
					   IS_ENABLED(CONFIG_SENSWEAR_DRV2605_DRIVER)
				   ? SENSWEAR_FEATURE_HAPTIC
				   : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_DAUGHTER_PPG) && IS_ENABLED(CONFIG_SHIELD_SENSWEAR_PPG)
				   ? SENSWEAR_FEATURE_PPG
				   : 0U) |
			  (IS_ENABLED(CONFIG_SHIELD_SENSWEAR_TEMPERATURE) &&
					   IS_ENABLED(CONFIG_SENSWEAR_RTC_DRIVER)
				   ? SENSWEAR_FEATURE_TEMPERATURE
				   : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_DAUGHTER_TOUCH) &&
					   IS_ENABLED(CONFIG_SHIELD_SENSWEAR_TOUCH)
				   ? SENSWEAR_FEATURE_TOUCH
				   : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_BQ27427_DRIVER) ? SENSWEAR_FEATURE_BATTERY : 0U) |
			  (IS_ENABLED(CONFIG_SENSWEAR_RTC_DRIVER) ? SENSWEAR_FEATURE_TIME : 0U)
		: 0U;

static ssize_t read_firmware_revision(struct bt_conn* conn,
									  const struct bt_gatt_attr* attr,
									  void* buf,
									  uint16_t len,
									  uint16_t offset) {
	/* DIS strings do not include a terminating NUL on the wire. */
	return bt_gatt_attr_read(conn,
							 attr,
							 buf,
							 len,
							 offset,
							 firmware_revision,
							 sizeof(firmware_revision) - 1U);
}

static ssize_t read_capabilities(struct bt_conn* conn,
								 const struct bt_gatt_attr* attr,
								 void* buf,
								 uint16_t len,
								 uint16_t offset) {
	uint8_t payload[SENSWEAR_CAPABILITIES_SIZE];

	BUILD_ASSERT(SENSWEAR_CAPABILITIES_SHIELD_OFFSET + sizeof(uint32_t) ==
				 SENSWEAR_CAPABILITIES_FEATURE_OFFSET);
	BUILD_ASSERT(SENSWEAR_CAPABILITIES_FEATURE_OFFSET + sizeof(uint32_t) == sizeof(payload));
	payload[0] = SENSWEAR_CAPABILITIES_SCHEMA_VERSION;
	sys_put_le32(shield_mask, &payload[SENSWEAR_CAPABILITIES_SHIELD_OFFSET]);
	sys_put_le32(feature_mask, &payload[SENSWEAR_CAPABILITIES_FEATURE_OFFSET]);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, sizeof(payload));
}

BT_GATT_SERVICE_DEFINE(senswear_device_information_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DIS_FIRMWARE_REVISION,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_firmware_revision,
											  NULL,
											  NULL));

BT_GATT_SERVICE_DEFINE(senswear_capabilities_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_SENSWEAR_DEVICE_INFO),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSWEAR_CAPABILITIES,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_capabilities,
											  NULL,
											  NULL));
