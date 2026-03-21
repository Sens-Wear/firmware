#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "haptic_lbs.h"

LOG_MODULE_REGISTER(SENSE_WEAR_HAPTIC_BLUETOOTH_LOGGER);

#define HAPTIC_LBS_PATTERN_HEADER_SIZE 4U
#define HAPTIC_LBS_PATTERN_FRAME_SIZE 3U

static const struct haptic_lbs_ops *haptic_ops;
static struct bt_conn *haptic_lbs_conn;

static ssize_t update_pattern(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct haptic_lbs_frame frames[HAPTIC_LBS_MAX_FRAMES];
	const uint8_t *data = buf;
	uint16_t frame_count;
	size_t expected_len;
	int rc;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < HAPTIC_LBS_PATTERN_HEADER_SIZE) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (data[0] != HAPTIC_LBS_PATTERN_VERSION) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (data[1] != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	frame_count = sys_get_le16(&data[2]);
	if (frame_count == 0U || frame_count > HAPTIC_LBS_MAX_FRAMES) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	expected_len = HAPTIC_LBS_PATTERN_HEADER_SIZE +
		       ((size_t)frame_count * HAPTIC_LBS_PATTERN_FRAME_SIZE);
	if (len != expected_len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	for (uint16_t i = 0; i < frame_count; i++) {
		size_t frame_offset = HAPTIC_LBS_PATTERN_HEADER_SIZE +
				     ((size_t)i * HAPTIC_LBS_PATTERN_FRAME_SIZE);

		frames[i].duration_ms = sys_get_le16(&data[frame_offset]);
		frames[i].intensity = data[frame_offset + 2U];

		if (frames[i].duration_ms == 0U) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}

	if (haptic_ops == NULL || haptic_ops->run_pattern == NULL) {
		LOG_ERR("No haptic BLE handler registered");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	rc = haptic_ops->run_pattern(frames, frame_count);
	if (rc != 0) {
		LOG_WRN("Haptic pattern rejected: %d", rc);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

BT_GATT_SERVICE_DEFINE(
	haptic_lbs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS_HAPTIC_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_HAPTIC_PATTERN_CONF,
			       BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
			       update_pattern, NULL));

void haptic_lbs_register_ops(const struct haptic_lbs_ops *ops)
{
	haptic_ops = ops;
}

void haptic_lbs_set_conn(struct bt_conn *conn)
{
	if (conn == NULL) {
		return;
	}

	if (haptic_lbs_conn != NULL) {
		bt_conn_unref(haptic_lbs_conn);
	}

	haptic_lbs_conn = bt_conn_ref(conn);
}

void haptic_lbs_clear_conn(void)
{
	if (haptic_lbs_conn != NULL) {
		bt_conn_unref(haptic_lbs_conn);
		haptic_lbs_conn = NULL;
	}
}
