#ifndef TEMPERATURE_LBS_H_
#define TEMPERATURE_LBS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>

#define BT_UUID_LBS_TEMPERATURE_SERVICE_VAL BT_UUID_128_ENCODE(0x8e83a64b, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF_VAL BT_UUID_128_ENCODE(0x8e83a64c, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x8e83a64d, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_VAL BT_UUID_128_ENCODE(0x8e83a64e, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)

#define BT_UUID_LBS_TEMPERATURE_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_SERVICE_VAL)
#define BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF_VAL)
#define BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_TEMPERATURE BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_VAL)

struct temperature_lbs_sample {
	int32_t temperature_mdeg_c;
} __packed;

typedef void (*temperature_lbs_notify_state_cb_t)(bool enabled, void *user_data);

void temperature_lbs_register_notify_cb(temperature_lbs_notify_state_cb_t cb, void *user_data);
void temperature_lbs_register_sampling_rate_cb(void (*callback)(uint16_t));
void temperature_lbs_register_transfer_interval_cb(void (*callback)(uint16_t));
void temperature_lbs_set_conn(struct bt_conn *conn);
void temperature_lbs_clear_conn(void);

int temperature_lbs_notify(const struct temperature_lbs_sample *sample);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
