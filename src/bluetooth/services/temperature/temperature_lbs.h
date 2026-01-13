#ifndef TEMPERATURE_LBS_H_
#define TEMPERATURE_LBS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#define BT_UUID_LBS_TEMPERATURE_SERVICE_VAL BT_UUID_128_ENCODE(0x8e83a64b, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF_VAL BT_UUID_128_ENCODE(0x8e83a64c, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x8e83a64d, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)
#define BT_UUID_LBS_TEMPERATURE_VAL BT_UUID_128_ENCODE(0x8e83a64f, 0x319d, 0x47c2, 0xba53, 0x6185fae0007f)

#define BT_UUID_LBS_TEMPERATURE_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_SERVICE_VAL)
#define BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_SAMPLING_RATE_CONF_VAL)
#define BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_TEMPERATURE BT_UUID_DECLARE_128(BT_UUID_LBS_TEMPERATURE_VAL)

void register_temperature_status_callback(void (*callback)(bool));
void register_temperature_sampling_rate_callback(void (*callback)(uint16_t));
void register_temperature_transfer_interval_callback(void (*callback)(uint16_t));

int temperature_lbs_send_sensor_notify(uint8_t* sensor_value, size_t size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif