#ifndef PRESSURE_LBS_H_
#define PRESSURE_LBS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#define BT_UUID_LBS_PRESSURE_SERVICE_VAL BT_UUID_128_ENCODE(0x3ef55cb7, 0xbe49, 0xbe49, 0x84ab, 0x950201eb3d3a)
#define BT_UUID_LBS_PRESSURE_SAMPLING_RATE_CONF_VAL BT_UUID_128_ENCODE(0x3ef55cb8, 0xbe49, 0xbe49, 0x84ab, 0x950201eb3d3a)
#define BT_UUID_LBS_PRESSURE_TRANSFER_INTERVAL_CONF_VAL BT_UUID_128_ENCODE(0x3ef55cb9, 0xbe49, 0xbe49, 0x84ab, 0x950201eb3d3a)
#define BT_UUID_LBS_PRESSURE_VAL BT_UUID_128_ENCODE(0x3ef55cba, 0xbe49, 0xbe49, 0x84ab, 0x950201eb3d3a)

#define BT_UUID_LBS_PRESSURE_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_PRESSURE_SERVICE_VAL)
#define BT_UUID_LBS_PRESSURE_SAMPLING_RATE_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PRESSURE_SAMPLING_RATE_CONF_VAL)
#define BT_UUID_LBS_PRESSURE_TRANSFER_INTERVAL_CONF BT_UUID_DECLARE_128(BT_UUID_LBS_PRESSURE_TRANSFER_INTERVAL_CONF_VAL)
#define BT_UUID_LBS_PRESSURE BT_UUID_DECLARE_128(BT_UUID_LBS_PRESSURE_VAL)

void register_pressure_status_callback(void (*callback)(bool));
void register_pressure_sampling_rate_callback(void (*callback)(uint16_t));
void register_pressure_transfer_interval_callback(void (*callback)(uint16_t));

int pressure_lbs_send_sensor_notify(uint8_t* sensor_value, size_t size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif