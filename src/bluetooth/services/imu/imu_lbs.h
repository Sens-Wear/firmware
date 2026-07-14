#ifndef IMU_LBS_H_
#define IMU_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <time.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

#define BT_UUID_LBS_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x7d2b6c10, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_QUAT_VAL BT_UUID_128_ENCODE(0x7d2b6c11, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_LACC_VAL BT_UUID_128_ENCODE(0x7d2b6c12, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_GYRO_VAL BT_UUID_128_ENCODE(0x7d2b6c13, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_GESTURE_VAL BT_UUID_128_ENCODE(0x7d2b6c14, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_ACTIVITY_VAL BT_UUID_128_ENCODE(0x7d2b6c15, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)

#define BT_UUID_LBS_IMU_CONFIG_SERVICE_VAL BT_UUID_128_ENCODE(0x7d2b6c20, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE_VAL BT_UUID_128_ENCODE(0x7d2b6c21, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD_VAL BT_UUID_128_ENCODE(0x7d2b6c22, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)

#define BT_UUID_LBS_IMU_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_SERVICE_VAL)
#define BT_UUID_LBS_IMU_QUAT BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_QUAT_VAL)
#define BT_UUID_LBS_IMU_LACC BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_LACC_VAL)
#define BT_UUID_LBS_IMU_GYRO BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_GYRO_VAL)
#define BT_UUID_LBS_IMU_GESTURE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_GESTURE_VAL)
#define BT_UUID_LBS_IMU_ACTIVITY BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_ACTIVITY_VAL)

#define BT_UUID_LBS_IMU_CONFIG_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_SERVICE_VAL)
#define BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_PHY_ENABLE_VAL)
#define BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_CONFIG_DRAIN_PERIOD_VAL)

struct imu_lbs_vec3 {
	time_t timestamp;
	int16_t x;
	int16_t y;
	int16_t z;
} __packed;

struct imu_lbs_quat {
	time_t timestamp;
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	uint16_t accuracy;
} __packed;

struct imu_lbs_gesture {
	time_t timestamp;
	uint8_t sensor_id;
	uint8_t gesture;
} __packed;

struct imu_lbs_activity {
	time_t timestamp;
	uint8_t sensor_id;
	uint8_t activity;
	uint8_t transition;
} __packed;

void imu_lbs_set_conn(struct bt_conn* conn);
void imu_lbs_clear_conn(void);
bool imu_lbs_streams_ready(void);
int imu_lbs_register_streams(void);

#ifdef __cplusplus
}
#endif

#endif
