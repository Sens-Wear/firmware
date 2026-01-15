#ifndef IMU_LBS_H_
#define IMU_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>

#define BT_UUID_LBS_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x7d2b6c10, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_QUAT_VAL BT_UUID_128_ENCODE(0x7d2b6c11, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)
#define BT_UUID_LBS_IMU_LACC_VAL BT_UUID_128_ENCODE(0x7d2b6c12, 0x9d78, 0x4f3c, 0xa122, 0x6d2c4e6d2a11)

#define BT_UUID_LBS_IMU_SERVICE BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_SERVICE_VAL)
#define BT_UUID_LBS_IMU_QUAT BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_QUAT_VAL)
#define BT_UUID_LBS_IMU_LACC BT_UUID_DECLARE_128(BT_UUID_LBS_IMU_LACC_VAL)

struct imu_lbs_quat {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	uint16_t accuracy;
} __packed;

struct imu_lbs_lacc {
	int16_t x;
	int16_t y;
	int16_t z;
} __packed;

typedef void (*imu_lbs_notify_state_cb_t)(bool enabled, void *user_data);

void imu_lbs_register_quat_notify_cb(imu_lbs_notify_state_cb_t cb, void *user_data);
void imu_lbs_register_lacc_notify_cb(imu_lbs_notify_state_cb_t cb, void *user_data);
void imu_lbs_set_conn(struct bt_conn *conn);
void imu_lbs_clear_conn(void);

int imu_lbs_notify_quat(const struct imu_lbs_quat *data);
int imu_lbs_notify_lacc(const struct imu_lbs_lacc *data);

#ifdef __cplusplus
}
#endif

#endif
