#include <stdbool.h>
#include <zephyr/kernel.h>

#define BHI360_CS_GPIO_NODE DT_NODELABEL(gpio1)
#define BHI360_CS_PIN (13)

struct imu_quat_data {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	uint16_t accuracy;
};

struct imu_lacc_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef void (*imu_quat_cb_t)(const struct imu_quat_data *data, void *user_data);
typedef void (*imu_lacc_cb_t)(const struct imu_lacc_data *data, void *user_data);

int imu_register_quaternion_callback(imu_quat_cb_t cb, void *user_data);
int imu_register_linear_accel_callback(imu_lacc_cb_t cb, void *user_data);
void imu_set_streaming_enabled(bool enabled);
int imu_start(void);
void imu_stop(void);

// Function to initialize sensor reading
void imu_sensor_init(void);
