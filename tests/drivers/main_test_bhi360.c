/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal bring-up test for the BHI360 IMU driver.
 *
 * Registers quaternion and linear-acceleration callbacks, starts the sensor
 * and enables streaming, then prints the most recent sample of each every
 * 2 seconds.
 *
 * Swap this file in for src/main.c (see tests/drivers/README.md) and flash to
 * exercise the driver on hardware.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bhi360.h"

#define TEST_POLL_INTERVAL K_SECONDS(2)

static struct imu_quat_data last_quat;
static struct imu_lacc_data last_lacc;
static atomic_t have_quat;
static atomic_t have_lacc;

static void on_quaternion(const struct imu_quat_data *data, void *user_data)
{
	ARG_UNUSED(user_data);
	last_quat = *data;
	atomic_set(&have_quat, 1);
}

static void on_linear_accel(const struct imu_lacc_data *data, void *user_data)
{
	ARG_UNUSED(user_data);
	last_lacc = *data;
	atomic_set(&have_lacc, 1);
}

int main(void)
{
	printk("\n=== BHI360 IMU driver test ===\n");

	imu_register_quaternion_callback(on_quaternion, NULL);
	imu_register_linear_accel_callback(on_linear_accel, NULL);

	imu_sensor_init();
	imu_set_streaming_enabled(true);

	if (imu_start() != 0) {
		printk("imu_start() failed\n");
		return 0;
	}

	printk("streaming; printing latest sample every 2 s...\n");

	while (1) {
		if (atomic_get(&have_quat)) {
			printk("quat:  x=%6d y=%6d z=%6d w=%6d acc=%u\n", last_quat.x,
			       last_quat.y, last_quat.z, last_quat.w, last_quat.accuracy);
		} else {
			printk("quat:  <no sample yet>\n");
		}

		if (atomic_get(&have_lacc)) {
			printk("lacc:  x=%6d y=%6d z=%6d\n", last_lacc.x, last_lacc.y,
			       last_lacc.z);
		} else {
			printk("lacc:  <no sample yet>\n");
		}

		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
