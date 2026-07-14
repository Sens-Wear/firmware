/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SenseWear device manager.
 *
 * The device manager (boards/common/device_manager) is the single publisher that
 * drains the shared device-event queue, drives each device's interrupt handler,
 * and republishes decoded data onto one zbus channel per message type. This test
 * enables the manager (CONFIG_SENSEWEAR_DEVICE_MANAGER, selected by the test
 * symbol), configures only devices that initialized, subscribes to ready
 * streams, and drives the pipeline:
 *
 *   - Fuel gauge: if present and enabled, configured and refreshed once for
 *     immediate, deterministic battery stream output; the manager's RTC-minute
 *     cadence then refreshes it again every minute.
 *   - PPG, temperature, touch, IMU, charger, and regulator: if present and
 *     enabled, configured or monitored through the manager before subscribing to
 *     their streams.
 *   - Haptic motor: if the sensewear_haptic shield is present, a short buzz plus
 *     a ROM waveform exercises the actuator control API.
 *
 * Build with the `test_device_manager` preset; see tests/device_manager/README.md
 * and BUILD.md. Add SHIELD=sensewear_haptic to also exercise the vibration motor.
 */

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
#include "bq27427.h"
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
#include "max30101.h"
#endif

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER) || defined(CONFIG_SENSEWEAR_BQ25180_DRIVER) || \
	defined(CONFIG_SENSEWEAR_BQ27427_DRIVER) || defined(CONFIG_SHIELD_SENSEWEAR_PPG) ||    \
	defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE) || defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
static bool configure_ready_device(enum device_manager_device_type device,
								   enum device_manager_stream_type stream,
								   const char* name) {
	/* A device is only reported "ready" once it has been configured, so
	 * configuration must come first — gating it on readiness would skip every
	 * device that merely initialized. A device that did not initialize fails to
	 * configure and is skipped here. */
	int ret = device_manager_config_device(device, NULL);

	if (ret != 0) {
		printk("device_manager_configure_device(%s) failed: %d\n", name, ret);
		return false;
	}

	if (!device_manager_stream_ready(stream)) {
		printk("%s configured but stream not ready; skipping\n", name);
		return false;
	}

	return true;
}
#endif

/* A basic subscriber notifies with the channel pointer; the consumer reads the
 * message from that channel. Streams are registered at run time only when the
 * producing device reports ready through the device manager. */
ZBUS_SUBSCRIBER_DEFINE(dm_sub, 16);

static const enum device_manager_stream_type dm_test_streams[] = {
	device_manager_stream_ImuQuaternion,
	device_manager_stream_ImuAccel,
	device_manager_stream_ImuGyro,
	device_manager_stream_ImuPedometer,
	device_manager_stream_ImuGesture,
	device_manager_stream_ImuActivity,
	device_manager_stream_Ppg,
	device_manager_stream_Temperature,
	device_manager_stream_Touch,
	device_manager_stream_TouchGesture,
	device_manager_stream_Battery,
	device_manager_stream_Charger,
	device_manager_stream_Regulator,
};

static bool dm_configured_streams[device_manager_stream_Count];

static void register_configured_streams(void) {
	for (size_t i = 0; i < ARRAY_SIZE(dm_test_streams); ++i) {
		enum device_manager_stream_type stream = dm_test_streams[i];

		if (!dm_configured_streams[stream]) {
			continue;
		}
		if (!device_manager_stream_ready(stream)) {
			continue;
		}

		int ret = device_manager_stream_register(stream, &dm_sub, K_MSEC(100));

		if (ret != 0) {
			printk("stream %d registration failed: %d\n", (int) stream, ret);
		}
	}
}

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
/** Slow FIFO-drain period used while the physical sensors are idle, in ms. */
#define DM_TEST_IMU_IDLE_DRAIN_MS 60000

static void enable_imu_streams(void) {
	dm_configured_streams[device_manager_stream_ImuQuaternion] = true;
	dm_configured_streams[device_manager_stream_ImuAccel] = true;
	dm_configured_streams[device_manager_stream_ImuGyro] = true;
	dm_configured_streams[device_manager_stream_ImuPedometer] = true;
	dm_configured_streams[device_manager_stream_ImuGesture] = true;
	dm_configured_streams[device_manager_stream_ImuActivity] = true;
}
#endif /* CONFIG_SENSEWEAR_BHI360_DRIVER */

/* One buffer large enough for any stream message. */
union dm_any_msg {
	struct imu_quaternion_msg_t imu_quaternion;
	struct imu_accel_msg_t imu_accel;
	struct imu_gyro_msg_t imu_gyro;
	struct imu_pedometer_msg_t imu_pedometer;
	struct imu_gesture_msg_t imu_gesture;
	struct imu_activity_msg_t imu_activity;
	struct ppg_msg_t ppg;
	struct temperature_msg_t temperature;
	struct touch_msg_t touch;
	struct touch_gesture_msg_t touch_gesture;
	struct battery_msg_t battery;
	struct charger_msg_t charger;
	struct regulator_msg_t regulator;
};

static void print_message(const struct zbus_channel* chan) {
	union dm_any_msg msg;
	enum device_manager_stream_type stream = device_manager_stream_from_channel(chan);

	if (zbus_chan_read(chan, &msg, K_MSEC(50)) != 0) {
		return;
	}

	switch (stream) {
	case device_manager_stream_Battery:
		printk("battery @ %lld: soc=%d.%d%%  v=%dmV  i=%dmA  rem=%dmAh  learning=%d\n",
			   (long long) msg.battery.timestamp,
			   msg.battery.state_of_charge_dpct / 10,
			   msg.battery.state_of_charge_dpct % 10,
			   msg.battery.voltage_mv,
			   msg.battery.average_current_ma,
			   msg.battery.remaining_capacity_mah,
			   msg.battery.learning_in_progress);
		break;
	case device_manager_stream_Charger:
		printk("charger @ %lld: pg=%d charging=%d charged=%d fault=%d flags=0x%08x irq=%lld\n",
			   (long long) msg.charger.timestamp,
			   msg.charger.power_good,
			   msg.charger.charging,
			   msg.charger.charged,
			   msg.charger.fault,
			   msg.charger.flags,
			   (long long) msg.charger.last_irq_time);
		break;
	case device_manager_stream_Regulator:
		printk("regulator @ %lld: enabled=%d vout=%u uV\n",
			   (long long) msg.regulator.timestamp,
			   msg.regulator.enabled,
			   msg.regulator.vout_uv);
		break;
	case device_manager_stream_Touch:
		printk("touch @ %lld: touched=%d x=%u y=%u\n",
			   (long long) msg.touch.timestamp,
			   msg.touch.touched,
			   msg.touch.x,
			   msg.touch.y);
		break;
	case device_manager_stream_TouchGesture:
		printk("touch gesture @ %lld: %d\n",
			   (long long) msg.touch_gesture.timestamp,
			   (int) msg.touch_gesture.gesture);
		break;
	case device_manager_stream_Temperature:
		printk("temperature @ %lld: %d m°C\n",
			   (long long) msg.temperature.timestamp,
			   msg.temperature.temperature_mdeg_c);
		break;
	case device_manager_stream_Ppg:
		printk("ppg @ %lld: ir=%u red=%u green=%u\n",
			   (long long) msg.ppg.timestamp,
			   msg.ppg.ir,
			   msg.ppg.red,
			   msg.ppg.green);
		break;
	case device_manager_stream_ImuQuaternion:
		printk("imu quat @ %lld: x=%d y=%d z=%d w=%d\n",
			   (long long) msg.imu_quaternion.timestamp,
			   msg.imu_quaternion.x,
			   msg.imu_quaternion.y,
			   msg.imu_quaternion.z,
			   msg.imu_quaternion.w);
		break;
	case device_manager_stream_ImuAccel:
		printk("imu accel @ %lld: x=%d y=%d z=%d\n",
			   (long long) msg.imu_accel.timestamp,
			   msg.imu_accel.x,
			   msg.imu_accel.y,
			   msg.imu_accel.z);
		break;
	case device_manager_stream_ImuGyro:
		printk("imu gyro @ %lld: x=%d y=%d z=%d\n",
			   (long long) msg.imu_gyro.timestamp,
			   msg.imu_gyro.x,
			   msg.imu_gyro.y,
			   msg.imu_gyro.z);
		break;
	case device_manager_stream_ImuPedometer:
		printk("imu pedometer @ %lld: steps=%u detected=%d\n",
			   (long long) msg.imu_pedometer.timestamp,
			   msg.imu_pedometer.step_count,
			   msg.imu_pedometer.step_detected);
		break;
	case device_manager_stream_ImuGesture:
		printk("imu gesture @ %lld: %d\n",
			   (long long) msg.imu_gesture.timestamp,
			   (int) msg.imu_gesture.gesture);
		break;
	case device_manager_stream_ImuActivity:
		printk("imu activity @ %lld: type=%d transition=%d\n",
			   (long long) msg.imu_activity.timestamp,
			   (int) msg.imu_activity.activity,
			   (int) msg.imu_activity.transition);
		break;
	default:
		break;
	}
}

int main(void) {
	printk("\n=== Device manager test ===\n");

#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	/* Give the wall clock a value so timestamps and the minute alarm work. */
	int rtc_ret = device_manager_set_rtc_time((time_t) 1767225600); /* 2026-01-01T00:00:00Z */

	if (rtc_ret != 0) {
		printk("device_manager_set_rtc_time() failed: %d\n", rtc_ret);
	}
#endif

	if (device_manager_init() != 0) {
		printk("device_manager_init() failed\n");
		return 0;
	}

	struct device_manager_config_t cfg;
	device_manager_get_default_config(&cfg);
	/* This test exercises the low-rate IMU event sensors (pedometer, gesture,
	 * activity), which are interrupt-driven; the high-rate physical streams stay
	 * off. Their FIFO is drained on a slow one-minute cadence, set up after
	 * configuration below, as a backstop to the hardware interrupts. */
	cfg.imu.phy_streams_enabled = false;
	cfg.gauge.update_period_min = 0;
	cfg.temperature.update_period_min = 0;

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	bool imu_configured = configure_ready_device(device_manager_device_Bhi360,
												 device_manager_stream_ImuQuaternion,
												 "Bhi360");
	if (imu_configured) {
		enable_imu_streams();
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
	bool charger_configured = configure_ready_device(device_manager_device_Bq25180,
													 device_manager_stream_Charger,
													 "Bq25180");
	if (charger_configured) {
		dm_configured_streams[device_manager_stream_Charger] = true;
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	/* Refresh the fuel gauge every minute only when the gauge initialized. */
	bool battery_configured = configure_ready_device(device_manager_device_Bq27427,
													 device_manager_stream_Battery,
													 "Bq27427");
	if (battery_configured) {
		dm_configured_streams[device_manager_stream_Battery] = true;
		cfg.gauge.update_period_min = 1;
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
	bool ppg_configured = configure_ready_device(device_manager_device_Max30101,
												 device_manager_stream_Ppg,
												 "Max30101");
	if (ppg_configured) {
		dm_configured_streams[device_manager_stream_Ppg] = true;

		/* Configuration alone does not start acquisition; enable multi-LED wrist-HR
		 * sampling so the PPG stream actually produces ir/red/green samples. A_FULL
		 * batching (per_sample_irq = false) drains once per FIFO almost-full. */
		int ppg_ret = max30101_enable_wrist_hr_sampling(false);

		if (ppg_ret != 0) {
			printk("max30101_enable_wrist_hr_sampling() failed: %d\n", ppg_ret);
		}
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	bool temperature_configured = configure_ready_device(device_manager_device_Max30208,
														 device_manager_stream_Temperature,
														 "Max30208");
	if (temperature_configured) {
		dm_configured_streams[device_manager_stream_Temperature] = true;
		cfg.temperature.update_period_min = 1;
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
	bool touch_configured = configure_ready_device(device_manager_device_Mtch6102,
												   device_manager_stream_Touch,
												   "Mtch6102");
	if (touch_configured) {
		dm_configured_streams[device_manager_stream_Touch] = true;
		dm_configured_streams[device_manager_stream_TouchGesture] = true;
	}
#endif

#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
	if (device_manager_stream_ready(device_manager_stream_Regulator)) {
		dm_configured_streams[device_manager_stream_Regulator] = true;
	} else {
		printk("Tpsm83102 not ready; skipping regulator stream\n");
	}
#endif

	int cfg_ret = device_manager_config(&cfg);

	if (cfg_ret != 0) {
		printk("device_manager_config() failed: %d\n", cfg_ret);
	}

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	/* The physical sensors are not sampling in this test, so device_manager_config()
	 * leaves the shared FIFO timer stopped. The low-rate event sensors (pedometer,
	 * gesture, activity) are interrupt-driven, but drain the FIFO once a minute as
	 * well so anything that does not raise its own interrupt still surfaces. */
	if (imu_configured) {
		int drain_ret = device_manager_set_imu_drain_period(DM_TEST_IMU_IDLE_DRAIN_MS);

		if (drain_ret != 0) {
			printk("device_manager_set_imu_drain_period() failed: %d\n", drain_ret);
		}
	}
#endif

	register_configured_streams();

	int start_ret = device_manager_start();

	if (start_ret != 0) {
		printk("device_manager_start() failed: %d\n", start_ret);
		return 0;
	}

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	if (battery_configured) {
		(void) bq27427_update_state(NULL);
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
	/* Actuator check: a short RTP buzz, then a ROM click effect. */
	static const uint8_t click_seq[] = {1};

	(void) device_manager_haptic_vibrate(200, 300);
	k_msleep(500);
	(void) device_manager_haptic_play_rom(click_seq, ARRAY_SIZE(click_seq));
#endif

	printk("Subscribed to ready streams; interact with the device to see events...\n");

	const struct zbus_channel* chan;

	while (zbus_sub_wait(&dm_sub, &chan, K_FOREVER) == 0) {
		print_message(chan);
	}

	return 0;
}
