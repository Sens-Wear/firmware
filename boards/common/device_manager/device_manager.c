/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_manager.c
 * @brief SenseWear device manager implementation.
 * @details Single publisher over per-message-type zbus streams. A consumer
 *          thread drains the shared device-event queue, drives each device's
 *          interrupt handler, and translates decoded driver events into the
 *          stable @ref sensewear_device_driver_messages contract, publishing each
 *          by value onto its channel.
 *
 * This translation unit is the one place that includes both the device-driver
 * headers and the message contract; it is compiled only when
 * CONFIG_SENSEWEAR_DEVICE_MANAGER is set, so isolated device/shield test images
 * do not pull it in. Per-device translators are guarded by the same Kconfig
 * symbols that gate the drivers themselves.
 */

#include "device_manager.h"

#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
#include "bhi360.h"
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
#include "drv2605.h"
#endif
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
#include "bq25180.h"
#endif
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
#include "bq27427.h"
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
#include "max30101.h"
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
#include "max30208.h"
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
#include "mtch6102.h"
#endif
#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
#include "tpsm83102.h"
#endif
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
#include "rtc.h"
#endif

LOG_MODULE_REGISTER(device_manager, CONFIG_LOG_DEFAULT_LEVEL);

/** Consumer-thread stack size, in bytes. */
#define DEVICE_MANAGER_STACK_SIZE 2048
/** Consumer-thread priority. */
#define DEVICE_MANAGER_THREAD_PRIO 7
/** Timeout for acquiring a channel when publishing, in milliseconds. */
#define DEVICE_MANAGER_PUB_TIMEOUT_MS 10
/** Upper bound on IMU samples copied from one batch event. */
#define DEVICE_MANAGER_IMU_BATCH_MAX 16

/* ------------------------------------------------------------------------- */
/* Streams: one channel per message type                                      */
/* ------------------------------------------------------------------------- */

ZBUS_CHAN_DEFINE(chan_imu_quaternion,
				 struct imu_quaternion_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_imu_accel,
				 struct imu_accel_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_imu_gyro,
				 struct imu_gyro_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_imu_pedometer,
				 struct imu_pedometer_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_imu_gesture,
				 struct imu_gesture_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_imu_activity,
				 struct imu_activity_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_ppg, struct ppg_msg_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_temperature,
				 struct temperature_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_touch,
				 struct touch_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_touch_gesture,
				 struct touch_gesture_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_battery,
				 struct battery_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_charger,
				 struct charger_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(chan_regulator,
				 struct regulator_msg_t,
				 NULL,
				 NULL,
				 ZBUS_OBSERVERS_EMPTY,
				 ZBUS_MSG_INIT(0));

/** Publish one message by value; best-effort, never blocks the manager long. */
static void device_manager_publish(const struct zbus_channel* chan, const void* msg) {
	int ret = zbus_chan_pub(chan, msg, K_MSEC(DEVICE_MANAGER_PUB_TIMEOUT_MS));

	if (ret != 0) {
		LOG_WRN("device manager: publish failed (%d)", ret);
	}
}

/* ------------------------------------------------------------------------- */
/* Configuration                                                              */
/* ------------------------------------------------------------------------- */

static struct device_manager_config_t device_manager_config_state;
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
static uint16_t device_manager_gauge_minutes; /**< RTC minutes elapsed since last gauge refresh. */
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
static uint16_t
	device_manager_temp_minutes; /**< RTC minutes elapsed since last temperature read. */
#endif

void device_manager_get_default_config(struct device_manager_config_t* config) {
	if (config == NULL) {
		return;
	}

	*config = (struct device_manager_config_t) {
		.imu = {.phy_streams_enabled = false, .drain_period_ms = 100},
		.gauge = {.update_period_min = 5},
		.temperature = {.update_period_min = 1},
	};
}

static bool device_manager_device_enum_valid(enum device_manager_device_type device) {
	return device > device_manager_device_Invalid && device < device_manager_device_Count;
}

static bool device_manager_device_supported(enum device_manager_device_type device) {
	switch (device) {
	case device_manager_device_Bhi360:
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
		return true;
#else
		return false;
#endif
	case device_manager_device_Bq25180:
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
		return true;
#else
		return false;
#endif
	case device_manager_device_Bq27427:
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
		return true;
#else
		return false;
#endif
	case device_manager_device_Max30101:
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
		return true;
#else
		return false;
#endif
	case device_manager_device_Max30208:
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
		return true;
#else
		return false;
#endif
	case device_manager_device_Mtch6102:
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
		return true;
#else
		return false;
#endif
	case device_manager_device_Tpsm83102:
#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
		return true;
#else
		return false;
#endif
	default:
		return false;
	}
}

static bool device_manager_device_ready(enum device_manager_device_type device) {
	switch (device) {
	case device_manager_device_Bhi360:
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
		return bhi360_is_ready();
#else
		return false;
#endif
	case device_manager_device_Bq25180:
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
		return bq25180_is_ready();
#else
		return false;
#endif
	case device_manager_device_Bq27427:
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
		return bq27427_is_ready();
#else
		return false;
#endif
	case device_manager_device_Max30101:
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
		return max30101_is_ready();
#else
		return false;
#endif
	case device_manager_device_Max30208:
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
		return max30208_is_ready();
#else
		return false;
#endif
	case device_manager_device_Mtch6102:
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
		return mtch6102_is_ready();
#else
		return false;
#endif
	case device_manager_device_Tpsm83102:
#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
		/* Devicetree-instantiated regulator: no SenseWear init/config step, so
		 * readiness is just the Zephyr device being ready. */
		return device_is_ready(DEVICE_DT_GET(DT_NODELABEL(tpsm83102)));
#else
		return false;
#endif
	default:
		return false;
	}
}

static enum device_manager_device_type device_manager_stream_device(
	enum device_manager_stream_type stream) {
	switch (stream) {
	case device_manager_stream_ImuQuaternion:
	case device_manager_stream_ImuAccel:
	case device_manager_stream_ImuGyro:
	case device_manager_stream_ImuPedometer:
	case device_manager_stream_ImuGesture:
	case device_manager_stream_ImuActivity:
		return device_manager_device_Bhi360;
	case device_manager_stream_Ppg:
		return device_manager_device_Max30101;
	case device_manager_stream_Temperature:
		return device_manager_device_Max30208;
	case device_manager_stream_Touch:
	case device_manager_stream_TouchGesture:
		return device_manager_device_Mtch6102;
	case device_manager_stream_Battery:
		return device_manager_device_Bq27427;
	case device_manager_stream_Charger:
		return device_manager_device_Bq25180;
	case device_manager_stream_Regulator:
		return device_manager_device_Tpsm83102;
	default:
		return device_manager_device_Invalid;
	}
}

static const struct zbus_channel* device_manager_stream_channel(
	enum device_manager_stream_type stream) {
	switch (stream) {
	case device_manager_stream_ImuQuaternion:
		return &chan_imu_quaternion;
	case device_manager_stream_ImuAccel:
		return &chan_imu_accel;
	case device_manager_stream_ImuGyro:
		return &chan_imu_gyro;
	case device_manager_stream_ImuPedometer:
		return &chan_imu_pedometer;
	case device_manager_stream_ImuGesture:
		return &chan_imu_gesture;
	case device_manager_stream_ImuActivity:
		return &chan_imu_activity;
	case device_manager_stream_Ppg:
		return &chan_ppg;
	case device_manager_stream_Temperature:
		return &chan_temperature;
	case device_manager_stream_Touch:
		return &chan_touch;
	case device_manager_stream_TouchGesture:
		return &chan_touch_gesture;
	case device_manager_stream_Battery:
		return &chan_battery;
	case device_manager_stream_Charger:
		return &chan_charger;
	case device_manager_stream_Regulator:
		return &chan_regulator;
	default:
		return NULL;
	}
}

bool device_manager_stream_valid(enum device_manager_stream_type stream) {
	enum device_manager_device_type device = device_manager_stream_device(stream);

	return device_manager_stream_channel(stream) != NULL &&
		   device_manager_device_supported(device) && device_manager_device_ready(device);
}

bool device_manager_stream_ready(enum device_manager_stream_type stream) {
	return device_manager_stream_valid(stream);
}

static bool device_manager_stream_exists(enum device_manager_stream_type stream) {
	enum device_manager_device_type device = device_manager_stream_device(stream);

	return device_manager_stream_channel(stream) != NULL && device_manager_device_supported(device);
}

enum device_manager_stream_type device_manager_stream_from_channel(
	const struct zbus_channel* chan) {
	if (chan == NULL) {
		return device_manager_stream_Invalid;
	}

	for (enum device_manager_stream_type stream = device_manager_stream_ImuQuaternion;
		 stream < device_manager_stream_Count;
		 stream++) {
		if (device_manager_stream_channel(stream) == chan) {
			return stream;
		}
	}

	return device_manager_stream_Invalid;
}

/** Per-stream registered-observer counts; gates device_manager_start(). */
static uint8_t device_manager_stream_observers[device_manager_stream_Count];

int device_manager_stream_register(enum device_manager_stream_type stream,
								   const struct zbus_observer* obs,
								   k_timeout_t timeout) {
	if (obs == NULL || !device_manager_stream_exists(stream)) {
		return -EINVAL;
	}
	if (!device_manager_stream_valid(stream)) {
		return -ENODEV;
	}

	int ret = zbus_chan_add_obs(device_manager_stream_channel(stream), obs, timeout);

	if (ret == 0) {
		device_manager_stream_observers[stream]++;
	}
	return ret;
}

int device_manager_stream_unregister(enum device_manager_stream_type stream,
									 const struct zbus_observer* obs,
									 k_timeout_t timeout) {
	const struct zbus_channel* chan = device_manager_stream_channel(stream);

	if (obs == NULL || !device_manager_stream_exists(stream) || chan == NULL) {
		return -EINVAL;
	}

	int ret = zbus_chan_rm_obs(chan, obs, timeout);

	if (ret == 0 && device_manager_stream_observers[stream] > 0) {
		device_manager_stream_observers[stream]--;
	}
	return ret;
}

int device_manager_get_default_device_config(enum device_manager_device_type device, void* config) {
	if (!device_manager_device_enum_valid(device) || config == NULL) {
		return -EINVAL;
	}

	switch (device) {
	case device_manager_device_Bhi360:
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
		bhi360_get_default_config((struct bhi360_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Bq25180:
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
		bq25180_get_default_lipo_usb_charger_config((struct bq25180_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Bq27427:
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
		bq27427_get_default_config((struct bq27427_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Max30101:
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
		max30101_get_default_config((struct max30101_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Max30208:
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
		max30208_get_default_config((struct max30208_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Mtch6102:
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
		mtch6102_get_default_config((struct mtch6102_config_t*) config);
		return 0;
#else
		return -ENOTSUP;
#endif
	default:
		return -EINVAL;
	}
}

int device_manager_config_device(enum device_manager_device_type device, const void* config) {
	if (!device_manager_device_enum_valid(device)) {
		return -EINVAL;
	}

	switch (device) {
	case device_manager_device_Bhi360: {
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
		struct bhi360_config_t default_config;
		const struct bhi360_config_t* imu_config = config;

		if (imu_config == NULL) {
			device_manager_get_default_device_config(device, &default_config);
			imu_config = &default_config;
		}
		if (!bhi360_config(imu_config)) {
			return -EIO;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	case device_manager_device_Bq25180: {
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
		const struct bq25180_config_t* charger_config = (const struct bq25180_config_t*) config;

		if (!bq25180_config(charger_config)) {
			return -EIO;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	case device_manager_device_Bq27427: {
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
		struct bq27427_config_t local_config;

		if (config == NULL) {
			bq27427_get_default_config(&local_config);
		} else {
			local_config = *(const struct bq27427_config_t*) config;
		}
		if (!bq27427_config(&local_config)) {
			return -EIO;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	case device_manager_device_Max30101: {
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
		struct max30101_config_t local_config;
		const struct max30101_config_t* ppg_config = NULL;

		if (config != NULL) {
			local_config = *(const struct max30101_config_t*) config;
			ppg_config = &local_config;
		}
		int ppg_ret = max30101_config(ppg_config);
		if (ppg_ret != 0) {
			return ppg_ret;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	case device_manager_device_Max30208: {
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
		struct max30208_config_t local_config;
		const struct max30208_config_t* temperature_config = NULL;

		if (config != NULL) {
			local_config = *(const struct max30208_config_t*) config;
			temperature_config = &local_config;
		}
		int temperature_ret = max30208_config(temperature_config);
		if (temperature_ret != 0) {
			return temperature_ret;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	case device_manager_device_Mtch6102: {
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
		struct mtch6102_config_t local_config;
		const struct mtch6102_config_t* touch_config = NULL;

		if (config != NULL) {
			local_config = *(const struct mtch6102_config_t*) config;
			touch_config = &local_config;
		}
		int touch_ret = mtch6102_config(touch_config);
		if (touch_ret != 0) {
			return touch_ret;
		}
		return 0;
#else
		ARG_UNUSED(config);
		return -ENOTSUP;
#endif
	}
	default:
		return -EINVAL;
	}
}

#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
/** Arm the RTC minute alarm when any minute-driven periodic update is active. */
static void device_manager_refresh_minute_alarm(void) {
	bool need = false;

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	need = need || (device_manager_config_state.gauge.update_period_min > 0);
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	need = need || (device_manager_config_state.temperature.update_period_min > 0);
#endif

	(void) rtc_enable_minute_alarm(need);
}
#endif /* CONFIG_SENSEWEAR_RTC_DRIVER */

/**
 * @brief Run one managed device's driver init.
 * @retval 0 The device initialized.
 * @retval -EIO The driver reports only boolean status and failed.
 * @retval -ENOTSUP The device's driver is not built.
 * @return A negative errno propagated from the driver.
 */
static int device_manager_init_device(enum device_manager_device_type device) {
	switch (device) {
	case device_manager_device_Bhi360:
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
		return bhi360_init() ? 0 : -EIO;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Bq25180:
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
		return bq25180_init() ? 0 : -EIO;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Bq27427:
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
		return bq27427_init() ? 0 : -EIO;
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Max30101:
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
		return max30101_init();
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Max30208:
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
		return max30208_init();
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Mtch6102:
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
		return mtch6102_init();
#else
		return -ENOTSUP;
#endif
	case device_manager_device_Tpsm83102:
#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
		/* Instantiated and initialized by devicetree at boot; nothing to do. */
		return 0;
#else
		return -ENOTSUP;
#endif
	default:
		return -EINVAL;
	}
}

int device_manager_config(const struct device_manager_config_t* config) {
	struct device_manager_config_t defaults;

	if (config == NULL) {
		device_manager_get_default_config(&defaults);
		config = &defaults;
	}

	/* Reject up front if a requested feature's device did not initialize, before
	 * any state is changed. */
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	if (config->imu.phy_streams_enabled && !bhi360_is_ready()) {
		LOG_ERR("device manager: IMU streaming requested but IMU not ready");
		return -ENODEV;
	}
#endif
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	if (config->gauge.update_period_min > 0 && !bq27427_is_ready()) {
		LOG_ERR("device manager: gauge cadence requested but gauge not ready");
		return -ENODEV;
	}
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	if (config->temperature.update_period_min > 0 && !max30208_is_ready()) {
		LOG_ERR("device manager: temperature cadence requested but temperature not ready");
		return -ENODEV;
	}
#endif

	device_manager_config_state = *config;

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	if (device_manager_config_state.imu.phy_streams_enabled) {
		/* Physical streams reuse the shared FIFO timer. Release any slow idle-drain
		 * period first so starting the streams cannot fail with -EBUSY when the
		 * timer is already running at a different period. */
		(void) bhi360_stop_periodic_timer();
		int ret = bhi360_start_phy_sensor_streams(device_manager_config_state.imu.drain_period_ms);
		if (ret != 0) {
			LOG_ERR("device manager: IMU stream start failed (%d)", ret);
			return ret;
		}
	} else {
		(void) bhi360_stop_phy_sensor_streams();
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	device_manager_gauge_minutes = 0;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	device_manager_temp_minutes = 0;
#endif
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	device_manager_refresh_minute_alarm();
#endif
	return 0;
}

int device_manager_set_imu_drain_period(uint32_t period_ms) {
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	if (!bhi360_is_ready()) {
		LOG_ERR("device manager: IMU not ready");
		return -ENODEV;
	}
	device_manager_config_state.imu.drain_period_ms = period_ms;
	return bhi360_start_periodic_timer(period_ms);
#else
	ARG_UNUSED(period_ms);
	return -ENOTSUP;
#endif
}

int device_manager_set_gauge_update_period(uint16_t minutes) {
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	if (minutes > 0 && !bq27427_is_ready()) {
		LOG_ERR("device manager: gauge not ready");
		return -ENODEV;
	}
	device_manager_gauge_minutes = 0;
#endif
	device_manager_config_state.gauge.update_period_min = minutes;
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	device_manager_refresh_minute_alarm();
#endif
	return 0;
}

int device_manager_set_body_temperature_update_period(uint16_t minutes) {
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	if (minutes > 0 && !max30208_is_ready()) {
		LOG_ERR("device manager: temperature not ready");
		return -ENODEV;
	}
	device_manager_temp_minutes = 0;
#endif
	device_manager_config_state.temperature.update_period_min = minutes;
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	device_manager_refresh_minute_alarm();
#endif
	return 0;
}

int device_manager_set_rtc_time(time_t unix_seconds) {
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	if (unix_seconds < (time_t) RTC_SANITY_MIN_UNIX) {
		return -EINVAL;
	}
	if (!rtc_is_ready() && !rtc_init()) {
		return -ENODEV;
	}
	if (!rtc_set_unix(unix_seconds)) {
		return -EIO;
	}
	return 0;
#else
	ARG_UNUSED(unix_seconds);
	return -ENOTSUP;
#endif
}

/* ------------------------------------------------------------------------- */
/* Vibration motor control (DRV2605)                                          */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_SHIELD_SENSEWEAR_HAPTIC)

static const struct device* const device_manager_haptic_dev =
	DEVICE_DT_GET(DT_ALIAS(sensewear_haptic));

/* The DRV2605 streams RTP frames asynchronously from these buffers, so the
 * manager owns them (static) and only overwrites them when no pattern is
 * playing. */
static uint8_t device_manager_rtp_amplitude[DEVICE_MANAGER_HAPTIC_RTP_MAX];
static uint32_t device_manager_rtp_hold[DEVICE_MANAGER_HAPTIC_RTP_MAX];
static struct drv2605_rtp_data device_manager_rtp;
static struct drv2605_rom_data device_manager_rom;

/** Configure the manager's RTP buffer as the source and begin playback. */
static int device_manager_haptic_play(size_t frames) {
	if (!device_is_ready(device_manager_haptic_dev)) {
		return -ENODEV;
	}
	if (drv2605_rtp_is_active(device_manager_haptic_dev)) {
		return -EBUSY;
	}

	device_manager_rtp.size = frames;
	device_manager_rtp.rtp_input = device_manager_rtp_amplitude;
	device_manager_rtp.rtp_hold_us = device_manager_rtp_hold;

	const union drv2605_config_data cfg = {.rtp_data = &device_manager_rtp};
	int ret = drv2605_haptic_config(device_manager_haptic_dev, DRV2605_HAPTICS_SOURCE_RTP, &cfg);

	if (ret != 0) {
		return ret;
	}
	return haptics_start_output(device_manager_haptic_dev);
}

int device_manager_haptic_vibrate(uint8_t amplitude, uint32_t duration_ms) {
	device_manager_rtp_amplitude[0] = amplitude;
	device_manager_rtp_hold[0] = duration_ms * 1000u;
	return device_manager_haptic_play(1);
}

int device_manager_haptic_start_rtp(const uint8_t* amplitude,
									const uint32_t* hold_us,
									size_t frames) {
	if (amplitude == NULL || hold_us == NULL || frames == 0) {
		return -EINVAL;
	}
	if (frames > DEVICE_MANAGER_HAPTIC_RTP_MAX) {
		return -E2BIG;
	}

	memcpy(device_manager_rtp_amplitude, amplitude, frames * sizeof(*amplitude));
	memcpy(device_manager_rtp_hold, hold_us, frames * sizeof(*hold_us));
	return device_manager_haptic_play(frames);
}

int device_manager_haptic_play_rom(const uint8_t* sequence, size_t count) {
	if (sequence == NULL || count == 0) {
		return -EINVAL;
	}
	if (count > DEVICE_MANAGER_HAPTIC_SEQ_MAX) {
		return -E2BIG;
	}
	if (!device_is_ready(device_manager_haptic_dev)) {
		return -ENODEV;
	}
	if (drv2605_rtp_is_active(device_manager_haptic_dev)) {
		return -EBUSY;
	}

	/* Zero-fill so unused sequencer slots terminate playback, then set the
	 * internal-trigger ROM source and copy the waveform sequence. The SenseWear
	 * haptic driver supports only the LRA library. */
	memset(&device_manager_rom, 0, sizeof(device_manager_rom));
	device_manager_rom.trigger = DRV2605_MODE_INTERNAL_TRIGGER;
	device_manager_rom.library = DRV2605_LIBRARY_LRA;
	memcpy(device_manager_rom.seq_regs, sequence, count * sizeof(*sequence));

	const union drv2605_config_data cfg = {.rom_data = &device_manager_rom};
	int ret = drv2605_haptic_config(device_manager_haptic_dev, DRV2605_HAPTICS_SOURCE_ROM, &cfg);

	if (ret != 0) {
		return ret;
	}
	return haptics_start_output(device_manager_haptic_dev);
}

int device_manager_haptic_stop(void) {
	if (!device_is_ready(device_manager_haptic_dev)) {
		return -ENODEV;
	}
	return haptics_stop_output(device_manager_haptic_dev);
}

bool device_manager_haptic_is_active(void) {
	return device_is_ready(device_manager_haptic_dev) &&
		   (drv2605_is_active(device_manager_haptic_dev) > 0);
}

#endif /* CONFIG_SHIELD_SENSEWEAR_HAPTIC */

/* ------------------------------------------------------------------------- */
/* Per-device translators                                                     */
/*                                                                            */
/* Each translator maps one device's decoded driver events onto the message   */
/* contract. On a device's raw INT event it drives that driver's handler;      */
/* otherwise it translates the decoded event onto the matching stream.         */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)

/** Publish one ::imu_activity_msg_t per set started/ended bit in @p bits. */
static void device_manager_publish_activity(time_t timestamp, uint8_t sensor_id, uint16_t bits) {
	static const struct {
		uint16_t started;
		uint16_t ended;
		enum imu_activity_type type;
	} map[] = {
		{BHY2_STILL_ACTIVITY_STARTED, BHY2_STILL_ACTIVITY_ENDED, imu_activity_Still},
		{BHY2_WALKING_ACTIVITY_STARTED, BHY2_WALKING_ACTIVITY_ENDED, imu_activity_Walking},
		{BHY2_RUNNING_ACTIVITY_STARTED, BHY2_RUNNING_ACTIVITY_ENDED, imu_activity_Running},
		{BHY2_ON_BICYCLE_ACTIVITY_STARTED, BHY2_ON_BICYCLE_ACTIVITY_ENDED, imu_activity_OnBicycle},
		{BHY2_IN_VEHICLE_ACTIVITY_STARTED, BHY2_IN_VEHICLE_ACTIVITY_ENDED, imu_activity_InVehicle},
		{BHY2_TILTING_ACTIVITY_STARTED, BHY2_TILTING_ACTIVITY_ENDED, imu_activity_Tilting},
	};

	for (size_t i = 0; i < ARRAY_SIZE(map); ++i) {
		if (bits & map[i].started) {
			struct imu_activity_msg_t msg = {
				.timestamp = timestamp,
				.sensor_id = sensor_id,
				.activity = map[i].type,
				.transition = imu_activity_transition_Started,
			};
			device_manager_publish(&chan_imu_activity, &msg);
		}
		if (bits & map[i].ended) {
			struct imu_activity_msg_t msg = {
				.timestamp = timestamp,
				.sensor_id = sensor_id,
				.activity = map[i].type,
				.transition = imu_activity_transition_Ended,
			};
			device_manager_publish(&chan_imu_activity, &msg);
		}
	}
}

static void device_manager_translate_imu(const struct device_driver_event_t* ev) {
	switch ((enum bhi360_event_type) ev->event_id) {
	case bhi360_event_Irq:
		(void) bhi360_irq_handler();
		break;
	case bhi360_event_QuaternionBatch: {
		struct bhi360_quat_data_t buf[DEVICE_MANAGER_IMU_BATCH_MAX];
		int n = bhi360_copy_quaternion(buf, ARRAY_SIZE(buf));

		for (int i = 0; i < n; ++i) {
			struct imu_quaternion_msg_t msg = {
				.timestamp = buf[i].timestamp,
				.x = buf[i].x,
				.y = buf[i].y,
				.z = buf[i].z,
				.w = buf[i].w,
				.accuracy = buf[i].accuracy,
			};
			device_manager_publish(&chan_imu_quaternion, &msg);
		}
		break;
	}
	case bhi360_event_LinearAccelerationBatch: {
		struct bhi360_lacc_data_t buf[DEVICE_MANAGER_IMU_BATCH_MAX];
		int n = bhi360_copy_linear_acceleration(buf, ARRAY_SIZE(buf));

		for (int i = 0; i < n; ++i) {
			struct imu_accel_msg_t msg = {
				.timestamp = buf[i].timestamp,
				.x = buf[i].x,
				.y = buf[i].y,
				.z = buf[i].z,
			};
			device_manager_publish(&chan_imu_accel, &msg);
		}
		break;
	}
	case bhi360_event_GyroBatch: {
		struct bhi360_gyro_data_t buf[DEVICE_MANAGER_IMU_BATCH_MAX];
		int n = bhi360_copy_gyro(buf, ARRAY_SIZE(buf));

		for (int i = 0; i < n; ++i) {
			struct imu_gyro_msg_t msg = {
				.timestamp = buf[i].timestamp,
				.x = buf[i].x,
				.y = buf[i].y,
				.z = buf[i].z,
			};
			device_manager_publish(&chan_imu_gyro, &msg);
		}
		break;
	}
	case bhi360_event_Pedometer: {
		const struct bhi360_pedometer_data_t* p =
			(const struct bhi360_pedometer_data_t*) ev->p_param;

		if (p != NULL) {
			struct imu_pedometer_msg_t msg = {
				.timestamp = p->timestamp,
				.sensor_id = p->sensor_id,
				.step_count = p->step_count,
				.step_detected = p->step_detected,
			};
			device_manager_publish(&chan_imu_pedometer, &msg);
		}
		break;
	}
	case bhi360_event_Gesture: {
		const struct bhi360_gesture_data_t* g = (const struct bhi360_gesture_data_t*) ev->p_param;

		if (g != NULL) {
			struct imu_gesture_msg_t msg = {
				.timestamp = g->timestamp,
				.sensor_id = g->sensor_id,
				.gesture = (enum imu_gesture_type) g->value,
			};
			device_manager_publish(&chan_imu_gesture, &msg);
		}
		break;
	}
	case bhi360_event_Activity: {
		const struct bhi360_activity_data_t* a = (const struct bhi360_activity_data_t*) ev->p_param;

		if (a != NULL) {
			device_manager_publish_activity(a->timestamp, a->sensor_id, a->activity);
		}
		break;
	}
	default:
		break;
	}
}

#endif /* CONFIG_SENSEWEAR_BHI360_DRIVER */

#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)

static void device_manager_translate_charger(const struct device_driver_event_t* ev) {
	struct bq25180_charger_state_t state;

	/* Read the full state once per INT; the individual decoded condition events
	 * are redundant with the flags carried in the published message. */
	if ((enum bq25180_event_type) ev->event_id != bq25180_event_Irq) {
		return;
	}
	if (bq25180_update_state(&state) != 0) {
		return;
	}

	struct charger_msg_t msg = {
		.timestamp = state.last_update_time,
		.last_irq_time = state.last_irq_time,
		.flags = state.state.value,
		.power_good = state.state.bits.bPowerGood,
		.charging = state.state.bits.bCharging,
		.charged = state.state.bits.bCharged,
		.fault = state.state.bits.bSafetyTimerFault || state.state.bits.bThermalSystemFault ||
				 state.state.bits.bBatteryUVLOFault || state.state.bits.bBatteryOCPFault,
	};
	device_manager_publish(&chan_charger, &msg);
}

#endif /* CONFIG_SENSEWEAR_BQ25180_DRIVER */

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)

static void device_manager_translate_gauge(const struct device_driver_event_t* ev) {
	const struct bq27427_battery_state_t* s;

	if ((enum bq27427_event_type) ev->event_id != bq27427_event_StateUpdated) {
		return;
	}

	s = (const struct bq27427_battery_state_t*) ev->p_param;
	if (s == NULL) {
		return;
	}

	struct battery_msg_t msg = {
		.timestamp = s->last_update_time,
		.temperature_ddeg_c = s->temperature,
		.voltage_mv = s->voltage,
		.average_current_ma = s->average_current,
		.average_power_mw = s->average_power,
		.state_of_charge_dpct = s->state_of_charge,
		.nominal_available_capacity_mah = s->nominal_available_capacity,
		.full_capacity_mah = s->full_battery_capacity,
		.remaining_capacity_mah = s->remaining_capacity,
		.learning_in_progress = s->learning_in_progress,
	};
	device_manager_publish(&chan_battery, &msg);
}

#endif /* CONFIG_SENSEWEAR_BQ27427_DRIVER */

#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)

static void device_manager_translate_regulator(const struct device_driver_event_t* ev) {
	struct regulator_msg_t msg = {
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
		/* The regulator event carries no timestamp; stamp it here. */
		.timestamp = rtc_get_timestamp_us(),
#else
		.timestamp = 0,
#endif
		.vout_uv = ev->v_param, /* All three event types carry VOUT in microvolts. */
	};

	switch ((enum tpsm83102_event_type) ev->event_id) {
	case tpsm83102_event_Enabled:
	case tpsm83102_event_VOUT_UPDATED:
		msg.enabled = true;
		break;
	case tpsm83102_event_Disabled:
		msg.enabled = false;
		break;
	default:
		return;
	}

	device_manager_publish(&chan_regulator, &msg);
}

#endif /* CONFIG_SENSEWEAR_TPSM83102_DRIVER */

#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)

static void device_manager_translate_ppg(const struct device_driver_event_t* ev) {
	enum max30101_event_type event = (enum max30101_event_type) ev->event_id;
	const struct max30101_ppg_sample_t* samples;
	uint32_t count;

	if (event == max30101_Irq) {
		(void) max30101_irq_handler();
		return;
	}
	if (event != max30101_event_FifoDataReady) {
		return;
	}

	count = ev->v_param;
	samples = (const struct max30101_ppg_sample_t*) ev->p_param;
	if (samples == NULL) {
		return;
	}

	for (uint32_t i = 0; i < count; ++i) {
		struct ppg_msg_t msg = {
			.timestamp = (time_t) samples[i].timestamp,
			.ir = samples[i].ir,
			.red = samples[i].red,
			.green = samples[i].green,
		};
		device_manager_publish(&chan_ppg, &msg);
	}
}

#endif /* CONFIG_SHIELD_SENSEWEAR_PPG */

#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)

static void device_manager_translate_temperature(const struct device_driver_event_t* ev) {
	enum max30208_event_type event = (enum max30208_event_type) ev->event_id;
	const struct temperature_sample_t* samples;
	uint32_t count;

	if (event == max30208_TimerIrq) {
		/* Drain the conversion(s); this posts max30208_event_SampleReady, which
		 * is handled below on the next iteration. */
		(void) max30208_get_samples(NULL, 0);
		return;
	}
	if (event != max30208_event_SampleReady) {
		return;
	}

	count = ev->v_param;
	samples = (const struct temperature_sample_t*) ev->p_param;
	if (samples == NULL) {
		return;
	}

	for (uint32_t i = 0; i < count; ++i) {
		struct temperature_msg_t msg = {
			.timestamp = samples[i].timestamp,
			.temperature_mdeg_c = samples[i].temperature_mdeg_c,
		};
		device_manager_publish(&chan_temperature, &msg);
	}
}

#endif /* CONFIG_SHIELD_SENSEWEAR_TEMPERATURE */

#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)

/** Map a decoded MTCH6102 event to a contract touch-gesture type. */
static enum touch_gesture_type device_manager_map_touch_gesture(enum mtch6102_event_type event) {
	switch (event) {
	case mtch6102_event_SingleClick:
		return touch_gesture_SingleClick;
	case mtch6102_event_ClickAndHold:
		return touch_gesture_ClickAndHold;
	case mtch6102_event_DoubleClick:
		return touch_gesture_DoubleClick;
	case mtch6102_event_DownSwipe:
		return touch_gesture_DownSwipe;
	case mtch6102_event_DownSwipeAndHold:
		return touch_gesture_DownSwipeAndHold;
	case mtch6102_event_RightSwipe:
		return touch_gesture_RightSwipe;
	case mtch6102_event_RightSwipeAndHold:
		return touch_gesture_RightSwipeAndHold;
	case mtch6102_event_UpSwipe:
		return touch_gesture_UpSwipe;
	case mtch6102_event_UpSwipeAndHold:
		return touch_gesture_UpSwipeAndHold;
	case mtch6102_event_LeftSwipe:
		return touch_gesture_LeftSwipe;
	case mtch6102_event_LeftSwipeAndHold:
		return touch_gesture_LeftSwipeAndHold;
	default:
		return touch_gesture_None;
	}
}

static void device_manager_translate_touch(const struct device_driver_event_t* ev) {
	enum mtch6102_event_type event = (enum mtch6102_event_type) ev->event_id;
	const struct touch_sensor_sample_t* sample = (const struct touch_sensor_sample_t*) ev->p_param;

	if (event == mtch6102_Irq) {
		(void) mtch6102_irq_handler();
		return;
	}

	if (sample == NULL) {
		return;
	}

	switch (event) {
	case mtch6102_event_TouchDetected:
	case mtch6102_event_TouchReleased: {
		struct touch_msg_t msg = {
			.timestamp = sample->timestamp,
			.touched = sample->position.touched,
			.x = sample->position.x,
			.y = sample->position.y,
			.touch_state = sample->position.touch_state,
		};
		device_manager_publish(&chan_touch, &msg);
		break;
	}
	default: {
		enum touch_gesture_type gesture = device_manager_map_touch_gesture(event);

		if (gesture == touch_gesture_None) {
			break;
		}

		struct touch_gesture_msg_t msg = {
			.timestamp = sample->timestamp,
			.gesture = gesture,
			.gesture_state = sample->gesture_state,
		};
		device_manager_publish(&chan_touch_gesture, &msg);
		break;
	}
	}
}

#endif /* CONFIG_SHIELD_SENSEWEAR_TOUCH */

#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)

/** Drive minute-cadence periodic updates from the RTC minute alarm. */
static void device_manager_handle_rtc(const struct device_driver_event_t* ev) {
	if ((enum rtc_event_type) ev->event_id != rtc_event_MinuteAlarm) {
		return;
	}

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	if (device_manager_config_state.gauge.update_period_min > 0 &&
		++device_manager_gauge_minutes >= device_manager_config_state.gauge.update_period_min) {
		device_manager_gauge_minutes = 0;
		(void) bq27427_update_state(NULL);
	}
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	/* NOTE: max30208_get_samples() triggers a one-shot conversion but requires
	 * the driver to already be in the sampling state (max30208_start()); wiring
	 * that enable path is left to the caller. */
	if (device_manager_config_state.temperature.update_period_min > 0 &&
		++device_manager_temp_minutes >=
			device_manager_config_state.temperature.update_period_min) {
		device_manager_temp_minutes = 0;
		(void) max30208_get_samples(NULL, 0);
	}
#endif
}

#endif /* CONFIG_SENSEWEAR_RTC_DRIVER */

/** Route one device event to its translator by producing device. */
static void device_manager_dispatch(const struct device_driver_event_t* ev) {
	switch (ev->device_id) {
#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	case BHI360_DEVICE_DTS_ID:
		device_manager_translate_imu(ev);
		break;
#endif
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
	case BQ25180_DEVICE_DTS_ID:
		device_manager_translate_charger(ev);
		break;
#endif
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	case BQ27427_DEVICE_DTS_ID:
		device_manager_translate_gauge(ev);
		break;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
	case MAX30101_DEVICE_DTS_ID:
		device_manager_translate_ppg(ev);
		break;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	case MAX30208_DEVICE_DTS_ID:
		device_manager_translate_temperature(ev);
		break;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
	case MTCH6102_DEVICE_DTS_ID:
		device_manager_translate_touch(ev);
		break;
#endif
#if defined(CONFIG_SENSEWEAR_TPSM83102_DRIVER)
	case TPSM83102_DEVICE_DTS_ID:
		device_manager_translate_regulator(ev);
		break;
#endif
#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	case RTC0_DEVICE_DTS_ID:
		device_manager_handle_rtc(ev);
		break;
#endif
	default:
		break;
	}
}

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* ------------------------------------------------------------------------- */

K_THREAD_STACK_DEFINE(device_manager_stack, DEVICE_MANAGER_STACK_SIZE);
static struct k_thread device_manager_thread;
static bool device_manager_started;

static void device_manager_thread_fn(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t ev;

	while (device_driver_event_wait(K_FOREVER, &ev)) {
		device_manager_dispatch(&ev);
	}
}

int device_manager_init(void) {
	/* Initialize every available device. A device that fails to come up is
	 * logged and skipped; the manager still serves the rest. */
	for (enum device_manager_device_type device = device_manager_device_Invalid + 1;
		 device < device_manager_device_Count;
		 device++) {
		if (!device_manager_device_supported(device)) {
			continue;
		}

		int ret = device_manager_init_device(device);

		if (ret != 0) {
			LOG_WRN("device manager: device %d init failed (%d); ignored", device, ret);
		}
	}

	return 0;
}

int device_manager_start(void) {
	if (device_manager_started) {
		return 0;
	}

	/* Refuse to start while any stream that already has observers is backed by a
	 * device that is not ready. */
	for (enum device_manager_stream_type stream = device_manager_stream_Invalid + 1;
		 stream < device_manager_stream_Count;
		 stream++) {
		if (device_manager_stream_observers[stream] > 0 && !device_manager_stream_ready(stream)) {
			LOG_ERR("device manager: stream %d has observers but its device is not ready", stream);
			return -ENODEV;
		}
	}

	if (device_driver_event_get_queue() == NULL) {
		LOG_ERR("device manager: event queue not initialized");
		return -ENODEV;
	}

	k_tid_t tid = k_thread_create(&device_manager_thread,
								  device_manager_stack,
								  K_THREAD_STACK_SIZEOF(device_manager_stack),
								  device_manager_thread_fn,
								  NULL,
								  NULL,
								  NULL,
								  DEVICE_MANAGER_THREAD_PRIO,
								  0,
								  K_NO_WAIT);
	if (tid == NULL) {
		LOG_ERR("device manager: failed to start consumer thread");
		return -EAGAIN;
	}

	k_thread_name_set(tid, "device_mgr");
	device_manager_started = true;
	return 0;
}
