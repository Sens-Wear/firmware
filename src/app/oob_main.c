/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Out-of-box main: application bring-up, owned by an auto-started monitor
 * thread (there is no main(); Zephyr's weak default is used).
 *
 * The thread brings up the device manager the same way the bring-up test
 * (tests/device_manager/main_test_device_manager.c) does — init, configure
 * every present device, publish configuration, start the publisher — and
 * starts the enabled BLE services on top of it:
 *
 *   - imu / power / body_temperature: their zbus listeners are registered
 *     here once the producing device reported ready.
 *   - ppg / touch: their services run retry threads that register themselves
 *     as soon as the stream is ready; nothing to do here beyond configuring
 *     the device.
 *   - led / haptic / time: pure GATT services (writes are forwarded to the
 *     device manager); they have no start step.
 *
 * While any enabled device remains unconfigured (absent at boot, transient
 * bus error, daughter board hot-plugged late, ...) the monitor retries its
 * bring-up once a minute and logs the per-stream state; once everything is
 * configured the thread exits.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "device_manager.h"

#include "bluetooth/services/imu/imu_lbs.h"
#include "bluetooth/services/power/power_lbs.h"

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
#include "bq27427.h"
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
#include "max30101.h"
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
#include "bluetooth/services/body_temperature/body_temperature_lbs.h"
#endif

LOG_MODULE_REGISTER(SENSE_WEAR_OOB_MAIN_LOGGER);

/** Retry cadence for devices that are still unconfigured, in ms. */
#define OOB_MAIN_MONITOR_PERIOD_MS 60000U

/** Slow FIFO-drain period used while the IMU physical sensors are idle, in ms.
 * The physical streams start disabled; the IMU config service enables them
 * (with a client-chosen drain period) over BLE on demand. */
#define OOB_MAIN_IMU_IDLE_DRAIN_MS 60000U

#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
/** Wall-clock seed so timestamps and the minute alarm work before a client
 * syncs real time through the Current Time service: 2026-01-01T00:00:00Z. */
#define OOB_MAIN_RTC_EPOCH_DEFAULT ((time_t) 1767225600)
#endif

#define OOB_MAIN_THREAD_STACK_SIZE 4096
#define OOB_MAIN_THREAD_PRIORITY 7

/* Desired device-manager configuration; per-device cadences are switched on
 * as their device configures, and the whole config is (re)published after
 * every bring-up pass that changed it. */
static struct device_manager_config_t oob_cfg;

/* Per-device configured state; the board-level flags are read unconditionally
 * (service start, state log), the shield flags only exist with their shield. */
static bool imu_configured;
static bool charger_configured;
static bool battery_configured;
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
static bool ppg_configured;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
static bool temperature_configured;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
static bool touch_configured;
#endif

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
/* One-shot follow-ups after the manager is running. */
static bool gauge_kicked;
#endif

static bool configure_ready_device(enum device_manager_device_type device,
								   enum device_manager_stream_type stream,
								   const char* name) {
	/* A device is only reported "ready" once it has been configured, so
	 * configuration must come first — gating it on readiness would skip every
	 * device that merely initialized. A device that did not initialize fails to
	 * configure and is retried on the next pass. */
	int ret = device_manager_config_device(device, NULL);

	if (ret != 0) {
		LOG_WRN("device_manager_config_device(%s) failed: %d", name, ret);
		return false;
	}

	if (!device_manager_stream_ready(stream)) {
		LOG_WRN("%s configured but stream not ready; will retry", name);
		return false;
	}

	return true;
}

/* Try to configure every enabled device that is not configured yet. Returns
 * true when a device newly configured (the published config must then be
 * refreshed). */
static bool oob_bring_up_pass(void) {
	bool changed = false;

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	if (!imu_configured && configure_ready_device(device_manager_device_Bhi360,
												  device_manager_stream_ImuQuaternion,
												  "Bhi360")) {
		imu_configured = true;
		changed = true;
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
	if (!charger_configured && configure_ready_device(device_manager_device_Bq25180,
													  device_manager_stream_Charger,
													  "Bq25180")) {
		charger_configured = true;
		changed = true;
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	if (!battery_configured && configure_ready_device(device_manager_device_Bq27427,
													  device_manager_stream_Battery,
													  "Bq27427")) {
		battery_configured = true;
		/* Refresh the fuel gauge every minute now that it initialized. */
		oob_cfg.gauge.update_period_min = 1;
		changed = true;
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
	if (!ppg_configured && configure_ready_device(device_manager_device_Max30101,
												  device_manager_stream_Ppg,
												  "Max30101")) {
		ppg_configured = true;
		changed = true;

		/* Configuration alone does not start acquisition; enable multi-LED
		 * wrist-HR sampling so the PPG stream actually produces ir/red/green
		 * samples. A_FULL batching (per_sample_irq = false) drains once per
		 * FIFO almost-full. The PPG service subscribes itself. */
		int ppg_ret = max30101_enable_wrist_hr_sampling(false);

		if (ppg_ret != 0) {
			LOG_WRN("max30101_enable_wrist_hr_sampling() failed: %d", ppg_ret);
		}
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	if (!temperature_configured && configure_ready_device(device_manager_device_Max30208,
														  device_manager_stream_Temperature,
														  "Max30208")) {
		temperature_configured = true;
		oob_cfg.temperature.update_period_min = 1;
		changed = true;
	}
#endif

#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
	/* The touch service subscribes itself once the streams are ready. */
	if (!touch_configured && configure_ready_device(device_manager_device_Mtch6102,
													device_manager_stream_Touch,
													"Mtch6102")) {
		touch_configured = true;
		changed = true;
	}
#endif

	return changed;
}

/* Publish the desired configuration and its follow-ups. */
static void oob_apply_config(void) {
	int ret = device_manager_config(&oob_cfg);

	if (ret != 0) {
		LOG_WRN("device_manager_config() failed: %d", ret);
	}

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	/* With the physical sensors idle, device_manager_config() leaves the shared
	 * FIFO timer stopped. The event sensors are interrupt-driven, but drain the
	 * FIFO on a slow cadence as well so anything that does not raise its own
	 * interrupt still surfaces. */
	if (imu_configured) {
		int drain_ret = device_manager_set_imu_drain_period(OOB_MAIN_IMU_IDLE_DRAIN_MS);

		if (drain_ret != 0) {
			LOG_WRN("device_manager_set_imu_drain_period() failed: %d", drain_ret);
		}
	}
#endif
}

/* Start the BLE services whose producing device is configured: register
 * their zbus listeners (idempotent) and run one-shot follow-ups. Only valid
 * once the device manager is started. */
static void oob_start_services(void) {
	if (imu_configured) {
		int ret = imu_lbs_register_streams();

		if (ret != 0) {
			LOG_WRN("imu_lbs_register_streams() failed: %d", ret);
		}
	}

	if (battery_configured || charger_configured) {
		int ret = power_lbs_register_streams();

		if (ret != 0) {
			LOG_WRN("power_lbs_register_streams() failed: %d", ret);
		}
	}

#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	if (temperature_configured) {
		int ret = body_temperature_lbs_register_stream();

		if (ret != 0) {
			LOG_WRN("body_temperature_lbs_register_stream() failed: %d", ret);
		}
	}
#endif

#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	/* One immediate refresh for deterministic battery output; the manager's
	 * RTC-minute cadence takes over from here. */
	if (battery_configured && !gauge_kicked) {
		(void) bq27427_update_state(NULL);
		gauge_kicked = true;
	}
#endif
}

static unsigned int oob_unconfigured_count(void) {
	unsigned int count = 0;

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	count += imu_configured ? 0U : 1U;
#endif
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
	count += charger_configured ? 0U : 1U;
#endif
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	count += battery_configured ? 0U : 1U;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
	count += ppg_configured ? 0U : 1U;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	count += temperature_configured ? 0U : 1U;
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
	count += touch_configured ? 0U : 1U;
#endif

	return count;
}

static void oob_log_device(const char* name,
						   bool configured,
						   enum device_manager_stream_type stream) {
	LOG_INF("  %-8s %-12s stream %s",
			name,
			configured ? "configured" : "unconfigured",
			device_manager_stream_ready(stream) ? "ready" : "not ready");
}

static void oob_log_state(void) {
	LOG_INF("device state (%u unconfigured):", oob_unconfigured_count());

#if defined(CONFIG_SENSEWEAR_BHI360_DRIVER)
	oob_log_device("Bhi360", imu_configured, device_manager_stream_ImuQuaternion);
#endif
#if defined(CONFIG_SENSEWEAR_BQ25180_DRIVER)
	oob_log_device("Bq25180", charger_configured, device_manager_stream_Charger);
#endif
#if defined(CONFIG_SENSEWEAR_BQ27427_DRIVER)
	oob_log_device("Bq27427", battery_configured, device_manager_stream_Battery);
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_PPG)
	oob_log_device("Max30101", ppg_configured, device_manager_stream_Ppg);
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
	oob_log_device("Max30208", temperature_configured, device_manager_stream_Temperature);
#endif
#if defined(CONFIG_SHIELD_SENSEWEAR_TOUCH)
	oob_log_device("Mtch6102", touch_configured, device_manager_stream_Touch);
#endif
}

static void oob_main_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

#if defined(CONFIG_SENSEWEAR_RTC_DRIVER)
	int rtc_ret = device_manager_set_rtc_time(OOB_MAIN_RTC_EPOCH_DEFAULT);

	if (rtc_ret != 0) {
		LOG_WRN("device_manager_set_rtc_time() failed: %d", rtc_ret);
	}
#endif

	if (device_manager_init() != 0) {
		LOG_ERR("device_manager_init() failed; bring-up aborted");
		return;
	}

	device_manager_get_default_config(&oob_cfg);
	/* The high-rate IMU physical streams stay off until a client enables them
	 * through the IMU config service; the low-rate event sensors (pedometer,
	 * gesture, activity) are interrupt-driven. Gauge and temperature cadences
	 * are enabled by the bring-up pass once their device configured. */
	oob_cfg.imu.phy_streams_enabled = false;
	oob_cfg.gauge.update_period_min = 0;
	oob_cfg.temperature.update_period_min = 0;

	(void) oob_bring_up_pass();
	oob_apply_config();

	if (device_manager_start() != 0) {
		LOG_ERR("device_manager_start() failed; bring-up aborted");
		return;
	}

	oob_start_services();
	oob_log_state();

	/* Retry any device that is still unconfigured once a minute. */
	while (oob_unconfigured_count() > 0U) {
		k_msleep(OOB_MAIN_MONITOR_PERIOD_MS);

		if (oob_bring_up_pass()) {
			oob_apply_config();
			oob_start_services();
		}

		oob_log_state();
	}

	LOG_INF("all enabled devices configured; monitor exiting");
}

K_THREAD_DEFINE(oob_main_thread_id,
		OOB_MAIN_THREAD_STACK_SIZE,
		oob_main_thread,
		NULL,
		NULL,
		NULL,
		OOB_MAIN_THREAD_PRIORITY,
		0,
		0);
