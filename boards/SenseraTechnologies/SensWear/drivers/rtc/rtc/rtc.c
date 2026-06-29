/**
 * rtc.c
 *
 * @file rtc.c
 * @brief SenseWear software RTC implementation.
 * @details The driver is a board-level singleton facade over the kernel wall
 *          clock (@ref SYS_CLOCK_REALTIME). Setting the time programs the wall
 *          clock through sys_clock_settime() so POSIX time() agrees; reading it
 *          returns sys_clock_gettime(). The current time is persisted to the
 *          Settings/NVS backend and restored at init. Alarms are backed by
 *          `k_timer` objects so the kernel schedules a single GRTC compare per
 *          boundary instead of polling.
 *
 * Timer expiry callbacks run in interrupt context and publish events through
 * the device-event manager using the ISR-safe post helper. Persistence touches
 * flash and therefore runs from the system workqueue, never from a timer
 * callback.
 */

#include "rtc.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/timeutil.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(rtc, CONFIG_LOG_DEFAULT_LEVEL);

/** Settings subtree and key used to persist the wall clock. */
#define RTC_SETTINGS_SUBTREE "rtc"
#define RTC_SETTINGS_KEY	 RTC_SETTINGS_SUBTREE "/time"

/** Seconds in each recurring-alarm period. */
#define RTC_MINUTE_SEC (60u)
#define RTC_HOUR_SEC   (3600u)
#define RTC_DAY_SEC	   (86400u)

/**
 * @brief Internal singleton driver context.
 * @details Holds the lifecycle flag, the alarm timers and their active state,
 *          the deferred persistence work, and the value recovered from Settings
 *          during rtc_init(). Private to this implementation unit.
 */
static struct rtc_t {
	/** rtc_init() completed successfully. */
	bool initialized;
	/** Recurring minute alarm is armed. */
	bool minute_on;
	/** Recurring hour alarm is armed. */
	bool hour_on;
	/** Recurring day alarm is armed. */
	bool day_on;
	/** One-shot specific-time alarm is armed. */
	bool oneshot_on;
	/** Unix target of the armed one-shot alarm. */
	time_t oneshot_target;
	/** Recurring minute-boundary alarm timer. */
	struct k_timer minute_timer;
	/** Recurring hour-boundary alarm timer. */
	struct k_timer hour_timer;
	/** Recurring day-boundary alarm timer. */
	struct k_timer day_timer;
	/** One-shot specific-time alarm timer. */
	struct k_timer oneshot_timer;
	/** Periodic persistence work item (system workqueue). */
	struct k_work_delayable persist_work;
	/** Unix time recovered from Settings during init. */
	int64_t restored_unix;
	/** Whether @ref restored_unix held a plausible value. */
	bool restored_valid;
} rtc;

static const char* const rtc_event_names[rtc_event_Count] = {
	[rtc_event_MinuteAlarm] = "MinuteAlarm",
	[rtc_event_HourAlarm] = "HourAlarm",
	[rtc_event_DayAlarm] = "DayAlarm",
	[rtc_event_TimeAlarm] = "TimeAlarm",
};

const char* rtc_event_name(uint32_t event_id) {
	if (event_id >= (uint32_t) rtc_event_Count || rtc_event_names[event_id] == NULL) {
		return "Unknown";
	}

	return rtc_event_names[event_id];
}

/* --------------------------------------------------------------------------
 * Wall-clock access
 * -------------------------------------------------------------------------- */

/** Program the kernel wall clock (and therefore POSIX time()) to @p unix_seconds. */
static bool rtc_seed_clock(time_t unix_seconds) {
	struct timespec ts = {
		.tv_sec = unix_seconds,
		.tv_nsec = 0,
	};

	int ret = sys_clock_settime(SYS_CLOCK_REALTIME, &ts);

	if (ret != 0) {
		LOG_ERR("sys_clock_settime failed (%d)", ret);
		return false;
	}

	return true;
}

time_t rtc_get_unix(void) {
	struct timespec ts;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) != 0) {
		return (time_t) -1;
	}

	return (time_t) ts.tv_sec;
}

bool rtc_get_time(struct tm* tm_utc) {
	if (tm_utc == NULL) {
		return false;
	}

	time_t now = rtc_get_unix();

	if (now == (time_t) -1) {
		return false;
	}

	return gmtime_r(&now, tm_utc) != NULL;
}

/* --------------------------------------------------------------------------
 * Persistence
 * -------------------------------------------------------------------------- */

/** Save the current wall clock to the Settings/NVS backend. */
static void rtc_persist(void) {
	time_t now = rtc_get_unix();

	if (now == (time_t) -1) {
		return;
	}

	int64_t value = (int64_t) now;
	int ret = settings_save_one(RTC_SETTINGS_KEY, &value, sizeof(value));

	if (ret != 0) {
		LOG_WRN("Failed to persist wall clock (%d)", ret);
	}
}

/** System-workqueue handler that persists the clock and reschedules itself. */
static void rtc_persist_work_handler(struct k_work* work) {
	ARG_UNUSED(work);

	rtc_persist();
	(void) k_work_schedule(&rtc.persist_work, K_SECONDS(RTC_PERSIST_INTERVAL_SEC));
}

/** Settings handler: capture the persisted Unix time during settings_load(). */
static int rtc_settings_set(const char* name,
							size_t len,
							settings_read_cb read_cb,
							void* cb_arg) {
	const char* next;

	if (!settings_name_steq(name, "time", &next) || next != NULL) {
		return -ENOENT;
	}

	int64_t value;

	if (len != sizeof(value)) {
		return -EINVAL;
	}

	ssize_t read = read_cb(cb_arg, &value, sizeof(value));

	if (read < 0) {
		return (int) read;
	}

	rtc.restored_unix = value;
	rtc.restored_valid = (value >= (int64_t) RTC_SANITY_MIN_UNIX);

	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(rtc_settings,
							   RTC_SETTINGS_SUBTREE,
							   NULL,
							   rtc_settings_set,
							   NULL,
							   NULL);

/* --------------------------------------------------------------------------
 * Alarms
 * -------------------------------------------------------------------------- */

/** Post an RTC event from interrupt context, tagging it with the current time. */
static void rtc_post_event_isr(enum rtc_event_type event) {
	time_t now = rtc_get_unix();
	uint32_t v_param = (now == (time_t) -1) ? 0u : (uint32_t) now;

	(void) device_driver_event_post_isr(RTC0_DEVICE_DTS_ID,
										(uint32_t) event,
										v_param,
										(uintptr_t) NULL);
}

static void rtc_minute_expiry(struct k_timer* timer) {
	ARG_UNUSED(timer);
	rtc_post_event_isr(rtc_event_MinuteAlarm);
}

static void rtc_hour_expiry(struct k_timer* timer) {
	ARG_UNUSED(timer);
	rtc_post_event_isr(rtc_event_HourAlarm);
}

static void rtc_day_expiry(struct k_timer* timer) {
	ARG_UNUSED(timer);
	rtc_post_event_isr(rtc_event_DayAlarm);
}

static void rtc_oneshot_expiry(struct k_timer* timer) {
	ARG_UNUSED(timer);
	rtc.oneshot_on = false;
	rtc_post_event_isr(rtc_event_TimeAlarm);
}

/** Seconds from @p now until the next multiple of @p period (always >= 1). */
static uint32_t rtc_secs_to_next_boundary(time_t now, uint32_t period) {
	uint32_t into = (uint32_t) (((int64_t) now) % (int64_t) period);

	return period - into; /* in [1, period] */
}

/** (Re)arm a recurring alarm so its first fire lands on the next boundary. */
static void rtc_arm_periodic(struct k_timer* timer, uint32_t period_sec) {
	time_t now = rtc_get_unix();

	if (now == (time_t) -1) {
		now = 0;
	}

	uint32_t initial = rtc_secs_to_next_boundary(now, period_sec);

	k_timer_start(timer, K_SECONDS(initial), K_SECONDS(period_sec));
}

/** Arm the one-shot alarm for an absolute Unix target in the future. */
static bool rtc_arm_oneshot(time_t target) {
	time_t now = rtc_get_unix();

	if (now == (time_t) -1) {
		return false;
	}

	int64_t delay = (int64_t) target - (int64_t) now;

	if (delay <= 0) {
		LOG_WRN("Time alarm target %lld is not in the future", (long long) target);
		return false;
	}

	rtc.oneshot_target = target;
	rtc.oneshot_on = true;
	k_timer_start(&rtc.oneshot_timer, K_SECONDS(delay), K_NO_WAIT);

	return true;
}

/** Realign every active alarm after the wall clock has been changed. */
static void rtc_realign_alarms(void) {
	if (rtc.minute_on) {
		rtc_arm_periodic(&rtc.minute_timer, RTC_MINUTE_SEC);
	}
	if (rtc.hour_on) {
		rtc_arm_periodic(&rtc.hour_timer, RTC_HOUR_SEC);
	}
	if (rtc.day_on) {
		rtc_arm_periodic(&rtc.day_timer, RTC_DAY_SEC);
	}
	if (rtc.oneshot_on && !rtc_arm_oneshot(rtc.oneshot_target)) {
		/* Target fell into the past after the adjustment. */
		k_timer_stop(&rtc.oneshot_timer);
		rtc.oneshot_on = false;
	}
}

bool rtc_enable_minute_alarm(bool enable) {
	if (!rtc.initialized) {
		return false;
	}

	rtc.minute_on = enable;

	if (enable) {
		rtc_arm_periodic(&rtc.minute_timer, RTC_MINUTE_SEC);
	} else {
		k_timer_stop(&rtc.minute_timer);
	}

	return true;
}

bool rtc_enable_hour_alarm(bool enable) {
	if (!rtc.initialized) {
		return false;
	}

	rtc.hour_on = enable;

	if (enable) {
		rtc_arm_periodic(&rtc.hour_timer, RTC_HOUR_SEC);
	} else {
		k_timer_stop(&rtc.hour_timer);
	}

	return true;
}

bool rtc_enable_day_alarm(bool enable) {
	if (!rtc.initialized) {
		return false;
	}

	rtc.day_on = enable;

	if (enable) {
		rtc_arm_periodic(&rtc.day_timer, RTC_DAY_SEC);
	} else {
		k_timer_stop(&rtc.day_timer);
	}

	return true;
}

bool rtc_set_time_alarm_unix(time_t unix_seconds) {
	if (!rtc.initialized) {
		return false;
	}

	return rtc_arm_oneshot(unix_seconds);
}

bool rtc_set_time_alarm(const struct tm* tm_utc) {
	if (!rtc.initialized || tm_utc == NULL) {
		return false;
	}

	struct tm copy = *tm_utc;
	time_t target = timeutil_timegm(&copy);

	if (target == (time_t) -1) {
		LOG_ERR("Invalid broken-down time for time alarm");
		return false;
	}

	return rtc_arm_oneshot(target);
}

bool rtc_cancel_time_alarm(void) {
	if (!rtc.initialized) {
		return false;
	}

	k_timer_stop(&rtc.oneshot_timer);
	rtc.oneshot_on = false;

	return true;
}

/* --------------------------------------------------------------------------
 * Time programming
 * -------------------------------------------------------------------------- */

bool rtc_set_unix(time_t unix_seconds) {
	if (unix_seconds < (time_t) RTC_SANITY_MIN_UNIX) {
		LOG_ERR("Refusing implausible time %lld", (long long) unix_seconds);
		return false;
	}

	if (!rtc_seed_clock(unix_seconds)) {
		return false;
	}

	rtc_persist();
	rtc_realign_alarms();

	return true;
}

bool rtc_set_time(const struct tm* tm_utc) {
	if (tm_utc == NULL) {
		return false;
	}

	struct tm copy = *tm_utc;
	time_t unix_seconds = timeutil_timegm(&copy);

	if (unix_seconds == (time_t) -1) {
		LOG_ERR("Invalid broken-down time");
		return false;
	}

	return rtc_set_unix(unix_seconds);
}

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

bool rtc_is_ready(void) {
	return rtc.initialized;
}

bool rtc_init(void) {
	if (rtc.initialized) {
		return true;
	}

	k_timer_init(&rtc.minute_timer, rtc_minute_expiry, NULL);
	k_timer_init(&rtc.hour_timer, rtc_hour_expiry, NULL);
	k_timer_init(&rtc.day_timer, rtc_day_expiry, NULL);
	k_timer_init(&rtc.oneshot_timer, rtc_oneshot_expiry, NULL);
	k_work_init_delayable(&rtc.persist_work, rtc_persist_work_handler);

	int ret = settings_subsys_init();

	if (ret != 0) {
		LOG_WRN("settings_subsys_init returned %d", ret);
	}

	ret = settings_load_subtree(RTC_SETTINGS_SUBTREE);

	if (ret != 0) {
		LOG_WRN("settings_load_subtree(%s) returned %d", RTC_SETTINGS_SUBTREE, ret);
	}

	time_t seed = rtc.restored_valid ? (time_t) rtc.restored_unix : (time_t) RTC_DEFAULT_UNIX;

	if (!rtc_seed_clock(seed)) {
		return false;
	}

	rtc.initialized = true;

	LOG_INF("RTC ready, wall clock seeded to %lld (%s)",
			(long long) seed,
			rtc.restored_valid ? "restored" : "default");

	(void) k_work_schedule(&rtc.persist_work, K_SECONDS(RTC_PERSIST_INTERVAL_SEC));

	return true;
}

#if defined(CONFIG_SENSEWEAR_RTC_AUTO_INIT)
/** System-init shim so the wall clock is correct before application code runs. */
static int rtc_sys_init(void) {
	(void) rtc_init();
	return 0;
}

SYS_INIT(rtc_sys_init, APPLICATION, CONFIG_SENSEWEAR_RTC_INIT_PRIORITY);
#endif /* CONFIG_SENSEWEAR_RTC_AUTO_INIT */
