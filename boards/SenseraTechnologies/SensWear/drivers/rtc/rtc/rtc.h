/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file rtc.h
 * @brief SensWear software real-time clock bound to the system wall clock.
 *
 * @defgroup senswear_rtc SensWear software RTC
 * @ingroup io_interfaces
 * @{
 *
 * The nRF54L15 has no battery-backed calendar RTC peripheral; its only
 * always-on time source is the GRTC, which already backs Zephyr's system clock.
 * This driver therefore does not talk to a dedicated RTC chip. Instead it is a
 * thin facade over the kernel wall clock (@ref SYS_CLOCK_REALTIME), which is
 * exactly the time base that POSIX `time()`, `gmtime_r()`, and
 * `clock_gettime(CLOCK_REALTIME, ...)` read:
 *
 * @code{.text}
 * application                       POSIX time(), gmtime_r(), clock_gettime()
 *      |                                       |
 *      | rtc_set_time(), rtc_get_time()        | reads SYS_CLOCK_REALTIME
 *      v                                       v
 * SensWear rtc facade  --- sys_clock_settime(SYS_CLOCK_REALTIME) ---> kernel
 *      |        \                                                       wall
 *      |         \--- NVS (settings) persistence -> restored at boot     clock
 *      v
 * k_timer alarms (minute / hour / day / specific-time)
 * @endcode
 *
 * Because the driver writes the wall clock through sys_clock_settime(), every
 * POSIX time call returns the value last programmed through rtc_set_time(),
 * advanced by the system clock. No periodic polling is performed for
 * timekeeping itself.
 *
 * @section senswear_rtc_persistence Persistence
 *
 * The current Unix time is saved to the Settings/NVS backend on every
 * rtc_set_time() and periodically every ::RTC_PERSIST_INTERVAL_SEC seconds. At
 * rtc_init() the most recent saved value is restored so the wall clock survives
 * a reboot to within the persist interval. Time cannot survive battery removal:
 * with no hardware RTC there is nothing to keep counting while unpowered. When
 * no value has ever been saved, the clock starts at ::RTC_DEFAULT_UNIX.
 *
 * @section senswear_rtc_alarms Alarms
 *
 * Alarms are backed by `k_timer` objects, whose timeouts the kernel turns into
 * GRTC hardware compares. The SoC therefore sleeps (System-ON idle) between
 * boundaries and wakes once per event rather than polling. Three recurring
 * boundary alarms and one one-shot alarm are provided:
 *
 * - rtc_enable_minute_alarm() fires at every wall-clock `:00` second.
 * - rtc_enable_hour_alarm()   fires at every wall-clock `:00:00`.
 * - rtc_enable_day_alarm()    fires at every wall-clock `00:00:00` UTC.
 * - rtc_set_time_alarm()      fires once at a specific UTC time.
 *
 * Each firing publishes a ::rtc_event_type through the SensWear device-event
 * manager from interrupt context; process it from the consumer thread.
 *
 * @note Alarms do not wake the SoC from System-OFF (deepest sleep). That would
 *       require a GRTC extended channel (z_nrf_grtc_timer_ext_chan_alloc()) and
 *       is intentionally out of scope here.
 *
 * @section senswear_rtc_time_repr Time representation
 *
 * Broken-down time uses the standard `struct tm` in UTC. Unix time is exposed
 * in seconds through `rtc_get_unix()`. For data streams that need sub-second
 * epoch tags, `rtc_get_timestamp_ms()` and `rtc_get_timestamp_us()` return Unix
 * timestamps in milliseconds and microseconds.
 *
 * @section senswear_rtc_example Typical usage
 *
 * @code{.c}
 * struct tm now = {
 *     .tm_year = 2026 - 1900, .tm_mon = 5, .tm_mday = 29,
 *     .tm_hour = 12, .tm_min = 0, .tm_sec = 0,
 * };
 *
 * rtc_init();
 * rtc_set_time(&now);          // wall clock + POSIX time() now agree
 * rtc_enable_minute_alarm(true);
 *
 * // From the device-event consumer thread:
 * //   case RTC0_DEVICE_DTS_ID:
 * //       if (ev.event_id == rtc_event_MinuteAlarm) { ... }
 * @endcode
 */

#ifndef SENSWEAR_RTC_H_
#define SENSWEAR_RTC_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default Unix time used when no value has been persisted.
 * @details 2026-01-01 00:00:00 UTC. Applications may override it before the
 *          driver is built.
 */
#ifndef RTC_DEFAULT_UNIX
#define RTC_DEFAULT_UNIX (1767225600)
#endif

/**
 * @brief Interval, in seconds, between periodic persists of the wall clock.
 * @details A smaller value loses less time across an unexpected reboot but
 *          writes the Settings/NVS backend more often. The clock is also
 *          persisted immediately on every rtc_set_time().
 */
#ifndef RTC_PERSIST_INTERVAL_SEC
#define RTC_PERSIST_INTERVAL_SEC (3600)
#endif

/**
 * @brief Oldest Unix time accepted as a plausible wall-clock value.
 * @details 2020-01-01 00:00:00 UTC. A restored or supplied timestamp below this
 *          is treated as uninitialised and rejected. Guards against seeding the
 *          clock from an erased/zeroed Settings record.
 */
#ifndef RTC_SANITY_MIN_UNIX
#define RTC_SANITY_MIN_UNIX (1577836800)
#endif

/**
 * @brief Driver-level RTC event identifiers.
 * @details Published as the event_id through the device-event manager under the
 *          RTC node's generated device id. This is a software namespace, not a
 *          hardware register encoding.
 */
enum rtc_event_type {
	rtc_event_MinuteAlarm = 0, /**< A wall-clock minute boundary was reached. */
	rtc_event_HourAlarm = 1,   /**< A wall-clock hour boundary was reached. */
	rtc_event_DayAlarm = 2,	   /**< A wall-clock day boundary (00:00:00 UTC) was reached. */
	rtc_event_TimeAlarm = 3,   /**< The one-shot specific-time alarm fired. */
	rtc_event_Count,		   /**< Number of valid event identifiers. */
};

/**
 * @brief Initialise the software RTC and restore persisted time.
 *
 * Initialises the Settings subsystem (idempotent), loads the most recently
 * saved Unix time, seeds the wall clock through sys_clock_settime() so POSIX
 * time calls are correct, and starts periodic persistence. When no valid value
 * is stored the clock is seeded with ::RTC_DEFAULT_UNIX. Calling this more than
 * once is harmless.
 *
 * @retval true The driver is ready.
 * @retval false The wall clock could not be seeded.
 */
bool rtc_init(void);

/**
 * @brief Report whether rtc_init() has completed successfully.
 *
 * @retval true The RTC facade is ready.
 * @retval false Initialisation has not succeeded.
 */
bool rtc_is_ready(void);

/**
 * @brief Set the wall-clock time from broken-down UTC time.
 *
 * Updates @ref SYS_CLOCK_REALTIME (so POSIX time() agrees), persists the value,
 * and realigns any active recurring alarms to the new time.
 *
 * @param tm_utc Broken-down UTC time. Must not be NULL and must represent a
 *        time at or after ::RTC_SANITY_MIN_UNIX.
 * @retval true The wall clock was updated.
 * @retval false @p tm_utc is NULL, out of range, or the clock write failed.
 */
bool rtc_set_time(const struct tm* tm_utc);

/**
 * @brief Set the wall-clock time from a Unix timestamp.
 *
 * @param unix_seconds Seconds since the Unix epoch, at or after
 *        ::RTC_SANITY_MIN_UNIX.
 * @retval true The wall clock was updated.
 * @retval false @p unix_seconds is out of range or the clock write failed.
 */
bool rtc_set_unix(time_t unix_seconds);

/**
 * @brief Read the current wall-clock time as broken-down UTC time.
 *
 * @param tm_utc Destination for the broken-down UTC time. Must not be NULL.
 * @retval true @p tm_utc was populated.
 * @retval false @p tm_utc is NULL or the clock could not be read.
 */
bool rtc_get_time(struct tm* tm_utc);

/**
 * @brief Read the current wall-clock time as a Unix timestamp.
 *
 * @return Seconds since the Unix epoch, or (time_t)-1 if the clock could not be
 *         read.
 */
time_t rtc_get_unix(void);

/**
 * @brief Read the current wall-clock time as a Unix timestamp in milliseconds.
 *
 * @return Milliseconds since the Unix epoch, or (time_t)-1 if the clock could
 *         not be read.
 */
time_t rtc_get_timestamp_ms(void);

/**
 * @brief Read the current wall-clock time as a Unix timestamp in microseconds.
 *
 * @return Microseconds since the Unix epoch, or (time_t)-1 if the clock could
 *         not be read.
 */
time_t rtc_get_timestamp_us(void);

/**
 * @brief Enable or disable the recurring minute alarm.
 *
 * When enabled, ::rtc_event_MinuteAlarm is published at every wall-clock minute
 * boundary (`:00` seconds).
 *
 * @param enable true to arm the alarm, false to cancel it.
 * @retval true The request was applied.
 * @retval false The driver is not ready.
 */
bool rtc_enable_minute_alarm(bool enable);

/**
 * @brief Enable or disable the recurring hour alarm.
 *
 * When enabled, ::rtc_event_HourAlarm is published at every wall-clock hour
 * boundary (`:00:00`).
 *
 * @param enable true to arm the alarm, false to cancel it.
 * @retval true The request was applied.
 * @retval false The driver is not ready.
 */
bool rtc_enable_hour_alarm(bool enable);

/**
 * @brief Enable or disable the recurring day alarm.
 *
 * When enabled, ::rtc_event_DayAlarm is published at every wall-clock day
 * boundary (`00:00:00` UTC).
 *
 * @param enable true to arm the alarm, false to cancel it.
 * @retval true The request was applied.
 * @retval false The driver is not ready.
 */
bool rtc_enable_day_alarm(bool enable);

/**
 * @brief Arm the one-shot specific-time alarm from broken-down UTC time.
 *
 * ::rtc_event_TimeAlarm is published once when the wall clock reaches @p tm_utc.
 * Re-arming replaces any pending one-shot alarm.
 *
 * @param tm_utc Future UTC time. Must not be NULL.
 * @retval true The alarm was armed.
 * @retval false The driver is not ready, @p tm_utc is NULL, or it is not in the
 *         future.
 */
bool rtc_set_time_alarm(const struct tm* tm_utc);

/**
 * @brief Arm the one-shot specific-time alarm from a Unix timestamp.
 *
 * @param unix_seconds Future UTC time as seconds since the Unix epoch.
 * @retval true The alarm was armed.
 * @retval false The driver is not ready or @p unix_seconds is not in the future.
 */
bool rtc_set_time_alarm_unix(time_t unix_seconds);

/**
 * @brief Cancel the one-shot specific-time alarm.
 *
 * @retval true The alarm is cancelled (whether or not one was pending).
 * @retval false The driver is not ready.
 */
bool rtc_cancel_time_alarm(void);

/**
 * @brief Return the printable name for an RTC event identifier.
 *
 * @param event_id Event identifier from ::rtc_event_type.
 * @return Constant string for the event, or "Unknown" when @p event_id is not
 *         valid.
 */
const char* rtc_event_name(uint32_t event_id);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SENSWEAR_RTC_H_ */
