/*
 * Copyright (c) 2024 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * SenseWear local copy:
 * Copied from Zephyr/NCS v3.3.0
 *   zephyr/include/zephyr/drivers/haptics/drv2605.h
 * This file is intentionally vendored as the patch target for SenseWear-specific
 * DRV2605 integration. Keep local changes documented in PATCHED_FROM_ZEPHYR.md.
 */

/**
 * @file
 * @brief API for the SenseWear-patched DRV2605 haptic driver.
 * @ingroup drv2605_interface
 *
 * @details
 * This is a vendored copy of Zephyr's DRV2605 haptics driver, patched for the
 * SenseWear platform so it can be changed without modifying the installed NCS
 * tree. The companion source is `drv2605.c`; the provenance, baseline, and the
 * full list of local changes are tracked in `PATCHED_FROM_ZEPHYR.md` next to
 * this file. Keep that file and this block in sync when patching.
 *
 * @par Provenance
 * - Baseline: NCS `v3.3.0` / Zephyr `4.3.99`
 *   (`/Users/yusein/Tools/nordic/ncs/v3.3.0/zephyr`).
 * - Copied from `zephyr/{drivers/haptics/drv2605.c,
 *   include/zephyr/drivers/haptics/drv2605.h, drivers/haptics/Kconfig.drv2605,
 *   dts/bindings/haptics/ti,drv2605.yaml}`.
 * - Zephyr's original copyright and SPDX headers are preserved.
 *
 * @par What is patched (vs. the Zephyr baseline)
 * - @b Compatible: binds to the namespaced `sensewear,drv2605`
 *   (`DT_DRV_COMPAT sensewear_drv2605`) instead of `ti,drv2605`, so it replaces,
 *   rather than collides with, Zephyr's upstream driver, binding, and
 *   `HAPTICS_DRV2605` Kconfig symbol. The driver is built when
 *   `CONFIG_SENSEWEAR_DRV2605_DRIVER` is set.
 * - @b Bus: register access is routed through the SenseWear `sys_i2c` ownership
 *   wrapper (`sys_i2c_write` / `sys_i2c_write_read`, `sys_i2c_lock` /
 *   `sys_i2c_release`) instead of the direct `i2c_dt_spec` helpers. Every
 *   dev-level operation acquires bus ownership on entry and fully releases it
 *   before returning.
 * - @b Enable @b pin: no devicetree `en-gpios`. The enable line is mandatory and
 *   claimed at init from the daughter-board GPIO arbiter as `daughter_if_GPIO0`;
 *   if that line is already owned the driver logs an error and asserts. The pin
 *   is held for the device's lifetime.
 * - @b RTP @b lifecycle: per-device atomic state (`rtp_active`,
 *   `rtp_stop_requested`, `rtp_active_seconds`). A second RTP start is rejected
 *   with `-EBUSY`; an external stop is observed mid-stream; a duplicate
 *   `Stopped` event is suppressed when the worker will post it.
 * - @b Events: playback lifecycle is reported through the SenseWear device-driver
 *   event queue under `DRV2605_DEVICE_DTS_ID`; see @ref drv2605_event_type.
 *
 * @par Features
 * - Zephyr haptics device API (`haptics_start_output()` /
 *   `haptics_stop_output()`) for start/stop.
 * - Five signal sources via drv2605_haptic_config(): ROM library waveforms, RTP
 *   streaming, audio-to-vibe, PWM, and analog (see @ref drv2605_haptics_source).
 * - Asynchronous RTP playback on a work queue; the caller's RTP buffers are
 *   streamed directly and must outlive playback. drv2605_rtp_is_active() lets a
 *   buffer owner serialise patterns and avoid reusing buffers mid-stream.
 * - Lifecycle/observability events (`Starting`, `Stopped`, `PlaybackActive`,
 *   `Error`) with a once-per-second active heartbeat; drv2605_event_name()
 *   returns printable names.
 * - Power management hooks (suspend/resume + enable-pin turn off/on).
 *
 * @par Typical use case (RTP playback)
 * @code{.c}
 * // Device comes from the devicetree node bound to "sensewear,drv2605".
 * const struct device *dev = DEVICE_DT_GET(DT_ALIAS(sensewear_haptic));
 *
 * if (!device_is_ready(dev)) {
 *         return -ENODEV;
 * }
 *
 * // Buffers must stay valid for the whole async stream; keep them static or
 * // otherwise owned until playback finishes.
 * static uint32_t hold_us[]  = { 100000, 50000, 100000 }; // per-frame hold time
 * static uint8_t  input[]    = {    255,   128,      0  }; // per-frame amplitude
 * static struct drv2605_rtp_data rtp = {
 *         .size        = ARRAY_SIZE(input),
 *         .rtp_hold_us = hold_us,
 *         .rtp_input   = input,
 * };
 * const union drv2605_config_data cfg = { .rtp_data = &rtp };
 *
 * if (drv2605_rtp_is_active(dev)) {
 *         return -EBUSY;            // a previous pattern is still streaming
 * }
 *
 * int ret = drv2605_haptic_config(dev, DRV2605_HAPTICS_SOURCE_RTP, &cfg);
 * if (ret == 0) {
 *         ret = haptics_start_output(dev);  // returns immediately; plays async
 * }
 * // ... later, to abort early:
 * // haptics_stop_output(dev);
 * @endcode
 *
 * For ROM library playback, populate a @ref drv2605_rom_data instead and pass it
 * through @ref union drv2605_config_data::rom_data with
 * @ref DRV2605_HAPTICS_SOURCE_ROM, then call haptics_start_output().
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_HAPTICS_DRV2605_H_
#define ZEPHYR_INCLUDE_DRIVERS_HAPTICS_DRV2605_H_

#include <zephyr/drivers/haptics.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup drv2605_interface DRV2605
 * @ingroup haptics_interface_ext
 * @brief DRV2605 Haptic Driver for ERM and LRA
 * @{
 */

/**
 * @name Helpers
 * @{
 */

/** Maximum number of waveforms that can be stored in the sequencer */
#define DRV2605_WAVEFORM_SEQUENCER_MAX 8

/**
 * @brief Creates a wait/delay value for the waveform sequencer.
 *
 * @details This macro generates a byte value that, when placed in the drv2605_rom_data::seq_regs
 * array, instructs the DRV2605 playback engine to idle for a specified duration.
 *
 * @param ms The desired delay in milliseconds (rounded down to the nearest 10ms). Valid range is
 * 10 to 1270.
 * @return A byte literal representing the wait time for the sequencer.
 */
#define DRV2605_WAVEFORM_SEQUENCER_WAIT_MS(ms) (0x80 | ((ms) / 10))

/** @} */

/**
 * @brief Effect libraries
 *
 * This enumeration defines the different effect libraries that can be used with the DRV2605 when
 * using ROM source. TouchSense 2220 libraries are for open-loop ERM motors,
 * @ref DRV2605_LIBRARY_LRA is to be used for closed-loop LRA motors.
 */
enum drv2605_library {
	DRV2605_LIBRARY_EMPTY = 0,   /**< Empty library */
	DRV2605_LIBRARY_TS2200_A,    /**< TouchSense 2220 A library */
	DRV2605_LIBRARY_TS2200_B,    /**< TouchSense 2220 B library */
	DRV2605_LIBRARY_TS2200_C,    /**< TouchSense 2220 C library */
	DRV2605_LIBRARY_TS2200_D,    /**< TouchSense 2220 D library */
	DRV2605_LIBRARY_TS2200_E,    /**< TouchSense 2220 E library */
	DRV2605_LIBRARY_LRA,         /**< Linear Resonance Actuator (LRA) library */
};

/**
 * @brief Modes of operation
 *
 * @details This enumeration defines the different modes of operation supported by the DRV2605.
 *
 * See Table 5 of the DRV2605 datasheet for more information on the various operation modes.
 */
enum drv2605_mode {
	DRV2605_MODE_INTERNAL_TRIGGER = 0,    /**< Internal trigger mode */
	DRV2605_MODE_EXTERNAL_EDGE_TRIGGER,   /**< External trigger mode (edge) */
	DRV2605_MODE_EXTERNAL_LEVEL_TRIGGER,  /**< External trigger mode (level) */
	DRV2605_MODE_PWM_ANALOG_INPUT,        /**< PWM or Analog input mode */
	DRV2605_MODE_AUDIO_TO_VIBE,           /**< Audio-to-vibe mode */
	DRV2605_MODE_RTP,                     /**< RTP mode */
	DRV2605_MODE_DIAGNOSTICS,             /**< Diagnostics mode */
	DRV2605_MODE_AUTO_CAL,                /**< Auto-calibration mode */
};

/**
 * @brief Types of haptic signal sources.
 *
 * @details This enumeration defines the different types of haptic signal sources supported by the
 * DRV2605.
 */
enum drv2605_haptics_source {
	DRV2605_HAPTICS_SOURCE_ROM,      /**< Playback from the pre-programmed ROM library. */
	DRV2605_HAPTICS_SOURCE_RTP,      /**< Playback from Real-Time Playback (RTP) data stream. */
	DRV2605_HAPTICS_SOURCE_AUDIO,    /**< Playback is generated from an audio signal. */
	DRV2605_HAPTICS_SOURCE_PWM,      /**< Playback is driven by an external PWM signal. */
	DRV2605_HAPTICS_SOURCE_ANALOG,   /**< Playback is driven by an external analog signal. */
};

/**
 * @brief Device-driver event identifiers posted by the DRV2605 driver.
 *
 * @details Events are posted to the SenseWear device-driver event queue with
 *          `DRV2605_DEVICE_DTS_ID` as the device id. For playback lifecycle
 *          events, `v_param` carries the active @ref drv2605_mode. For
 *          @ref drv2605_event_PlaybackActive, `v_param` carries elapsed RTP
 *          playback time in seconds. For @ref drv2605_event_Error, `v_param`
 *          carries the positive errno value that caused playback to stop early.
 */
enum drv2605_event_type {
	drv2605_event_Starting = 0,       /**< Playback start was accepted. */
	drv2605_event_Stopped = 1,        /**< Playback stopped or completed. */
	drv2605_event_PlaybackActive = 2, /**< RTP playback is still active. */
	drv2605_event_Error = 3,          /**< Playback stopped because of an error. */
	drv2605_event_Count,              /**< Number of valid event identifiers. */
};

/**
 * @brief ROM data configuration
 *
 * @details This structure contains configuration data for when the DRV2605 is using the internal
 * ROM as the haptic source (@ref DRV2605_HAPTICS_SOURCE_ROM).
 */
struct drv2605_rom_data {
	/** Mode of operation for triggering the ROM effects. */
	enum drv2605_mode trigger;
	/** Effect library to use for playback. */
	enum drv2605_library library;
	/**
	 * @brief Waveform sequencer contents.
	 *
	 * This array contains the register values for the sequencer registers (0x04 to 0x0B).
	 * Each byte can describe either:
	 * - A waveform identifier (1 to 123) from the selected library to be played.
	 * - A wait time, if the MSB is set. The lower 7 bits are a multiple of 10 ms.
	 * Playback stops at the first zero entry.
	 *
	 * See Table 8 of the DRV2605 datasheet.
	 */
	uint8_t seq_regs[DRV2605_WAVEFORM_SEQUENCER_MAX];
	/**
	 * @brief Overdrive time offset.
	 *
	 * A signed 8-bit value that adds a time offset to the overdrive portion of the waveform.
	 * The offset is `overdrive_time * 5 ms`.
	 *
	 * See Table 10 of the DRV2605 datasheet.
	 */
	uint8_t overdrive_time;
	/**
	 * @brief Sustain positive time offset.
	 *
	 * A signed 8-bit value that adds a time offset to the positive sustain portion of the
	 * waveform. The offset is `sustain_pos_time * 5 ms`.
	 *
	 * See Table 11 of the DRV2605 datasheet.
	 */
	uint8_t sustain_pos_time;
	/**
	 * @brief Sustain negative time offset.
	 *
	 * A signed 8-bit value that adds a time offset to the negative sustain portion of the
	 * waveform. The offset is `sustain_neg_time * 5 ms`.
	 *
	 * See Table 12 of the DRV2605 datasheet.
	 */
	uint8_t sustain_neg_time;
	/**
	 * @brief Brake time offset.
	 *
	 * A signed 8-bit value that adds a time offset to the braking portion of the waveform.
	 * The offset is `brake_time * 5 ms`.
	 *
	 * See Table 13 of the DRV2605 datasheet.
	 */
	uint8_t brake_time;
};

/**
 * @brief Real-Time Playback (RTP) data configuration.
 *
 * @details This structure contains configuration data for when the DRV2605 is in RTP mode
 * (@ref DRV2605_HAPTICS_SOURCE_RTP). It allows for streaming custom haptic waveforms.
 */
struct drv2605_rtp_data {
	/** The number of entries in the @ref rtp_hold_us and @ref rtp_input arrays. */
	size_t size;
	/**
	 * @brief Pointer to an array of hold times.
	 *
	 * Each value specifies the duration in microseconds to hold the corresponding
	 * amplitude from the `rtp_input` array.
	 */
	uint32_t *rtp_hold_us;
	/**
	 * @brief Pointer to an array of RTP amplitude values.
	 *
	 * Each value is an 8-bit amplitude that will be written to the RTP input register (0x02).
	 * These values can be signed or unsigned depending on device configuration.
	 */
	uint8_t *rtp_input;
};

/**
 * @brief Configuration data union.
 *
 * @details This union holds a pointer to the specific configuration data struct required
 * for the selected haptic source. The correct member must be populated before calling
 * drv2605_haptic_config().
 */
union drv2605_config_data {
	/** Pointer to ROM configuration data. Use when source is @ref DRV2605_HAPTICS_SOURCE_ROM */
	struct drv2605_rom_data *rom_data;
	/** Pointer to RTP configuration data. Use when source is @ref DRV2605_HAPTICS_SOURCE_RTP */
	struct drv2605_rtp_data *rtp_data;
};

/**
 * @brief Configure the DRV2605 device for a particular signal source
 *
 * @param dev Pointer to the device structure for haptic device instance
 * @param source The type of haptic signal source desired
 * @param config_data Pointer to the configuration data union for the source
 *
 * @retval 0 success
 * @retval -ENOTSUP signal source not supported
 * @retval -errno another negative error code on failure
 */
int drv2605_haptic_config(const struct device *dev, enum drv2605_haptics_source source,
			  const union drv2605_config_data *config_data);

/**
 * @brief Return a printable DRV2605 event name.
 *
 * @param event_id Event identifier from @ref drv2605_event_type.
 * @return Static string for known event ids, otherwise `"Unknown"`.
 */
const char *drv2605_event_name(enum drv2605_event_type event_id);

/**
 * @brief Report whether an RTP stream is currently playing.
 *
 * @details RTP playback runs asynchronously on a work queue. A caller that owns
 * the RTP data buffers passed to drv2605_haptic_config() can use this to avoid
 * reconfiguring or freeing those buffers while the worker is still streaming
 * from them, and to serialise back-to-back patterns.
 *
 * @param dev Pointer to the device structure for haptic device instance.
 * @retval true RTP playback is active.
 * @retval false No RTP playback is in progress.
 */
bool drv2605_rtp_is_active(const struct device *dev);


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
