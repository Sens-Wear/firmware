/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file bhi360.c
 * @brief SenseWear BHI360 smart IMU driver implementation.
 *
 * @section bhi360_impl_overview Implementation Overview
 *
 * This driver manages a BHI360 smart IMU through Bosch's BHY2 host library.
 * The device is used as an interrupt-driven virtual-sensor hub: the physical
 * IRQ line only signals that work is pending, while all actual samples and
 * firmware notifications are retrieved later from thread context by draining
 * the device FIFO.
 *
 * The operational flow is:
 * - bhi360_init() prepares shared SPI access, chip-select/reset GPIOs, and the
 *   IRQ callback.
 * - bhi360_config() binds the BHY2 transport hooks, probes the product,
 *   uploads firmware, discovers available virtual sensors, and enables the
 *   driver's default sensor sets.
 * - bhi360_irq_callback() posts bhi360_event_Irq from ISR context.
 * - bhi360_irq_handler() runs from caller thread context, reads interrupt
 *   status, and drains the FIFO. BHY2 dispatches FIFO packets into the parser
 *   callbacks below.
 * - Parser callbacks decode payloads into cached driver-owned structs and post
 *   higher-level device_driver_event_t messages for the application.
 *
 * @section bhi360_impl_design Driver Design
 *
 * The driver is implemented as a singleton because the board contains exactly
 * one BHI360 instance. The singleton owns:
 * - the shared SPI device specification,
 * - GPIO specifications for chip select, reset, IRQ, and the two auxiliary
 *   user-routable GPIO pins,
 * - a BHY2 device context,
 * - a FIFO work buffer used by bhy2_get_and_process_fifo(), and
 * - per-event cached sample storage whose addresses are published in p_param.
 *
 * The high-rate streams (quaternion, linear acceleration, gyroscope) cache their
 * samples in per-drain arrays: a single interrupt can decode many samples of the
 * same stream. These streams are not posted per sample. Instead each drain emits
 * one summary event (bhi360_event_QuaternionBatch, _LinearAccelerationBatch,
 * _GyroBatch) whose v_param is the number of samples collected and whose p_param
 * points at the first element of the array. The arrays are reset at the start of
 * each bhi360_irq_handler() pass, so a published pointer stays valid only until
 * the next FIFO drain; bhi360_copy_*() returns a thread-safe snapshot. The
 * remaining low-rate classes are still posted one event per sample and reuse one
 * cached struct each. Consumers that need long-lived copies must duplicate the
 * pointed-to data.
 *
 * @section bhi360_impl_enabled_sensors Enabled Sensors
 *
 * bhi360_config() enables the activity-class virtual sensors requested by
 * struct bhi360_config_t. Passing NULL selects the SenseWear default list:
 * motion/no-motion, wrist gesture/wear, wear-aware activity recognition, step
 * counter, and step detector. High-rate physical streams (rotation vector,
 * acceleration, gyroscope) are enabled later through
 * bhi360_start_phy_sensor_streams().
 *
 * Meta-event streams are also registered so firmware status notifications are
 * surfaced to the event system and logs.
 */

#include <assert.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "bhi3_defs.h"
#include "bhi360.h"
#include "bhi360_api_error.h"
#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"
#include "device_driver_dts_ids.h"
#include "device_driver_events.h"
#include "rtc.h"
#include "sys_spi.h"

#define BHY2_RD_WR_LEN 256
#define WORK_BUFFER_SIZE 2048
#define BHI360_NODE DT_NODELABEL(bhi360)

BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, cs_gpios), "BHI360 is missing its chip-select GPIO");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, int_gpios), "BHI360 is missing its IRQ GPIO");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, reset_gpios), "BHI360 is missing its reset GPIO");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, gpio0_gpios), "BHI360 is missing GPIO0 wiring");
BUILD_ASSERT(DT_NODE_HAS_PROP(BHI360_NODE, gpio1_gpios), "BHI360 is missing GPIO1 wiring");

LOG_MODULE_REGISTER(bhi360, CONFIG_LOG_DEFAULT_LEVEL);

// TODO: Implement loading the firmware from external flash
/* Uncomment to upload firmware to flash instead of RAM */
/*#define UPLOAD_FIRMWARE_TO_FLASH*/

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "firmware/bhi360/BHI260AP-flash.fw.h"
#else
#include "firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#endif

#define BHI360_SPI_TIMEOUT K_MSEC(100)
#define BHI360_REPORT_LATENCY_MS 100U
#define BHI360_WAKE_FIFO_WATERMARK_BYTES 8U
#define BHI360_NONWAKE_FIFO_WATERMARK_BYTES 8U

/** @brief Sentinel for sensor_time_offset before the first successful sync. */
#define BHI360_TIME_OFFSET_UNSET ((time_t) -1)
/** @brief Minimum age of the last time sync before it is refreshed, in microseconds. */
#define BHI360_TIME_SYNC_PERIOD_US ((time_t) 60 * 1000 * 1000)
/** @brief Settle time after a config-time timestamp-event request, in milliseconds. */
#define BHI360_TIME_SYNC_INIT_DELAY_MS 5

/**
 * @brief Maximum number of high-rate samples cached per FIFO drain.
 * @details A single bhi360_irq_handler() pass can decode several samples of the
 *          same high-rate stream (quaternion, accelerometer, gyroscope). Each
 *          decoded sample is stored in its own slot so the pointer published in
 *          a posted event stays valid until the next FIFO drain, instead of
 *          being overwritten by the next sample in the same batch. At 100 Hz with
 *          a 100 ms report latency a batch holds roughly ten samples per stream;
 *          this bound leaves comfortable headroom. Excess samples in an unusually
 *          large batch are dropped with a warning.
 */
#define BHI360_MAX_SAMPLES_PER_IRQ 32U

/**
 * @brief Sensor configuration descriptor used during enable/disable passes.
 * @details Each entry binds a BHY2 virtual-sensor ID to a desired output rate,
 *          latency budget, a human-readable name for logging, and the FIFO
 *          parser callback that translates raw BHY2 packets into SenseWear
 *          driver events.
 */
struct bhi360_sensor_enable {
	/** @brief BHY2 sensor ID constant (e.g., BHY2_SENSOR_ID_GAMERV_WU, BHY2_SENSOR_ID_GYRO_WU). */
	uint8_t sensor_id;
	/** @brief Human-readable sensor name for logging. */
	const char* name;
	/** @brief Desired output sample rate in Hz. */
	bhy2_float sample_rate_hz;
	/** @brief Acceptable latency in milliseconds; 0 for real-time. */
	uint32_t latency_ms;
	/** @brief FIFO parser callback function that decodes and posts events. */
	bhy2_fifo_parse_callback_t parser;
};

/**
 * @brief Static metadata for one supported activity-class virtual sensor.
 */
struct bhi360_activity_sensor_descriptor {
	enum bhi360_activity_sensor_type sensor;
	uint8_t sensor_id;
	const char* name;
	bhy2_fifo_parse_callback_t parser;
};

/**
 * @brief Internal lifecycle state bits for the singleton driver instance.
 * @details These flags track which initialization stages have completed and are
 *          consulted by the SPI transport hooks and public control flow.
 */
union bhi360_state_t {
	/** @brief Raw byte value. */
	uint8_t value;
	/** @brief Individual state flags. */
	struct {
		/** @brief SPI, GPIOs, and IRQ handlers ready; ready for configuration. */
		uint8_t initialized : 1;
		/** @brief Firmware uploaded and sensors enabled; ready for IRQ processing. */
		uint8_t configured : 1;
		/** @brief Device communication successful; used internally during probe. */
		uint8_t probed : 1;
		/** @brief Product ID verified to be BHY2_PRODUCT_ID. */
		uint8_t device_found : 1;
		/** @brief GPIO interrupt callbacks registered and armed. */
		uint8_t irq_ready : 1;
	} bits;
};

/**
 * @brief Singleton BHI360 driver state.
 * @details Owns hardware descriptors, BHY2 state, the FIFO work buffer, and
 *          cached payload structs referenced by posted events.
 */
static struct bhi360_t {
	/** @brief Shared SPI bus descriptor used by the BHY2 transport callbacks. */
	struct sys_spi_dt_spec spi;
	/** @brief Manual chip-select GPIO for the BHI360 SPI target. */
	struct gpio_dt_spec cs_gpio;
	/** @brief Host interrupt GPIO driven by the BHI360 HIRQ pin. */
	struct gpio_dt_spec irq_gpio;
	/** @brief Hardware reset GPIO connected to the BHI360 RESETN pin. */
	struct gpio_dt_spec reset_gpio;
	/** @brief Auxiliary BHI360 GPIO0 DTS specification exposed through bhi360_get_gpio0(). */
	struct gpio_dt_spec gpio0;
	/** @brief Auxiliary BHI360 GPIO1 DTS specification exposed through bhi360_get_gpio1(). */
	struct gpio_dt_spec gpio1;
	/** @brief Zephyr GPIO callback object registered on the interrupt GPIO. */
	struct gpio_callback irq_cb;
	/** @brief Bosch BHY2 host-library device context. */
	struct bhy2_dev bhy2;
	/** @brief Lifecycle state flags for initialization, probing, configuration, and IRQ setup. */
	union bhi360_state_t state;
	/** @brief Human-readable driver name used in log messages. */
	char name[32];

	/** @brief Driver-owned FIFO work buffer passed to bhy2_get_and_process_fifo(). */
	uint8_t work_buffer[WORK_BUFFER_SIZE];
	/**< @brief Flag indicating whether the physical sensor streams are enabled. */
	bool enable_phy_sensor_streams;
	/**< @brief Flag indicating whether the sample timer is running. */
	bool timer_running;
	/**< @brief Timer period in milliseconds. */
	uint32_t timer_period_ms;
	/**< @brief Zephyr timer used to trigger FIFO processing at a fixed interval. */
	struct k_timer fifo_timer;

	/** @brief Guards the per-drain sample arrays and counts against concurrent drain/copy access.
	 */
	struct k_mutex lock;
	/** @brief Quaternion samples decoded in the current FIFO drain, published as one batch event.
	 */
	struct bhi360_quat_data_t quat_data[BHI360_MAX_SAMPLES_PER_IRQ];
	/** @brief Number of valid entries in quat_data for the current FIFO drain. */
	size_t quat_count;
	size_t quat_skipped_count; /**< @brief Number of quaternion samples skipped due to array
								  overflow. */
	/** @brief Linear-acceleration samples decoded in the current FIFO drain, published as one batch
	 * event. */
	struct bhi360_lacc_data_t lacc_data[BHI360_MAX_SAMPLES_PER_IRQ];
	/** @brief Number of valid entries in lacc_data for the current FIFO drain. */
	size_t lacc_count;
	size_t lacc_skipped_count; /**< @brief Number of linear-acceleration samples skipped due to
								  array overflow. */
	/** @brief Gyroscope samples decoded in the current FIFO drain, published as one batch event. */
	struct bhi360_gyro_data_t gyro_data[BHI360_MAX_SAMPLES_PER_IRQ];
	/** @brief Number of valid entries in gyro_data for the current FIFO drain. */
	size_t gyro_count;
	size_t gyro_skipped_count; /**< @brief Number of gyroscope samples skipped due to array
								  overflow. */
	/** @brief Cached pedometer sample published through bhi360_event_Pedometer p_param. */
	struct bhi360_pedometer_data_t pedometer_data;
	/** @brief Cached gesture sample published through bhi360_event_Gesture p_param. */
	struct bhi360_gesture_data_t gesture_data;
	/** @brief Cached activity sample published through bhi360_event_Activity p_param. */
	struct bhi360_activity_data_t activity_data;

	/** @brief Host-minus-sensor timestamp offset in microseconds; established at
	 *         config and re-synced at most once per ::BHI360_TIME_SYNC_PERIOD_US. */
	time_t sensor_time_offset;
	/** @brief RTC time (microseconds) of the last time-sync attempt, or 0 if never. */
	time_t last_time_sync_us;
	time_t last_irq_timestamp; /**< @brief Timestamp of the most recent IRQ (milliseconds since the
								  Unix epoch). */
} bhi360 = {
	.spi = SYS_SPI_DT_SPEC_GET(BHI360_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
	.cs_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, cs_gpios),
	.irq_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, int_gpios),
	.reset_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, reset_gpios),
	.gpio0 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio0_gpios),
	.gpio1 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio1_gpios),
	.name = "BHI360",
	.sensor_time_offset = BHI360_TIME_OFFSET_UNSET,
	.last_time_sync_us = 0,
	.last_irq_timestamp = -1,
};

static const char* const bhi360_event_names[bhi360_event_Count] = {
	[bhi360_event_Irq] = "Irq",
	[bhi360_event_QuaternionBatch] = "QuaternionBatch",
	[bhi360_event_LinearAccelerationBatch] = "LinearAccelerationBatch",
	[bhi360_event_GyroBatch] = "GyroBatch",
	[bhi360_event_Pedometer] = "Pedometer",
	[bhi360_event_Gesture] = "Gesture",
	[bhi360_event_Activity] = "Activity",
	[bhi360_event_MetaEvent] = "MetaEvent",
};

/**
 * @brief Look up the top-level BHI360 event name.
 * @param event_id Event ID from @ref bhi360_event_type.
 * @return Constant event-name string, or "Unknown" if the ID is invalid.
 */
static const char* bhi360_base_event_name(uint32_t event_id) {
	if ((event_id >= (uint32_t) bhi360_event_Count) || (bhi360_event_names[event_id] == NULL)) {
		return "Unknown";
	}

	return bhi360_event_names[event_id];
}

/**
 * @brief Look up a pedometer event name.
 * @param event Pedometer subevent decoded from bhi360_event_Pedometer v_param.
 * @return Constant parent-and-subevent-name string, or "Pedometer: Unknown" if unsupported.
 */
static const char* bhi360_pedometer_event_name(enum bhi360_pedometer_event_type event) {
	switch (event) {
	case bhi360_pedometer_event_StepCounter:
		return "Pedometer: StepCounter";
	case bhi360_pedometer_event_StepCounterWakeup:
		return "Pedometer: StepCounterWakeup";
	case bhi360_pedometer_event_StepCounterLowPower:
		return "Pedometer: StepCounterLowPower";
	case bhi360_pedometer_event_StepCounterLowPowerWakeup:
		return "Pedometer: StepCounterLowPowerWakeup";
	case bhi360_pedometer_event_StepDetector:
		return "Pedometer: StepDetector";
	case bhi360_pedometer_event_StepDetectorWakeup:
		return "Pedometer: StepDetectorWakeup";
	case bhi360_pedometer_event_StepDetectorLowPower:
		return "Pedometer: StepDetectorLowPower";
	case bhi360_pedometer_event_StepDetectorLowPowerWakeup:
		return "Pedometer: StepDetectorLowPowerWakeup";
	default:
		return "Pedometer: Unknown";
	}
}

/**
 * @brief Look up a gesture event name.
 * @param event Gesture sensor subevent decoded from bits 15:8 of bhi360_event_Gesture v_param.
 * @return Constant parent-and-subevent-name string, or "Gesture: Unknown" if unsupported.
 */
static const char* bhi360_gesture_event_name(enum bhi360_gesture_event_type event) {
	switch (event) {
	case bhi360_gesture_event_Wake:
		return "Gesture: WakeGesture";
	case bhi360_gesture_event_Glance:
		return "Gesture: GlanceGesture";
	case bhi360_gesture_event_Pickup:
		return "Gesture: PickupGesture";
	case bhi360_gesture_event_WristTilt:
		return "Gesture: WristTiltGesture";
	case bhi360_gesture_event_TiltDetector:
		return "Gesture: TiltDetector";
	case bhi360_gesture_event_StationaryDetector:
		return "Gesture: StationaryDetector";
	case bhi360_gesture_event_MotionDetector:
		return "Gesture: MotionDetector";
	case bhi360_gesture_event_SignificantMotion:
		return "Gesture: SignificantMotion";
	case bhi360_gesture_event_SignificantMotionLowPower:
		return "Gesture: SignificantMotionLowPower";
	case bhi360_gesture_event_SignificantMotionLowPowerWakeup:
		return "Gesture: SignificantMotionLowPowerWakeup";
	case bhi360_gesture_event_AnyMotionLowPower:
		return "Gesture: AnyMotionLowPower";
	case bhi360_gesture_event_AnyMotionLowPowerWakeup:
		return "Gesture: AnyMotionLowPowerWakeup";
	case bhi360_gesture_event_NoMotionLowPowerWakeup:
		return "Gesture: NoMotionLowPowerWakeup";
	case bhi360_gesture_event_WristGestureDetectLowPowerWakeup:
		return "Gesture: WristGestureDetectLowPowerWakeup";
	case bhi360_gesture_event_WristWearLowPowerWakeup:
		return "Gesture: WristWearLowPowerWakeup";
	default:
		return "Gesture: Unknown";
	}
}

/**
 * @brief Look up an activity-recognition source event name.
 * @param event Activity source decoded from bits 23:16 of bhi360_event_Activity v_param.
 * @return Constant parent-and-source-event-name string, or "Activity: Unknown" if unsupported.
 */
static const char* bhi360_activity_event_name(enum bhi360_activity_event_type event) {
	switch (event) {
	case bhi360_activity_event_Recognition:
		return "Activity: ActivityRecognition";
	case bhi360_activity_event_WearRecognitionWakeup:
		return "Activity: WearActivityRecognitionWakeup";
	default:
		return "Activity: Unknown";
	}
}

/**
 * @brief Look up an activity transition bit name.
 * @param event Single activity-transition bit decoded from bits 15:0 of bhi360_event_Activity
 * v_param.
 * @return Constant parent-and-transition-name string, or NULL if the value is zero, multi-bit, or
 * unsupported.
 */
static const char* bhi360_activity_transition_name(enum bhi360_activity_transition_type event) {
	switch (event) {
	case bhi360_activity_transition_StillEnded:
		return "Activity: StillEnded";
	case bhi360_activity_transition_WalkingEnded:
		return "Activity: WalkingEnded";
	case bhi360_activity_transition_RunningEnded:
		return "Activity: RunningEnded";
	case bhi360_activity_transition_BicycleEnded:
		return "Activity: BicycleEnded";
	case bhi360_activity_transition_VehicleEnded:
		return "Activity: VehicleEnded";
	case bhi360_activity_transition_TiltingEnded:
		return "Activity: TiltingEnded";
	case bhi360_activity_transition_StillStarted:
		return "Activity: StillStarted";
	case bhi360_activity_transition_WalkingStarted:
		return "Activity: WalkingStarted";
	case bhi360_activity_transition_RunningStarted:
		return "Activity: RunningStarted";
	case bhi360_activity_transition_BicycleStarted:
		return "Activity: BicycleStarted";
	case bhi360_activity_transition_VehicleStarted:
		return "Activity: VehicleStarted";
	case bhi360_activity_transition_TiltingStarted:
		return "Activity: TiltingStarted";
	default:
		return NULL;
	}
}

/**
 * @brief Look up a firmware meta-event name.
 * @param event Meta-event type decoded from bits 23:16 of bhi360_event_MetaEvent v_param.
 * @return Constant parent-and-meta-event-name string, or "MetaEvent: Unknown" if unsupported.
 */
static const char* bhi360_meta_event_name(enum bhi360_meta_event_type event) {
	switch (event) {
	case bhi360_meta_event_FlushComplete:
		return "MetaEvent: FlushComplete";
	case bhi360_meta_event_SampleRateChanged:
		return "MetaEvent: SampleRateChanged";
	case bhi360_meta_event_PowerModeChanged:
		return "MetaEvent: PowerModeChanged";
	case bhi360_meta_event_AlgorithmEvents:
		return "MetaEvent: AlgorithmEvents";
	case bhi360_meta_event_SensorStatus:
		return "MetaEvent: SensorStatus";
	case bhi360_meta_event_BsxDoStepsMain:
		return "MetaEvent: BsxDoStepsMain";
	case bhi360_meta_event_BsxDoStepsCalib:
		return "MetaEvent: BsxDoStepsCalib";
	case bhi360_meta_event_BsxGetOutputSignal:
		return "MetaEvent: BsxGetOutputSignal";
	case bhi360_meta_event_SensorError:
		return "MetaEvent: SensorError";
	case bhi360_meta_event_FifoOverflow:
		return "MetaEvent: FifoOverflow";
	case bhi360_meta_event_DynamicRangeChanged:
		return "MetaEvent: DynamicRangeChanged";
	case bhi360_meta_event_FifoWatermark:
		return "MetaEvent: FifoWatermark";
	case bhi360_meta_event_Initialized:
		return "MetaEvent: Initialized";
	case bhi360_meta_event_TransferCause:
		return "MetaEvent: TransferCause";
	case bhi360_meta_event_SensorFramework:
		return "MetaEvent: SensorFramework";
	case bhi360_meta_event_Reset:
		return "MetaEvent: Reset";
	case bhi360_meta_event_Spacer:
		return "MetaEvent: Spacer";
	default:
		return "MetaEvent: Unknown";
	}
}

/** Assert the BHI360 chip-select line. */
static inline void bhi360_cs_select(struct bhi360_t* imu) {
	gpio_pin_set_dt(&imu->cs_gpio, 1);
}

/** Release the BHI360 chip-select line. */
static inline void bhi360_cs_deselect(struct bhi360_t* imu) {
	gpio_pin_set_dt(&imu->cs_gpio, 0);
}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref);
static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info* callback_info,
									  void* callback_ref);
static void parse_gyro(const struct bhy2_fifo_parse_data_info* callback_info, void* callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref);
static void parse_scalar_event(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref);
static void parse_step_counter(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref);
static void parse_activity(const struct bhy2_fifo_parse_data_info* callback_info,
						   void* callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev* dev);
static int8_t bhi360_try_sync_sensor_time_offset(struct bhi360_t* dev, time_t now_us);
static void bhi360_init_sensor_time_offset(struct bhi360_t* dev);
static void bhi360_maybe_sync_sensor_time_offset(struct bhi360_t* dev);

/**
 * @brief Program and log the BHI360 host interrupt routing.
 * @details The firmware image can reinitialize host-interface registers during
 *          boot, so this is applied after firmware boot before sensors are
 *          enabled. Edge mode is used because the firmware generates HIRQ pulses.
 *
 *          Wake and non-wake FIFO interrupt sources remain enabled. Status and
 *          debug HIRQ sources are disabled to avoid meta/status traffic waking
 *          the host independently of FIFO data.
 *
 *          The HIRQ line (P1.09 MPU_IRQ#) is a dedicated point-to-point signal,
 *          so it is driven push-pull rather than open-drain: the firmware
 *          actively drives both edges and no external/internal pull-up is
 *          required. Open-drain would need a pull-up to return the active-low
 *          line high between pulses; without one the line floats after the first
 *          pulse and no further GPIO_INT_EDGE_TO_ACTIVE edges are seen, so no
 *          interrupts fire at all.
 */
static bool bhi360_configure_host_interrupts(struct bhi360_t* imu) {
	uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_DEBUG | BHY2_ICTL_ACTIVE_LOW | BHY2_ICTL_EDGE;
	int8_t rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	LOG_INF("%s: host interrupt ctrl=0x%02x W:%s NW:%s ST:%s DBG:%s polarity:%s drive:%s mode:%s",
			imu->name,
			hintr_ctrl,
			(hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "off" : "on",
			(hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "off" : "on",
			(hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "off" : "on",
			(hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "off" : "on",
			(hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active-low" : "active-high",
			(hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open-drain" : "push-pull",
			(hintr_ctrl & BHY2_ICTL_EDGE) ? "edge" : "level");

	return true;
}

/**
 * @brief Program FIFO watermarks used for host interrupt generation.
 * @details The BHY2 helper performs a FIFO-control parameter read, write, and
 *          read-back verification. Run this during configuration before sensor
 *          streaming starts; doing the same transaction while async status and
 *          stream data are active can collide with status-channel traffic.
 */
static bool bhi360_configure_fifo_watermark(struct bhi360_t* imu) {
	int8_t rslt = bhy2_set_fifo_wmark_wkup(BHI360_WAKE_FIFO_WATERMARK_BYTES, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: failed to set wake FIFO watermark to %u bytes",
				imu->name,
				BHI360_WAKE_FIFO_WATERMARK_BYTES);
		return false;
	}

	rslt = bhy2_set_fifo_wmark_nonwkup(BHI360_NONWAKE_FIFO_WATERMARK_BYTES, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: failed to set non-wake FIFO watermark to %u bytes",
				imu->name,
				BHI360_NONWAKE_FIFO_WATERMARK_BYTES);
		return false;
	}

	uint32_t wake_watermark = 0;
	rslt = bhy2_get_fifo_wmark_wkup(&wake_watermark, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	uint32_t nonwake_watermark = 0;
	rslt = bhy2_get_fifo_wmark_nonwkup(&nonwake_watermark, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	LOG_INF("%s: FIFO watermarks W=%u bytes NW=%u bytes",
			imu->name,
			wake_watermark,
			nonwake_watermark);
	if ((wake_watermark != BHI360_WAKE_FIFO_WATERMARK_BYTES) ||
		(nonwake_watermark != BHI360_NONWAKE_FIFO_WATERMARK_BYTES)) {
		LOG_ERR("%s: FIFO watermark readback mismatch, expected W=%u bytes NW=%u bytes",
				imu->name,
				BHI360_WAKE_FIFO_WATERMARK_BYTES,
				BHI360_NONWAKE_FIFO_WATERMARK_BYTES);
		return false;
	}

	return true;
}

/**
 * @brief Physical sensors enabled by default.
 * @details These are the primary motion outputs exposed by the public API.
 */
static const struct bhi360_sensor_enable bhi360_phy_sensors[] = {
	/* Use wake-up variants for high-rate streams so their samples go through the
	 * wake FIFO. Step Counter LP is only available as a non-wake stream in this
	 * firmware, so both wake and non-wake FIFO watermarks are configured small. */
	{BHY2_SENSOR_ID_GAMERV_WU, "Quaternion", 100.0f, BHI360_REPORT_LATENCY_MS, parse_quaternion},
	{BHY2_SENSOR_ID_ACC_WU, "Accel.", 100.0f, BHI360_REPORT_LATENCY_MS, parse_linear_acceleration},
	{BHY2_SENSOR_ID_GYRO_WU, "Gyroscope", 100.0f, BHI360_REPORT_LATENCY_MS, parse_gyro},
};

/** Supported low-rate activity-class sensors. */
static const struct bhi360_activity_sensor_descriptor bhi360_activity_sensor_descriptors[] = {
	{bhi360_activity_sensor_AnyMotionLowPower,
	 BHY2_SENSOR_ID_ANY_MOTION_LP,
	 "Any motion",
	 parse_scalar_event},
	{bhi360_activity_sensor_NoMotionLowPowerWakeup,
	 BHI3_SENSOR_ID_NO_MOTION_LP_WU,
	 "No motion",
	 parse_scalar_event},
	{bhi360_activity_sensor_WristGestureDetectLowPowerWakeup,
	 BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU,
	 "Wrist gesture detect",
	 parse_scalar_event},
	{bhi360_activity_sensor_WristWearLowPowerWakeup,
	 BHI3_SENSOR_ID_WRIST_WEAR_LP_WU,
	 "Wrist wear",
	 parse_scalar_event},
	{bhi360_activity_sensor_WearRecognitionWakeup,
	 BHI3_SENSOR_ID_AR_WEAR_WU,
	 "Wear activity",
	 parse_activity},
	{bhi360_activity_sensor_StepCounterLowPower,
	 BHY2_SENSOR_ID_STC_LP,
	 "Step counter",
	 parse_step_counter},
	{bhi360_activity_sensor_StepDetectorLowPower,
	 BHY2_SENSOR_ID_STD_LP,
	 "Step detector",
	 parse_scalar_event},
};

/** SenseWear default activity-class sensor configuration. */
static const struct bhi360_activity_sensor_config_t bhi360_default_activity_sensors[] = {
	/* Report latency 0 on every low-rate event sensor: these are on-change/event
	 * sensors, so a zero max-report-latency makes the firmware raise a host
	 * interrupt the moment a frame (motion, step, activity transition) enters the
	 * FIFO, rather than batching it for BHI360_REPORT_LATENCY_MS and depending on
	 * the byte watermark or the periodic software drain to surface it. */
	{bhi360_activity_sensor_AnyMotionLowPower, 1.0f, 0},
	{bhi360_activity_sensor_NoMotionLowPowerWakeup, 1.0f, 0},
	{bhi360_activity_sensor_WristGestureDetectLowPowerWakeup, 1.0f, 0},
	{bhi360_activity_sensor_WristWearLowPowerWakeup, 1.0f, 0},
	{bhi360_activity_sensor_WearRecognitionWakeup, 1.0f, 0},
	{bhi360_activity_sensor_StepCounterLowPower, 1.0f, 0},
	{bhi360_activity_sensor_StepDetectorLowPower, 1.0f, 0},
};

/**
 * @brief Post a BHI360 event from thread context.
 * @details Wraps device_driver_event_post() with the generated BHI360 device ID.
 */
static void bhi360_post_event(enum bhi360_event_type event, uint32_t v_param, void* p_param) {
	(void) device_driver_event_post(BHI360_DEVICE_DTS_ID,
									(uint32_t) event,
									v_param,
									(uintptr_t) p_param,
									K_NO_WAIT);
}

/**
 * @brief Post a BHI360 event from ISR context.
 * @details Used exclusively by the GPIO interrupt callback to publish the raw
 *          IRQ event without parsing FIFO contents inside the ISR.
 */
static void bhi360_post_event_isr(enum bhi360_event_type event, uint32_t v_param) {
	(void) device_driver_event_post_isr(BHI360_DEVICE_DTS_ID, (uint32_t) event, v_param, 0U);
}

/**
 * @brief Enable every sensor listed in a descriptor table.
 * @details For each available virtual sensor, this registers the parser callback
 *          and programs the requested sample rate and latency.
 */
static void bhi360_enable_sensor_table(struct bhi360_t* dev,
									   const struct bhi360_sensor_enable* sensors,
									   size_t count) {
	for (size_t i = 0; i < count; i++) {
		const struct bhi360_sensor_enable* sensor = &sensors[i];

		if (!bhy2_is_sensor_available(sensor->sensor_id, &dev->bhy2)) {
			LOG_WRN("%s: %s (id %u) is not available", dev->name, sensor->name, sensor->sensor_id);
			continue;
		}

		int8_t rslt =
			bhy2_register_fifo_parse_callback(sensor->sensor_id, sensor->parser, dev, &dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt != BHY2_OK) {
			continue;
		}

		rslt = bhy2_set_virt_sensor_cfg(sensor->sensor_id,
										sensor->sample_rate_hz,
										sensor->latency_ms,
										&dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt == BHY2_OK) {
			LOG_INF("%s: enabled %s (id %u) at %.2f Hz",
					dev->name,
					sensor->name,
					sensor->sensor_id,
					(double) sensor->sample_rate_hz);
		}
	}
}

static const struct bhi360_activity_sensor_descriptor* bhi360_activity_sensor_descriptor(
	enum bhi360_activity_sensor_type sensor_type) {
	for (size_t i = 0; i < ARRAY_SIZE(bhi360_activity_sensor_descriptors); i++) {
		if (bhi360_activity_sensor_descriptors[i].sensor == sensor_type) {
			return &bhi360_activity_sensor_descriptors[i];
		}
	}

	return NULL;
}

static bool bhi360_enable_activity_sensors(struct bhi360_t* dev,
										   const struct bhi360_activity_sensor_config_t* sensors,
										   size_t count) {
	if (count > 0U && sensors == NULL) {
		LOG_ERR("%s: activity sensor config list is NULL", dev->name);
		return false;
	}

	for (size_t i = 0; i < count; i++) {
		const struct bhi360_activity_sensor_config_t* config = &sensors[i];
		const struct bhi360_activity_sensor_descriptor* sensor =
			bhi360_activity_sensor_descriptor(config->sensor);

		if (sensor == NULL) {
			LOG_ERR("%s: unsupported activity sensor %d", dev->name, (int) config->sensor);
			return false;
		}
		if (!bhy2_is_sensor_available(sensor->sensor_id, &dev->bhy2)) {
			LOG_WRN("%s: %s (id %u) is not available", dev->name, sensor->name, sensor->sensor_id);
			continue;
		}

		int8_t rslt =
			bhy2_register_fifo_parse_callback(sensor->sensor_id, sensor->parser, dev, &dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt != BHY2_OK) {
			return false;
		}

		rslt = bhy2_set_virt_sensor_cfg(sensor->sensor_id,
										(bhy2_float) config->sample_rate_hz,
										config->latency_ms,
										&dev->bhy2);
		print_api_error(rslt, &dev->bhy2);
		if (rslt != BHY2_OK) {
			return false;
		}

		LOG_INF("%s: enabled %s (id %u) at %.2f Hz",
				dev->name,
				sensor->name,
				sensor->sensor_id,
				(double) config->sample_rate_hz);
	}

	return true;
}

/**
 * @brief Disable every sensor listed in a descriptor table.
 * @details Sensor output rate is set to zero and each FIFO stream is flushed.
 */
static void bhi360_disable_sensor_table(struct bhi360_t* dev,
										const struct bhi360_sensor_enable* sensors,
										size_t count) {
	for (size_t i = 0; i < count; i++) {
		(void) bhy2_set_virt_sensor_cfg(sensors[i].sensor_id, 0.0f, 0, &dev->bhy2);
		(void) bhy2_flush_fifo(sensors[i].sensor_id, &dev->bhy2);
	}
	ARG_UNUSED(dev);
}

static void bhi360_disable_activity_sensors(struct bhi360_t* dev) {
	for (size_t i = 0; i < ARRAY_SIZE(bhi360_activity_sensor_descriptors); i++) {
		const struct bhi360_activity_sensor_descriptor* sensor =
			&bhi360_activity_sensor_descriptors[i];

		(void) bhy2_set_virt_sensor_cfg(sensor->sensor_id, 0.0f, 0, &dev->bhy2);
		(void) bhy2_flush_fifo(sensor->sensor_id, &dev->bhy2);
	}
}

/**
 * @brief Log a BHY2 API failure with interface context when available.
 */
static void print_api_error(int8_t rslt, struct bhy2_dev* dev) {
	if (rslt != BHY2_OK) {
		LOG_ERR("API error: %s (%d)", bhi360_api_get_error(rslt), rslt);
		if ((rslt == BHY2_E_IO) && (dev != NULL)) {
			LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
			dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
		}
	}
}

/**
 * @brief Upload the selected BHI360 firmware image.
 * @details The firmware is streamed in BHY2-compatible chunks rounded up to a
 *          four-byte boundary. Depending on the compile-time option it targets
 *          RAM or flash upload helpers.
 */
static int8_t upload_firmware(struct bhy2_dev* dev) {
	uint32_t incr = 256;
	uint32_t len = sizeof(bhy2_firmware_image);
	int8_t rslt = BHY2_OK;

	if ((incr % 4) != 0) {
		incr = ((incr >> 2) + 1) << 2;
	}

	for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr) {
		if (incr > (len - i)) {
			incr = len - i;
			if ((incr % 4) != 0) {
				incr = ((incr >> 2) + 1) << 2;
			}
		}

#ifdef UPLOAD_FIRMWARE_TO_FLASH
		rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
		rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif

		LOG_INF("%.2f%% complete", (double) (i + incr) / (double) len * 100.0);
	}

	return rslt;
}

/**
 * @brief GPIO callback that translates hardware IRQ edges into queue events.
 * @details No SPI/FIFO work is performed here; only the pin bitmap is forwarded
 *          to the central driver-event queue as bhi360_event_Irq. The consumer
 *          calls bhi360_irq_handler() from its own thread context.
 */
static void bhi360_irq_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	bhi360.last_irq_timestamp = rtc_get_timestamp_us();
	bhi360_post_event_isr(bhi360_event_Irq, pins);
}

static void bhi360_phy_sensor_stream_timer_callback(struct k_timer* timer) {
	ARG_UNUSED(timer);

	bhi360_post_event_isr(bhi360_event_Irq, 0);
}

/**
 * @brief Configure and register the hardware IRQ callback.
 */
static int bhi360_irq_init(struct bhi360_t* imu) {
	if (imu->state.bits.irq_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&imu->irq_gpio)) {
		LOG_ERR("BHI360 IRQ GPIO not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&imu->irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&imu->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure BHI360 IRQ interrupt (%d)", ret);
		return ret;
	}

	gpio_init_callback(&imu->irq_cb, bhi360_irq_callback, BIT(imu->irq_gpio.pin));
	ret = gpio_add_callback(imu->irq_gpio.port, &imu->irq_cb);
	if (ret != 0) {
		LOG_ERR("Failed to add BHI360 IRQ callback (%d)", ret);
		return ret;
	}

	imu->state.bits.irq_ready = 1U;
	return 0;
}

static int bhi360_init_fifo_timer(void) {
	if (bhi360.state.bits.initialized != 0U) {
		return 0;
	}
	if (bhi360.state.bits.configured != 0U) {
		return -EINVAL;
	}
	bhi360.enable_phy_sensor_streams = false;
	bhi360.timer_running = false;
	bhi360.timer_period_ms = UINT32_MAX;
	k_timer_init(&bhi360.fifo_timer,
				 bhi360_phy_sensor_stream_timer_callback,
				 NULL); /* stop callback optional */
	return 0;
}
/**
 * @brief BHY2 transport hook for SPI register reads.
 * @details Shared SPI ownership is acquired for the transaction, the read bit
 *          is applied to the register address, and the data phase is issued as
 *          a separate SPI read.
 */
static int8_t bhi360_spi_read(uint8_t reg_addr,
							  uint8_t* reg_data,
							  uint32_t length,
							  void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || (reg_data == NULL) || (length == 0U)) {
		return BHY2_E_IO;
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = (uint8_t) (reg_addr | 0x80);

	const struct spi_buf tx_buf = {
		.buf = &tx_cmd,
		.len = sizeof(tx_cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	const struct spi_buf rx_buf = {
		.buf = reg_data,
		.len = length,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	if (ret == 0) {
		ret = sys_spi_read(&imu->spi, &rx);
	}
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO;
	}

	return BHY2_INTF_RET_SUCCESS;
}

/**
 * @brief BHY2 transport hook for SPI register writes.
 * @details Shared SPI ownership is acquired for the transaction and the write
 *          command byte and payload are issued in one transfer.
 */
static int8_t bhi360_spi_write(uint8_t reg_addr,
							   const uint8_t* reg_data,
							   uint32_t length,
							   void* intf_ptr) {
	struct bhi360_t* imu = (struct bhi360_t*) intf_ptr;

	if ((imu == NULL) || ((reg_data == NULL) && (length > 0U))) {
		return BHY2_E_IO;
	}

	if (!imu->state.bits.initialized) {
		return BHY2_E_IO;
	}

	if (length > BHY2_RD_WR_LEN) {
		return BHY2_E_IO;
	}

	int ret = sys_spi_lock(&imu->spi, BHI360_SPI_TIMEOUT);
	if (ret != 0) {
		return BHY2_E_IO;
	}

	if (!sys_spi_is_ready(&imu->spi)) {
		(void) sys_spi_unlock(&imu->spi);
		return BHY2_E_IO;
	}

	uint8_t tx_cmd = reg_addr;
	struct spi_buf tx_bufs[2] = {
		{
			.buf = &tx_cmd,
			.len = sizeof(tx_cmd),
		},
		{
			.buf = (void*) reg_data,
			.len = length,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = (length > 0U) ? 2U : 1U,
	};

	bhi360_cs_select(imu);
	ret = sys_spi_write(&imu->spi, &tx);
	bhi360_cs_deselect(imu);

	int unlock_ret = sys_spi_unlock(&imu->spi);
	if (ret == 0) {
		ret = unlock_ret;
	}

	if (ret != 0) {
		return BHY2_E_IO;
	}

	return BHY2_INTF_RET_SUCCESS;
}

/**
 * @brief BHY2 timing hook.
 * @details The Bosch host library uses this callback when it needs short delays
 *          during boot, probing, or register synchronization.
 */
static void bhi360_delay_us(uint32_t period_us, void* intf_ptr) {
	ARG_UNUSED(intf_ptr);
	k_usleep(period_us);
}

const struct gpio_dt_spec* bhi360_get_gpio0(void) {
	return &bhi360.gpio0;
}

const struct gpio_dt_spec* bhi360_get_gpio1(void) {
	return &bhi360.gpio1;
}

const char* bhi360_event_name(enum bhi360_event_type event_id, uint32_t v_param) {
	switch (event_id) {
	case bhi360_event_Pedometer:
		return bhi360_pedometer_event_name((enum bhi360_pedometer_event_type) v_param);
	case bhi360_event_Gesture:
		return bhi360_gesture_event_name((enum bhi360_gesture_event_type)((v_param >> 8) & 0xFFU));
	case bhi360_event_Activity: {
		const uint16_t transition = (uint16_t) (v_param & 0xFFFFU);
		const char* transition_name;

		if ((transition != 0U) && ((transition & (transition - 1U)) == 0U)) {
			transition_name =
				bhi360_activity_transition_name((enum bhi360_activity_transition_type) transition);
			if (transition_name != NULL) {
				return transition_name;
			}
		}

		return bhi360_activity_event_name(
			(enum bhi360_activity_event_type)((v_param >> 16) & 0xFFU));
	}
	case bhi360_event_MetaEvent:
		return bhi360_meta_event_name((enum bhi360_meta_event_type)((v_param >> 16) & 0xFFU));
	default:
		return bhi360_base_event_name(event_id);
	}
}

bool bhi360_init(void) {
	struct bhi360_t* imu = &bhi360;
	int ret;

	if (imu->state.bits.initialized) {
		return true;
	}

	k_mutex_init(&imu->lock);

	LOG_INF("%s: Starting initialization", imu->name);

	if (!sys_spi_is_ready(&imu->spi)) {
		LOG_ERR("%s: SYS_SPI device not ready", imu->name);
		return false;
	}

	if (!gpio_is_ready_dt(&imu->cs_gpio)) {
		LOG_ERR("%s: chip-select GPIO not ready", imu->name);
		return false;
	}

	if (!gpio_is_ready_dt(&imu->reset_gpio)) {
		LOG_ERR("%s: reset GPIO not ready", imu->name);
		return false;
	}

	ret = gpio_pin_configure_dt(&imu->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: failed to configure chip-select GPIO (%d)", imu->name, ret);
		return false;
	}

	ret = gpio_pin_configure_dt(&imu->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: failed to configure reset GPIO (%d)", imu->name, ret);
		return false;
	}

	ret = bhi360_irq_init(imu);
	if (ret != 0) {
		return false;
	}

	ret = bhi360_init_fifo_timer();
	if (ret != 0) {
		return false;
	}

	imu->state.bits.initialized = 1U;
	return true;
}

bool bhi360_is_ready(void) {
	return bhi360.state.bits.initialized != 0U && bhi360.state.bits.configured != 0U;
}

void bhi360_get_default_config(struct bhi360_config_t* config) {
	if (config == NULL) {
		return;
	}

	config->activity_sensors = bhi360_default_activity_sensors;
	config->activity_sensor_count = ARRAY_SIZE(bhi360_default_activity_sensors);
}

bool bhi360_config(const struct bhi360_config_t* config) {
	struct bhi360_t* imu = &bhi360;
	struct bhi360_config_t default_config;
	int8_t rslt;
	uint8_t product_id = 0;
	uint16_t version = 0;
	uint8_t hif_ctrl;
	uint8_t boot_status;

	if (imu->state.bits.configured) {
		return true;
	}

	if (!bhi360_init()) {
		return false;
	}
	if (config == NULL) {
		bhi360_get_default_config(&default_config);
		config = &default_config;
	}

	LOG_INF("%s: Starting configuration", imu->name);

	k_sleep(K_USEC(1));

	rslt = bhy2_init(BHY2_SPI_INTERFACE,
					 bhi360_spi_read,
					 bhi360_spi_write,
					 bhi360_delay_us,
					 BHY2_RD_WR_LEN,
					 imu,
					 &imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Initialization failed", imu->name);
		return false;
	}

	rslt = bhy2_soft_reset(&imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Soft reset failed with code %d", imu->name, rslt);
		return false;
	}

	bool id_read_success = false;
	imu->state.bits.probed = 1U;
	for (int retry = 0; retry < 20; retry++) {
		rslt = bhy2_get_product_id(&product_id, &imu->bhy2);
		if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID) {
			LOG_INF("%s: Product ID verified on attempt %d", imu->name, retry + 1);
			id_read_success = true;
			imu->state.bits.device_found = 1U;
			break;
		}
		k_msleep(10);
	}

	if (!id_read_success) {
		LOG_ERR("%s: Failed to verify product ID", imu->name);
		return false;
	}

	rslt = bhy2_get_boot_status(&boot_status, &imu->bhy2);
	if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
		LOG_ERR("%s: Host interface not ready", imu->name);
		return false;
	}

	LOG_INF("%s: Uploading firmware", imu->name);
	rslt = upload_firmware(&imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Firmware upload failed", imu->name);
		return false;
	}

	rslt = bhy2_boot_from_ram(&imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	rslt = bhy2_get_kernel_version(&version, &imu->bhy2);
	if (rslt != BHY2_OK || version == 0U) {
		LOG_ERR("%s: Boot failed", imu->name);
		return false;
	}
	LOG_INF("%s: Boot successful, kernel version %u", imu->name, version);

	if (!bhi360_configure_host_interrupts(imu)) {
		return false;
	}

	hif_ctrl = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	/* This populates dev->event_size[], which the FIFO parser uses to advance past
	 * each frame. If it fails, the table stays zero and the parser cannot advance
	 * past a meta-event frame (read position never moves), spinning and flooding
	 * "Invalid meta event size 0". Treat a failure as fatal for configuration. */
	LOG_INF("%s: Updating virtual sensor list", imu->name);
	rslt = bhy2_update_virtual_sensor_list(&imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		LOG_ERR("%s: Failed to update virtual sensor list", imu->name);
		return false;
	}

	hif_ctrl = 0U;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	if (!bhi360_configure_fifo_watermark(imu)) {
		return false;
	}

	hif_ctrl = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	LOG_INF("%s: Registering meta-event callbacks", imu->name);
	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT,
											 parse_meta_event,
											 imu,
											 &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU,
											 parse_meta_event,
											 imu,
											 &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	/* FIFO control is configured above before streaming starts. Do not query it
	 * again while sensors are running; the parameter exchange shares the status
	 * channel with async firmware status traffic. */

	if (!bhi360_enable_activity_sensors(imu,
										config->activity_sensors,
										config->activity_sensor_count)) {
		return false;
	}
	bhi360.enable_phy_sensor_streams = false;

	/* Establish the host/sensor timestamp mapping here so the per-IRQ hot path
	 * only re-syncs it every BHI360_TIME_SYNC_PERIOD_US. Retries past the first
	 * miss; if it still fails the offset stays unset and the next IRQ retries. */
	bhi360_init_sensor_time_offset(imu);

	imu->state.bits.configured = 1U;
	LOG_INF("%s: Configuration complete", imu->name);
	return true;
}

int bhi360_start_phy_sensor_streams(uint32_t period_ms) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}
	if (bhi360.enable_phy_sensor_streams) {
		return -EBUSY;
	}
	if (bhi360.timer_running && bhi360.timer_period_ms != period_ms) {
		return -EBUSY;
	}
	k_mutex_lock(&bhi360.lock, K_FOREVER);

	bhi360_enable_sensor_table(&bhi360, bhi360_phy_sensors, ARRAY_SIZE(bhi360_phy_sensors));
	if (!bhi360.timer_running) {
		k_timer_start(&bhi360.fifo_timer, K_MSEC(period_ms), K_MSEC(period_ms));
		bhi360.enable_phy_sensor_streams = true;
		bhi360.timer_running = true;
		bhi360.timer_period_ms = period_ms;
	}
	k_mutex_unlock(&bhi360.lock);
	return 0;
}

int bhi360_stop_phy_sensor_streams(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}
	if (!bhi360.enable_phy_sensor_streams) {
		return -EINVAL;
	}
	k_mutex_lock(&bhi360.lock, K_FOREVER);
	k_timer_stop(&bhi360.fifo_timer);
	bhi360_disable_sensor_table(&bhi360, bhi360_phy_sensors, ARRAY_SIZE(bhi360_phy_sensors));
	bhi360.enable_phy_sensor_streams = false;
	bhi360.timer_running = false;
	bhi360.timer_period_ms = UINT32_MAX;
	bhi360_post_event(bhi360_event_Irq, 0, 0);
	k_mutex_unlock(&bhi360.lock);
	return 0;
}

int bhi360_start_periodic_timer(uint32_t period_ms) {
	assert(period_ms > 0 && period_ms < UINT32_MAX);
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}
	if (bhi360.timer_running) {
		return (bhi360.timer_period_ms == period_ms) ? 0 : -EBUSY;
	}
	k_mutex_lock(&bhi360.lock, K_FOREVER);
	k_timer_start(&bhi360.fifo_timer, K_MSEC(period_ms), K_MSEC(period_ms));
	bhi360.timer_running = true;
	bhi360.timer_period_ms = period_ms;
	bhi360_post_event(bhi360_event_Irq, 0, 0);
	k_mutex_unlock(&bhi360.lock);
	return 0;
}

int bhi360_stop_periodic_timer(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}
	if (!bhi360.timer_running) {
		return -EINVAL;
	}
	k_mutex_lock(&bhi360.lock, K_FOREVER);
	k_timer_stop(&bhi360.fifo_timer);
	bhi360.timer_running = false;
	bhi360.timer_period_ms = UINT32_MAX;
	if (bhi360.enable_phy_sensor_streams) {
		bhi360_disable_sensor_table(&bhi360, bhi360_phy_sensors, ARRAY_SIZE(bhi360_phy_sensors));
		bhi360.enable_phy_sensor_streams = false;
	}
	bhi360_post_event(bhi360_event_Irq, 0, 0);
	k_mutex_unlock(&bhi360.lock);
	return 0;
}

int bhi360_irq_handler(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}

	/* Hold the cache lock across the whole drain so the high-rate parsers can
	 * refill the per-drain arrays without a copy-out reader observing a partial
	 * batch. Start a fresh batch by resetting the counts before draining. */
	k_mutex_lock(&bhi360.lock, K_FOREVER);

	bhi360_maybe_sync_sensor_time_offset(&bhi360);

	bhi360.quat_count = 0;
	bhi360.lacc_count = 0;
	bhi360.gyro_count = 0;

	bhi360.quat_skipped_count = 0;
	bhi360.lacc_skipped_count = 0;
	bhi360.gyro_skipped_count = 0;
	LOG_DBG("bhi360_irq_handler() starting processing... ");
	/* BHY2 reads INT_STATUS internally; a separate pre-read can consume FIFO cause bits. */
	int8_t rslt =
		bhy2_get_and_process_fifo(bhi360.work_buffer, sizeof(bhi360.work_buffer), &bhi360.bhy2);

	/* Snapshot the per-stream counts collected during this drain, then release the
	 * lock before posting so the queue operations run unlocked. */
	size_t quat_count = bhi360.quat_count;
	size_t lacc_count = bhi360.lacc_count;
	size_t gyro_count = bhi360.gyro_count;

	k_mutex_unlock(&bhi360.lock);

	print_api_error(rslt, &bhi360.bhy2);
	if (rslt != BHY2_OK) {
		return -EIO;
	}

	/* Post one summary event per high-rate stream that produced samples this drain.
	 * v_param carries the number of samples; p_param points at the first element of
	 * the driver-owned array, which stays valid until the next drain. */
	if (quat_count > 0U) {
		bhi360_post_event(bhi360_event_QuaternionBatch, (uint32_t) quat_count, bhi360.quat_data);
	}
	if (lacc_count > 0U) {
		bhi360_post_event(bhi360_event_LinearAccelerationBatch,
						  (uint32_t) lacc_count,
						  bhi360.lacc_data);
	}
	if (gyro_count > 0U) {
		bhi360_post_event(bhi360_event_GyroBatch, (uint32_t) gyro_count, bhi360.gyro_data);
	}

	if (bhi360.quat_skipped_count > 0) {
		LOG_WRN("Quaternion sample dropped: batch exceeded %u samples. %u skipped",
				BHI360_MAX_SAMPLES_PER_IRQ,
				bhi360.quat_skipped_count);
	}
	if (bhi360.lacc_skipped_count > 0) {
		LOG_WRN("Linear acceleration sample dropped: batch exceeded %u samples. %u skipped",
				BHI360_MAX_SAMPLES_PER_IRQ,
				bhi360.lacc_skipped_count);
	}
	if (bhi360.gyro_skipped_count > 0) {
		LOG_WRN("Gyro sample dropped: batch exceeded %u samples. %u skipped",
				BHI360_MAX_SAMPLES_PER_IRQ,
				bhi360.gyro_skipped_count);
	}
	LOG_DBG("bhi360_irq_handler() finished processing. ");
	return 0;
}

/**
 * @brief Copy a cached high-rate stream out under the driver lock.
 * @details Reads the sample count and array contents while holding the cache
 *          mutex so a concurrent FIFO drain cannot publish a partial batch. The
 *          count is read inside the lock to stay consistent with the data.
 * @param array Base address of the cached sample array.
 * @param count_ptr Pointer to the live sample count for @p array.
 * @param elem_size Size of one sample in bytes.
 * @param out Destination buffer for up to @p max_samples elements.
 * @param max_samples Capacity of @p out in elements.
 * @return Number of samples copied (>= 0), or a negative errno on error.
 */
static int bhi360_copy_stream(const void* array,
							  const size_t* count_ptr,
							  size_t elem_size,
							  void* out,
							  size_t max_samples) {
	if ((out == NULL) || (max_samples == 0U)) {
		return -EINVAL;
	}

	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}

	k_mutex_lock(&bhi360.lock, K_FOREVER);
	size_t count = MIN(*count_ptr, max_samples);
	if (count > 0U) {
		memcpy(out, array, count * elem_size);
	}
	k_mutex_unlock(&bhi360.lock);

	return (int) count;
}

int bhi360_copy_quaternion(struct bhi360_quat_data_t* out, size_t max_samples) {
	return bhi360_copy_stream(bhi360.quat_data,
							  &bhi360.quat_count,
							  sizeof(bhi360.quat_data[0]),
							  out,
							  max_samples);
}

int bhi360_copy_linear_acceleration(struct bhi360_lacc_data_t* out, size_t max_samples) {
	return bhi360_copy_stream(bhi360.lacc_data,
							  &bhi360.lacc_count,
							  sizeof(bhi360.lacc_data[0]),
							  out,
							  max_samples);
}

int bhi360_copy_gyro(struct bhi360_gyro_data_t* out, size_t max_samples) {
	return bhi360_copy_stream(bhi360.gyro_data,
							  &bhi360.gyro_count,
							  sizeof(bhi360.gyro_data[0]),
							  out,
							  max_samples);
}

/**
 * @brief Read the firmware FIFO control parameter into a status snapshot.
 * @details The FIFO control read is a status-channel parameter exchange. The
 *          caller must ensure the status channel has been drained first (a FIFO
 *          drain does this) so no async status message larger than the library's
 *          internal buffer is pending, otherwise the read returns BHY2_E_BUFFER.
 */
static int bhi360_read_fifo_status(struct bhi360_t* imu, struct bhi360_fifo_status* out) {
	uint32_t fifo_ctrl[4] = {0};
	int8_t rslt = bhy2_get_fifo_ctrl(fifo_ctrl, &imu->bhy2);
	if (rslt != BHY2_OK) {
		print_api_error(rslt, &imu->bhy2);
		return -EIO;
	}

	out->wakeup_watermark = fifo_ctrl[0];
	out->wakeup_size = fifo_ctrl[1];
	out->nonwakeup_watermark = fifo_ctrl[2];
	out->nonwakeup_size = fifo_ctrl[3];
	return 0;
}

int bhi360_get_fifo_status(struct bhi360_fifo_status* out) {
	if (out == NULL) {
		return -EINVAL;
	}

	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}

	/* Serialize the status-channel exchange against a concurrent FIFO drain;
	 * both touch the shared BHY2 device context. */
	k_mutex_lock(&bhi360.lock, K_FOREVER);
	int ret = bhi360_read_fifo_status(&bhi360, out);
	k_mutex_unlock(&bhi360.lock);

	return ret;
}

void bhi360_stop(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return;
	}

	bhi360_disable_sensor_table(&bhi360, bhi360_phy_sensors, ARRAY_SIZE(bhi360_phy_sensors));
	bhi360_disable_activity_sensors(&bhi360);
	(void) bhy2_soft_reset(&bhi360.bhy2);
	/* Sensor time restarts on soft reset; drop the offset so reconfigure re-syncs. */
	bhi360.sensor_time_offset = BHI360_TIME_OFFSET_UNSET;
	bhi360.last_time_sync_us = 0;
	bhi360.state.bits.configured = 0U;
}

static inline time_t bhy2_timestamp_to_elapsed_us(uint64_t bhy2_timestamp) {
	/* BHY2 FIFO timestamps are raw 15.625 us ticks since sensor boot.
	 * Convert them to elapsed microseconds since the BHI360 firmware boot.
	 * These are not Unix-epoch timestamps.
	 */
	return (time_t) ((bhy2_timestamp * UINT64_C(15625)) / UINT64_C(1000));
}

/**
 * @brief Attempt one host-minus-sensor timestamp sync.
 * @details Reads the BHI360 hardware timestamp latched at the most recent host
 *          interrupt (BHY2_REG_HOST_INTR_TIME) and, on success, stores the
 *          difference from @p now_us as host time minus sensor time. This is a
 *          plain register read: unlike bhy2_hif_req_and_get_hw_timestamp() it does
 *          not request a fresh timestamp event and poll ~50 us for the register to
 *          change, so it cannot spuriously return BHY2_E_TIMEOUT. Called at IRQ
 *          entry, where the triggering interrupt has just latched a fresh value.
 *          The attempt is stamped regardless of outcome so a failed read does not
 *          re-fire on every IRQ; an offset that has never been established is
 *          retried on the next IRQ instead. Returns the BHY2 result without logging
 *          so callers can choose whether to log.
 *
 * @param dev    Driver instance.
 * @param now_us RTC time in microseconds sampled just before the read.
 * @return The BHY2 API result; ::BHY2_OK on success.
 */
static int8_t bhi360_try_sync_sensor_time_offset(struct bhi360_t* dev, time_t now_us) {
	uint64_t sensor_timestamp = 0;
	int8_t rslt = bhy2_hif_get_hw_timestamp(&sensor_timestamp, &dev->bhy2.hif);

	dev->last_time_sync_us = now_us;

	if (rslt == BHY2_OK) {
		dev->sensor_time_offset = now_us - bhy2_timestamp_to_elapsed_us(sensor_timestamp);
	}
	return rslt;
}

/**
 * @brief Establish the timestamp offset at config.
 * @details At config there may not have been a host interrupt yet, so the latched
 *          HOST_INTR_TIME could be stale. Request a single timestamp event, let it
 *          propagate, then read it back. This avoids bhy2_hif_req_and_get_hw_timestamp(),
 *          whose ~50 us in-call poll for the register to change reliably misses
 *          right after config (HOST_INTR_TIME has not updated yet, so *ts == ts_old)
 *          and returns BHY2_E_TIMEOUT. Failure is not fatal: the offset stays unset
 *          and the first IRQ re-syncs against its freshly latched timestamp.
 */
static void bhi360_init_sensor_time_offset(struct bhi360_t* dev) {
	(void) bhy2_set_timestamp_event_req(1, &dev->bhy2);
	k_msleep(BHI360_TIME_SYNC_INIT_DELAY_MS);

	int8_t rslt = bhi360_try_sync_sensor_time_offset(dev, rtc_get_timestamp_us());

	if (rslt != BHY2_OK) {
		LOG_WRN("%s: initial time sync failed (%d); will retry on first IRQ", dev->name, rslt);
	}
}

/**
 * @brief Re-sync the timestamp offset only when stale, off the IRQ hot path.
 * @details Skips the slow, timeout-prone hardware-timestamp request unless the
 *          offset has never been established, or the last sync is older than
 *          ::BHI360_TIME_SYNC_PERIOD_US. Called at the top of each FIFO drain.
 */
static void bhi360_maybe_sync_sensor_time_offset(struct bhi360_t* dev) {
	time_t now_us = rtc_get_timestamp_us();

	if (dev->sensor_time_offset != BHI360_TIME_OFFSET_UNSET &&
		(now_us - dev->last_time_sync_us) < BHI360_TIME_SYNC_PERIOD_US) {
		return;
	}

	int8_t rslt = bhi360_try_sync_sensor_time_offset(dev, now_us);

	if (rslt != BHY2_OK) {
		print_api_error(rslt, &dev->bhy2);
	}
}

/**
 * @brief Parse a rotation-vector FIFO packet into the per-drain quaternion array.
 * @details Samples are only collected here; no event is posted per sample.
 *          bhi360_irq_handler() posts a single bhi360_event_QuaternionBatch once
 *          the drain finishes, carrying the collected count in v_param.
 */
static void parse_quaternion(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_quaternion data;

	if (callback_info->data_size != 11) {
		LOG_ERR("Invalid data size: %d", callback_info->data_size);
		return;
	}

	if (dev->quat_count >= BHI360_MAX_SAMPLES_PER_IRQ) {
		bhi360.quat_skipped_count++;
		return;
	}

	bhy2_parse_quaternion(callback_info->data_ptr, &data);

	struct bhi360_quat_data_t* slot = &dev->quat_data[dev->quat_count++];
	slot->x = data.x;
	slot->y = data.y;
	slot->z = data.z;
	slot->w = data.w;
	slot->accuracy = data.accuracy;
	slot->timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
}

/**
 * @brief Parse a linear-acceleration FIFO packet into the per-drain array.
 * @details Samples are only collected here; no event is posted per sample.
 *          bhi360_irq_handler() posts a single bhi360_event_LinearAccelerationBatch
 *          once the drain finishes, carrying the collected count in v_param.
 */
static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info* callback_info,
									  void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;

	if (dev->lacc_count >= BHI360_MAX_SAMPLES_PER_IRQ) {
		bhi360.lacc_skipped_count++;
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	struct bhi360_lacc_data_t* slot = &dev->lacc_data[dev->lacc_count++];
	slot->x = data.x;
	slot->y = data.y;
	slot->z = data.z;
	slot->timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
}

/**
 * @brief Parse a gyroscope FIFO packet into the per-drain gyroscope array.
 * @details Samples are only collected here; no event is posted per sample.
 *          bhi360_irq_handler() posts a single bhi360_event_GyroBatch once the
 *          drain finishes, carrying the collected count in v_param.
 */
static void parse_gyro(const struct bhy2_fifo_parse_data_info* callback_info, void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;

	if (dev->gyro_count >= BHI360_MAX_SAMPLES_PER_IRQ) {
		bhi360.gyro_skipped_count++;
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	struct bhi360_gyro_data_t* slot = &dev->gyro_data[dev->gyro_count++];
	slot->x = data.x;
	slot->y = data.y;
	slot->z = data.z;
	slot->timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
}

/**
 * @brief Parse scalar classifier outputs used for gestures and step detectors.
 * @details Step-detector sensor IDs are normalized into bhi360_event_Pedometer
 *          with step_detected set true. All other scalar outputs are emitted as
 *          bhi360_event_Gesture with v_param packed as `(sensor_id << 8) | value`.
 */
static void parse_scalar_event(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	uint8_t value = (callback_info->data_size > 0) ? callback_info->data_ptr[0] : 1U;
	uint32_t event_value = ((uint32_t) callback_info->sensor_id << 8) | value;

	if ((callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP_WU)) {
		dev->pedometer_data.sensor_id = callback_info->sensor_id;
		dev->pedometer_data.step_detected = true;
		dev->pedometer_data.timestamp =
			bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
		bhi360_post_event(bhi360_event_Pedometer, callback_info->sensor_id, &dev->pedometer_data);
		return;
	}

	dev->gesture_data.sensor_id = callback_info->sensor_id;
	dev->gesture_data.value = value;
	dev->gesture_data.timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
	bhi360_post_event(bhi360_event_Gesture, event_value, &dev->gesture_data);
	LOG_INF("Gesture/event sensor id %u value 0x%02x", callback_info->sensor_id, value);
}

/**
 * @brief Parse a step-counter FIFO packet into pedometer state.
 * @details The firmware may emit one-, two-, or four-byte little-endian counts.
 *          The decoded count is posted as bhi360_event_Pedometer with
 *          step_detected cleared.
 */
static void parse_step_counter(const struct bhy2_fifo_parse_data_info* callback_info,
							   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	uint32_t count = 0;

	if (callback_info->data_size >= 4) {
		count = BHY2_LE2U32(callback_info->data_ptr);
	} else if (callback_info->data_size >= 2) {
		count = BHY2_LE2U16(callback_info->data_ptr);
	} else if (callback_info->data_size == 1) {
		count = callback_info->data_ptr[0];
	}

	dev->pedometer_data.sensor_id = callback_info->sensor_id;
	dev->pedometer_data.step_count = count;
	dev->pedometer_data.step_detected = false;
	dev->pedometer_data.timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
	bhi360_post_event(bhi360_event_Pedometer, callback_info->sensor_id, &dev->pedometer_data);
	LOG_INF("Step counter sensor id %u count %u", callback_info->sensor_id, count);
}

/**
 * @brief Parse an activity-recognition FIFO packet.
 * @details The activity payload is a 16-bit bitmask of start/end transitions.
 *          It is cached and posted as bhi360_event_Activity with v_param packed
 *          as `(sensor_id << 16) | activity`.
 */
static void parse_activity(const struct bhy2_fifo_parse_data_info* callback_info,
						   void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;

	if (callback_info->data_size < 2) {
		LOG_WRN("Activity event sensor id %u has invalid size %u",
				callback_info->sensor_id,
				callback_info->data_size);
		return;
	}

	uint16_t activity = BHY2_LE2U16(callback_info->data_ptr);
	dev->activity_data.sensor_id = callback_info->sensor_id;
	dev->activity_data.activity = activity;
	dev->activity_data.timestamp =
		bhy2_timestamp_to_elapsed_us(*(callback_info->time_stamp)) + bhi360.sensor_time_offset;
	bhi360_post_event(bhi360_event_Activity,
					  ((uint32_t) callback_info->sensor_id << 16) | activity,
					  &dev->activity_data);
	if (activity & BHY2_STILL_ACTIVITY_ENDED) {
		LOG_INF("Activity: still ended");
	}
	if (activity & BHY2_WALKING_ACTIVITY_ENDED) {
		LOG_INF("Activity: walking ended");
	}
	if (activity & BHY2_RUNNING_ACTIVITY_ENDED) {
		LOG_INF("Activity: running ended");
	}
	if (activity & BHY2_ON_BICYCLE_ACTIVITY_ENDED) {
		LOG_INF("Activity: bicycle ended");
	}
	if (activity & BHY2_IN_VEHICLE_ACTIVITY_ENDED) {
		LOG_INF("Activity: vehicle ended");
	}
	if (activity & BHY2_TILTING_ACTIVITY_ENDED) {
		LOG_INF("Activity: tilting ended");
	}
	if (activity & BHY2_STILL_ACTIVITY_STARTED) {
		LOG_INF("Activity: still started");
	}
	if (activity & BHY2_WALKING_ACTIVITY_STARTED) {
		LOG_INF("Activity: walking started");
	}
	if (activity & BHY2_RUNNING_ACTIVITY_STARTED) {
		LOG_INF("Activity: running started");
	}
	if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED) {
		LOG_INF("Activity: bicycle started");
	}
	if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED) {
		LOG_INF("Activity: vehicle started");
	}
	if (activity & BHY2_TILTING_ACTIVITY_STARTED) {
		LOG_INF("Activity: tilting started");
	}
}

/**
 * @brief Pack and publish a firmware meta-event.
 * @details Meta-events do not use p_param; instead `(type << 16) | (byte1 << 8)
 *          | byte2` is carried in v_param.
 */
static void bhi360_post_meta_event(uint8_t type, uint8_t byte1, uint8_t byte2) {
	bhi360_post_event(bhi360_event_MetaEvent,
					  ((uint32_t) type << 16) | ((uint32_t) byte1 << 8) | byte2,
					  NULL);
}

/**
 * @brief Decide whether a meta-event should be visible to the application.
 * @details Routine spacer and initialized meta packets can be emitted repeatedly
 *          by the firmware and are not actionable during normal streaming. Keep
 *          them out of the shared application event queue while preserving
 *          diagnostics such as errors, overflows, resets, status, and watermark
 *          notifications.
 */
static bool bhi360_should_publish_meta_event(uint8_t type) {
	switch (type) {
	case BHY2_META_EVENT_SPACER:
	case BHY2_META_EVENT_INITIALIZED:
		return false;
	default:
		return true;
	}
}

/**
 * @brief Parse and publish a BHY2 meta-event packet.
 * @details Both the regular and wake-up meta-event streams are routed here.
 *          The function validates the three-byte payload and publishes actionable
 *          packed bhi360_event_MetaEvent notifications for consumers.
 */
static void parse_meta_event(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref) {
	ARG_UNUSED(callback_ref);

	if (callback_info->data_size < 3) {
		LOG_WRN("Invalid meta event size %u", callback_info->data_size);
		return;
	}

	uint8_t meta_event_type = callback_info->data_ptr[0];
	uint8_t byte1 = callback_info->data_ptr[1];
	uint8_t byte2 = callback_info->data_ptr[2];

	if ((callback_info->sensor_id != BHY2_SYS_ID_META_EVENT) &&
		(callback_info->sensor_id != BHY2_SYS_ID_META_EVENT_WU)) {
		return;
	}

	if (!bhi360_should_publish_meta_event(meta_event_type)) {
		return;
	}

	bhi360_post_meta_event(meta_event_type, byte1, byte2);
}
