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
 * - bhi360_configure() binds the BHY2 transport hooks, probes the product,
 *   uploads firmware, discovers available virtual sensors, and enables the
 *   driver's default sensor sets.
 * - bhi360_irq_callback() posts bhi360_event_Irq from ISR context.
 * - bhi360_process_irq() reads interrupt status and drains the FIFO from thread
 *   context. BHY2 dispatches FIFO packets into the parser callbacks below.
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
 * - per-event cached sample structs whose addresses are published in p_param.
 *
 * Sample payload pointers remain valid until the next event of the same class is
 * parsed, because each event class reuses one cached struct inside the driver.
 * Consumers that need long-lived copies must duplicate the pointed-to data.
 *
 * @section bhi360_impl_enabled_sensors Enabled Sensors
 *
 * The default configuration enables three groups of virtual sensors:
 * - Base sensors: rotation vector, linear acceleration, gyroscope, step count,
 *   and step detector variants.
 * - Gesture sensors: wake, glance, pickup, wrist tilt, motion/stationary, and
 *   related low-power gesture-classifier outputs.
 * - Activity sensors: activity recognition and wear-aware activity recognition.
 *
 * Meta-event streams are also registered so firmware status notifications are
 * surfaced to the event system and logs.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdio.h>

#include "bhi3_defs.h"
#include "bhi360.h"
#include "bhi360_api_error.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "device_driver_dts_ids.h"
#include "device_driver_events.h"
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

/**
 * @brief Sensor configuration descriptor used during enable/disable passes.
 * @details Each entry binds a BHY2 virtual-sensor ID to a desired output rate,
 *          latency budget, a human-readable name for logging, and the FIFO
 *          parser callback that translates raw BHY2 packets into SenseWear
 *          driver events.
 */
struct bhi360_sensor_enable {
	/** @brief BHY2 sensor ID constant (e.g., BHY2_SENSOR_ID_RV, BHY2_SENSOR_ID_GYRO). */
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
	/** @brief Cached quaternion sample published through bhi360_event_Quaternion p_param. */
	struct bhi360_quat_data quat_data;
	/** @brief Cached linear-acceleration sample published through bhi360_event_LinearAcceleration p_param. */
	struct bhi360_lacc_data lacc_data;
	/** @brief Cached gyroscope sample published through bhi360_event_Gyro p_param. */
	struct bhi360_gyro_data gyro_data;
	/** @brief Cached pedometer sample published through bhi360_event_Pedometer p_param. */
	struct bhi360_pedometer_data pedometer_data;
	/** @brief Cached gesture sample published through bhi360_event_Gesture p_param. */
	struct bhi360_gesture_data gesture_data;
	/** @brief Cached activity sample published through bhi360_event_Activity p_param. */
	struct bhi360_activity_data activity_data;
} bhi360 = {
	.spi = SYS_SPI_DT_SPEC_GET(BHI360_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
	.cs_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, cs_gpios),
	.irq_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, int_gpios),
	.reset_gpio = GPIO_DT_SPEC_GET(BHI360_NODE, reset_gpios),
	.gpio0 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio0_gpios),
	.gpio1 = GPIO_DT_SPEC_GET(BHI360_NODE, gpio1_gpios),
	.name = "BHI360",
};

static const char* const bhi360_event_names[bhi360_event_Count] = {
	[bhi360_event_Irq] = "Irq",
	[bhi360_event_Quaternion] = "Quaternion",
	[bhi360_event_LinearAcceleration] = "LinearAcceleration",
	[bhi360_event_Gyro] = "Gyro",
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
 * @param event Single activity-transition bit decoded from bits 15:0 of bhi360_event_Activity v_param.
 * @return Constant parent-and-transition-name string, or NULL if the value is zero, multi-bit, or unsupported.
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

/**
 * @brief Base motion and pedometer sensors enabled by default.
 * @details These are the primary motion outputs exposed by the public API.
 */
static const struct bhi360_sensor_enable bhi360_base_sensors[] = {
	{BHY2_SENSOR_ID_RV, "Rotation vector", 100.0f, 0, parse_quaternion},
	{BHY2_SENSOR_ID_LACC, "Linear acceleration", 100.0f, 0, parse_linear_acceleration},
	{BHY2_SENSOR_ID_GYRO, "Gyroscope", 100.0f, 0, parse_gyro},
	{BHY2_SENSOR_ID_STC, "Step counter", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_WU, "Step counter wake-up", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_LP, "Step counter low-power", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STC_LP_WU, "Step counter low-power wake-up", 1.0f, 0, parse_step_counter},
	{BHY2_SENSOR_ID_STD, "Step detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_WU, "Step detector wake-up", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_LP, "Step detector low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STD_LP_WU, "Step detector low-power wake-up", 1.0f, 0, parse_scalar_event},
};

/**
 * @brief Gesture and motion-classifier sensors enabled by default.
 * @details All of these report through parse_scalar_event() and become
 *          bhi360_event_Gesture events.
 */
static const struct bhi360_sensor_enable bhi360_gesture_sensors[] = {
	{BHY2_SENSOR_ID_WAKE_GESTURE, "Wake gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_GLANCE_GESTURE, "Glance gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_PICKUP_GESTURE, "Pickup gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_WRIST_TILT_GESTURE, "Wrist tilt gesture", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_TILT_DETECTOR, "Tilt detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_STATIONARY_DET, "Stationary detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_MOTION_DET, "Motion detector", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG, "Significant motion", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG_LP, "Significant motion low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_SIG_LP_WU, "Significant motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_ANY_MOTION_LP, "Any motion low-power", 1.0f, 0, parse_scalar_event},
	{BHY2_SENSOR_ID_ANY_MOTION_LP_WU, "Any motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHI3_SENSOR_ID_NO_MOTION_LP_WU, "No motion low-power wake-up", 1.0f, 0, parse_scalar_event},
	{BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU,
	 "Wrist gesture detect low-power wake-up",
	 1.0f,
	 0,
	 parse_scalar_event},
	{BHI3_SENSOR_ID_WRIST_WEAR_LP_WU, "Wrist wear low-power wake-up", 1.0f, 0, parse_scalar_event},
};

/**
 * @brief Activity-recognition sensors enabled by default.
 * @details These produce packed activity bitfields and are published as
 *          bhi360_event_Activity.
 */
static const struct bhi360_sensor_enable bhi360_activity_sensors[] = {
	{BHY2_SENSOR_ID_AR, "Activity recognition", 1.0f, 0, parse_activity},
	{BHI3_SENSOR_ID_AR_WEAR_WU, "Wear activity recognition wake-up", 1.0f, 0, parse_activity},
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
 * @details No FIFO work is performed here; only the pin bitmap is forwarded to
 *          the central driver-event queue as bhi360_event_Irq.
 */
static void bhi360_irq_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	bhi360_post_event_isr(bhi360_event_Irq, pins);
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
		return bhi360_gesture_event_name((enum bhi360_gesture_event_type) ((v_param >> 8) & 0xFFU));
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
			(enum bhi360_activity_event_type) ((v_param >> 16) & 0xFFU));
	}
	case bhi360_event_MetaEvent:
		return bhi360_meta_event_name((enum bhi360_meta_event_type) ((v_param >> 16) & 0xFFU));
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

	imu->state.bits.initialized = 1U;
	return true;
}

bool bhi360_configure(void) {
	struct bhi360_t* imu = &bhi360;
	int8_t rslt;
	uint8_t product_id = 0;
	uint16_t version = 0;
	uint8_t hintr_ctrl;
	uint8_t hif_ctrl;
	uint8_t boot_status;

	if (imu->state.bits.configured) {
		return true;
	}

	if (!bhi360_init()) {
		return false;
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

	hintr_ctrl = BHY2_ICTL_ACTIVE_LOW | BHY2_ICTL_OPEN_DRAIN;
	rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
		return false;
	}

	hif_ctrl = BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);
	print_api_error(rslt, &imu->bhy2);
	if (rslt != BHY2_OK) {
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

	LOG_INF("%s: Updating virtual sensor list", imu->name);
	rslt = bhy2_update_virtual_sensor_list(&imu->bhy2);
	print_api_error(rslt, &imu->bhy2);

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

	bhi360_enable_sensor_table(imu, bhi360_base_sensors, ARRAY_SIZE(bhi360_base_sensors));
	bhi360_enable_sensor_table(imu, bhi360_gesture_sensors, ARRAY_SIZE(bhi360_gesture_sensors));
	bhi360_enable_sensor_table(imu, bhi360_activity_sensors, ARRAY_SIZE(bhi360_activity_sensors));

	imu->state.bits.configured = 1U;
	LOG_INF("%s: Configuration complete", imu->name);
	return true;
}

int bhi360_process_irq(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return -ENODEV;
	}

	uint8_t int_status = 0;
	int8_t rslt = bhy2_get_interrupt_status(&int_status, &bhi360.bhy2);
	print_api_error(rslt, &bhi360.bhy2);
	if (rslt != BHY2_OK) {
		return -EIO;
	}

	ARG_UNUSED(int_status);

	rslt = bhy2_get_and_process_fifo(bhi360.work_buffer, sizeof(bhi360.work_buffer), &bhi360.bhy2);
	print_api_error(rslt, &bhi360.bhy2);
	if (rslt != BHY2_OK) {
		return -EIO;
	}

	return 0;
}

void bhi360_stop(void) {
	if (!bhi360.state.bits.initialized || !bhi360.state.bits.configured) {
		return;
	}

	bhi360_disable_sensor_table(&bhi360, bhi360_base_sensors, ARRAY_SIZE(bhi360_base_sensors));
	bhi360_disable_sensor_table(&bhi360,
								bhi360_gesture_sensors,
								ARRAY_SIZE(bhi360_gesture_sensors));
	bhi360_disable_sensor_table(&bhi360,
								bhi360_activity_sensors,
								ARRAY_SIZE(bhi360_activity_sensors));
	(void) bhy2_soft_reset(&bhi360.bhy2);
	bhi360.state.bits.configured = 0U;
}

/**
 * @brief Parse a rotation-vector FIFO packet into quaternion output.
 * @details The decoded quaternion sample is written into the cached quaternion
 *          buffer and posted as bhi360_event_Quaternion. v_param carries the
 *          source sensor ID and p_param points at struct bhi360_quat_data.
 */
static void parse_quaternion(const struct bhy2_fifo_parse_data_info* callback_info,
							 void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_quaternion data;

	if (callback_info->data_size != 11) {
		LOG_ERR("Invalid data size: %d", callback_info->data_size);
		return;
	}

	bhy2_parse_quaternion(callback_info->data_ptr, &data);

	dev->quat_data.x = data.x;
	dev->quat_data.y = data.y;
	dev->quat_data.z = data.z;
	dev->quat_data.w = data.w;
	dev->quat_data.accuracy = data.accuracy;

	bhi360_post_event(bhi360_event_Quaternion, callback_info->sensor_id, &dev->quat_data);
}

/**
 * @brief Parse a linear-acceleration FIFO packet into cached XYZ output.
 * @details The decoded sample is posted as bhi360_event_LinearAcceleration with
 *          the sensor ID in v_param and struct bhi360_lacc_data in p_param.
 */
static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info* callback_info,
									  void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	dev->lacc_data.x = data.x;
	dev->lacc_data.y = data.y;
	dev->lacc_data.z = data.z;

	bhi360_post_event(bhi360_event_LinearAcceleration, callback_info->sensor_id, &dev->lacc_data);
}

/**
 * @brief Parse a gyroscope FIFO packet into cached XYZ angular velocity.
 * @details The decoded sample is posted as bhi360_event_Gyro with the sensor ID
 *          in v_param and struct bhi360_gyro_data in p_param.
 */
static void parse_gyro(const struct bhy2_fifo_parse_data_info* callback_info, void* callback_ref) {
	struct bhi360_t* dev = (callback_ref != NULL) ? (struct bhi360_t*) callback_ref : &bhi360;
	struct bhy2_data_xyz data;

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	dev->gyro_data.x = data.x;
	dev->gyro_data.y = data.y;
	dev->gyro_data.z = data.z;

	bhi360_post_event(bhi360_event_Gyro, callback_info->sensor_id, &dev->gyro_data);
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

	if ((callback_info->sensor_id == BHY2_SENSOR_ID_STD) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_WU) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP) ||
		(callback_info->sensor_id == BHY2_SENSOR_ID_STD_LP_WU)) {
		dev->pedometer_data.sensor_id = callback_info->sensor_id;
		dev->pedometer_data.step_detected = true;
		bhi360_post_event(bhi360_event_Pedometer, callback_info->sensor_id, &dev->pedometer_data);
		return;
	}

	dev->gesture_data.sensor_id = callback_info->sensor_id;
	dev->gesture_data.value = value;
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
 * @brief Parse and publish a BHY2 meta-event packet.
 * @details Both the regular and wake-up meta-event streams are routed here.
 *          The function validates the three-byte payload and publishes a packed
 *          bhi360_event_MetaEvent for consumers.
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

	bhi360_post_meta_event(meta_event_type, byte1, byte2);
}
