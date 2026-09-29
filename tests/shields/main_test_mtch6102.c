/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the SensWear MTCH6102 touch driver on the senswear_touch
 * shield.
 *
 * init() powers and probes the controller, config() selects acquisition,
 * and start() enables SYNC frame-complete events plus recovery polling.
 * The consumer handles coalesced mtch6102_Irq requests and prints host-decoded
 * slider position/gestures from immutable, owned sample events. Every sample
 * is released after printing. See boards/shields/senswear_touch/README.md for
 * physical mapping and the complete hardware validation procedure.
 *
 * Build with the test_mtch6102 preset; this is an on-target test.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/barrier.h>
#include <errno.h>

#include "mtch6102.h"
#include "device_driver_events.h"
#include "device_driver_dts_ids.h"
#include "sys_i2c.h"
#include "tpsm83102_registers.h"

#define TEST_EVENT_WAIT_MS 1000
#define TEST_EVENT_THREAD_PRIO 7
#define TEST_EVENT_STACK_SIZE 2048

/* Debugger-only calibration interface. Word arrays avoid ABI/padding assumptions.
 * Read
 * snapshot[1] before/after the whole array; accept equal, even versions.
 * Commands: write
 * parameters first and request ID [0] last. The main thread
 * owns every controller mutation; a
 * debugger never calls I2C or halts acquisition.
 * No calibration interface is present in the
 * production application.
 */
volatile uint32_t mtch6102_calibration_snapshot[80];
/* Command 2 parameters [2..6]: X/Y threshold, hysteresis, scans, baseline
 * interval. Optional
 * [7]: 0 preserves filtering; 1..3 selects IIR strength.
 * Command 3 parameters [2..6]: debounce
 * up/down (1..10), positive/negative
 * baseline slew limits (1..255), idle period (1..65535).
 * Active period is kept. */
volatile uint32_t mtch6102_calibration_command[12];

static int apply_calibration_config(struct mtch6102_config_t* config,
									const struct mtch6102_config_t* candidate) {
	mtch6102_stop();
	int ret = mtch6102_config(candidate);
	if (ret == 0) {
		*config = *candidate;
	} else {
		int restore_ret = mtch6102_config(config);
		if (restore_ret != 0) {
			printk("Calibration restore failed: %d\n", restore_ret);
		}
	}
	int start_ret = mtch6102_start();
	return ret == 0 ? start_ret : ret;
}

static void calibration_loop(void) {
	struct mtch6102_config_t config;
	struct mtch6102_diagnostic_snapshot frame;
	uint32_t request_done = 0;
	uint32_t revision = 0;
	mtch6102_get_default_config(&config);
	mtch6102_calibration_snapshot[0] = 0x53435432U;
	for (;;) {
		uint32_t request = mtch6102_calibration_command[0];
		if (request != request_done) {
			int ret = -EINVAL;
			if (mtch6102_calibration_command[1] == 1U) {
				ret = mtch6102_force_baseline();
			} else if (mtch6102_calibration_command[1] == 2U) {
				uint32_t tx = mtch6102_calibration_command[2];
				uint32_t ty = mtch6102_calibration_command[3];
				uint32_t hysteresis = mtch6102_calibration_command[4];
				uint32_t scans = mtch6102_calibration_command[5];
				uint32_t base_interval = mtch6102_calibration_command[6];
				uint32_t filter_strength = mtch6102_calibration_command[7];
				if (tx > 0U && tx <= 255U && ty > 0U && ty <= 255U && hysteresis < tx &&
					hysteresis < ty && scans > 0U && scans <= 32U && base_interval > 0U &&
					base_interval <= 65535U && filter_strength <= 3U) {
					struct mtch6102_config_t candidate = config;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TouchThreshX)] =
						(uint8_t) tx;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_TouchThreshY)] =
						(uint8_t) ty;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_Hysteresis)] =
						(uint8_t) hysteresis;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_ScanCount)] =
						(uint8_t) scans;
					candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
						mtch6102_config_BaseIntervalL)] = (uint8_t) base_interval;
					candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
						mtch6102_config_BaseIntervalH)] = (uint8_t) (base_interval >> 8);
					if (filter_strength != 0U) {
						candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
							mtch6102_config_FilterType)] = 2U;
						candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
							mtch6102_config_FilterStrength)] = (uint8_t) filter_strength;
					}
					ret = apply_calibration_config(&config, &candidate);
				}
			} else if (mtch6102_calibration_command[1] == 3U) {
				uint32_t debounce_up = mtch6102_calibration_command[2];
				uint32_t debounce_down = mtch6102_calibration_command[3];
				uint32_t base_positive = mtch6102_calibration_command[4];
				uint32_t base_negative = mtch6102_calibration_command[5];
				uint32_t idle_period = mtch6102_calibration_command[6];
				if (debounce_up >= 1U && debounce_up <= 10U && debounce_down >= 1U &&
					debounce_down <= 10U && base_positive >= 1U && base_positive <= 255U &&
					base_negative >= 1U && base_negative <= 255U && idle_period >= 1U &&
					idle_period <= 65535U) {
					struct mtch6102_config_t candidate = config;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DebounceUp)] =
						(uint8_t) debounce_up;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DebounceDown)] =
						(uint8_t) debounce_down;
					candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
						mtch6102_config_BasePosFilter)] = (uint8_t) base_positive;
					candidate.configuration[MTCH6102_CONFIGURATION_INDEX(
						mtch6102_config_BaseNegFilter)] = (uint8_t) base_negative;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_IdlePeriodL)] =
						(uint8_t) idle_period;
					candidate
						.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_IdlePeriodH)] =
						(uint8_t) (idle_period >> 8);
					ret = apply_calibration_config(&config, &candidate);
				}
			}
			mtch6102_calibration_command[8] = (uint32_t) ret;
			request_done = request;
			barrier_dmem_fence_full();
			mtch6102_calibration_command[9] = request_done;
		}
		int ret = mtch6102_get_diagnostic_snapshot(&frame);
		mtch6102_calibration_snapshot[1] = ++revision;
		barrier_dmem_fence_full();
		mtch6102_calibration_snapshot[2] = (uint32_t) ret;
		if (ret == 0) {
			mtch6102_calibration_snapshot[3] = frame.frame_sequence;
			mtch6102_calibration_snapshot[4] = (uint32_t) frame.uptime_ms;
			mtch6102_calibration_snapshot[5] = frame.sample.position.touched;
			mtch6102_calibration_snapshot[6] = frame.sample.position.x;
			mtch6102_calibration_snapshot[7] = frame.sample.position.y;
			mtch6102_calibration_snapshot[8] = frame.sample.gesture_state;
			mtch6102_calibration_snapshot[9] = frame.sample.position.touch_state;
			for (size_t i = 0; i < 15; ++i) {
				mtch6102_calibration_snapshot[10 + i] = frame.sensor_values[i];
				mtch6102_calibration_snapshot[25 + i] =
					frame.raw_values[2 * i] | ((uint32_t) frame.raw_values[2 * i + 1] << 8);
				mtch6102_calibration_snapshot[40 + i] =
					frame.base_values[2 * i] | ((uint32_t) frame.base_values[2 * i + 1] << 8);
			}
			for (size_t i = 0; i < 4; ++i) {
				uint32_t packed = 0;
				for (size_t j = 0; j < 4 && 4 * i + j < 15; ++j) {
					packed |= (uint32_t) frame.sensor_compensation[4 * i + j] << (8 * j);
				}
				mtch6102_calibration_snapshot[55 + i] = packed;
			}
			for (size_t i = 0; i < 9; ++i) {
				uint32_t packed = 0;
				for (size_t j = 0; j < 4; ++j) {
					packed |= (uint32_t) frame.config_values[4 * i + j] << (8 * j);
				}
				mtch6102_calibration_snapshot[59 + i] = packed;
			}
		}
		barrier_dmem_fence_full();
		mtch6102_calibration_snapshot[1] = ++revision;
		k_msleep(20);
	}
}

K_THREAD_STACK_DEFINE(test_event_stack, TEST_EVENT_STACK_SIZE);
static struct k_thread test_event_thread;

/* Only run after a failed probe in this isolated bring-up image. These reads
 * do not change rail
 * settings or device addresses. The scan reads one byte
 * at each non-reserved address; it may
 * consume pending peripheral status.
 */
static void print_probe_diagnostics(void) {
	static struct sys_i2c_dt_spec bus = SYS_I2C_DT_SPEC_GET(DT_NODELABEL(tpsm83102));
	const uint8_t registers[] = {
		tpsm83102_register_CONTROL1,
		tpsm83102_register_VOUT,
		tpsm83102_register_CONTROL2,
	};
	int ret = sys_i2c_lock(&bus, K_MSEC(100));
	if (ret != 0) {
		printk("diagnostic bus lock failed: %d\n", ret);
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(registers); ++i) {
		uint8_t value = 0;
		ret = sys_i2c_write_read(&bus, &registers[i], 1, &value, 1);
		mtch6102_calibration_snapshot[68 + 2 * i] = (uint32_t) ret;
		mtch6102_calibration_snapshot[69 + 2 * i] = value;
		printk("regulator register 0x%02x: ret=%d value=0x%02x\n", registers[i], ret, value);
	}
	ret = sys_i2c_release(&bus);
	if (ret != 0) {
		printk("diagnostic bus release failed: %d\n", ret);
		return;
	}
	for (uint16_t address = 0x08; address <= 0x77; ++address) {
		bus.config.addr = address;
		ret = sys_i2c_lock(&bus, K_MSEC(100));
		if (ret != 0) {
			printk("scan bus lock failed: %d\n", ret);
			break;
		}
		uint8_t value;
		int read_ret = sys_i2c_read(&bus, &value, 1);
		ret = sys_i2c_release(&bus);
		if (read_ret == 0) {
			mtch6102_calibration_snapshot[74 + address / 32U] |= BIT(address % 32U);
			printk("I2C read ACK at 0x%02x\n", address);
		}
		if (ret != 0) {
			printk("scan bus release failed: %d\n", ret);
			break;
		}
	}
	printk("Probe diagnostics complete; regulator readback is not a voltage measurement.\n");
}

/* The queued event owns this immutable sample until the consumer releases it. */
static void print_sample(const struct device_driver_event_t* event) {
	const struct touch_sensor_sample_t* sample =
		(const struct touch_sensor_sample_t*) event->p_param;

	if (sample == NULL) {
		return;
	}

	printk("  ts=%lld us  touched=%d  x=%u  y=%u  touch_state=0x%02x  gesture=0x%02x\n",
		   (long long) sample->timestamp,
		   sample->position.touched,
		   sample->position.x,
		   sample->position.y,
		   sample->position.touch_state,
		   sample->gesture_state);
}

static void mtch6102_event_consumer_thread(void* a, void* b, void* c) {
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct device_driver_event_t event;

	while (1) {
		if (device_driver_event_wait(K_MSEC(TEST_EVENT_WAIT_MS), &event)) {
			if (event.device_id != MTCH6102_DEVICE_DTS_ID) {
				continue;
			}

			if (event.event_id == mtch6102_Irq) {
				/* Read the completed acquisition frame and publish its host sample. */
				mtch6102_irq_handler();
				continue;
			}

			/* Classified touch/gesture event: name it and print the sample. */
			printk("event: %s (touch_state=0x%02x)\n",
				   mtch6102_event_name((enum mtch6102_event_type) event.event_id),
				   event.v_param);
			print_sample(&event);
			mtch6102_release_sample((const struct touch_sensor_sample_t*) event.p_param);
		}
	}
}

static bool start_mtch6102_event_consumer(void) {
	/* The backing queue is created by the device_driver_events SYS_INIT hook
	 * (device_driver_events_init.c) at POST_KERNEL, so no manual init here. */
	if (device_driver_event_get_queue() == NULL) {
		printk("device driver event queue is not initialized\n");
		return false;
	}

	k_tid_t tid = k_thread_create(&test_event_thread,
								  test_event_stack,
								  K_THREAD_STACK_SIZEOF(test_event_stack),
								  mtch6102_event_consumer_thread,
								  NULL,
								  NULL,
								  NULL,
								  TEST_EVENT_THREAD_PRIO,
								  0,
								  K_NO_WAIT);

	if (tid == NULL) {
		printk("failed to start event consumer thread\n");
		return false;
	}

	k_thread_name_set(tid, "mtch6102_evt");
	return true;
}

int main(void) {
	printk("\n=== MTCH6102 touch test (senswear_touch shield) ===\n");
	mtch6102_calibration_snapshot[0] = 0x53435432U;
	mtch6102_calibration_command[10] = 1;

	int ret = mtch6102_init();
	/* Diagnostic images have no device manager to retry a transient probe. */
	for (uint32_t attempt = 1; ret != 0 && attempt < 3; ++attempt) {
		k_msleep(250);
		mtch6102_calibration_command[11] = attempt;
		ret = mtch6102_init();
	}
	if (ret != 0) {
		mtch6102_calibration_command[8] = (uint32_t) ret;
		mtch6102_calibration_snapshot[2] = (uint32_t) ret;
		printk("mtch6102_init() failed: %d\n", ret);
		print_probe_diagnostics();
		return 0;
	}

	mtch6102_calibration_command[10] = 2;
	ret = mtch6102_config(NULL);
	if (ret != 0) {
		mtch6102_calibration_command[8] = (uint32_t) ret;
		printk("mtch6102_config() failed\n");
		return 0;
	}

	if (!mtch6102_is_ready()) {
		printk("MTCH6102 not ready after configuration\n");
		return 0;
	}

	mtch6102_calibration_command[10] = 3;
	ret = mtch6102_start();
	if (ret != 0) {
		mtch6102_calibration_command[8] = (uint32_t) ret;
		printk("mtch6102_start() failed\n");
		return 0;
	}

	if (!start_mtch6102_event_consumer()) {
		mtch6102_calibration_command[8] = (uint32_t) -EIO;
		printk("failed to start MTCH6102 event consumer\n");
		return 0;
	}

	printk(
		"Acquisition started; touch the pad to see decoded events from the consumer thread...\n");
	mtch6102_calibration_command[10] = 4;
	calibration_loop();

	return 0;
}
