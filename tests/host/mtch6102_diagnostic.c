/* SPDX-License-Identifier: Apache-2.0 */
#include "mtch6102.h"
#include "mtch6102_platform.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Exercise the actual diagnostic driver through public APIs and modeled I2C
 * frames. These single-threaded mocks do not model hardware signal behavior.
 */
const struct device mtch6102_mock_device = {.name = "host touch platform"};
static const struct gpio_dt_spec gpio_pins[daughter_if_gpio_count] = {
	{&mtch6102_mock_device, 14},
	{&mtch6102_mock_device, 13},
	{&mtch6102_mock_device, 12},
	{&mtch6102_mock_device, 11},
};
static bool claimed[daughter_if_gpio_count];
static bool supply_enabled;
static int32_t supply_uv;
static int64_t now_ms;
static unsigned int lock_depth;
static unsigned int mutex_depth;
static unsigned int probe_failures;
static unsigned int probe_reads;
static unsigned int rail_enables;
static unsigned int gpio_claims;
static unsigned int gpio_callbacks;
static unsigned int cfg_commands;
static unsigned int cfg_reads;
static uint8_t registers[256];
static struct gpio_callback* frame_callback;
static unsigned int bs_reads;
static bool stuck_bs;
static int overlap_reg = -1;
static uint32_t observed_frames;
static int fail_read_reg = -1;
static int fail_write;
static int fail_bus;
static int fail_mutex;
static int fail_release;
static unsigned int command;
static unsigned int compensation_reads;
static unsigned int configuration_reads;
static void next_frame(void) {
	now_ms += 20;
	++observed_frames;
	frame_callback->handler(&mtch6102_mock_device, frame_callback, frame_callback->mask);
}

bool device_is_ready(const struct device* device) {
	assert(device == &mtch6102_mock_device);
	return true;
}

int k_mutex_lock(struct k_mutex* mutex, k_timeout_t timeout) {
	assert(timeout == K_FOREVER || timeout == 250);
	if (fail_mutex) {
		int e = fail_mutex;
		fail_mutex = 0;
		return e;
	}
	++mutex->depth;
	++mutex_depth;
	return 0;
}

int k_mutex_unlock(struct k_mutex* mutex) {
	assert(mutex->depth > 0 && mutex_depth > 0);
	--mutex->depth;
	--mutex_depth;
	return 0;
}

int64_t k_uptime_get(void) {
	return now_ms;
}

int32_t k_msleep(int32_t duration_ms) {
	assert(duration_ms >= 0);
	now_ms += duration_ms;
	return 0;
}

bool regulator_is_enabled(const struct device* device) {
	assert(device_is_ready(device));
	return supply_enabled;
}

int regulator_get_voltage(const struct device* device, int32_t* voltage_uv) {
	assert(device_is_ready(device));
	*voltage_uv = supply_uv;
	return 0;
}

int regulator_set_voltage(const struct device* device, int32_t min_uv, int32_t max_uv) {
	assert(device_is_ready(device));
	assert(!supply_enabled && min_uv == max_uv);
	supply_uv = min_uv;
	return 0;
}

int regulator_enable(const struct device* device) {
	assert(device_is_ready(device));
	assert(!supply_enabled && supply_uv == MTCH6102_SUPPLY_VOLTAGE_UV);
	supply_enabled = true;
	++rail_enables;
	return 0;
}

bool sys_i2c_is_ready(const struct sys_i2c_dt_spec* spec) {
	assert(spec->address == 0x25);
	return true;
}

int sys_i2c_lock(const struct sys_i2c_dt_spec* spec, k_timeout_t timeout) {
	assert(sys_i2c_is_ready(spec));
	assert(timeout > 0 && lock_depth == 0);
	if (fail_bus) {
		int e = fail_bus;
		fail_bus = 0;
		return e;
	}
	++lock_depth;
	return 0;
}

int sys_i2c_release(const struct sys_i2c_dt_spec* spec) {
	assert(sys_i2c_is_ready(spec));
	assert(lock_depth == 1);
	--lock_depth;
	int e = fail_release;
	fail_release = 0;
	return e;
}

int sys_i2c_write(const struct sys_i2c_dt_spec* spec, const uint8_t* bytes, uint32_t length) {
	assert(sys_i2c_is_ready(spec));
	assert(supply_enabled && lock_depth == 1 && length == 2);
	assert(bytes[0] != mtch6102_config_I2CAddr);
	registers[bytes[0]] = bytes[1];
	if (bytes[0] == mtch6102_core_CMD) {
		assert(bytes[1] == BIT(5) || bytes[1] == BIT(0));
		command = bytes[1];
		if (command == BIT(5)) {
			++cfg_commands;
			cfg_reads = 0;
		} else
			bs_reads = 0;
		if (fail_write) {
			int e = fail_write;
			fail_write = 0;
			return e;
		}
	}
	return 0;
}

int sys_i2c_write_read(const struct sys_i2c_dt_spec* spec,
					   const void* write,
					   size_t write_length,
					   void* read,
					   size_t read_length) {
	assert(sys_i2c_is_ready(spec));
	assert(supply_enabled && lock_depth == 1 && write_length == 1);
	uint8_t reg = *(const uint8_t*) write;
	if (reg == fail_read_reg) {
		fail_read_reg = -1;
		return -EIO;
	}
	if (reg == mtch6102_core_FWMajor) {
		const uint8_t id[] = {2, 5, 0, 0x12};
		++probe_reads;
		assert(read_length == sizeof(id));
		if (probe_failures) {
			--probe_failures;
			return -EIO;
		}
		memcpy(read, id, sizeof(id));
	} else if (reg == mtch6102_core_CMD) {
		assert(read_length == 1);
		if (command == BIT(5))
			*(uint8_t*) read = cfg_reads++ == 0 ? BIT(5) : 0;
		else {
			assert(command == BIT(0));
			*(uint8_t*) read = stuck_bs || bs_reads++ == 0 ? BIT(0) : 0;
		}
	} else {
		assert((unsigned int) reg + read_length <= sizeof(registers));
		if (reg == mtch6102_compensation_SENSORCOMP_RX0) {
			assert(read_length == 15);
			++compensation_reads;
		}
		if (reg == mtch6102_config_NumberOfXChannels) {
			assert(read_length == MTCH6102_CONFIGURATION_REGISTER_COUNT);
			++configuration_reads;
		}
		memcpy(read, &registers[reg], read_length);
		if (reg == overlap_reg) {
			overlap_reg = -1;
			next_frame();
		}
	}
	return 0;
}

const struct gpio_dt_spec* daughter_if_gpio_claim(enum daughter_if_gpio line) {
	assert(line == daughter_if_GPIO2 || line == daughter_if_GPIO3);
	assert(!claimed[line]);
	claimed[line] = true;
	++gpio_claims;
	return &gpio_pins[line];
}

int daughter_if_gpio_release(enum daughter_if_gpio line) {
	assert(line < daughter_if_gpio_count && claimed[line]);
	claimed[line] = false;
	return 0;
}

int gpio_pin_configure_dt(const struct gpio_dt_spec* pin, unsigned int flags) {
	assert(pin->port == &mtch6102_mock_device && flags == GPIO_INPUT);
	return 0;
}

void gpio_init_callback(struct gpio_callback* callback,
						void (*handler)(const struct device*, struct gpio_callback*, uint32_t),
						uint32_t mask) {
	callback->handler = handler;
	callback->mask = mask;
}

int gpio_add_callback(const struct device* port, struct gpio_callback* callback) {
	assert(port == &mtch6102_mock_device && callback->handler != NULL);
	++gpio_callbacks;
	frame_callback = callback;
	return 0;
}

int gpio_remove_callback(const struct device* port, struct gpio_callback* callback) {
	assert(port == &mtch6102_mock_device && callback->handler != NULL);
	assert(gpio_callbacks > 0);
	--gpio_callbacks;
	return 0;
}

int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec* pin, unsigned int flags) {
	assert(pin == &gpio_pins[daughter_if_GPIO2]);
	assert(flags == GPIO_INT_EDGE_FALLING);
	return 0;
}

/* Acquisition is deliberately outside these startup/configuration scenarios. */
int gpio_pin_get_raw(const struct device* port, uint8_t pin) {
	ARG_UNUSED(port);
	ARG_UNUSED(pin);
	return 0;
}

void k_timer_start(struct k_timer* timer, k_timeout_t delay, k_timeout_t period) {
	ARG_UNUSED(timer);
	ARG_UNUSED(delay);
	ARG_UNUSED(period);
}

void k_timer_stop(struct k_timer* timer) {
	ARG_UNUSED(timer);
}

int k_mem_slab_alloc(struct k_mem_slab* slab, void** memory, k_timeout_t timeout) {
	ARG_UNUSED(slab);
	assert(timeout == K_NO_WAIT);
	*memory = malloc(sizeof(struct touch_sensor_sample_t));
	return *memory ? 0 : -ENOMEM;
}

void k_mem_slab_free(struct k_mem_slab* slab, void* memory) {
	ARG_UNUSED(slab);
	free(memory);
}

int device_driver_event_post(uint32_t device,
							 uint32_t event,
							 uint32_t value,
							 uintptr_t pointer,
							 k_timeout_t timeout) {
	ARG_UNUSED(device);
	ARG_UNUSED(event);
	ARG_UNUSED(value);
	assert(timeout == K_NO_WAIT);
	if (pointer)
		free((void*) pointer);
	return 0;
}

int device_driver_event_post_isr(uint32_t device,
								 uint32_t event,
								 uint32_t value,
								 uintptr_t pointer) {
	return device_driver_event_post(device, event, value, pointer, K_NO_WAIT);
}

time_t rtc_get_timestamp_us(void) {
	return now_ms * 1000;
}

static void sample_frame(struct mtch6102_diagnostic_snapshot* snapshot) {
	next_frame();
	assert(mtch6102_irq_handler() == 0);
	assert(mtch6102_get_diagnostic_snapshot(snapshot) == 0);
	assert(snapshot->frame_sequence == observed_frames);
	assert(snapshot->uptime_ms == (uint64_t) now_ms);
	assert(snapshot->sample.timestamp == now_ms * 1000);
	assert(memcmp(snapshot->sensor_values, &registers[0x80], 15) == 0);
	assert(memcmp(snapshot->raw_values, &registers[0x90], 30) == 0);
	assert(memcmp(snapshot->base_values, &registers[0xb0], 30) == 0);
	assert(memcmp(snapshot->sensor_compensation, &registers[0x50], 15) == 0);
	assert(memcmp(snapshot->config_values,
				  &registers[mtch6102_config_NumberOfXChannels],
				  MTCH6102_CONFIGURATION_REGISTER_COUNT) == 0);
	assert(lock_depth == 0 && mutex_depth == 0);
}

static void assert_cached(const struct mtch6102_diagnostic_snapshot* expected) {
	struct mtch6102_diagnostic_snapshot actual;
	assert(mtch6102_get_diagnostic_snapshot(&actual) == 0);
	assert(memcmp(&actual, expected, sizeof(actual)) == 0);
	assert(lock_depth == 0 && mutex_depth == 0);
}

int main(void) {
	struct mtch6102_diagnostic_snapshot d, old;
	struct mtch6102_config_t config;
	mtch6102_get_default_config(&config);
	unsigned int debounce_down =
		config.configuration[MTCH6102_CONFIGURATION_INDEX(mtch6102_config_DebounceDown)];
	assert(debounce_down > 0);
	assert(mtch6102_get_diagnostic_snapshot(NULL) == -EINVAL);
	assert(mtch6102_get_diagnostic_snapshot(&d) == -EAGAIN);
	assert(mtch6102_force_baseline() == -EAGAIN);
	for (unsigned int i = 0; i < 15; i++)
		registers[0x50 + i] = (uint8_t) (70 + i);
	registers[mtch6102_config_I2CAddr] = 0x25;
	assert(mtch6102_config(&config) == 0);
	assert(compensation_reads == 1 && configuration_reads == 1);
	assert(mtch6102_force_baseline() == -EAGAIN);
	assert(mtch6102_start() == 0);
	assert(mtch6102_irq_handler() == -EAGAIN);
	for (unsigned int i = 0; i < 30; i++) {
		registers[0x90 + i] = (uint8_t) (30 + i);
		registers[0xb0 + i] = (uint8_t) (60 + i);
	}
	/* Touch comes from SENSORVALUES; RAW and BASE bytes are only diagnostics. */
	registers[0x80] = 80;
	for (unsigned int frame = 0; frame < debounce_down; ++frame) {
		sample_frame(&d);
		assert(d.sample.position.touched == (frame + 1 == debounce_down));
	}
	assert(d.sample.position.touched && d.sample.position.x == 0);
	old = d;
	assert(mtch6102_irq_handler() == -EAGAIN);
	assert_cached(&old);

	/* Neither a failed RAW/BASE read nor an overlapping frame may advance the
	 * diagnostic cache or publish partly updated register blocks. */
	const int failure_regs[] = {mtch6102_acquisition_RAWVALUES_RX0_L,
								mtch6102_acquisition_BASEVALUES_RX0_L};
	for (size_t i = 0; i < sizeof(failure_regs) / sizeof(failure_regs[0]); ++i) {
		next_frame();
		++registers[0x90];
		++registers[0xb0];
		fail_read_reg = failure_regs[i];
		assert(mtch6102_irq_handler() == -EIO);
		assert_cached(&old);
		next_frame();
		overlap_reg = failure_regs[i];
		assert(mtch6102_irq_handler() == -EAGAIN);
		assert_cached(&old);
	}
	sample_frame(&d);
	assert(d.sample.position.touched);
	assert(d.raw_values[0] != old.raw_values[0] && d.base_values[0] != old.base_values[0]);
	assert(mtch6102_force_baseline() == 0);
	assert(bs_reads == 2 && lock_depth == 0 && mutex_depth == 0);
	assert(mtch6102_get_diagnostic_snapshot(&d) == -EAGAIN);
	assert(mtch6102_irq_handler() == -EAGAIN);
	registers[0x80] = 0;
	sample_frame(&d);
	assert(!d.sample.position.touched);
	old = d;
	sample_frame(&d);
	assert(!d.sample.position.touched && d.frame_sequence > old.frame_sequence);
	assert(compensation_reads == 1 && configuration_reads == 1);

	stuck_bs = true;
	int64_t start = now_ms;
	assert(mtch6102_force_baseline() == -ETIMEDOUT);
	assert(now_ms - start == 250);
	assert(lock_depth == 0 && mutex_depth == 0);
	assert(mtch6102_get_diagnostic_snapshot(&d) == -EAGAIN);
	stuck_bs = false;
	fail_write = -EIO;
	fail_release = -ENXIO;
	assert(mtch6102_force_baseline() == -EIO);
	assert(lock_depth == 0 && mutex_depth == 0);
	fail_read_reg = mtch6102_core_CMD;
	assert(mtch6102_force_baseline() == -EIO);
	assert(lock_depth == 0 && mutex_depth == 0);
	fail_bus = -EBUSY;
	assert(mtch6102_force_baseline() == -EBUSY);
	fail_mutex = -EBUSY;
	assert(mtch6102_force_baseline() == -EBUSY);
	assert(lock_depth == 0 && mutex_depth == 0);
	sample_frame(&d);
	mtch6102_stop();
	assert(mtch6102_get_diagnostic_snapshot(&d) == -EAGAIN);
	assert(mtch6102_force_baseline() == -EAGAIN);
	assert(supply_enabled);
	assert(mtch6102_config(&config) == 0);
	assert(compensation_reads == 2 && configuration_reads == 2);
	assert(mtch6102_start() == 0);
	assert(mtch6102_irq_handler() == -EAGAIN);
	sample_frame(&d);
	assert(!d.sample.position.touched);
	puts("MTCH6102 diagnostic cache/coherence, fresh frames, baseline command errors: passed");
	return 0;
}
