/* SPDX-License-Identifier: Apache-2.0 */
#include "mtch6102.h"
#include "mtch6102_platform.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

/* Exercise public entry points against the actual driver, without reaching
 * into its private state. Each scenario runs in a fresh process.
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

bool device_is_ready(const struct device* device) {
	assert(device == &mtch6102_mock_device);
	return true;
}

int k_mutex_lock(struct k_mutex* mutex, k_timeout_t timeout) {
	assert(timeout == K_FOREVER);
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
	++lock_depth;
	return 0;
}

int sys_i2c_release(const struct sys_i2c_dt_spec* spec) {
	assert(sys_i2c_is_ready(spec));
	assert(lock_depth == 1);
	--lock_depth;
	return 0;
}

int sys_i2c_write(const struct sys_i2c_dt_spec* spec, const uint8_t* bytes, uint32_t length) {
	assert(sys_i2c_is_ready(spec));
	assert(supply_enabled && lock_depth == 1 && length == 2);
	/* Changing the address would disconnect the controller from this spec. */
	assert(bytes[0] != mtch6102_config_I2CAddr);
	registers[bytes[0]] = bytes[1];
	if (bytes[0] == mtch6102_core_CMD) {
		assert(bytes[1] == BIT(5));
		++cfg_commands;
		cfg_reads = 0;
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
	if (reg == mtch6102_core_FWMajor) {
		const uint8_t firmware_id[] = {0x02, 0x05, 0x00, 0x12};
		assert(read_length == sizeof(firmware_id));
		++probe_reads;
		if (probe_failures > 0) {
			--probe_failures;
			return -EIO;
		}
		memcpy(read, firmware_id, sizeof(firmware_id));
	} else {
		assert(reg == mtch6102_core_CMD && read_length == 1);
		assert(cfg_commands > 0);
		/* Model a command that stays busy for one poll before completion. */
		*(uint8_t*) read = cfg_reads++ == 0 ? BIT(5) : 0;
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
	assert(false);
	return -ENOSYS;
}

void k_timer_start(struct k_timer* timer, k_timeout_t delay, k_timeout_t period) {
	ARG_UNUSED(timer);
	ARG_UNUSED(delay);
	ARG_UNUSED(period);
	assert(false);
}

void k_timer_stop(struct k_timer* timer) {
	ARG_UNUSED(timer);
	assert(false);
}

int k_mem_slab_alloc(struct k_mem_slab* slab, void** memory, k_timeout_t timeout) {
	ARG_UNUSED(slab);
	ARG_UNUSED(memory);
	ARG_UNUSED(timeout);
	assert(false);
	return -ENOSYS;
}

void k_mem_slab_free(struct k_mem_slab* slab, void* memory) {
	ARG_UNUSED(slab);
	ARG_UNUSED(memory);
	assert(false);
}

int device_driver_event_post(uint32_t device,
							 uint32_t event,
							 uint32_t value,
							 uintptr_t pointer,
							 k_timeout_t timeout) {
	ARG_UNUSED(device);
	ARG_UNUSED(event);
	ARG_UNUSED(value);
	ARG_UNUSED(pointer);
	ARG_UNUSED(timeout);
	assert(false);
	return -ENOSYS;
}

int device_driver_event_post_isr(uint32_t device,
								 uint32_t event,
								 uint32_t value,
								 uintptr_t pointer) {
	return device_driver_event_post(device, event, value, pointer, K_NO_WAIT);
}

time_t rtc_get_timestamp_us(void) {
	assert(false);
	return 0;
}

static void assert_ready_defaults(void) {
	assert(mtch6102_is_ready());
	assert(registers[mtch6102_config_NumberOfXChannels] == 12);
	assert(registers[mtch6102_config_NumberOfYChannels] == 3);
	assert(registers[mtch6102_core_MODE] == 3);
	assert(cfg_commands > 0 && cfg_reads == 2);
	assert(lock_depth == 0 && mutex_depth == 0);
}

static void transient_probe_failure(void) {
	probe_failures = 1;
	assert(mtch6102_init() == -EIO);
	assert(!mtch6102_is_ready());
	assert(rail_enables == 1 && probe_reads == 1 && gpio_claims == 0);
	assert(mtch6102_config(NULL) == 0);
	assert_ready_defaults();
	assert(probe_reads == 2 && rail_enables == 1);
	assert(gpio_claims == 2 && gpio_callbacks == 1);
	/* Later retries/configuration must not reacquire existing resources. */
	assert(mtch6102_init() == 0);
	assert(mtch6102_config(NULL) == 0);
	assert_ready_defaults();
	assert(probe_reads == 2 && rail_enables == 1);
	assert(gpio_claims == 2 && gpio_callbacks == 1 && cfg_commands == 2);
}

static void persistent_probe_failure(void) {
	probe_failures = 3;
	assert(mtch6102_init() == -EIO);
	assert(mtch6102_config(NULL) == -EIO);
	assert(mtch6102_config(NULL) == -EIO);
	assert(!mtch6102_is_ready());
	assert(probe_reads == 3 && cfg_commands == 0);
	assert(rail_enables == 1 && gpio_claims == 0);
	assert(lock_depth == 0 && mutex_depth == 0);
}

static void configure_without_explicit_init(void) {
	assert(mtch6102_config(NULL) == 0);
	assert_ready_defaults();
	assert(probe_reads == 1 && rail_enables == 1);
	assert(gpio_claims == 2 && gpio_callbacks == 1);
}

int main(int argc, char** argv) {
	assert(argc == 2);
	if (strcmp(argv[1], "transient") == 0) {
		transient_probe_failure();
	} else if (strcmp(argv[1], "persistent") == 0) {
		persistent_probe_failure();
	} else if (strcmp(argv[1], "direct-config") == 0) {
		configure_without_explicit_init();
	} else {
		fprintf(stderr, "Unknown scenario: %s\n", argv[1]);
		return 1;
	}
	printf("MTCH6102 lifecycle %s: passed\n", argv[1]);
	return 0;
}
