/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SENSWEAR_MTCH6102_HOST_PLATFORM_H_
#define SENSWEAR_MTCH6102_HOST_PLATFORM_H_

/* Minimal platform boundary for compiling the production driver on a host.
 * These mocks are single-threaded; they do not validate Zephyr scheduling.
 */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#define BIT(n) (1UL << (n))
#define ARG_UNUSED(value) ((void) (value))
#define BUILD_ASSERT(condition, message) _Static_assert(condition, message)
#define __ASSERT(condition, ...) assert(condition)
#define DT_ALIAS(name) 0
#define DT_NODE_HAS_STATUS(node, status) 1
#define DT_NODE_HAS_PROP(node, prop) 1
#define DT_PHANDLE(node, prop) 0
#define DEVICE_DT_GET(node) (&mtch6102_mock_device)
#define CONFIG_LOG_DEFAULT_LEVEL 0
#define LOG_MODULE_REGISTER(...)
#define LOG_ERR(...)
#define LOG_WRN(...)
#define LOG_INF(...)
#define LOG_DBG(...)

struct device {
	const char* name;
};
extern const struct device mtch6102_mock_device;
bool device_is_ready(const struct device* device);

typedef int64_t k_timeout_t;
#define K_FOREVER ((k_timeout_t) -1)
#define K_NO_WAIT ((k_timeout_t) 0)
#define K_MSEC(ms) ((k_timeout_t) (ms))
struct k_mutex {
	unsigned int depth;
};
#define K_MUTEX_DEFINE(name) struct k_mutex name
int k_mutex_lock(struct k_mutex* mutex, k_timeout_t timeout);
int k_mutex_unlock(struct k_mutex* mutex);
int64_t k_uptime_get(void);
int32_t k_msleep(int32_t duration_ms);

struct k_timer {
	void (*expiry)(struct k_timer* timer);
};
#define K_TIMER_DEFINE(name, expiry_fn, stop_fn) struct k_timer name = {expiry_fn}
void k_timer_start(struct k_timer* timer, k_timeout_t delay, k_timeout_t period);
void k_timer_stop(struct k_timer* timer);
struct k_mem_slab {
	int unused;
};
#define K_MEM_SLAB_DEFINE(name, size, count, alignment) struct k_mem_slab name
int k_mem_slab_alloc(struct k_mem_slab* slab, void** memory, k_timeout_t timeout);
void k_mem_slab_free(struct k_mem_slab* slab, void* memory);

typedef int atomic_t;
typedef int atomic_val_t;
static inline atomic_val_t atomic_get(const atomic_t* value) {
	return *value;
}
static inline void atomic_set(atomic_t* value, atomic_val_t next) {
	*value = next;
}
static inline void atomic_clear(atomic_t* value) {
	*value = 0;
}
static inline void atomic_inc(atomic_t* value) {
	++*value;
}
static inline bool atomic_cas(atomic_t* value, atomic_val_t old, atomic_val_t next) {
	if (*value != old) {
		return false;
	}
	*value = next;
	return true;
}

struct gpio_dt_spec {
	const struct device* port;
	uint8_t pin;
};
struct gpio_callback {
	void (*handler)(const struct device*, struct gpio_callback*, uint32_t);
	uint32_t mask;
};
#define GPIO_INPUT 1U
#define GPIO_INT_EDGE_FALLING 2U
void gpio_init_callback(struct gpio_callback* callback,
						void (*handler)(const struct device*, struct gpio_callback*, uint32_t),
						uint32_t mask);
int gpio_pin_configure_dt(const struct gpio_dt_spec* pin, unsigned int flags);
int gpio_add_callback(const struct device* port, struct gpio_callback* callback);
int gpio_remove_callback(const struct device* port, struct gpio_callback* callback);
int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec* pin, unsigned int flags);
int gpio_pin_get_raw(const struct device* port, uint8_t pin);

bool regulator_is_enabled(const struct device* device);
int regulator_get_voltage(const struct device* device, int32_t* voltage_uv);
int regulator_set_voltage(const struct device* device, int32_t min_uv, int32_t max_uv);
int regulator_enable(const struct device* device);

struct sys_i2c_dt_spec {
	unsigned int address;
};
#define SYS_I2C_DT_SPEC_GET(node) \
	{ 0x25U }
bool sys_i2c_is_ready(const struct sys_i2c_dt_spec* spec);
int sys_i2c_lock(const struct sys_i2c_dt_spec* spec, k_timeout_t timeout);
int sys_i2c_release(const struct sys_i2c_dt_spec* spec);
int sys_i2c_write(const struct sys_i2c_dt_spec* spec, const uint8_t* bytes, uint32_t length);
int sys_i2c_write_read(const struct sys_i2c_dt_spec* spec,
					   const void* write,
					   size_t write_length,
					   void* read,
					   size_t read_length);

enum daughter_if_gpio {
	daughter_if_GPIO0,
	daughter_if_GPIO1,
	daughter_if_GPIO2,
	daughter_if_GPIO3,
	daughter_if_gpio_count,
};
const struct gpio_dt_spec* daughter_if_gpio_claim(enum daughter_if_gpio line);
int daughter_if_gpio_release(enum daughter_if_gpio line);

#define MTCH6102_DEVICE_DTS_ID 1U
int device_driver_event_post(uint32_t device,
							 uint32_t event,
							 uint32_t value,
							 uintptr_t pointer,
							 k_timeout_t timeout);
int device_driver_event_post_isr(uint32_t device,
								 uint32_t event,
								 uint32_t value,
								 uintptr_t pointer);
time_t rtc_get_timestamp_us(void);

#endif
