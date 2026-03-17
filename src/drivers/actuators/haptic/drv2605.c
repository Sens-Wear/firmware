#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "drv2605.h"

LOG_MODULE_REGISTER(SENS_WEAR_HAPTIC_SENSOR_LOGGER);

#define HAPTIC_PLAYBACK_THREAD_STACK_SIZE 1024
#define HAPTIC_PLAYBACK_THREAD_PRIORITY 8
#define HAPTIC_TRAILING_OFF_FRAME_MS 1U

#define DRV2605_REG_MODE 0x01
#define DRV2605_REG_RT_PLAYBACK_INPUT 0x02
#define DRV2605_MODE_MASK GENMASK(2, 0)
#define DRV2605_MODE_INTERNAL_TRIGGER 0x00
#define DRV2605_MODE_RTP 0x05

static const struct device *const haptic_dev = DEVICE_DT_GET(DT_NODELABEL(drv2605));
static const struct i2c_dt_spec haptic_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(drv2605));

static K_THREAD_STACK_DEFINE(haptic_playback_thread_stack, HAPTIC_PLAYBACK_THREAD_STACK_SIZE);
static struct k_thread haptic_playback_thread_data;
static K_SEM_DEFINE(haptic_playback_sem, 0, 1);
static K_MUTEX_DEFINE(haptic_state_mutex);

static bool haptic_thread_started;
static bool haptic_is_playing;
static size_t haptic_queued_frame_count;
static struct haptic_actuator_frame
	haptic_queued_frames[HAPTIC_ACTUATOR_MAX_FRAMES + 1U];

static int haptic_drv2605_set_mode(uint8_t mode)
{
	return i2c_reg_update_byte_dt(&haptic_i2c, DRV2605_REG_MODE, DRV2605_MODE_MASK, mode);
}

static int haptic_drv2605_set_intensity(uint8_t intensity)
{
	return i2c_reg_write_byte_dt(&haptic_i2c, DRV2605_REG_RT_PLAYBACK_INPUT, intensity);
}

static int haptic_drv2605_enter_idle(void)
{
	int ret;

	ret = haptic_drv2605_set_intensity(0U);
	if (ret < 0) {
		return ret;
	}

	return haptic_drv2605_set_mode(DRV2605_MODE_INTERNAL_TRIGGER);
}

static int haptic_drv2605_prepare_rtp(void)
{
	int ret;

	ret = haptic_drv2605_set_intensity(0U);
	if (ret < 0) {
		return ret;
	}

	return haptic_drv2605_set_mode(DRV2605_MODE_RTP);
}

static void haptic_playback_thread(void *arg1, void *arg2, void *arg3)
{
	struct haptic_actuator_frame local_frames[ARRAY_SIZE(haptic_queued_frames)];
	size_t local_frame_count;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		int ret;

		k_sem_take(&haptic_playback_sem, K_FOREVER);

		k_mutex_lock(&haptic_state_mutex, K_FOREVER);
		local_frame_count = haptic_queued_frame_count;
		memcpy(local_frames, haptic_queued_frames,
		       local_frame_count * sizeof(local_frames[0]));
		k_mutex_unlock(&haptic_state_mutex);

		ret = haptic_drv2605_prepare_rtp();
		if (ret < 0) {
			LOG_ERR("Failed to enter RTP mode: %d", ret);
			goto finish_playback;
		}

		for (size_t i = 0; i < local_frame_count; i++) {
			ret = haptic_drv2605_set_intensity(local_frames[i].intensity);
			if (ret < 0) {
				LOG_ERR("Failed to write RTP frame %u: %d", (unsigned int)i, ret);
				break;
			}

			k_msleep(local_frames[i].duration_ms);
		}

finish_playback:
		ret = haptic_drv2605_enter_idle();
		if (ret < 0) {
			LOG_ERR("Failed to return haptic driver to idle: %d", ret);
		}

		k_mutex_lock(&haptic_state_mutex, K_FOREVER);
		haptic_is_playing = false;
		haptic_queued_frame_count = 0U;
		k_mutex_unlock(&haptic_state_mutex);
	}
}

int haptic_actuator_init(void)
{
	int ret;

	if (!device_is_ready(haptic_dev)) {
		LOG_ERR("DRV2605 device is not ready");
		return -ENODEV;
	}

	if (!i2c_is_ready_dt(&haptic_i2c)) {
		LOG_ERR("DRV2605 I2C bus is not ready");
		return -ENODEV;
	}

	ret = haptic_drv2605_enter_idle();
	if (ret < 0) {
		LOG_ERR("Failed to initialize DRV2605 idle state: %d", ret);
		return ret;
	}

	if (!haptic_thread_started) {
		k_thread_create(&haptic_playback_thread_data, haptic_playback_thread_stack,
				K_THREAD_STACK_SIZEOF(haptic_playback_thread_stack),
				haptic_playback_thread, NULL, NULL, NULL,
				HAPTIC_PLAYBACK_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&haptic_playback_thread_data, "haptic_playback");
		haptic_thread_started = true;
	}

	return 0;
}

int haptic_actuator_play_pattern(const struct haptic_actuator_frame *frames, size_t frame_count)
{
	size_t queued_count;

	if (frames == NULL || frame_count == 0U || frame_count > HAPTIC_ACTUATOR_MAX_FRAMES) {
		return -EINVAL;
	}

	k_mutex_lock(&haptic_state_mutex, K_FOREVER);

	if (haptic_is_playing) {
		k_mutex_unlock(&haptic_state_mutex);
		return -EBUSY;
	}

	for (size_t i = 0; i < frame_count; i++) {
		if (frames[i].duration_ms == 0U) {
			k_mutex_unlock(&haptic_state_mutex);
			return -EINVAL;
		}
	}

	memcpy(haptic_queued_frames, frames, frame_count * sizeof(haptic_queued_frames[0]));
	queued_count = frame_count;

	if (haptic_queued_frames[queued_count - 1U].intensity != 0U) {
		haptic_queued_frames[queued_count].duration_ms = HAPTIC_TRAILING_OFF_FRAME_MS;
		haptic_queued_frames[queued_count].intensity = 0U;
		queued_count++;
	}

	haptic_queued_frame_count = queued_count;
	haptic_is_playing = true;

	k_mutex_unlock(&haptic_state_mutex);

	k_sem_give(&haptic_playback_sem);
	return 0;
}

bool haptic_actuator_is_busy(void)
{
	bool is_busy;

	k_mutex_lock(&haptic_state_mutex, K_FOREVER);
	is_busy = haptic_is_playing;
	k_mutex_unlock(&haptic_state_mutex);

	return is_busy;
}
