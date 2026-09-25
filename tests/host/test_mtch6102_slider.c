/* SPDX-License-Identifier: Apache-2.0 */
#include "mtch6102_slider.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

struct fixture {
	struct mtch6102_slider_config config;
	struct mtch6102_slider_state state;
	struct mtch6102_slider_output output;
	uint8_t values[MTCH6102_SLIDER_ELECTRODES];
};

static void init(struct fixture* f) {
	memset(f, 0, sizeof(*f));
	mtch6102_slider_config_defaults(&f->config);
	mtch6102_slider_reset(&f->state);
	assert(mtch6102_slider_config_valid(&f->config));
}

static void set_pad(struct fixture* f, uint8_t pad, uint8_t excess) {
	uint8_t rx = f->config.physical_to_rx[pad];
	f->values[rx] =
		(rx < f->config.x_channels ? f->config.threshold_x : f->config.threshold_y) + excess;
}

static void frame(struct fixture* f, uint64_t now_ms) {
	/* A clear TCH bit must never hide touches confined to either RX bank. */
	mtch6102_slider_process(&f->state, &f->config, f->values, 0xA0U, now_ms, false, &f->output);
	assert(f->output.position.y == 0U);
	assert(f->output.position.touch_state == 0xA0U);
}

static void touch(struct fixture* f, uint8_t pad, uint64_t now_ms) {
	memset(f->values, 0, sizeof(f->values));
	set_pad(f, pad, 40U);
	frame(f, now_ms);
	assert(f->output.position.touched);
	assert(f->output.position.x == pad * MTCH6102_SLIDER_PITCH);
}

static void release(struct fixture* f, uint64_t now_ms) {
	memset(f->values, 0, sizeof(f->values));
	frame(f, now_ms);
	assert(!f->output.position.touched);
	assert(f->output.position.x == 0U);
}

static void test_electrodes(void) {
	struct fixture f;
	for (uint8_t pad = 0; pad < MTCH6102_SLIDER_ELECTRODES; ++pad) {
		init(&f);
		touch(&f, pad, 0U);
		release(&f, 20U);
	}
	assert(f.config.physical_to_rx[0] == 0U);
	assert(f.config.physical_to_rx[1] == 2U);
	assert(f.config.physical_to_rx[2] == 1U);
	assert(MTCH6102_SLIDER_MAX_X == 896U);

	/* An arbitrary legal permutation must also produce physical coordinates. */
	for (uint8_t pad = 0; pad < MTCH6102_SLIDER_ELECTRODES; ++pad) {
		init(&f);
		for (uint8_t i = 0; i < MTCH6102_SLIDER_ELECTRODES; ++i) {
			f.config.physical_to_rx[i] = MTCH6102_SLIDER_ELECTRODES - 1U - i;
		}
		touch(&f, pad, 0U);
	}
}

static void test_centroid(void) {
	struct fixture f;
	init(&f);
	set_pad(&f, 11U, 40U);
	set_pad(&f, 12U, 40U);
	frame(&f, 0U);
	assert(f.output.position.touched);
	assert(f.output.position.x == 11U * MTCH6102_SLIDER_PITCH + MTCH6102_SLIDER_PITCH / 2U);

	init(&f);
	set_pad(&f, 0U, 60U);
	set_pad(&f, 14U, 40U);
	frame(&f, 0U);
	assert(f.output.position.x == 0U);
	init(&f);
	set_pad(&f, 0U, 40U);
	set_pad(&f, 14U, 60U);
	frame(&f, 0U);
	assert(f.output.position.x == MTCH6102_SLIDER_MAX_X);

	/* Exact threshold and release threshold must have nonzero centroid weight. */
	init(&f);
	f.values[0] = 55U;
	frame(&f, 0U);
	assert(f.output.position.touched);
	f.values[0] = 51U;
	frame(&f, 10U);
	assert(f.output.position.touched);
	f.values[0] = 50U;
	frame(&f, 20U);
	assert(!f.output.position.touched);
	init(&f);
	f.values[14] = 40U;
	frame(&f, 0U);
	assert(f.output.position.touched);
	f.values[14] = 36U;
	frame(&f, 10U);
	assert(f.output.position.touched);
	f.values[14] = 35U;
	frame(&f, 20U);
	assert(!f.output.position.touched);
	init(&f);
	f.values[0] = 54U;
	f.values[14] = 39U;
	frame(&f, 0U);
	assert(!f.output.position.touched);
	release(&f, 10U);
}

static void test_debounce_and_status(void) {
	struct fixture f;
	init(&f);
	f.config.debounce_down = 2U;
	f.config.debounce_up = 2U;
	set_pad(&f, 3U, 40U);
	frame(&f, 0U);
	assert(!f.output.position.touched);
	frame(&f, 10U);
	assert(f.output.position.touched);
	memset(f.values, 0, sizeof(f.values));
	frame(&f, 20U);
	assert(f.output.position.touched);
	assert(f.output.position.x == 3U * MTCH6102_SLIDER_PITCH);
	frame(&f, 30U);
	assert(!f.output.position.touched);

	/* Even a set hardware TCH bit cannot synthesize a host touch from zeros. */
	init(&f);
	mtch6102_slider_process(&f.state, &f.config, f.values, 0xFFU, 0U, false, &f.output);
	assert(!f.output.position.touched);
	assert(f.output.position.touch_state == 0xFFU);
}

static void test_taps(void) {
	struct fixture f;
	init(&f);
	touch(&f, 2U, 0U);
	release(&f, 60U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 310U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 311U);
	assert(f.output.gesture == mtch6102_gesture_SingleClick);
	frame(&f, 500U);
	assert(f.output.gesture == mtch6102_gesture_None);

	init(&f);
	touch(&f, 2U, 0U);
	release(&f, 60U);
	touch(&f, 2U, 200U);
	release(&f, 230U);
	assert(f.output.gesture == mtch6102_gesture_DoubleClick);
	frame(&f, 800U);
	assert(f.output.gesture == mtch6102_gesture_None);

	/* A distant second tap does not become a double click. */
	init(&f);
	touch(&f, 2U, 0U);
	release(&f, 60U);
	touch(&f, 10U, 100U);
	assert(f.output.gesture == mtch6102_gesture_SingleClick);
	release(&f, 150U);
	frame(&f, 401U);
	assert(f.output.gesture == mtch6102_gesture_SingleClick);
}

static void test_hold_and_swipes(void) {
	struct fixture f;
	init(&f);
	touch(&f, 0U, 0U);
	frame(&f, 499U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 500U);
	assert(f.output.gesture == mtch6102_gesture_ClickAndHold);
	frame(&f, 1000U);
	assert(f.output.gesture == mtch6102_gesture_None);
	release(&f, 1100U);
	frame(&f, 1500U);
	assert(f.output.gesture == mtch6102_gesture_None);

	for (uint8_t direction = 0U; direction < 2U; ++direction) {
		uint8_t start = direction == 0U ? 2U : 8U;
		uint8_t finish = direction == 0U ? 8U : 2U;
		init(&f);
		touch(&f, start, 0U);
		touch(&f, finish, 60U);
		assert(f.output.gesture == mtch6102_gesture_None);
		release(&f, 100U);
		assert(f.output.gesture ==
			   (direction == 0U ? mtch6102_gesture_RightSwipe : mtch6102_gesture_LeftSwipe));
		frame(&f, 500U);
		assert(f.output.gesture == mtch6102_gesture_None);
		init(&f);
		touch(&f, start, 0U);
		touch(&f, finish, 60U);
		frame(&f, 379U);
		assert(f.output.gesture == mtch6102_gesture_None);
		frame(&f, 380U);
		assert(f.output.gesture == (direction == 0U ? mtch6102_gesture_RightSwipeAndHold
													: mtch6102_gesture_LeftSwipeAndHold));
		frame(&f, 500U);
		assert(f.output.gesture == mtch6102_gesture_None);
		release(&f, 600U);
		assert(f.output.gesture == mtch6102_gesture_None);
	}

	/* Preserve both gestures when a potential second tap becomes a swipe. */
	init(&f);
	touch(&f, 2U, 0U);
	release(&f, 60U);
	touch(&f, 2U, 100U);
	touch(&f, 8U, 160U);
	release(&f, 200U);
	assert(f.output.gesture == mtch6102_gesture_SingleClick);
	frame(&f, 210U);
	assert(f.output.gesture == mtch6102_gesture_RightSwipe);
}

static void test_reset_and_time(void) {
	struct fixture f;
	init(&f);
	touch(&f, 3U, 0U);
	release(&f, 60U);
	mtch6102_slider_process(&f.state, &f.config, f.values, 0U, 500U, true, &f.output);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 1000U);
	assert(f.output.gesture == mtch6102_gesture_None);
	touch(&f, 3U, 2000U);
	mtch6102_slider_process(&f.state, &f.config, f.values, 0U, 3000U, true, &f.output);
	assert(f.output.position.touched);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 3499U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 3500U);
	assert(f.output.gesture == mtch6102_gesture_ClickAndHold);
	frame(&f, 100U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, 600U);
	assert(f.output.gesture == mtch6102_gesture_ClickAndHold);

	init(&f);
	uint64_t start_ms = (uint64_t) UINT32_MAX - 100U;
	touch(&f, 2U, start_ms);
	frame(&f, start_ms + 499U);
	assert(f.output.gesture == mtch6102_gesture_None);
	frame(&f, start_ms + 500U);
	assert(f.output.gesture == mtch6102_gesture_ClickAndHold);
}

static void test_invalid_config(void) {
	struct fixture f;
	init(&f);
	f.config.physical_to_rx[0] = 15U;
	assert(!mtch6102_slider_config_valid(&f.config));
	frame(&f, 0U);
	assert(!f.output.position.touched);
	init(&f);
	f.config.physical_to_rx[0] = f.config.physical_to_rx[1];
	assert(!mtch6102_slider_config_valid(&f.config));
	init(&f);
	f.config.threshold_x = 0U;
	assert(!mtch6102_slider_config_valid(&f.config));
	frame(&f, 0U);
	assert(!f.output.position.touched);
	init(&f);
	f.config.hysteresis = f.config.threshold_y;
	assert(!mtch6102_slider_config_valid(&f.config));
}

int main(void) {
	test_electrodes();
	test_centroid();
	test_debounce_and_status();
	test_taps();
	test_hold_and_swipes();
	test_reset_and_time();
	test_invalid_config();
	puts("MTCH6102 slider tests passed");
	return 0;
}
