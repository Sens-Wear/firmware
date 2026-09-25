/* SPDX-License-Identifier: Apache-2.0 */
#include "mtch6102_slider.h"

#include <stddef.h>
#include <string.h>

void mtch6102_slider_config_defaults(struct mtch6102_slider_config* config) {
	memset(config, 0, sizeof(*config));
	for (uint8_t i = 0; i < MTCH6102_SLIDER_ELECTRODES; ++i) {
		config->physical_to_rx[i] = i;
	}
	config->physical_to_rx[1] = 2U;
	config->physical_to_rx[2] = 1U;
	config->x_channels = 12U;
	config->threshold_x = 55U;
	config->threshold_y = 40U;
	config->hysteresis = 4U;
	config->debounce_down = 1U;
	config->debounce_up = 1U;
	config->tap_distance = 25U;
	config->double_tap_distance = 64U;
	config->swipe_distance = 64U;
	config->swipe_hold_boundary = 25U;
	config->tap_hold_ms = 500U;
	config->double_tap_ms = 250U;
	config->swipe_hold_ms = 320U;
}

bool mtch6102_slider_config_valid(const struct mtch6102_slider_config* config) {
	if (config == NULL || config->x_channels > MTCH6102_SLIDER_ELECTRODES ||
		config->threshold_x == 0U || config->threshold_y == 0U ||
		config->hysteresis >= config->threshold_x || config->hysteresis >= config->threshold_y ||
		config->tap_hold_ms == 0U || config->double_tap_ms == 0U || config->swipe_hold_ms == 0U ||
		config->swipe_distance == 0U) {
		return false;
	}

	uint16_t seen = 0U;
	for (uint8_t i = 0; i < MTCH6102_SLIDER_ELECTRODES; ++i) {
		uint8_t rx = config->physical_to_rx[i];
		if (rx >= MTCH6102_SLIDER_ELECTRODES || (seen & (1U << rx)) != 0U) {
			return false;
		}
		seen |= (uint16_t) (1U << rx);
	}
	return true;
}

void mtch6102_slider_reset(struct mtch6102_slider_state* state) {
	memset(state, 0, sizeof(*state));
}

static uint16_t distance(uint16_t a, uint16_t b) {
	return a > b ? a - b : b - a;
}

static uint8_t threshold(const struct mtch6102_slider_config* config, uint8_t rx) {
	return rx < config->x_channels ? config->threshold_x : config->threshold_y;
}

static bool locate(const struct mtch6102_slider_config* config,
				   const uint8_t* values,
				   bool touched,
				   uint16_t* x) {
	int best = -1;
	uint8_t peak = 0U;
	for (uint8_t pad = 0; pad < MTCH6102_SLIDER_ELECTRODES; ++pad) {
		uint8_t rx = config->physical_to_rx[pad];
		int level = threshold(config, rx) - (touched ? config->hysteresis : 0U);
		int strength = values[rx] - level;
		if (strength > best) {
			best = strength;
			peak = pad;
		}
	}
	if (best < 0) {
		return false;
	}

	/* Use only the strongest pad and its immediate physical neighbours. A
	 * second, distant contact must not create a false coordinate between them.
	 * Subtract each RX bank's release threshold to interpolate across its seam.
	 */
	uint8_t first = peak == 0U ? 0U : peak - 1U;
	uint8_t last = peak + 1U < MTCH6102_SLIDER_ELECTRODES ? peak + 1U : peak;
	uint32_t weighted_x = 0U;
	uint32_t weight_sum = 0U;
	for (uint8_t pad = first; pad <= last; ++pad) {
		uint8_t rx = config->physical_to_rx[pad];
		int weight = values[rx] - (threshold(config, rx) - config->hysteresis) + 1;
		if (weight > 0) {
			weighted_x += (uint32_t) weight * pad * MTCH6102_SLIDER_PITCH;
			weight_sum += (uint32_t) weight;
		}
	}
	/* The selected peak contributes at least one even exactly at threshold. */
	*x = (uint16_t) ((weighted_x + weight_sum / 2U) / weight_sum);
	return true;
}

static void emit(struct mtch6102_slider_state* state,
				 struct mtch6102_slider_output* output,
				 uint8_t gesture) {
	if (output->gesture == mtch6102_gesture_None) {
		output->gesture = gesture;
	} else {
		/* A delayed first tap and the second contact's gesture may finish in
		 * the same frame. Deliver the second event on the following frame.
		 */
		state->queued_gesture = gesture;
	}
}

static void flush_tap(struct mtch6102_slider_state* state, struct mtch6102_slider_output* output) {
	if (state->tap_pending) {
		emit(state, output, mtch6102_gesture_SingleClick);
		state->tap_pending = false;
	}
	state->double_candidate = false;
}

static void begin_contact(struct mtch6102_slider_state* state,
						  const struct mtch6102_slider_config* config,
						  uint64_t now_ms,
						  struct mtch6102_slider_output* output) {
	state->start_x = state->x;
	state->stationary_x = state->x;
	state->start_ms = now_ms;
	state->stationary_ms = now_ms;
	state->max_travel = 0U;
	state->swipe = mtch6102_gesture_None;
	state->hold_sent = false;
	state->double_candidate = state->tap_pending &&
							  now_ms - state->tap_ms <= config->double_tap_ms &&
							  distance(state->x, state->tap_x) <= config->double_tap_distance;
	if (!state->double_candidate) {
		flush_tap(state, output);
	}
}

static void track_contact(struct mtch6102_slider_state* state,
						  const struct mtch6102_slider_config* config,
						  uint64_t now_ms,
						  struct mtch6102_slider_output* output) {
	uint16_t travel = distance(state->x, state->start_x);
	if (travel > state->max_travel) {
		state->max_travel = travel;
	}
	if (distance(state->x, state->stationary_x) > config->swipe_hold_boundary) {
		state->stationary_x = state->x;
		state->stationary_ms = now_ms;
	}
	if (state->hold_sent) {
		return;
	}
	if (travel >= config->swipe_distance && state->swipe == mtch6102_gesture_None) {
		state->swipe = state->x > state->start_x ? mtch6102_gesture_RightSwipe
												 : mtch6102_gesture_LeftSwipe;
		state->stationary_x = state->x;
		state->stationary_ms = now_ms;
	}
	uint8_t gesture = mtch6102_gesture_None;
	if (state->swipe != mtch6102_gesture_None &&
		now_ms - state->stationary_ms >= config->swipe_hold_ms) {
		gesture = state->swipe == mtch6102_gesture_RightSwipe ? mtch6102_gesture_RightSwipeAndHold
															  : mtch6102_gesture_LeftSwipeAndHold;
	} else if (state->max_travel <= config->tap_distance &&
			   now_ms - state->start_ms >= config->tap_hold_ms) {
		gesture = mtch6102_gesture_ClickAndHold;
	}
	if (gesture != mtch6102_gesture_None) {
		flush_tap(state, output);
		emit(state, output, gesture);
		state->hold_sent = true;
	}
}

static void end_contact(struct mtch6102_slider_state* state,
						const struct mtch6102_slider_config* config,
						uint64_t now_ms,
						struct mtch6102_slider_output* output) {
	if (state->hold_sent) {
		return;
	}
	if (state->swipe != mtch6102_gesture_None) {
		flush_tap(state, output);
		emit(state, output, state->swipe);
	} else if (state->max_travel <= config->tap_distance &&
			   now_ms - state->start_ms < config->tap_hold_ms) {
		if (state->double_candidate) {
			state->tap_pending = false;
			state->double_candidate = false;
			emit(state, output, mtch6102_gesture_DoubleClick);
		} else {
			state->tap_pending = true;
			state->tap_x = state->x;
			state->tap_ms = now_ms;
		}
	} else {
		flush_tap(state, output);
	}
}

void mtch6102_slider_process(struct mtch6102_slider_state* state,
							 const struct mtch6102_slider_config* config,
							 const uint8_t sensor_values[MTCH6102_SLIDER_ELECTRODES],
							 uint8_t touch_state,
							 uint64_t now_ms,
							 bool force_reset,
							 struct mtch6102_slider_output* output) {
	memset(output, 0, sizeof(*output));
	output->position.touch_state = touch_state;
	if (!mtch6102_slider_config_valid(config)) {
		mtch6102_slider_reset(state);
		return;
	}
	if (force_reset || (state->has_time && now_ms < state->last_ms)) {
		mtch6102_slider_reset(state);
	}
	state->has_time = true;
	state->last_ms = now_ms;
	output->gesture = state->queued_gesture;
	state->queued_gesture = mtch6102_gesture_None;
	if (state->tap_pending && !state->touched && now_ms - state->tap_ms > config->double_tap_ms) {
		flush_tap(state, output);
	}

	bool was_touched = state->touched;
	uint16_t x = state->x;
	bool present = locate(config, sensor_values, state->touched, &x);
	if (present) {
		state->up_frames = 0U;
		if (state->down_frames < UINT8_MAX) {
			++state->down_frames;
		}
		if (state->down_frames >= config->debounce_down) {
			state->touched = true;
		}
		state->x = x;
	} else {
		state->down_frames = 0U;
		if (state->up_frames < UINT8_MAX) {
			++state->up_frames;
		}
		if (state->up_frames >= config->debounce_up) {
			state->touched = false;
		}
	}
	if (!was_touched && state->touched) {
		begin_contact(state, config, now_ms, output);
	} else if (was_touched && !state->touched) {
		end_contact(state, config, now_ms, output);
	} else if (state->touched) {
		track_contact(state, config, now_ms, output);
	}
	output->position.touched = state->touched;
	output->position.x = state->touched ? state->x : 0U;
}
