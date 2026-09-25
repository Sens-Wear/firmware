/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SENSWEAR_MTCH6102_SLIDER_H_
#define SENSWEAR_MTCH6102_SLIDER_H_

#include "mtch6102_registers.h"

#define MTCH6102_SLIDER_ELECTRODES 15U
#define MTCH6102_SLIDER_PITCH 64U
#define MTCH6102_SLIDER_MAX_X ((MTCH6102_SLIDER_ELECTRODES - 1U) * MTCH6102_SLIDER_PITCH)

/* SENSORVALUES are positive, baseline-subtracted channel signals. The map is
 * indexed by physical pad order, beginning at x=0, and contains RX indices.
 */
struct mtch6102_slider_config {
	uint8_t physical_to_rx[MTCH6102_SLIDER_ELECTRODES];
	uint8_t x_channels;
	uint8_t threshold_x;
	uint8_t threshold_y;
	uint8_t hysteresis;
	uint8_t debounce_down;
	uint8_t debounce_up;
	uint16_t tap_distance;
	uint16_t double_tap_distance;
	uint16_t swipe_distance;
	uint16_t swipe_hold_boundary;
	uint32_t tap_hold_ms;
	uint32_t double_tap_ms;
	uint32_t swipe_hold_ms;
};

/* Caller-owned state; initialize with mtch6102_slider_reset(). */
struct mtch6102_slider_state {
	bool touched;
	bool has_time;
	bool hold_sent;
	bool tap_pending;
	bool double_candidate;
	uint8_t down_frames;
	uint8_t up_frames;
	uint8_t swipe;
	uint8_t queued_gesture;
	uint16_t x;
	uint16_t start_x;
	uint16_t max_travel;
	uint16_t stationary_x;
	uint16_t tap_x;
	uint64_t last_ms;
	uint64_t start_ms;
	uint64_t stationary_ms;
	uint64_t tap_ms;
};

struct mtch6102_slider_output {
	struct mtch6102_position position;
	uint8_t gesture;
};

/* Defaults retain the existing RX-bank thresholds and use the SensWear touch
 * board's connector-to-tip pad order (RX0, RX2, RX1, RX3, ..., RX14).
 * Gesture times are explicit host-side milliseconds, independent of the
 * controller's gesture configuration register encoding.
 */
void mtch6102_slider_config_defaults(struct mtch6102_slider_config* config);
bool mtch6102_slider_config_valid(const struct mtch6102_slider_config* config);
void mtch6102_slider_reset(struct mtch6102_slider_state* state);

/* Process one complete acquisition frame, including untouched frames so that
 * delayed single taps can complete. touch_state is diagnostic only. Each
 * gesture is emitted once; a single tap waits for the double-tap window.
 * Reset on resume/reconfiguration to discard stale contacts and pending taps.
 * Invalid configuration returns an untouched result without indexing the map.
 */
void mtch6102_slider_process(struct mtch6102_slider_state* state,
							 const struct mtch6102_slider_config* config,
							 const uint8_t sensor_values[MTCH6102_SLIDER_ELECTRODES],
							 uint8_t touch_state,
							 uint64_t now_ms,
							 bool force_reset,
							 struct mtch6102_slider_output* output);

#endif /* SENSWEAR_MTCH6102_SLIDER_H_ */
