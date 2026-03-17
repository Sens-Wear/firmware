#ifndef DRV2605_H
#define DRV2605_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAPTIC_ACTUATOR_MAX_FRAMES 64U

struct haptic_actuator_frame {
	uint16_t duration_ms;
	uint8_t intensity;
};

int haptic_actuator_init(void);
int haptic_actuator_play_pattern(const struct haptic_actuator_frame *frames, size_t frame_count);
bool haptic_actuator_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif
