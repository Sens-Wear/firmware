#ifndef DAUGHTER_BOARD_MANAGER_H_
#define DAUGHTER_BOARD_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "bluetooth/services/power/power_lbs.h"

typedef void (*daughter_board_manager_status_cb_t)(const struct power_lbs_daughter_state *state,
						   void *user_data);

int daughter_board_manager_init(void);
int daughter_board_manager_get_status(struct power_lbs_daughter_state *state);
int daughter_board_manager_set_ppg_active(bool active);
void daughter_board_manager_register_status_cb(daughter_board_manager_status_cb_t cb,
					       void *user_data);

#ifdef __cplusplus
}
#endif

#endif
