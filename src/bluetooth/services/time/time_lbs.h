#ifndef TIME_LBS_H_
#define TIME_LBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#define BT_UUID_LBS_CURRENT_TIME_SERVICE_VAL 0x1805
#define BT_UUID_LBS_CURRENT_TIME_VAL 0x2A2B
#define BT_UUID_LBS_LOCAL_TIME_INFORMATION_VAL 0x2A0F
#define BT_UUID_LBS_REFERENCE_TIME_INFORMATION_VAL 0x2A14

#define BT_UUID_LBS_CURRENT_TIME_SERVICE BT_UUID_DECLARE_16(BT_UUID_LBS_CURRENT_TIME_SERVICE_VAL)
#define BT_UUID_LBS_CURRENT_TIME BT_UUID_DECLARE_16(BT_UUID_LBS_CURRENT_TIME_VAL)
#define BT_UUID_LBS_LOCAL_TIME_INFORMATION BT_UUID_DECLARE_16(BT_UUID_LBS_LOCAL_TIME_INFORMATION_VAL)
#define BT_UUID_LBS_REFERENCE_TIME_INFORMATION BT_UUID_DECLARE_16(BT_UUID_LBS_REFERENCE_TIME_INFORMATION_VAL)

enum time_lbs_adjust_reason {
	TIME_LBS_ADJUST_REASON_MANUAL_TIME_UPDATE = BIT(0),
	TIME_LBS_ADJUST_REASON_EXTERNAL_REFERENCE_TIME_UPDATE = BIT(1),
	TIME_LBS_ADJUST_REASON_CHANGE_OF_TIME_ZONE = BIT(2),
	TIME_LBS_ADJUST_REASON_CHANGE_OF_DST = BIT(3),
};

struct time_lbs_exact_time_256 {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t day_of_week;
	uint8_t fractions256;
} __packed;

struct time_lbs_current_time {
	struct time_lbs_exact_time_256 exact_time_256;
	uint8_t adjust_reason;
} __packed;

struct time_lbs_local_time_information {
	int8_t time_zone;
	uint8_t dst_offset;
} __packed;

struct time_lbs_reference_time_information {
	uint8_t source;
	uint8_t accuracy;
	uint8_t days_since_update;
	uint8_t hours_since_update;
} __packed;

void time_lbs_set_conn(struct bt_conn* conn);
void time_lbs_clear_conn(void);

#ifdef __cplusplus
}
#endif

#endif
