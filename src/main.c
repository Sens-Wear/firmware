/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>

#include "app/led_ble_bridge.h"
#include "app/imu_ble_bridge.h"
#include "app/daughter_board_manager.h"
#include "app/power_ble_bridge.h"
#include "bluetooth/services/imu/imu_lbs.h"
#include "bluetooth/services/led/led_lbs.h"
#include "bluetooth/services/power/power_lbs.h"

#include "drivers/memory/eeprom/m95p.h"
#include "drivers/power/charger/bq25180.h"
#include "drivers/power/gauge/bq27427.h"
#include "drivers/sensors/imu/bhi360.h"

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
#include "app/haptic_ble_bridge.h"
#include "bluetooth/services/haptic/haptic_lbs.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
#include "app/ppg_ble_bridge.h"
#include "bluetooth/services/ppg/ppg_lbs.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
#include "app/temperature_ble_bridge.h"
#include "bluetooth/services/temperature/temperature_lbs.h"
#endif

#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
#include "app/touch_ble_bridge.h"
#include "bluetooth/services/touch/touch_lbs.h"
#endif

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	800, /* Min Advertising Interval 500ms (800*0.625ms) */
	801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
	NULL); /* Set to NULL for undirected advertising */

LOG_MODULE_REGISTER(SENSE_WEAR_LOGGER);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define BT_UUID_LBS_VAL BT_UUID_128_ENCODE(0x56966294, 0x9cb8, 0x4c92, 0x9d74, 0x834187f486de)

static void adv_restart_work_fn(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_fn);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

static struct bt_conn *current_conn = NULL;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void adv_restart_work_fn(struct k_work *work)
{
    int err;

    /* If advertising might still be on for any reason, stop it first.
     * Ignore "not advertising" type errors.
     */
    err = bt_le_adv_stop();
    if (err && err != -EALREADY && err != -EINVAL) {
        LOG_WRN("bt_le_adv_stop returned %d", err);
    }

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising restart failed (err %d)", err);
    } else {
        LOG_INF("Advertising restarted");
    }
}

void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_INF("Connection error %d", err);
        return;
    }
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
	imu_lbs_set_conn(conn);
	led_lbs_set_conn(conn);
	power_lbs_set_conn(conn);
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
	haptic_lbs_set_conn(conn);
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	ppg_lbs_set_conn(conn);
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
	temperature_lbs_set_conn(conn);
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
	touch_lbs_set_conn(conn);
#endif
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected. Reason %u", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
	imu_lbs_clear_conn();
	led_lbs_clear_conn();
	power_lbs_clear_conn();
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
	haptic_lbs_clear_conn();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	ppg_lbs_clear_conn();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
	temperature_lbs_clear_conn();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
	touch_lbs_clear_conn();
#endif

	/* Defer restart briefly so the stack has time to finish disconnect cleanup. */
	k_work_schedule(&adv_restart_work, K_MSEC(1000));
}


struct bt_conn_cb connection_callbacks = {
    .connected              = on_connected,
    .disconnected           = on_disconnected,
};

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	LOG_INF("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            LOG_ERR("Settings load failed (err %d)", err);
        }
    }
	bt_conn_cb_register(&connection_callbacks);
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}

	// static struct bq25180_config_t bq25180_config_val;
	// static union bq25180_charger_state_t bq25180_state;
	// bq25180_init();
	// bq25180_reset(NULL);
	// bq25180_get_default_lipo_usb_charger_config(&bq25180_config_val);
	// bq25180_config_val.battery_uvlo = bq25180_battery_UVLO_threshold_2V8;

	// bq25180_config(&bq25180_config_val);
	// bq25180_update_state(&bq25180_state);
	// bq25180_print_state();

	// // Initialize battery gauge
	// static struct bq27427_config_t bq27427_config_val;
	// bq27427_init();
	// bq27427_reset(NULL);
	// bq27427_get_default_config(&bq27427_config_val);
	// bq27427_config_val.battery_capacity = 450;
	// // bq27427_config_val.battery_type = bq27427_chemistry_Lipo4V35;
	// bq27427_config(&bq27427_config_val);
	// bq27427_update_state(NULL);
	// bq27427_print_state();

	// Initialize touch sensor
    // touch_sensor_init();

	// Initialize PPG sensor
	// ppg_sensor_init();

	// haptic_actuator_init();
	// led_controller_init();
	// led_controller_configure();

	err = daughter_board_manager_init();
	if (err) {
		LOG_ERR("Daughter board manager init failed: %d", err);
	}

	led_ble_bridge_init();
    imu_ble_bridge_init();
	power_ble_bridge_init();
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
	temperature_ble_bridge_init();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_PPG)
	ppg_ble_bridge_init();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
	touch_ble_bridge_init();
#endif
#if IS_ENABLED(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
	haptic_ble_bridge_init();
#endif


	sys_memory_init();
	test_memory();
		

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
