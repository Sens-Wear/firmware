#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/sys/util.h>
#include <sys/errno.h>

#include "bq25180.h"

LOG_MODULE_REGISTER(SENSE_WEAR_CHARGER_POWER_LOGGER);

static const struct device *chgdev = DEVICE_DT_GET(DT_NODELABEL(bq25180));
static const struct i2c_dt_spec bq_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(bq25180));

// Initialize the sensor thread
int charger_power_init(void)
{
	union charger_propval val;
	int ret;

	if (chgdev == NULL) {
		LOG_INF("Error: no charger device found.\n");
		return 0;
	}

	if (!device_is_ready(chgdev)) {
		LOG_INF("Error: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       chgdev->name);
		return 0;
	}

	LOG_INF("Found device \"%s\", getting charger data\n", chgdev->name);

#ifdef CONFIG_CHARGER_DISCHARGE_CURRENT_NOTIFICATIONS
	val.discharge_current_notification.current_ua =
		CONFIG_APP_DISCHARGE_CURRENT_NOTIFICATION_THRESHOLD_UA;
	val.discharge_current_notification.severity = CHARGER_SEVERITY_WARNING;
	val.discharge_current_notification.duration_us =
		CONFIG_APP_DISCHARGE_CURRENT_NOTIFICATION_DURATION_US;

	ret = charger_set_prop(chgdev, CHARGER_PROP_DISCHARGE_CURRENT_NOTIFICATION, &val);
	if (ret < 0) {
		return ret;
	}
#endif

#ifdef CONFIG_CHARGER_SYSTEM_VOLTAGE_NOTIFICATIONS
	val.system_voltage_notification =
		CONFIG_APP_SYSTEM_VOLTAGE_NOTIFICATION_THRESHOLD_UV;

	ret = charger_set_prop(chgdev, CHARGER_PROP_SYSTEM_VOLTAGE_NOTIFICATION_UV, &val);
	if (ret < 0) {
		return ret;
	}
#endif
	while (1) {
		/* Poll until external power is presented to the charger */
		do {
			ret = charger_get_prop(chgdev, CHARGER_PROP_ONLINE, &val);
			if (ret < 0) {
				return ret;
			}

			k_msleep(100);
		} while (val.online == CHARGER_ONLINE_OFFLINE);

		val.status = CHARGER_STATUS_CHARGING;

		ret = charger_charge_enable(chgdev, true);
		if (ret == -ENOTSUP) {
			LOG_INF("Enabling charge not supported, assuming auto charge enable\n");
			continue;
		} else if (ret < 0) {
			return ret;
		}

		k_msleep(500);

        ret = charger_get_prop(chgdev, CHARGER_PROP_ONLINE, &val);
		if (ret < 0) {
			return ret;
		}
        LOG_INF("CHARGER_PROP_ONLINE is \"%d\"", val.status);

		ret = charger_get_prop(chgdev, CHARGER_PROP_STATUS, &val);
		if (ret < 0) {
			return ret;
		}
        LOG_INF("CHARGER_STATUS_CHARGING is \"%d\"", val.status);
		switch (val.status) {
            case CHARGER_STATUS_CHARGING:
                LOG_INF("Charging in progress...\n");

                ret = charger_get_prop(chgdev, CHARGER_PROP_CHARGE_TYPE, &val);
                if (ret < 0) {
                    return ret;
                }

                LOG_INF("Device \"%s\" charge type is %d\n", chgdev->name, val.charge_type);
                break;
            case CHARGER_STATUS_NOT_CHARGING:
                LOG_INF("Charging halted...\n");

                ret = charger_get_prop(chgdev, CHARGER_PROP_HEALTH, &val);
                if (ret < 0) {
                    return ret;
                }

                LOG_INF("Device \"%s\" health is %d\n", chgdev->name, val.health);
                break;
            case CHARGER_STATUS_FULL:
                LOG_INF("Charging complete!");
                return 0;
            case CHARGER_STATUS_DISCHARGING:
                LOG_INF("External power removed, discharging\n");

                ret = charger_get_prop(chgdev, CHARGER_PROP_ONLINE, &val);
                if (ret < 0) {
                    return ret;
                }
                break;
            default:
                return -EIO;
		}

		k_msleep(500);
	}
}


