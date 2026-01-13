#include "common.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(COMMON);

const char *get_api_error(int8_t error_code)
{
    switch (error_code)
    {
        case BHY2_OK: return "BHY2_OK";
        case BHY2_E_NULL_PTR: return "BHY2_E_NULL_PTR";
        case BHY2_E_INVALID_PARAM: return "BHY2_E_INVALID_PARAM";
        case BHY2_E_IO: return "BHY2_E_IO";
        case BHY2_E_MAGIC: return "BHY2_E_MAGIC";
        case BHY2_E_TIMEOUT: return "BHY2_E_TIMEOUT";
        case BHY2_E_BUFFER: return "BHY2_E_BUFFER";
        default: return "Unknown error code";
    }
}

const char *get_sensor_error_text(uint8_t sensor_error)
{
    switch (sensor_error)
    {
        case 0: return "No error";
        default: return "Unknown sensor error";
    }
}

const char *get_sensor_name(uint8_t sensor_id)
{
    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_RV: return "Rotation Vector";
        default: return "Unknown sensor";
    }
} 