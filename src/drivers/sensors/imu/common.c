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
        case 0x00: return "No error";
        case 0x10: return "Firmware expected version mismatch";
        case 0x11: return "Firmware upload failed: bad header CRC";
        case 0x12: return "Firmware upload failed: SHA hash mismatch";
        case 0x13: return "Firmware upload failed: bad image CRC";
        case 0x14: return "Firmware upload failed: ECDSA signature verification failed";
        case 0x15: return "Firmware upload failed: bad public key CRC";
        case 0x16: return "Firmware upload failed: signed firmware required";
        case 0x17: return "Firmware upload failed: firmware header missing";
        case 0x19: return "Unexpected watchdog reset";
        case 0x1A: return "ROM version mismatch";
        case 0x1B: return "Fatal firmware error";
        case 0x1C: return "Chained firmware error: next payload not found";
        case 0x1D: return "Chained firmware error: payload not valid";
        case 0x1E: return "Chained firmware error: payload entries invalid";
        case 0x1F: return "Bootloader error: OTP CRC invalid";
        case 0x20: return "Firmware init failed";
        case 0x21: return "Sensor init failed: unexpected device ID";
        case 0x22: return "Sensor init failed: no response from device";
        case 0x23: return "Sensor init failed: unknown";
        case 0x24: return "Sensor error: no valid data";
        case 0x25: return "Slow sample rate";
        case 0x26: return "Data overflow";
        case 0x27: return "Stack overflow";
        case 0x28: return "Insufficient free RAM";
        case 0x29: return "Sensor init failed: driver parsing error";
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
