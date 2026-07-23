#ifndef FDC1004_H
#define FDC1004_H

#include <zephyr/kernel.h>

// Define buffer size (10 readings for 1 second)
#define SENSOR_BUFFER_SIZE 10  

// Function to initialize sensor reading
void pressure_sensor_init(void);

// Function to dynamically change the data collection frequency
void set_pressure_sensor_frequency(uint16_t hz);

// Function to dynamically change BLE transmission frequency
void set_pressure_ble_frequency(uint16_t hz);

#endif // SENSOR_H
