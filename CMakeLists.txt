# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(firmware)

# Add BHY2 sensor API source files
file(GLOB BHY2_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BHY2-Sensor-API/*.c"
)

target_sources(app PRIVATE
    src/main.c
    src/app/led_ble_bridge.c
    src/app/imu_ble_bridge.c
    src/app/power_ble_bridge.c
    src/bluetooth/services/led/led_lbs.c
    src/bluetooth/services/imu/imu_lbs.c
    src/bluetooth/services/power/power_lbs.c
    src/drivers/sensors/pressure/fdc1004.c
    src/drivers/actuators/led/led_controller.c
    src/drivers/actuators/led/LP5562/lp5562.c
    src/drivers/actuators/haptic/drv2605.c
    src/drivers/memory/eeprom/m95p.c
    src/drivers/power/charger/bq25180.c
    src/drivers/sensors/imu/common.c
    src/drivers/sensors/imu/bhi360.c
    src/drivers/sensors/temperature/max30208.c
    src/drivers/sensors/touch/mtch6102.c
    src/drivers/sensors/ppg/max30101.c
    src/drivers/power/regulator/tpsm83102.c
    src/drivers/power/gauge/bq27427.c
    src/bluetooth/services/pressure/pressure_lbs.c
    ${BHY2_SOURCES}
)

# Include directories - simplified paths
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/sensors/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BHY2-Sensor-API
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/sensors/temperature/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/actuators/led/
)

# Add compile definitions if needed
target_compile_definitions(app PRIVATE
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)
