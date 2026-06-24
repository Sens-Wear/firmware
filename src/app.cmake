# SPDX-License-Identifier: Apache-2.0
#
# Application sources and include directories, collected into the `app_src`
# INTERFACE library (mirrors the board_drivers.cmake pattern). The top-level
# CMakeLists links `app_src` into Zephyr's `app` target, so these sources are
# compiled as part of the application and the include directories / compile
# definitions propagate to it.
#
# Must be included after find_package(Zephyr) so CONFIG_* are available.

set(APP_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(app_src INTERFACE)

# Core application sources (always built).
target_sources(app_src INTERFACE
    ${APP_SRC_DIR}/main.c
    ${APP_SRC_DIR}/app/led_ble_bridge.c
    ${APP_SRC_DIR}/app/imu_ble_bridge.c
    ${APP_SRC_DIR}/app/daughter_board_manager.c
    ${APP_SRC_DIR}/app/power_ble_bridge.c
    ${APP_SRC_DIR}/bluetooth/services/led/led_lbs.c
    ${APP_SRC_DIR}/bluetooth/services/imu/imu_lbs.c
    ${APP_SRC_DIR}/bluetooth/services/power/power_lbs.c
    ${APP_SRC_DIR}/bluetooth/services/pressure/pressure_lbs.c
    ${APP_SRC_DIR}/drivers/actuators/led/led_controller.c
)

# Daughter-board sources, selected by the active SENSEWEAR_DAUGHTER_* choice.
if(CONFIG_SENSEWEAR_DAUGHTER_HAPTIC)
    target_sources(app_src INTERFACE
        ${APP_SRC_DIR}/app/haptic_ble_bridge.c
        ${APP_SRC_DIR}/bluetooth/services/haptic/haptic_lbs.c
    )
endif()

if(CONFIG_SENSEWEAR_DAUGHTER_PPG)
    target_sources(app_src INTERFACE
        ${APP_SRC_DIR}/app/ppg_ble_bridge.c
        ${APP_SRC_DIR}/bluetooth/services/ppg/ppg_lbs.c
    )
endif()

if(CONFIG_SENSEWEAR_DAUGHTER_TEMPERATURE)
    target_sources(app_src INTERFACE
        ${APP_SRC_DIR}/app/temperature_ble_bridge.c
        ${APP_SRC_DIR}/bluetooth/services/temperature/temperature_lbs.c
    )
endif()

if(CONFIG_SENSEWEAR_DAUGHTER_TOUCH)
    target_sources(app_src INTERFACE
        ${APP_SRC_DIR}/app/touch_ble_bridge.c
        ${APP_SRC_DIR}/bluetooth/services/touch/touch_lbs.c
    )
endif()

target_include_directories(app_src INTERFACE
    ${APP_SRC_DIR}
    ${APP_SRC_DIR}/drivers/actuators/led
    ${CMAKE_SOURCE_DIR}/boards/SenseraTechnologies/SensWear
    ${CMAKE_SOURCE_DIR}/boards/SenseraTechnologies/SensWear/drivers/bus
    ${CMAKE_SOURCE_DIR}/boards/SenseraTechnologies/SensWear/drivers/common
)

# The application uses the BHY2 sensor API; link the libs aggregate so the
# include directory and define propagate to app_src sources.
target_link_libraries(app_src INTERFACE libs)
