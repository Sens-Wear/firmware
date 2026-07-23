# SPDX-License-Identifier: Apache-2.0
#
# Bluetooth GATT services, collected into the `ble_services` INTERFACE library
# (mirrors the board_drivers.cmake pattern). Core services are always built;
# daughter-board services are gated on their CONFIG_SENSWEAR_DAUGHTER_*
# Kconfig choice, so include this after find_package(Zephyr).
#
# The interface include directory is the src/ root, so consumers include
# headers as "bluetooth/services/<name>/<name>_lbs.h".
#
# Included from both the top-level CMakeLists and src/app.cmake; the guard
# keeps the second include from redefining the target.

include_guard(GLOBAL)

set(BLE_SERVICES_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(ble_services INTERFACE)

function(add_ble_service_dir service_name)
    set(service_dir ${BLE_SERVICES_DIR}/${service_name})

    file(GLOB service_sources CONFIGURE_DEPENDS
        ${service_dir}/*.c
    )

    target_sources(ble_services INTERFACE
        ${service_sources}
    )
    target_include_directories(ble_services INTERFACE
        ${service_dir}
    )
endfunction()

# Core services (always built).
add_ble_service_dir(imu)
add_ble_service_dir(led)
add_ble_service_dir(power)
add_ble_service_dir(time)

target_include_directories(ble_services INTERFACE
    ${CMAKE_SOURCE_DIR}/boards/common/device_manager
)

# Daughter-board services, selected by the active SENSWEAR_DAUGHTER_* choice.
if(CONFIG_SENSWEAR_DAUGHTER_HAPTIC)
    add_ble_service_dir(haptic)
endif()

if(CONFIG_SENSWEAR_DAUGHTER_PPG)
    add_ble_service_dir(ppg)
endif()

if(CONFIG_SENSWEAR_DAUGHTER_TOUCH)
    add_ble_service_dir(touch)
endif()

# Shield-backed services, selected by the active shield config.
if(CONFIG_SHIELD_SENSWEAR_TEMPERATURE)
    add_ble_service_dir(body_temperature)
endif()
