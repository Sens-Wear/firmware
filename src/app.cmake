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

# Bluetooth GATT services (ble_services INTERFACE library).
include(${APP_SRC_DIR}/bluetooth/services/services.cmake)

add_library(app_src INTERFACE)

# Core application sources (always built). There is no main.c: the out-of-box
# monitor thread in oob_main.c owns the application bring-up and Zephyr's weak
# default main() is used. The LED actuator driver (src/drivers/actuators/led)
# is not compiled for now; it will be placed appropriately later.
target_sources(app_src INTERFACE
    ${APP_SRC_DIR}/app/oob_main.c
)

target_include_directories(app_src INTERFACE
    ${APP_SRC_DIR}
)

# ble_services carries the GATT service sources, their include directory, and
# common_drivers (device_manager.h). Device driver headers (bq27427.h,
# max30101.h, ...) come from the board/shield driver aggregates that the
# top-level CMakeLists links into `app`; app_src sources compile as part of
# `app`, so those include directories already apply.
target_link_libraries(app_src INTERFACE ble_services)
