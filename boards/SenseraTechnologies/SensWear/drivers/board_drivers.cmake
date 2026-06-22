# SenseWear board driver aggregation.
#
# Each driver type provides a <type>_drivers.cmake that appends its enabled
# sources and public include directories to BOARD_DRIVER_SOURCES and
# BOARD_DRIVER_INCLUDE_DIRS. The collected lists are exposed to the application
# through the `board_drivers` INTERFACE library.

set(BOARD_DRIVER_SOURCES "")
set(BOARD_DRIVER_INCLUDE_DIRS "")

include(${CMAKE_CURRENT_LIST_DIR}/bus/bus_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/common/common_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/charger/charger_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/gauge/gauge_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/imu/imu_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/led_controller/led_controller_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/memory/memory_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/regulator/regulator_drivers.cmake)

add_library(board_drivers INTERFACE)

target_sources(board_drivers INTERFACE
    ${BOARD_DRIVER_SOURCES}
)

target_include_directories(board_drivers INTERFACE
    ${BOARD_DRIVER_INCLUDE_DIRS}
)
