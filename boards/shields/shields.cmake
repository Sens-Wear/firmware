# SPDX-License-Identifier: Apache-2.0

# SensWear shield driver aggregation.
#
# Each enabled shield contributes its colocated driver sources and public include
# directories to the `shield_drivers` INTERFACE library.

set(SHIELD_DRIVER_SOURCES "")
set(SHIELD_DRIVER_INCLUDE_DIRS "")

if(CONFIG_SHIELD_SENSWEAR_HAPTIC)
    include(${CMAKE_CURRENT_LIST_DIR}/senswear_haptic/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSWEAR_PPG)
    include(${CMAKE_CURRENT_LIST_DIR}/senswear_ppg/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSWEAR_TEMPERATURE)
    include(${CMAKE_CURRENT_LIST_DIR}/senswear_temperature/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSWEAR_TOUCH)
    include(${CMAKE_CURRENT_LIST_DIR}/senswear_touch/drivers/shield_drivers.cmake)
endif()

add_library(shield_drivers INTERFACE)

target_sources(shield_drivers INTERFACE
    ${SHIELD_DRIVER_SOURCES}
)

target_include_directories(shield_drivers INTERFACE
    ${SHIELD_DRIVER_INCLUDE_DIRS}
)
