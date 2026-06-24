# SPDX-License-Identifier: Apache-2.0

# SenseWear shield driver aggregation.
#
# Each enabled shield contributes its colocated driver sources and public include
# directories to the `shield_drivers` INTERFACE library.

set(SHIELD_DRIVER_SOURCES "")
set(SHIELD_DRIVER_INCLUDE_DIRS "")

if(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
    include(${CMAKE_CURRENT_LIST_DIR}/sensewear_haptic/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSEWEAR_PPG)
    include(${CMAKE_CURRENT_LIST_DIR}/sensewear_ppg/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSEWEAR_TEMPERATURE)
    include(${CMAKE_CURRENT_LIST_DIR}/sensewear_temperature/drivers/shield_drivers.cmake)
endif()

if(CONFIG_SHIELD_SENSEWEAR_TOUCH)
    include(${CMAKE_CURRENT_LIST_DIR}/sensewear_touch/drivers/shield_drivers.cmake)
endif()

add_library(shield_drivers INTERFACE)

target_sources(shield_drivers INTERFACE
    ${SHIELD_DRIVER_SOURCES}
)

target_include_directories(shield_drivers INTERFACE
    ${SHIELD_DRIVER_INCLUDE_DIRS}
)
