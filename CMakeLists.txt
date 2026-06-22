# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND SHIELD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
# Board-specific devicetree bindings live under the board directory
# (boards/.../SensWear/dts/bindings), keeping them co-located with the board
# and its out-of-tree drivers.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SenseraTechnologies/SensWear)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(firmware)

set(BOARD_FLASH_RUNNER pyocd)

# Board driver toggles. Each device driver is collected into the board_drivers
# interface library only when its option is ON (all enabled by default).
option(HAVE_BQ25180_DRIVER "Build the BQ25180 charger driver" ON)
option(HAVE_BQ27427_DRIVER "Build the BQ27427 fuel-gauge driver" ON)
option(HAVE_BHI360_DRIVER "Build the BHI360 IMU driver" ON)
option(HAVE_LP5562_DRIVER "Build the LP5562 LED controller driver" ON)
option(HAVE_M95P_DRIVER "Build the M95P EEPROM driver" ON)
option(HAVE_TPSM83102_DRIVER "Build the TPSM83102 regulator driver" ON)

# Each HAVE_<driver>_DRIVER CMake option must agree with its
# CONFIG_SENSEWEAR_<driver>_DRIVER Kconfig symbol (set via prj.conf). Reject
# configuration if a driver is enabled on one side and disabled on the other.
foreach(_drv BQ25180 BQ27427 BHI360 LP5562 M95P TPSM83102)
    if(HAVE_${_drv}_DRIVER)
        set(_cmake_enabled TRUE)
    else()
        set(_cmake_enabled FALSE)
    endif()
    if(CONFIG_SENSEWEAR_${_drv}_DRIVER)
        set(_kconfig_enabled TRUE)
    else()
        set(_kconfig_enabled FALSE)
    endif()
    if(NOT _cmake_enabled STREQUAL _kconfig_enabled)
        message(FATAL_ERROR
            "SenseWear ${_drv} driver configuration mismatch: "
            "HAVE_${_drv}_DRIVER=${_cmake_enabled} but "
            "CONFIG_SENSEWEAR_${_drv}_DRIVER=${_kconfig_enabled}. "
            "Both must agree: change the CMake option "
            "(-DHAVE_${_drv}_DRIVER=ON/OFF) or the Kconfig symbol in prj.conf "
            "(CONFIG_SENSEWEAR_${_drv}_DRIVER=y/n).")
    endif()
endforeach()

# Driver bring-up tests (tests/drivers). All OFF by default. Enabling any
# individual TEST_<driver>_DRIVER implies TEST_DRIVERS; when TEST_DRIVERS is on
# the selected test main() replaces the application main (src/main.c).
option(TEST_DRIVERS "Build a driver bring-up test instead of the application" OFF)
option(TEST_BQ25180_DRIVER "Build the BQ25180 charger bring-up test" OFF)
option(TEST_BQ27427_DRIVER "Build the BQ27427 fuel-gauge bring-up test" OFF)
option(TEST_BHI360_DRIVER "Build the BHI360 IMU bring-up test" OFF)
option(TEST_LP5562_DRIVER "Build the LP5562 LED controller bring-up test" OFF)
option(TEST_M95P_DRIVER "Build the M95P EEPROM bring-up test" OFF)
option(TEST_TPSM83102_DRIVER "Build the TPSM83102 regulator bring-up test" OFF)

foreach(_t BQ25180 BQ27427 BHI360 LP5562 M95P TPSM83102)
    if(TEST_${_t}_DRIVER)
        set(TEST_DRIVERS ON)
    endif()
endforeach()

# Vendored third-party libraries (BHY2 sensor API, ...) collected into the
# `libs` INTERFACE library. Linked always: board_drivers' bhi360.c uses BHY2.
include(${CMAKE_CURRENT_SOURCE_DIR}/libs/libs.cmake)

# Board driver subsystem: per-type cmakes collect enabled driver sources and
# include directories into the `board_drivers` INTERFACE library.
include(${CMAKE_CURRENT_SOURCE_DIR}/boards/SenseraTechnologies/SensWear/drivers/board_drivers.cmake)

target_link_libraries(app PRIVATE board_drivers libs)

if(TEST_DRIVERS)
    # A driver bring-up test replaces the application: the test provides main()
    # and links only against the board drivers. The application sources
    # (src/app.cmake -> app_src) are not built at all.
    include(${CMAKE_CURRENT_SOURCE_DIR}/tests/drivers/tests.cmake)
else()
    # Normal build: application sources / includes (app_src INTERFACE library).
    include(${CMAKE_CURRENT_SOURCE_DIR}/src/app.cmake)
    target_link_libraries(app PRIVATE app_src)
endif()
