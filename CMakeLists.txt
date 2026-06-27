# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND SHIELD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND EXTRA_ZEPHYR_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/config/cmake/zephyr_module)
# Board-specific devicetree bindings live under the board directory
# (boards/.../SensWear/dts/bindings), keeping them co-located with the board
# and its out-of-tree drivers.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SenseraTechnologies/SensWear)
# The sensewear_haptic shield ships its own DRV2605 driver and binding
# (sensewear,drv2605); register the shield as a DTS root so its bindings under
# dts/bindings are discovered. Only do so when the shield is actually selected.
# DTS_ROOT must be set before find_package(Zephyr), so this keys off the SHIELD
# input variable (set on the command line / preset / environment) rather than
# CONFIG_SHIELD_SENSEWEAR_HAPTIC, which is not known until Kconfig has run.
if("${SHIELD}" MATCHES "sensewear_haptic" OR "$ENV{SHIELD}" MATCHES "sensewear_haptic")
    list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/boards/shields/sensewear_haptic)
endif()

# Emit compile_commands.json for clangd. Set before find_package(Zephyr): the
# variable initialises each target's EXPORT_COMPILE_COMMANDS property at creation
# time, so it must be in effect before Zephyr defines its library targets, or
# most entries would be excluded. Sysbuild does not forward CMAKE_* variables to
# images, so enabling it here in the application's own CMake (rather than at the
# sysbuild top level) is what makes it take effect for this image.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(firmware)

# Keep a stable compile_commands.json at the project root for clangd (.clangd
# points there). Every configuration builds in its own tree -- the application
# in build/, each driver test in build/tests/<name>/ -- and CMake writes that
# tree's database to ${CMAKE_BINARY_DIR}/compile_commands.json. We symlink the
# project-root entry to whichever image just configured, so clangd always reads
# the active build's commands (e.g. a test build includes its main_test_*.c that
# the application build omits). Only the main app image does this; sibling
# sysbuild images (mcuboot, ...) leave the link alone.
if(CMAKE_EXPORT_COMPILE_COMMANDS AND (NOT SYSBUILD OR SYSBUILD_MAIN_APP))
    file(CREATE_LINK
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
        SYMBOLIC)
endif()

set(BOARD_FLASH_RUNNER pyocd)

# Each device driver is collected into the board_drivers interface library only
# when its CONFIG_SENSEWEAR_<driver>_DRIVER Kconfig symbol is set (all enabled by
# default; see Kconfig). Driving driver presence purely from Kconfig means a
# single source of truth that also propagates through sysbuild / CMake presets,
# so no CMake-side option or consistency check is needed here -- the per-type
# cmakes under boards/.../drivers gate directly on the CONFIG_ symbols.

# Driver bring-up tests (tests/drivers). All off by default. The selection is
# driven through Kconfig (CONFIG_SENSEWEAR_TEST_<driver>_DRIVER) rather than CMake
# options: under sysbuild only Kconfig symbols are forwarded from the top-level
# build (and thus from a CMake preset) down to this image, so a CMake option here
# would never receive the override. Enabling any individual test implies
# TEST_DRIVERS; when TEST_DRIVERS is on the selected test main() replaces the
# application main (src/main.c). The CONFIG_ symbols are available as CMake
# variables because find_package(Zephyr) above has already run Kconfig.
set(TEST_DRIVERS OFF)
foreach(_t BQ25180 BQ27427 BHI360 LP5562 M95P TPSM83102)
    if(CONFIG_SENSEWEAR_TEST_${_t}_DRIVER)
        set(TEST_DRIVERS ON)
    endif()
endforeach()

# Shield driver bring-up tests (tests/shields). Same Kconfig-driven selection as
# the board-driver tests above; these exercise drivers shipped by a shield, so
# the matching shield must also be enabled (e.g. SHIELD=sensewear_haptic).
set(TEST_SHIELDS OFF)
foreach(_t DRV2605)
    if(CONFIG_SENSEWEAR_TEST_${_t}_DRIVER)
        set(TEST_SHIELDS ON)
    endif()
endforeach()

# Vendored third-party libraries (BHY2 sensor API, ...) collected into the
# `libs` INTERFACE library. Linked always: board_drivers' bhi360.c uses BHY2.
include(${CMAKE_CURRENT_SOURCE_DIR}/libs/libs.cmake)

# Board driver subsystem: per-type cmakes collect enabled driver sources and
# include directories into the `board_drivers` INTERFACE library.
include(${CMAKE_CURRENT_SOURCE_DIR}/boards/SenseraTechnologies/SensWear/drivers/board_drivers.cmake)

# Shield driver subsystem: the shields aggregator checks enabled shields and
# collects their colocated driver sources into the `shield_drivers` INTERFACE library.
include(${CMAKE_CURRENT_SOURCE_DIR}/boards/shields/shields.cmake)

target_link_libraries(app PRIVATE board_drivers shield_drivers libs)

if(TEST_DRIVERS AND TEST_SHIELDS)
    message(FATAL_ERROR
        "A board-driver test and a shield test are both enabled; each defines "
        "main(). Enable exactly one SENSEWEAR_TEST_<driver>_DRIVER.")
endif()

if(TEST_DRIVERS OR TEST_SHIELDS)
    # A bring-up test replaces the application: the test provides main() and links
    # against the board/shield drivers. The application sources
    # (src/app.cmake -> app_src) are not built at all.
    if(TEST_DRIVERS)
        include(${CMAKE_CURRENT_SOURCE_DIR}/tests/drivers/tests.cmake)
    endif()
    if(TEST_SHIELDS)
        include(${CMAKE_CURRENT_SOURCE_DIR}/tests/shields/tests.cmake)
    endif()
else()
    # Normal build: application sources / includes (app_src INTERFACE library).
    include(${CMAKE_CURRENT_SOURCE_DIR}/src/app.cmake)
    target_link_libraries(app PRIVATE app_src)
endif()
