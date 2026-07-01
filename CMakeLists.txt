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
foreach(_t BQ25180 BQ27427 BHI360 LP5562 M95P TPSM83102 RTC)
    if(CONFIG_SENSEWEAR_TEST_${_t}_DRIVER)
        set(TEST_DRIVERS ON)
    endif()
endforeach()

# Shield driver bring-up tests (tests/shields). Same Kconfig-driven selection as
# the board-driver tests above; these exercise drivers shipped by a shield, so
# the matching shield must also be enabled (e.g. SHIELD=sensewear_haptic).
set(TEST_SHIELDS OFF)
foreach(_t DRV2605 MAX30101 MAX30208 MTCH6102)
    if(CONFIG_SENSEWEAR_TEST_${_t}_DRIVER)
        set(TEST_SHIELDS ON)
    endif()
endforeach()

# Vendored third-party libraries (BHY2 sensor API, ...) collected into the
# `libs` INTERFACE library. Linked always: board_drivers' bhi360.c uses BHY2.
include(${CMAKE_CURRENT_SOURCE_DIR}/libs/libs.cmake)

# Board-level driver subsystem: aggregates the shared `common_drivers`, the
# base-board `board_drivers`, and the selected `shield_drivers` INTERFACE
# libraries (see boards/drivers.cmake).
include(${CMAKE_CURRENT_SOURCE_DIR}/boards/drivers.cmake)

# Count the enabled SenseWear shields (one per daughter board). The CONFIG_
# symbols are readable here as CMake variables because find_package(Zephyr) has
# already run Kconfig. The total is handed to the application as the
# SENSEWEAR_NUM_SHIELDS compile definition.
set(SENSEWEAR_SHIELDS SENSEWEAR_HAPTIC SENSEWEAR_PPG SENSEWEAR_TEMPERATURE SENSEWEAR_TOUCH)
set(_shield_count 0)
foreach(_s ${SENSEWEAR_SHIELDS})
    if(CONFIG_SHIELD_${_s})
        math(EXPR _shield_count "${_shield_count} + 1")
    endif()
endforeach()

# Count, straight from devicetree, how many enabled devices depend on the shared
# daughter-connector regulator (TPSM83102 / VDD_DAUGHTER). Each daughter device
# declares `vin-supply = <&tpsm83102>` in its shield overlay. Zephyr's CMake
# devicetree API cannot enumerate a node's children, so instead of walking the
# daughter I2C bus we check each known daughter device by its node label and tally
# the okay ones whose vin-supply resolves to the regulator node. The connector
# exposes a single regulated rail, so more than one is a hard error. The dt_*
# helpers read the parsed devicetree, available now that find_package(Zephyr) has
# run. dt_prop() leaves its output variable empty when the property is absent.
set(SENSEWEAR_DAUGHTER_DEVICES drv2605 max30101 max30208 mtch6102)
dt_nodelabel(_reg_path NODELABEL tpsm83102)
set(_regulator_dep_count 0)
if(_reg_path)
    foreach(_label ${SENSEWEAR_DAUGHTER_DEVICES})
        dt_nodelabel(_dev_path NODELABEL ${_label})
        if(NOT _dev_path)
            continue()
        endif()
        dt_node_has_status(_ok PATH "${_dev_path}" STATUS okay)
        if(NOT _ok)
            continue()
        endif()
        dt_prop(_supply PATH "${_dev_path}" PROPERTY "vin-supply")
        if(_supply STREQUAL "${_reg_path}")
            math(EXPR _regulator_dep_count "${_regulator_dep_count} + 1")
        endif()
    endforeach()
endif()

if(_regulator_dep_count GREATER 1)
    message(FATAL_ERROR
        "${_regulator_dep_count} enabled devices depend on the shared "
        "daughter-connector regulator (TPSM83102 / VDD_DAUGHTER), but the "
        "connector provides a single regulated rail. Enable at most one "
        "regulator-dependent daughter shield.")
endif()

# At most one regulator-dependent daughter device: publish the active shield
# count to the application.
target_compile_definitions(app PRIVATE SENSEWEAR_NUM_SHIELDS=${_shield_count})

target_link_libraries(app PRIVATE common_drivers board_drivers shield_drivers libs)

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
    # include(${CMAKE_CURRENT_SOURCE_DIR}/src/app.cmake)
    # target_link_libraries(app PRIVATE app_src)
endif()
