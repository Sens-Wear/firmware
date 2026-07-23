# SPDX-License-Identifier: Apache-2.0
#
# Shield driver bring-up test targets. Reached only when TEST_SHIELDS is ON (see
# the top-level CMakeLists). Each test is gated on its own
# CONFIG_SENSWEAR_TEST_<driver>_DRIVER Kconfig symbol and built as a
# `test_<driver>` INTERFACE library linked into `app`.
#
# Every test file defines its own main(), so exactly one bring-up test (across
# tests/drivers and tests/shields) may be enabled at a time; the app main
# (src/main.c) is omitted while a test is on. The relevant shield must be
# enabled (e.g. SHIELD=senswear_haptic) so its driver and devicetree node are
# present.

set(SHIELD_TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})
set(_enabled_shield_tests "")

# Create a test target for <name> (sources tests/shields/main_test_<name>.c)
# when its CONFIG_<SYM> Kconfig symbol is set, and link it into the application.
macro(senswear_add_shield_test _name _sym)
    if(${_sym})
        add_library(test_${_name} INTERFACE)
        target_sources(test_${_name} INTERFACE ${SHIELD_TESTS_DIR}/main_test_${_name}.c)
        target_link_libraries(app PRIVATE test_${_name})
        list(APPEND _enabled_shield_tests ${_name})
    endif()
endmacro()

senswear_add_shield_test(drv2605 CONFIG_SENSWEAR_TEST_DRV2605_DRIVER)
senswear_add_shield_test(max30101 CONFIG_SENSWEAR_TEST_MAX30101_DRIVER)
senswear_add_shield_test(max30208 CONFIG_SENSWEAR_TEST_MAX30208_DRIVER)
senswear_add_shield_test(mtch6102 CONFIG_SENSWEAR_TEST_MTCH6102_DRIVER)

list(LENGTH _enabled_shield_tests _enabled_shield_count)
if(_enabled_shield_count GREATER 1)
    message(FATAL_ERROR
        "Multiple shield tests enabled (${_enabled_shield_tests}); each defines "
        "main(). Enable exactly one SENSWEAR_TEST_<driver>_DRIVER.")
endif()
