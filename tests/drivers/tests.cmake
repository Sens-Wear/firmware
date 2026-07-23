# SPDX-License-Identifier: Apache-2.0
#
# Driver bring-up test targets. Reached only when TEST_DRIVERS is ON (see the
# top-level CMakeLists). Each test is gated on its own
# CONFIG_SENSWEAR_TEST_<driver>_DRIVER Kconfig symbol and built as a
# `test_<driver>` INTERFACE library linked into `app`.
#
# Every test file defines its own main(), so exactly one test may be enabled at
# a time; the app main (src/main.c) is omitted while TEST_DRIVERS is on.

set(TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})

# Create a test target for <name> (sources tests/drivers/main_test_<name>.c)
# when its CONFIG_<SYM> Kconfig symbol is set, and link it into the application.
macro(senswear_add_driver_test _name _sym)
    if(${_sym})
        add_library(test_${_name} INTERFACE)
        target_sources(test_${_name} INTERFACE ${TESTS_DIR}/main_test_${_name}.c)
        target_link_libraries(app PRIVATE test_${_name})
        list(APPEND _enabled_bringup_tests ${_name})
    endif()
endmacro()

senswear_add_driver_test(bq25180   CONFIG_SENSWEAR_TEST_BQ25180_DRIVER)
senswear_add_driver_test(bq27427   CONFIG_SENSWEAR_TEST_BQ27427_DRIVER)
senswear_add_driver_test(bhi360    CONFIG_SENSWEAR_TEST_BHI360_DRIVER)
senswear_add_driver_test(lp5562    CONFIG_SENSWEAR_TEST_LP5562_DRIVER)
senswear_add_driver_test(m95p      CONFIG_SENSWEAR_TEST_M95P_DRIVER)
senswear_add_driver_test(tpsm83102 CONFIG_SENSWEAR_TEST_TPSM83102_DRIVER)
senswear_add_driver_test(rtc       CONFIG_SENSWEAR_TEST_RTC_DRIVER)
