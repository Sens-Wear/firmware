# SPDX-License-Identifier: Apache-2.0
#
# Driver bring-up test targets. Reached only when TEST_DRIVERS is ON (see the
# top-level CMakeLists). Each test is gated on its own TEST_<driver>_DRIVER
# option and built as a `test_<driver>` INTERFACE library linked into `app`.
#
# Every test file defines its own main(), so exactly one test may be enabled at
# a time; the app main (src/main.c) is omitted while TEST_DRIVERS is on.

set(TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})
set(_enabled_tests "")

# Create a test target for <name> (sources tests/drivers/main_test_<name>.c)
# when its TEST_<OPT> option is ON, and link it into the application.
macro(senswear_add_driver_test _name _opt)
    if(${_opt})
        add_library(test_${_name} INTERFACE)
        target_sources(test_${_name} INTERFACE ${TESTS_DIR}/main_test_${_name}.c)
        target_link_libraries(app PRIVATE test_${_name})
        list(APPEND _enabled_tests ${_name})
    endif()
endmacro()

senswear_add_driver_test(bq25180   TEST_BQ25180_DRIVER)
senswear_add_driver_test(bq27427   TEST_BQ27427_DRIVER)
senswear_add_driver_test(bhi360    TEST_BHI360_DRIVER)
senswear_add_driver_test(lp5562    TEST_LP5562_DRIVER)
senswear_add_driver_test(m95p      TEST_M95P_DRIVER)
senswear_add_driver_test(tpsm83102 TEST_TPSM83102_DRIVER)

list(LENGTH _enabled_tests _enabled_count)
if(_enabled_count EQUAL 0)
    message(FATAL_ERROR
        "TEST_DRIVERS is ON but no TEST_<driver>_DRIVER is enabled. "
        "Enable exactly one, e.g. -DTEST_M95P_DRIVER=ON.")
elseif(_enabled_count GREATER 1)
    message(FATAL_ERROR
        "Multiple driver tests enabled (${_enabled_tests}); each defines "
        "main(). Enable exactly one TEST_<driver>_DRIVER.")
endif()
