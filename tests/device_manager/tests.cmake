# SPDX-License-Identifier: Apache-2.0
#
# Device-manager bring-up test target. Reached only when TEST_DRIVERS is ON
# (see the top-level CMakeLists). The test is gated on
# CONFIG_SENSEWEAR_TEST_DEVICE_MANAGER and built as a `test_device_manager`
# INTERFACE library linked into `app`.
#
# The test defines its own main(), so exactly one bring-up test (across
# tests/drivers, tests/device_manager, and tests/shields) may be enabled at a
# time; the app main (src/main.c) is omitted while a test is on.

set(DEVICE_MANAGER_TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})

if(CONFIG_SENSEWEAR_TEST_DEVICE_MANAGER)
    add_library(test_device_manager INTERFACE)
    target_sources(test_device_manager INTERFACE
        ${DEVICE_MANAGER_TESTS_DIR}/main_test_device_manager.c
    )
    target_link_libraries(app PRIVATE test_device_manager)
    list(APPEND _enabled_bringup_tests device_manager)
endif()
