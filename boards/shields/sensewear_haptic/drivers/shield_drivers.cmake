# SPDX-License-Identifier: Apache-2.0

# SenseWear haptic shield drivers.
if(CONFIG_SENSEWEAR_DRV2605_DRIVER)
    list(APPEND SHIELD_DRIVER_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/haptic/drv2605/drv2605.c
    )
endif()

list(APPEND SHIELD_DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/haptic/drv2605
)
