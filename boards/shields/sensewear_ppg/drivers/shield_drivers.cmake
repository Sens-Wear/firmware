# SPDX-License-Identifier: Apache-2.0

# SenseWear PPG shield drivers.
list(APPEND SHIELD_DRIVER_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/ppg/max30101/max30101.c
)

list(APPEND SHIELD_DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/ppg/max30101
    ${CMAKE_SOURCE_DIR}/src
)
