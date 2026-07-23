# SPDX-License-Identifier: Apache-2.0

# SensWear temperature shield drivers.
list(APPEND SHIELD_DRIVER_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/temperature/max30208/max30208.c
)

list(APPEND SHIELD_DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/temperature/max30208
)
