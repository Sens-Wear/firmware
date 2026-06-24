# SPDX-License-Identifier: Apache-2.0

# SenseWear touch shield drivers.
list(APPEND SHIELD_DRIVER_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/touch/mtch6102/mtch6102.c
)

list(APPEND SHIELD_DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/touch/mtch6102
)
