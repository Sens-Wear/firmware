# SPDX-License-Identifier: Apache-2.0
#
# Bosch BHY2 sensor API (vendored). Collected into the `bhy2_sensors` INTERFACE
# library: consumers that link it pick up the sources, the include directory
# (which also covers the firmware/ blobs) and the BHY2 configuration define.

set(BHY2_DIR ${CMAKE_CURRENT_LIST_DIR}/BHY2-Sensor-API)

file(GLOB BHY2_SOURCES CONFIGURE_DEPENDS ${BHY2_DIR}/*.c)

add_library(bhy2_sensors INTERFACE)

target_sources(bhy2_sensors INTERFACE ${BHY2_SOURCES})

target_include_directories(bhy2_sensors INTERFACE ${BHY2_DIR})

target_compile_definitions(bhy2_sensors INTERFACE
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)
