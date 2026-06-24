# SPDX-License-Identifier: Apache-2.0
#
# Third-party / vendored libraries. Each library has its own cmake that defines
# an INTERFACE library; the aggregate `libs` INTERFACE library links them so
# consumers depend on a single target.
#
# Add a new library by creating libs/<name>.cmake (defining its INTERFACE
# library), including it here, and linking it into `libs` below.



add_library(libs INTERFACE)

if(CONFIG_SENSEWEAR_BHI360_DRIVER)
    include(${CMAKE_CURRENT_LIST_DIR}/BHY2-Sensor-API/bhy2-sensors.cmake)
    target_link_libraries(libs INTERFACE
        bhy2_sensors
    )
endif()
