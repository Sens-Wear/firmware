# Common driver infrastructure (device event manager, message contract, ...).
#
# This is shared by both the base-board drivers and the shield drivers, so it is
# collected into its own `common_drivers` INTERFACE library rather than folded
# into either driver aggregate.
file(GLOB _common_src CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/device_driver_events/*.c)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_sensewear_device_ids_header ${CMAKE_CURRENT_BINARY_DIR}/generated/device_driver_dts_ids.h)
set(_sensewear_device_ids_script ${CMAKE_CURRENT_LIST_DIR}/generate_device_driver_dts_ids.py)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)

# Generate the header at configure time. The merged devicetree (zephyr.dts) is
# already produced by find_package(Zephyr) before this file is included, so the
# header always has real, DTS-derived content before any source is compiled.
#
# This runs purely at configure time, so it needs no build target dependency
# (and makes no assumption about the application target name). The devicetree
# sources are configure dependencies of the Zephyr build, so any change to the
# devicetree triggers a reconfigure, which re-runs this generation.
execute_process(
	COMMAND ${Python3_EXECUTABLE} ${_sensewear_device_ids_script}
		--input ${PROJECT_BINARY_DIR}/zephyr/zephyr.dts
		--output ${_sensewear_device_ids_header}
	RESULT_VARIABLE _sensewear_device_ids_result
)
if(NOT _sensewear_device_ids_result EQUAL 0)
	message(FATAL_ERROR
		"Failed to generate device_driver_dts_ids.h from "
		"${PROJECT_BINARY_DIR}/zephyr/zephyr.dts (exit ${_sensewear_device_ids_result})")
endif()

add_library(common_drivers INTERFACE)

target_sources(common_drivers INTERFACE ${_common_src})

target_include_directories(common_drivers INTERFACE
	${CMAKE_CURRENT_BINARY_DIR}/generated
	${CMAKE_CURRENT_LIST_DIR}/device_driver_events
	${CMAKE_CURRENT_LIST_DIR}/device_manager
)
