# Common driver infrastructure (device event manager, ...).
file(GLOB _common_src CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.c)
list(APPEND BOARD_DRIVER_SOURCES ${_common_src})
list(APPEND BOARD_DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
