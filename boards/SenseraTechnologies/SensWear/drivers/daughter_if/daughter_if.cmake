# Daughter-board interface driver (connector GPIO ownership arbiter).
if(CONFIG_SENSWEAR_DAUGHTER_IF_DRIVER)
    file(GLOB _daughter_if_src CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.c)
    list(APPEND BOARD_DRIVER_SOURCES ${_daughter_if_src})
    list(APPEND BOARD_DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
endif()
