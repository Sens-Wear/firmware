# LED controller drivers.
if(CONFIG_SENSWEAR_LP5562_DRIVER)
    file(GLOB _lp5562_src CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/lp5562/*.c)
    list(APPEND BOARD_DRIVER_SOURCES ${_lp5562_src})
    list(APPEND BOARD_DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lp5562)

    if(CONFIG_SENSWEAR_DEVICE_MANAGER)
        # Application-facing setup and RGBW channel wrapper used by
        # device_manager.
        set(_led_controller_wrapper_dir
            ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/actuators/led)
        list(APPEND BOARD_DRIVER_SOURCES
            ${_led_controller_wrapper_dir}/led_controller.c)
        list(APPEND BOARD_DRIVER_INCLUDE_DIRS
            ${_led_controller_wrapper_dir})
    endif()
endif()
