# Shared system-bus ownership wrappers. Gated by their existing Kconfig options.
if(CONFIG_SENSWEAR_SYS_I2C)
    list(APPEND BOARD_DRIVER_SOURCES ${CMAKE_CURRENT_LIST_DIR}/sys_i2c.c)
endif()
if(CONFIG_SENSWEAR_SYS_SPI)
    list(APPEND BOARD_DRIVER_SOURCES ${CMAKE_CURRENT_LIST_DIR}/sys_spi.c)
endif()
list(APPEND BOARD_DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
