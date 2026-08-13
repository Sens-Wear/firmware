if (CONFIG_SOC_NRF54L15_CPUAPP)
  board_runner_args(jlink "--device=nRF54L15_M33" "--speed=4000")
elseif (CONFIG_SOC_NRF54L15_CPUFLPR)
  board_runner_args(jlink "--device=nRF54L15_RV32" "--speed=4000")
endif()

if(CONFIG_BOARD_SENSWEAR_NRF54L15_NS)
  set(TFM_PUBLIC_KEY_FORMAT "full")
endif()

if(CONFIG_TFM_FLASH_MERGED_BINARY)
  set_property(TARGET runners_yaml_props_target PROPERTY hex_file "${CMAKE_BINARY_DIR}/tfm_merged.hex")
endif()

# OpenOCD runner (`west flash|debug|attach|debugserver --runner openocd`).
#
# The nRF54L15 stores code in RRAM, for which OpenOCD defines no flash bank, so
# the stock `flash write_image` path cannot program this part. The repository's
# `config/openocd-nrf54l15.cfg` works around that with a `nrf54l-load` proc that
# enables the RRAM controller before writing; `--cmd-load` points the runner at
# it. Arguments set here are appended after the defaults from
# `openocd.board.cmake`, so they win in argparse; `--cmd-verify` restates that
# file's default, since the runner itself refuses to flash a hex image when
# either command is unset.
get_filename_component(SENSWEAR_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)

# Which probe the OpenOCD runner drives. Unlike the VS Code launch
# configurations, which offer one entry per probe, west binds the adapter file
# at configure time. Override per build directory with
# `-DSENSWEAR_OPENOCD_INTERFACE=stlink-v2`, or export the same name in the
# environment (needed under sysbuild, which forwards only Kconfig symbols to
# this image, not CMake cache variables).
set(SENSWEAR_OPENOCD_INTERFACE "cmsis-dap" CACHE STRING
    "Adapter config for the OpenOCD runner: cmsis-dap or stlink-v2")
if(DEFINED ENV{SENSWEAR_OPENOCD_INTERFACE})
  set(SENSWEAR_OPENOCD_INTERFACE $ENV{SENSWEAR_OPENOCD_INTERFACE})
endif()

set(SENSWEAR_OPENOCD_ADAPTER_CFG
    ${SENSWEAR_ROOT_DIR}/config/openocd-${SENSWEAR_OPENOCD_INTERFACE}.cfg)
if(NOT EXISTS ${SENSWEAR_OPENOCD_ADAPTER_CFG})
  message(FATAL_ERROR
          "SENSWEAR_OPENOCD_INTERFACE=${SENSWEAR_OPENOCD_INTERFACE} selects a "
          "missing adapter config: ${SENSWEAR_OPENOCD_ADAPTER_CFG}")
endif()

# Which OpenOCD to run. Zephyr's own discovery is `find_program(OPENOCD
# openocd)`, which caches OPENOCD-NOTFOUND here because the OpenOCD build this
# board needs is not on PATH. Point the runner at the same wrapper the VS Code
# launch configurations use instead: it reads OPENOCD and OPENOCD_SCRIPTS from
# `.env` at run time (see `.env.example`), so the path is resolved per
# invocation rather than frozen into the build directory at configure time.
# `--openocd` is a common west runner option, accepted here because west parses
# these arguments with the same parser that defines it.
if(CMAKE_HOST_WIN32)
  set(SENSWEAR_OPENOCD_WRAPPER ${SENSWEAR_ROOT_DIR}/config/scripts/openocd-with-env.cmd)
else()
  set(SENSWEAR_OPENOCD_WRAPPER ${SENSWEAR_ROOT_DIR}/config/scripts/openocd-with-env.sh)
endif()

board_runner_args(openocd
                  "--openocd=${SENSWEAR_OPENOCD_WRAPPER}"
                  "--config=${SENSWEAR_OPENOCD_ADAPTER_CFG}"
                  "--config=${SENSWEAR_ROOT_DIR}/config/openocd-nrf54l15.cfg"
                  "--cmd-load=nrf54l-load"
                  "--cmd-verify=verify_image"
                  # Mirror the launch configurations' preLaunchCommands: the
                  # cfg disables the GDB memory map, so spell out the read-only
                  # code region for hardware breakpoints.
                  "--gdb-init=set mem inaccessible-by-default off"
                  "--gdb-init=mem 0x00000000 0x00180000 ro")

include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)