# SPDX-License-Identifier: Apache-2.0
#
# Board-level driver aggregation.
#
# Collects all driver code linked into the application, in three INTERFACE
# libraries, from one place:
#   - common_drivers : infrastructure shared by the board and shield drivers
#                      (device event manager, message contract, generated IDs)
#   - board_drivers  : base-board device drivers
#   - shield_drivers : the selected daughter-board (shield) drivers
#
# Keeping the shared `common/` include here means neither the board nor the
# shield aggregate has to reach across trees to pull it in.

include(${CMAKE_CURRENT_LIST_DIR}/common/common_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/SenseraTechnologies/SensWear/drivers/board_drivers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/shields/shields.cmake)
