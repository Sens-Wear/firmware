# SPDX-License-Identifier: Apache-2.0

# The M95P disk backend is supplied by the board_drivers target rather than
# Zephyr's in-tree drivers/disk directory. CONFIG_DISK_ACCESS still creates the
# drivers__disk library, so explicitly permit that library to remain empty.
zephyr_library_amend()
zephyr_library_property(ALLOW_EMPTY TRUE)
