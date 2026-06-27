# SPDX-License-Identifier: Apache-2.0

# The DRV2605 haptics driver is supplied by the shield_drivers target rather than
# Zephyr's in-tree drivers/haptics directory. CONFIG_HAPTICS (selected by the
# SenseWear driver so the haptics_* syscalls are generated) still creates the
# drivers__haptics library, and with no in-tree haptics driver enabled it has no
# sources, so explicitly permit that library to remain empty.
zephyr_library_amend()
zephyr_library_property(ALLOW_EMPTY TRUE)
