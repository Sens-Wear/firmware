# SPDX-License-Identifier: Apache-2.0

# Zephyr discovers modules before Kconfig is evaluated, so this module must be
# registered unconditionally. Its M95P-specific extension is loaded only when
# both the M95P driver and its disk backend are enabled in the resolved
# configuration.
if(CONFIG_SENSWEAR_M95P_DRIVER AND CONFIG_SENSWEAR_M95P_DISK)
    add_subdirectory(drivers/disk)
endif()

# The DRV2605 haptics driver lives in shield_drivers, not Zephyr's in-tree
# drivers/haptics, but it selects CONFIG_HAPTICS for the haptics_* syscalls. That
# leaves the in-tree drivers__haptics library empty; permit it to stay empty.
if(CONFIG_SENSWEAR_DRV2605_DRIVER)
    add_subdirectory(drivers/haptics)
endif()
