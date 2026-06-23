# SPDX-License-Identifier: Apache-2.0

# Zephyr discovers modules before Kconfig is evaluated, so this module must be
# registered unconditionally. Its M95P-specific extension is loaded only when
# both the M95P driver and its disk backend are enabled in the resolved
# configuration.
if(CONFIG_SENSEWEAR_M95P_DRIVER AND CONFIG_SENSEWEAR_M95P_DISK)
    add_subdirectory(drivers/disk)
endif()
