/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file m95p_disk.h
 * @brief Maps the M95P EEPROM onto Zephyr's filesystem stack.
 *
 * @defgroup sensewear_m95p_disk SenseWear M95P disk / filesystem mapping
 * @ingroup io_interfaces
 * @{
 *
 * This unit registers the @ref sensewear_bq25180-style M95P driver with the
 * Zephyr disk-access subsystem under @ref M95P_DISK_NAME. A 512-byte logical
 * sector maps one-to-one to an M95P page, so any filesystem layered on the
 * disk-access block device (LittleFS via `CONFIG_FS_LITTLEFS_BLK_DEV`, or
 * FatFs) can use the EEPROM as backing storage.
 *
 * The disk is registered automatically during system init. Mounting is
 * separate: either enable @kconfig{CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT} to
 * mount LittleFS at boot, or call m95p_fs_mount() from application code.
 */

#ifndef SENSEWEAR_M95P_DISK_H_
#define SENSEWEAR_M95P_DISK_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Name the M95P is registered under with the disk-access subsystem. */
#define M95P_DISK_NAME "eeprom"

/** Default mount point for the M95P LittleFS filesystem. */
#define M95P_FS_MOUNT_POINT "/eeprom"

/**
 * @brief Mount the LittleFS filesystem on the M95P disk.
 *
 * On the first mount of a blank device LittleFS formats the medium
 * automatically (unless mounted read-only).
 *
 * @retval 0 The filesystem was mounted.
 * @retval -ENOSYS LittleFS support is not enabled in the build.
 * @return A negative errno from fs_mount() on failure.
 */
int m95p_fs_mount(void);

/**
 * @brief Unmount the M95P LittleFS filesystem.
 *
 * @retval 0 The filesystem was unmounted.
 * @retval -ENOSYS LittleFS support is not enabled in the build.
 * @return A negative errno from fs_unmount() on failure.
 */
int m95p_fs_unmount(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SENSEWEAR_M95P_DISK_H_ */
