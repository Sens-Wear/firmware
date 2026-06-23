/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file m95p_disk.c
 * @brief Disk-access driver and filesystem mapping for the M95P EEPROM.
 *
 * Bridges the board's M95P driver to Zephyr's disk-access subsystem so a
 * filesystem can be mounted on the EEPROM. Public design notes are in
 * @ref sensewear_m95p_disk.
 *
 * Sector model: one logical disk sector is @ref M95P_DISK_SECTOR_SIZE bytes,
 * which equals one M95P page (M95P_PAGE_SIZE). Logical sector N therefore maps
 * to M95P page N, and to byte address N * 512. The factory golden section
 * occupies the top pages and is excluded from the reported sector count so a
 * filesystem never maps onto it.
 */

#include "m95p_disk.h"

#include <errno.h>

#ifdef CONFIG_SENSEWEAR_M95P_DISK

#include "m95p.h"
#include "m95p_organization.h"

#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(m95p_disk, CONFIG_LOG_DEFAULT_LEVEL);

/** Logical disk sector size; one sector maps to one M95P page. */
#define M95P_DISK_SECTOR_SIZE (M95P_PAGE_SIZE)

/** Number of logical sectors exposed, excluding the reserved golden section. */
static inline uint32_t m95p_disk_sector_count(void) {
	return (uint32_t) (M95P_PAGE_COUNT - m95p_get_golden_section_page_count());
}

/** Ensure the underlying M95P driver is initialized. */
static int m95p_disk_init(struct disk_info* disk) {
	ARG_UNUSED(disk);

	if (!m95p_is_ready()) {
		m95p_init(NULL);
	}

	return m95p_is_ready() ? 0 : -EIO;
}

/** Report disk presence and write-protection status. */
static int m95p_disk_status(struct disk_info* disk) {
	ARG_UNUSED(disk);

	if (!m95p_is_ready()) {
		return DISK_STATUS_UNINIT;
	}
	if (m95p_is_write_protected()) {
		return DISK_STATUS_WR_PROTECT;
	}
	return DISK_STATUS_OK;
}

/** Read @p num_sector logical sectors starting at @p start_sector. */
static int m95p_disk_read(struct disk_info* disk,
						  uint8_t* data_buf,
						  uint32_t start_sector,
						  uint32_t num_sector) {
	ARG_UNUSED(disk);

	if ((uint64_t) start_sector + num_sector > m95p_disk_sector_count()) {
		return -EINVAL;
	}

	/* Reads may span pages, so a single transfer covers the whole range. */
	if (!m95p_read((uint32_t) (start_sector * M95P_DISK_SECTOR_SIZE),
				   data_buf,
				   (size_t) num_sector * M95P_DISK_SECTOR_SIZE)) {
		return -EIO;
	}
	return 0;
}

/** Write @p num_sector logical sectors starting at @p start_sector. */
static int m95p_disk_write(struct disk_info* disk,
						   const uint8_t* data_buf,
						   uint32_t start_sector,
						   uint32_t num_sector) {
	ARG_UNUSED(disk);

	if ((uint64_t) start_sector + num_sector > m95p_disk_sector_count()) {
		return -EINVAL;
	}

	/*
	 * Writes are page-bounded: one logical sector == one M95P page. The block
	 * device has no separate erase step, and filesystems overwrite sectors in
	 * place, so each sector must be erased and reprogrammed atomically. Use the
	 * page-write (PGWR) path; page-program (PGPR) only programs already-erased
	 * pages and would set the program-fail flag on any non-blank sector.
	 */
	for (uint32_t i = 0; i < num_sector; i++) {
		const uint8_t* page = data_buf + (size_t) i * M95P_DISK_SECTOR_SIZE;

		if (!m95p_write_sector(start_sector + i, page, M95P_DISK_SECTOR_SIZE)) {
			return -EIO;
		}
	}
	return 0;
}

/** Handle disk-access control queries. */
static int m95p_disk_ioctl(struct disk_info* disk, uint8_t cmd, void* buff) {
	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t*) buff = m95p_disk_sector_count();
		return 0;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t*) buff = M95P_DISK_SECTOR_SIZE;
		return 0;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		/* The EEPROM is page-writable, so the erase unit is one sector. */
		*(uint32_t*) buff = 1U;
		return 0;
	case DISK_IOCTL_CTRL_SYNC:
		while (m95p_is_busy()) {
			k_msleep(1);
		}
		return 0;
	case DISK_IOCTL_CTRL_INIT:
		return m95p_disk_init(disk);
	case DISK_IOCTL_CTRL_DEINIT:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct disk_operations m95p_disk_ops = {
	.init = m95p_disk_init,
	.status = m95p_disk_status,
	.read = m95p_disk_read,
	.write = m95p_disk_write,
	.ioctl = m95p_disk_ioctl,
};

static struct disk_info m95p_disk = {
	.name = M95P_DISK_NAME,
	.ops = &m95p_disk_ops,
};

/** Register the M95P with the disk-access subsystem during system init. */
static int m95p_disk_register(void) {
	int rc = disk_access_register(&m95p_disk);

	if (rc != 0) {
		LOG_ERR("M95P disk-access registration failed (%d)", rc);
	}
	return rc;
}
SYS_INIT(m95p_disk_register, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#ifdef CONFIG_FILE_SYSTEM_LITTLEFS

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(m95p_lfs_data);

static struct fs_mount_t m95p_mount = {
	.type = FS_LITTLEFS,
	.fs_data = &m95p_lfs_data,
	.storage_dev = (void*) M95P_DISK_NAME,
	.mnt_point = M95P_FS_MOUNT_POINT,
	.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
};

int m95p_fs_mount(void) {
	int rc = fs_mount(&m95p_mount);

	if (rc != 0) {
		LOG_ERR("M95P LittleFS mount failed (%d)", rc);
	} else {
		LOG_INF("M95P LittleFS mounted at %s", M95P_FS_MOUNT_POINT);
	}
	return rc;
}

int m95p_fs_unmount(void) {
	return fs_unmount(&m95p_mount);
}

#ifdef CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT
/** Mount the filesystem at boot, after the disk has been registered. */
static int m95p_fs_automount(void) {
	(void) m95p_fs_mount();
	return 0;
}
/*
 * Run one priority level after m95p_disk_register so the disk exists before we
 * mount. The SYS_INIT priority is stringized into the init-entry section name,
 * so it must be a single integer token: use UTIL_INC() to increment at
 * preprocessor time rather than "CONFIG_APPLICATION_INIT_PRIORITY + 1", which
 * would emit a malformed ".z_init_..._P_90 + 1_..." section.
 */
SYS_INIT(m95p_fs_automount, APPLICATION, UTIL_INC(CONFIG_APPLICATION_INIT_PRIORITY));
#endif /* CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT */

#else /* CONFIG_FILE_SYSTEM_LITTLEFS */

int m95p_fs_mount(void) {
	return -ENOSYS;
}

int m95p_fs_unmount(void) {
	return -ENOSYS;
}

#endif /* CONFIG_FILE_SYSTEM_LITTLEFS */

#else /* CONFIG_SENSEWEAR_M95P_DISK */

int m95p_fs_mount(void) {
	return -ENOSYS;
}

int m95p_fs_unmount(void) {
	return -ENOSYS;
}

#endif /* CONFIG_SENSEWEAR_M95P_DISK */
