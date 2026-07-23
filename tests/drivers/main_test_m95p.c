/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bring-up test for the M95P EEPROM driver, exercised through LittleFS.
 *
 * Initialises the device, prints the JEDEC id and geometry, then mounts the
 * LittleFS filesystem layered on the M95P disk and performs write/read-back
 * round-trips using the POSIX file API (open/write/read/close): one text file
 * and one binary file. Finally it prints the busy/ready status every 2 seconds.
 *
 * The filesystem is auto-mounted at boot (CONFIG_SENSWEAR_M95P_DISK_AUTOMOUNT),
 * and POSIX file operations are provided by the POSIX layer (CONFIG_POSIX_API);
 * see prj.conf. Swap this file in for src/main.c (see tests/drivers/README.md)
 * and flash to exercise the driver on hardware.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/posix/dirent.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/stat.h>
#include <zephyr/posix/unistd.h>

#include "m95p.h"
#include "m95p_disk.h"

#define TEST_POLL_INTERVAL K_SECONDS(2)

#define TEXT_FILE_PATH M95P_FS_MOUNT_POINT "/hello.txt"
#define BIN_FILE_PATH M95P_FS_MOUNT_POINT "/data.bin"

/* Write the whole buffer, looping over short writes. Returns 0 or -errno. */
static int write_all(int fd, const void* buf, size_t len) {
	const uint8_t* p = buf;

	while (len > 0) {
		ssize_t n = write(fd, p, len);

		if (n < 0) {
			return -errno;
		}
		if (n == 0) {
			return -EIO;
		}
		p += n;
		len -= (size_t) n;
	}
	return 0;
}

/* Number of times the text block is appended to grow the file across pages. */
#define TEXT_REPEAT 100

/*
 * Round-trip a text file through the POSIX file API. The same block is appended
 * TEXT_REPEAT times so the file spans many 512-byte EEPROM pages, then the file
 * is closed, reopened, and read back in chunks and verified against the
 * repeating pattern (avoids holding the whole file in RAM).
 */
static bool test_text_file(void) {
	static const char text[] = "M95P LittleFS POSIX round-trip\nline two\n";
	const size_t text_len = strlen(text);
	const size_t expected_len = text_len * TEXT_REPEAT;

	int fd = open(TEXT_FILE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (fd < 0) {
		printk("text: open(write) failed (errno %d)\n", errno);
		return false;
	}

	for (int i = 0; i < TEXT_REPEAT; i++) {
		int rc = write_all(fd, text, text_len);

		if (rc != 0) {
			printk("text: write of block %d failed (%d)\n", i, rc);
			close(fd);
			return false;
		}
	}
	close(fd);

	fd = open(TEXT_FILE_PATH, O_RDONLY);
	if (fd < 0) {
		printk("text: open(read) failed (errno %d)\n", errno);
		return false;
	}

	/* Read back in chunks, comparing each byte to the expected repeated text. */
	char chunk[128];
	size_t pos = 0;

	for (;;) {
		ssize_t n = read(fd, chunk, sizeof(chunk));

		if (n < 0) {
			printk("text: read failed (errno %d) at offset %zu\n", errno, pos);
			close(fd);
			return false;
		}
		if (n == 0) {
			break; /* EOF */
		}
		for (ssize_t i = 0; i < n; i++) {
			if (chunk[i] != text[(pos + (size_t) i) % text_len]) {
				printk("text: MISMATCH at offset %zu\n", pos + (size_t) i);
				close(fd);
				return false;
			}
		}
		pos += (size_t) n;
	}
	close(fd);

	if (pos != expected_len) {
		printk("text: length MISMATCH (read %zu, expected %zu)\n", pos, expected_len);
		return false;
	}

	printk("text round-trip: OK (%zu bytes, %d blocks)\n", pos, TEXT_REPEAT);
	return true;
}

/* Number of times the binary block is appended to grow the file across pages. */
#define BIN_REPEAT 100

/*
 * Round-trip a binary file through the POSIX file API. A 256-byte 0x00..0xFF
 * block is appended BIN_REPEAT times so the file spans many 512-byte EEPROM
 * pages, then the file is closed, reopened, and read back in chunks and
 * verified against the repeating pattern (avoids holding the whole file in RAM).
 */
static bool test_binary_file(void) {
	uint8_t block[256];
	const size_t expected_len = sizeof(block) * BIN_REPEAT;

	/* Full 0x00..0xFF span exercises NUL and non-text bytes end to end. */
	for (size_t i = 0; i < sizeof(block); i++) {
		block[i] = (uint8_t) i;
	}

	int fd = open(BIN_FILE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (fd < 0) {
		printk("binary: open(write) failed (errno %d)\n", errno);
		return false;
	}

	for (int i = 0; i < BIN_REPEAT; i++) {
		int rc = write_all(fd, block, sizeof(block));

		if (rc != 0) {
			printk("binary: write of block %d failed (%d)\n", i, rc);
			close(fd);
			return false;
		}
	}
	close(fd);

	fd = open(BIN_FILE_PATH, O_RDONLY);
	if (fd < 0) {
		printk("binary: open(read) failed (errno %d)\n", errno);
		return false;
	}

	/*
	 * Read back in chunks, comparing each byte to the expected repeated block.
	 * The block is 0x00..0xFF, so byte at offset o equals (uint8_t)(o % 256).
	 */
	uint8_t chunk[256];
	size_t pos = 0;

	for (;;) {
		ssize_t n = read(fd, chunk, sizeof(chunk));

		if (n < 0) {
			printk("binary: read failed (errno %d) at offset %zu\n", errno, pos);
			close(fd);
			return false;
		}
		if (n == 0) {
			break; /* EOF */
		}
		for (ssize_t i = 0; i < n; i++) {
			if (chunk[i] != (uint8_t) ((pos + (size_t) i) % sizeof(block))) {
				printk("binary: MISMATCH at offset %zu\n", pos + (size_t) i);
				close(fd);
				return false;
			}
		}
		pos += (size_t) n;
	}
	close(fd);

	if (pos != expected_len) {
		printk("binary: length MISMATCH (read %zu, expected %zu)\n", pos, expected_len);
		return false;
	}

	printk("binary round-trip: OK (%zu bytes, %d blocks)\n", pos, BIN_REPEAT);
	return true;
}

#define WALK_MAX_SUBDIRS 8 /* subdir names buffered per directory level */
#define WALK_NAME_MAX 48   /* max remembered subdir-name length */
#define WALK_PATH_MAX 128  /* max full path length */

/*
 * Recursively list a directory tree through the POSIX directory API
 * (opendir/readdir/closedir) and report each entry's type and size via stat().
 *
 * Each directory is fully read and closed before descending into its
 * subdirectories, so only one DIR handle is open at a time (POSIX_OPEN_MAX is
 * small). Subdirectory names are buffered to enable that ordering.
 */
static void walk_dir(const char* path, int depth) {
	DIR* dir = opendir(path);

	if (dir == NULL) {
		printk("%*sopendir(%s) failed (errno %d)\n", depth * 2, "", path, errno);
		return;
	}

	char subdirs[WALK_MAX_SUBDIRS][WALK_NAME_MAX];
	int n_subdirs = 0;
	struct dirent* ent;

	while ((ent = readdir(dir)) != NULL) {
		char child[WALK_PATH_MAX];
		int len = snprintf(child, sizeof(child), "%s/%s", path, ent->d_name);

		if (len < 0 || (size_t) len >= sizeof(child)) {
			printk("%*s%s  (path too long, skipped)\n", depth * 2, "", ent->d_name);
			continue;
		}

		struct stat st;

		if (stat(child, &st) != 0) {
			printk("%*s%s  (stat failed, errno %d)\n", depth * 2, "", ent->d_name, errno);
			continue;
		}

		if (S_ISDIR(st.st_mode)) {
			printk("%*s%s/\n", depth * 2, "", ent->d_name);
			if (n_subdirs < WALK_MAX_SUBDIRS && strlen(ent->d_name) < WALK_NAME_MAX) {
				strcpy(subdirs[n_subdirs++], ent->d_name);
			}
		} else {
			printk("%*s%s  (%zu bytes)\n", depth * 2, "", ent->d_name, (size_t) st.st_size);
		}
	}
	closedir(dir); /* close before recursing so only one DIR is open at a time */

	for (int i = 0; i < n_subdirs; i++) {
		char child[WALK_PATH_MAX];

		if (snprintf(child, sizeof(child), "%s/%s", path, subdirs[i]) < (int) sizeof(child)) {
			walk_dir(child, depth + 1);
		}
	}
}

int main(void) {
	printk("\n=== M95P EEPROM LittleFS test ===\n");

	if (!m95p_init(NULL)) {
		printk("m95p_init() failed\n");
		return 0;
	}

	if (!m95p_is_ready()) {
		printk("m95p_is_ready() == false\n");
		return 0;
	}

	const union m95p_jedec_id_t id = m95p_get_jedec_id();

	printk("JEDEC id: manufacturer=0x%02x type=0x%02x capacity=0x%02x\n",
		   id.fields.manufacturer_id,
		   id.fields.memory_type,
		   id.fields.capacity);
	printk("geometry: size=%zu page=%zu sector=%zu pages=%zu sectors=%zu\n",
		   m95p_get_size(),
		   m95p_get_page_size(),
		   m95p_get_sector_size(),
		   m95p_get_page_count(),
		   m95p_get_sector_count());

	/*
	 * With automount the disk driver already mounted the filesystem during
	 * system init, so re-mounting would just log an "already mounted" error.
	 * Only mount explicitly when automount is disabled.
	 */
#if !defined(CONFIG_SENSWEAR_M95P_DISK_AUTOMOUNT)
	int rc = m95p_fs_mount();

	if (rc != 0) {
		printk("LittleFS mount failed (%d)\n", rc);
		return 0;
	}
#endif
	printk("LittleFS available at %s\n", M95P_FS_MOUNT_POINT);

	bool ok = test_text_file();

	ok = test_binary_file() && ok;
	printk("filesystem round-trips: %s\n", ok ? "ALL OK" : "FAILED");

	printk("filesystem tree under %s:\n", M95P_FS_MOUNT_POINT);
	walk_dir(M95P_FS_MOUNT_POINT, 1);

	printk("polling status every 2 s...\n");

	while (1) {
		printk("ready=%d busy=%d\n", m95p_is_ready(), m95p_is_busy());
		k_sleep(TEST_POLL_INTERVAL);
	}

	return 0;
}
