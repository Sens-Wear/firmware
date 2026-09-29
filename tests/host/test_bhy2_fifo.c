/* Exercise the actual Bosch parser with deterministic FIFO transport fixtures.
 * Including the source also permits the internal leftover-copy boundary check.
 * The HIF implementation satisfies the remaining library APIs; only the four
 * FIFO entry points used by these scenarios are replaced below. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef BHY2_TEST_SOURCE
#define BHY2_TEST_SOURCE "../../libs/BHY2-Sensor-API/bhy2.c"
#endif
#include BHY2_TEST_SOURCE

#define bhy2_hif_get_interrupt_status unused_hif_get_interrupt_status
#define bhy2_hif_get_wakeup_fifo unused_hif_get_wakeup_fifo
#define bhy2_hif_get_nonwakeup_fifo unused_hif_get_nonwakeup_fifo
#define bhy2_hif_get_status_fifo_async unused_hif_get_status_fifo_async
int8_t unused_hif_get_interrupt_status(uint8_t* status, struct bhy2_hif_dev* hif);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "../../libs/BHY2-Sensor-API/bhy2_hif.c"
#pragma GCC diagnostic pop
#undef bhy2_hif_get_interrupt_status
#undef bhy2_hif_get_wakeup_fifo
#undef bhy2_hif_get_nonwakeup_fifo
#undef bhy2_hif_get_status_fifo_async

static const uint8_t* fixture;
static size_t fixture_size;
static size_t fixture_offset;
static uint32_t fixture_chunk;
static uint8_t fixture_status;
static unsigned int callback_count;
static uint8_t callback_payloads[4];

int8_t bhy2_hif_get_interrupt_status(uint8_t* status, struct bhy2_hif_dev* hif) {
	(void)hif;
	*status = fixture_status;
	return BHY2_OK;
}

static int8_t fixture_read(uint8_t expected_status, uint8_t* data, uint32_t capacity,
						   uint32_t* read_count, uint32_t* remaining) {
	assert(fixture_status == expected_status);
	size_t count = fixture_size - fixture_offset;
	if (count > capacity) {
		count = capacity;
	}
	if (count > fixture_chunk) {
		count = fixture_chunk;
	}
	assert(count > 0);
	memcpy(data, fixture + fixture_offset, count);
	fixture_offset += count;
	*read_count = (uint32_t)count;
	*remaining = (uint32_t)(fixture_size - fixture_offset);
	return BHY2_OK;
}

int8_t bhy2_hif_get_wakeup_fifo(uint8_t* data, uint32_t capacity, uint32_t* read_count,
							  uint32_t* remaining, struct bhy2_hif_dev* hif) {
	(void)hif;
	return fixture_read(BHY2_IST_FIFO_W_DRDY, data, capacity, read_count, remaining);
}

int8_t bhy2_hif_get_nonwakeup_fifo(uint8_t* data, uint32_t capacity, uint32_t* read_count,
								 uint32_t* remaining, struct bhy2_hif_dev* hif) {
	(void)hif;
	return fixture_read(BHY2_IST_FIFO_NW_DRDY, data, capacity, read_count, remaining);
}

int8_t bhy2_hif_get_status_fifo_async(uint8_t* data, uint32_t capacity, uint32_t* read_count,
									uint32_t* remaining, struct bhy2_hif_dev* hif) {
	(void)hif;
	return fixture_read(BHY2_IST_MASK_DEBUG, data, capacity, read_count, remaining);
}

static void meta_callback(const struct bhy2_fifo_parse_data_info* info, void* ref) {
	assert(ref == &callback_count);
	assert(info->sensor_id == BHY2_SYS_ID_META_EVENT);
	assert(info->data_size == 4);
	assert(callback_count < sizeof(callback_payloads));
	callback_payloads[callback_count++] = info->data_ptr[0];
}

static struct bhy2_dev prepare(const uint8_t* bytes, size_t size, uint8_t status,
							 uint32_t chunk) {
	struct bhy2_dev dev = {0};
	fixture = bytes;
	fixture_size = size;
	fixture_offset = 0;
	fixture_status = status;
	fixture_chunk = chunk;
	callback_count = 0;
	memset(callback_payloads, 0, sizeof(callback_payloads));
	dev.table[0].sensor_id = BHY2_SYS_ID_META_EVENT;
	dev.table[0].callback = meta_callback;
	dev.table[0].callback_ref = &callback_count;
	return dev;
}

static void unknown_event(uint8_t status, uint8_t unknown_id) {
	const uint8_t bytes[] = {BHY2_SYS_ID_META_EVENT, 1, 2, 3, unknown_id};
	struct bhy2_dev dev = prepare(bytes, sizeof(bytes), status, sizeof(bytes));
	uint8_t work[32];
	assert(bhy2_get_and_process_fifo(work, sizeof(work), &dev) == BHY2_E_INVALID_EVENT_SIZE);
	assert(callback_count == 1);
	assert(callback_payloads[0] == 1);
	assert(fixture_offset == sizeof(bytes));
}

static void callback_isolation(void) {
	const uint8_t bytes[] = {BHY2_SYS_ID_META_EVENT, 1, 2, 3, 42, 8, 9};
	struct bhy2_dev dev = prepare(bytes, sizeof(bytes), BHY2_IST_MASK_DEBUG, sizeof(bytes));
	dev.event_size[42] = 3; /* Valid event size, with no registered consumer. */
	uint8_t work[32];
	assert(bhy2_get_and_process_fifo(work, sizeof(work), &dev) == BHY2_OK);
	assert(callback_count == 1);
	assert(callback_payloads[0] == 1);
}

static void unknown_first(void) {
	/* Same stalled suffix observed on hardware, without any earlier callback. */
	const uint8_t bytes[] = {0x21, 0x21, 0x21, 0, 0};
	struct bhy2_dev dev = prepare(bytes, sizeof(bytes), BHY2_IST_MASK_DEBUG, sizeof(bytes));
	uint8_t work[32];
	assert(bhy2_get_and_process_fifo(work, sizeof(work), &dev) == BHY2_E_INVALID_EVENT_SIZE);
	assert(callback_count == 0);
}

static void partial_event(void) {
	const uint8_t bytes[] = {BHY2_SYS_ID_META_EVENT, 1, 2, 3,
							BHY2_SYS_ID_META_EVENT, 4, 5, 6};
	struct bhy2_dev dev = prepare(bytes, sizeof(bytes), BHY2_IST_FIFO_NW_DRDY, 5);
	uint8_t work[32];
	assert(bhy2_get_and_process_fifo(work, sizeof(work), &dev) == BHY2_OK);
	assert(callback_count == 2);
	assert(callback_payloads[0] == 1 && callback_payloads[1] == 4);
	assert(fixture_offset == sizeof(bytes));
}

static void long_leftover(void) {
	uint8_t bytes[307];
	uint8_t expected[300];
	for (size_t i = 0; i < sizeof(bytes); ++i) {
		bytes[i] = (uint8_t)(i * 7U);
	}
	memcpy(expected, bytes + 7, sizeof(expected));
	struct bhy2_fifo_buffer fifo = {
		.read_pos = 7,
		.read_length = sizeof(bytes),
		.buffer_size = sizeof(bytes),
		.buffer = bytes,
	};
	assert(parse_fifo_support(&fifo) == BHY2_OK);
	assert(fifo.read_length == sizeof(expected));
	assert(memcmp(bytes, expected, sizeof(expected)) == 0);
}

int main(int argc, char** argv) {
	assert(argc == 2);
	if (strcmp(argv[1], "unknown-wakeup") == 0) {
		unknown_event(BHY2_IST_FIFO_W_DRDY, 42);
	} else if (strcmp(argv[1], "unknown-nonwakeup") == 0) {
		unknown_event(BHY2_IST_FIFO_NW_DRDY, 42);
	} else if (strcmp(argv[1], "unknown-status") == 0) {
		unknown_event(BHY2_IST_MASK_DEBUG, 42);
	} else if (strcmp(argv[1], "reserved-id") == 0) {
		unknown_event(BHY2_IST_MASK_DEBUG, 249);
	} else if (strcmp(argv[1], "callback-isolation") == 0) {
		callback_isolation();
	} else if (strcmp(argv[1], "unknown-first") == 0) {
		unknown_first();
	} else if (strcmp(argv[1], "partial-event") == 0) {
		partial_event();
	} else if (strcmp(argv[1], "long-leftover") == 0) {
		long_leftover();
	} else {
		fprintf(stderr, "Unknown scenario: %s\n", argv[1]);
		return EXIT_FAILURE;
	}
	printf("PASS %s\n", argv[1]);
	return EXIT_SUCCESS;
}
