# SensWear local changes

The vendored `bhy2.c` identifies itself as Bosch Sensor API v1.6.0 (2023-03-24).
Keep these changes isolated when replacing or comparing the upstream library.

## FIFO parser progress and callback isolation

On the assembled board, the shared device-manager thread stopped processing
touch while the MTCH6102 continued generating SYNC frames. A live stack placed
the manager inside the BHI360 STATUS FIFO parser: `read_pos = 29`,
`read_length = 34`, remaining hardware bytes zero, and the unconsumed suffix was
`21 21 21 00 00`. Sensor ID `0x21` had event size zero. The parser repeatedly
called the previous meta-event callback with this unrelated ID and zero size,
then advanced its cursor by zero. Both the touch mutex and shared I2C bus were
free; this was an IMU parser loop starving the shared consumer.

The local correction:

- Clears callback lookup output before each search. A known-size event without
  a registered consumer is skipped without reusing an earlier callback.
- Returns `BHY2_E_INVALID_EVENT_SIZE` before calling a callback for a zero-size
  event, rather than attempting to guess the frame boundary.
- Preserves a wake-FIFO parser error through its helper's return value.
- Uses a 32-bit index for the internal leftover-copy routine's 32-bit length.
  This prevents index wrap if that routine receives 256 or more leftover bytes;
  it was not the cause observed on the live board.

An application-only check cannot interrupt the original parser once it enters
the non-progressing loop, so the parser correction belongs in this vendor file.
The change does not establish why the unexpected STATUS FIFO bytes appeared.
The caller remains responsible for error reporting and acquisition recovery.

`tests/host/test_bhy2_fifo.c` includes the actual vendor implementation and
supplies deterministic FIFO reads. It covers malformed events in all three FIFO
classes, reserved ID 249, the observed `0x21` suffix, callback isolation, valid
partial frames, and the leftover-copy boundary. The pre-fix source times out on
the `0x21` suffix and long-copy cases under a four-second process deadline;
the patched source passes all cases. Valid partial frames pass both versions.
See `tests/host/README.md` for commands. These tests do not validate the physical
SPI transport or touch sensitivity through the enclosure.
