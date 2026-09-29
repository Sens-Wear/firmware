# Host driver tests

These tests exercise the SensWear 15-electrode slider and host gesture decoder
without Zephyr, a touch controller, or BLE. They verify the physical RX mapping,
both RX banks, the bank seam, local centroid selection, hysteresis, debounce,
release, diagnostic status, gestures, reset, and 64-bit timing.

From the firmware repository in PowerShell with GCC on `PATH`:

```powershell
New-Item -ItemType Directory -Force build/host | Out-Null
gcc -std=c11 -Wall -Wextra -Werror `
  -I boards/shields/senswear_touch/drivers/touch/mtch6102 `
  tests/host/test_mtch6102_slider.c `
  boards/shields/senswear_touch/drivers/touch/mtch6102/mtch6102_slider.c `
  -o build/host/test_mtch6102_slider.exe
./build/host/test_mtch6102_slider.exe
```

This does not validate electrode sensitivity, acquisition timing, or physical
hardware. Use the touch shield bring-up target for those checks.

Standalone host builds default to ring-enclosure thresholds (16/6). Repeat the
slider and driver tests with `-DCONFIG_SENSWEAR_TOUCH_NO_ENCLOSURE=1` added to
the compiler commands to exercise the bare-board thresholds (55/40). Firmware
builds select the same option through Kconfig, as documented in the touch shield
README.

## Touch startup recovery

`mtch6102_lifecycle.c` compiles the production `mtch6102.c` and slider against
the small platform mocks in `mtch6102_mocks/`. It injects probe I2C failures and
checks recovery through the public configuration API, persistent failure errno,
default register application and CFG completion, and reuse of an already enabled
rail and claimed GPIOs. Each scenario runs in a fresh process so the tests never
access or reset the driver's private state.

```powershell
New-Item -ItemType Directory -Force build/host | Out-Null
gcc -std=c11 -Wall -Wextra -Werror `
  -I tests/host/mtch6102_mocks `
  -I boards/shields/senswear_touch/drivers/touch/mtch6102 `
  tests/host/mtch6102_lifecycle.c `
  boards/shields/senswear_touch/drivers/touch/mtch6102/mtch6102.c `
  boards/shields/senswear_touch/drivers/touch/mtch6102/mtch6102_slider.c `
  -o build/host/mtch6102_lifecycle.exe
./build/host/mtch6102_lifecycle.exe transient
./build/host/mtch6102_lifecycle.exe persistent
./build/host/mtch6102_lifecycle.exe direct-config
```

These are single-threaded startup tests. They do not validate interrupt timing,
Zephyr concurrency, physical I2C behavior, electrode response, or BLE streaming.

## Touch calibration diagnostics

`mtch6102_diagnostic.c` compiles the test-only diagnostic APIs against the same
platform mocks. It checks coherent SENSORVALUES/RAWVALUES/BASEVALUES snapshots,
cached compensation/configuration readback, cache preservation on failed or
overlapping reads, fresh frames after start and baseline commands, idle snapshot
refresh, and CMD.BS
completion, timeout, and transport errors. Touch decoding still uses native
SENSORVALUES; RAWVALUES and BASEVALUES remain diagnostics.

```powershell
gcc -std=c11 -Wall -Wextra -Werror `
  -DCONFIG_SENSWEAR_TEST_MTCH6102_DRIVER=1 `
  -I tests/host/mtch6102_mocks `
  -I boards/shields/senswear_touch/drivers/touch/mtch6102 `
  tests/host/mtch6102_diagnostic.c `
  boards/shields/senswear_touch/drivers/touch/mtch6102/mtch6102.c `
  boards/shields/senswear_touch/drivers/touch/mtch6102/mtch6102_slider.c `
  -o build/host/mtch6102_diagnostic.exe
./build/host/mtch6102_diagnostic.exe
```

The mocks model API ordering and error handling. They do not establish physical
baseline behavior, through-cover sensitivity, gesture quality, or controller
timing. The touch shield README describes the required hardware measurements.

## BHI360 FIFO parser progress

`test_bhy2_fifo.c` exercises the actual vendored Bosch parser with deterministic
FIFO reads. It verifies rejection of zero-size events in all three FIFO classes,
the observed `0x21` stalled suffix, reserved ID 249, callback isolation between
registered and unregistered events, preservation of partial valid frames, and
copying more than 255 leftover bytes. The test includes the vendor source to
exercise that internal copy boundary directly; it does not reimplement parsing.

```powershell
New-Item -ItemType Directory -Force build/host | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -I libs/BHY2-Sensor-API `
  tests/host/test_bhy2_fifo.c -o build/host/test_bhy2_fifo.exe
@'
import subprocess
cases = (
    "unknown-first", "unknown-wakeup", "unknown-nonwakeup", "unknown-status",
    "reserved-id", "callback-isolation", "partial-event", "long-leftover",
)
for case in cases:
    subprocess.run(["build/host/test_bhy2_fifo.exe", case], check=True, timeout=4)
'@ | python -
```

The timeout is part of the regression: malformed input must return an error
instead of trapping the shared device-manager thread. The local vendor patch
and observed hardware evidence are documented in
[`LOCAL_CHANGES.md`](../../libs/BHY2-Sensor-API/LOCAL_CHANGES.md).
This harness does not establish physical SPI behavior or hardware recovery.
