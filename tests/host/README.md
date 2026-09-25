# Host touch decoder tests

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
