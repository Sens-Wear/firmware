# Shield driver bring-up tests

Standalone, on-target bring-up tests for drivers shipped by a SenseWear shield —
one `main_test_<driver>.c` per shield driver. Each file defines its own `main()`
and exercises a single driver, mirroring `tests/drivers` but for shield-scoped
drivers (which also require the shield to be enabled).

| File | Driver | What it does |
|------|--------|--------------|
| `main_test_drv2605.c` | DRV2605 haptics (sensewear_haptic shield) | check device ready → alternate an RTP amplitude ramp and a ROM library click every 2 s |
| `main_test_max30101.c` | MAX30101 PPG (sensewear_ppg shield) | power VDD_DAUGHTER (5.0 V) → probe and configure → start wrist-HR sampling → drain interrupt-driven IR/Red/Green samples from the internal stream |
| `main_test_max30208.c` | MAX30208 temperature (sensewear_temperature shield) | probe and configure the FIFO → start timer-paced sampling → drain temperature samples from the internal stream on each `max30208_TimerIrq` |

## Building a test

Like `tests/drivers`, tests are selected with **Kconfig symbols** (all off by
default) so a normal build is unaffected, and they are Kconfig — not CMake
options — so the selection propagates through sysbuild / a CMake preset.

- `CONFIG_SENSEWEAR_TEST_<DRIVER>_DRIVER` — build that driver's test, e.g.
  `CONFIG_SENSEWEAR_TEST_DRV2605_DRIVER`.

Because the driver belongs to a shield, the shield must also be enabled so its
devicetree node and driver are present. The easiest way is the matching CMake
preset, which enables both:

```sh
cmake --preset test_drv2605
cmake --build --preset test_drv2605
```

Equivalently:

```sh
west build -b SensWear/nrf54l15/cpuapp -- \
    -DSHIELD=sensewear_haptic \
    -DCONFIG_SENSEWEAR_TEST_DRV2605_DRIVER=y
```

Enable exactly **one** bring-up test across `tests/drivers` and `tests/shields`
(each defines its own `main()`; the build fails fast if two are on). When a test
is enabled, the application main (`src/main.c`) is omitted and the test's
`main()` takes its place.

Wiring: the top `CMakeLists.txt` reads the `CONFIG_SENSEWEAR_TEST_*` symbols,
derives `TEST_SHIELDS`, and includes `tests/shields/tests.cmake` when it is on;
that file creates a `test_<driver>` INTERFACE library per enabled test and links
it into `app`.
