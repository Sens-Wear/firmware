# SenseWear Firmware

Zephyr-based firmware for the SenseWear wearable platform (board
`SensWear/nrf54l15/cpuapp`, vendor SenseraTechnologies). It brings up BLE plus a
set of on-board sensors, actuators, and power peripherals, and optionally one
daughter-board sensor selected at build time.

For toolchain and build setup see [SETUP.md](SETUP.md); for the source-tree
layout see [ORGANIZATION.md](ORGANIZATION.md); for build-system mechanics and
tests see [BUILD.md](BUILD.md).

## Architecture

The firmware is one Zephyr application image. Functionality is split into
**devices** (one out-of-tree driver each), **shared infrastructure** they depend
on, and **vendored libraries**. Everything is selected at configure time through
Kconfig — there are no parallel CMake options — and each piece compiles into one
of three interface libraries (`board_drivers`, `shield_drivers`, `libs`) that
link into the application. See [BUILD.md](BUILD.md) for the full mechanism.

### Base-board devices

These devices are permanently fitted to the main board. Their drivers live under
`boards/SenseraTechnologies/SensWear/drivers/` and are collected into
`board_drivers`. Each is gated by a Kconfig symbol that is **`default y`**, so it
is enabled out of the box and disabled by setting the symbol to `n`:

| Device | Function | Bus | Enable / disable symbol |
| --- | --- | --- | --- |
| BQ25180 | Battery charger / power path | I²C | `CONFIG_SENSEWEAR_BQ25180_DRIVER` |
| BQ27427 | Fuel gauge | I²C | `CONFIG_SENSEWEAR_BQ27427_DRIVER` |
| BHI360 | IMU | SPI | `CONFIG_SENSEWEAR_BHI360_DRIVER` |
| LP5562 | LED controller | I²C | `CONFIG_SENSEWEAR_LP5562_DRIVER` |
| M95P | Serial-page EEPROM | SPI | `CONFIG_SENSEWEAR_M95P_DRIVER` |
| TPSM83102 | Buck regulator (daughter rail) | I²C | `CONFIG_SENSEWEAR_TPSM83102_DRIVER` |

### Daughter-board devices (shields)

Optional daughter hardware is modeled as Zephyr **shields** under
`boards/shields/`. A shield is selected with `SHIELD=sensewear_<name>` (or
`--shield`), which sets `CONFIG_SHIELD_SENSEWEAR_<NAME>`; only then is the
shield's driver compiled (into `shield_drivers`) and its devicetree overlay
applied. Selecting a shield also drives the `SENSEWEAR_DAUGHTER_BOARD` Kconfig
choice, which adds that board's BLE bridge and GATT service to the application.

| Shield | Device | Function |
| --- | --- | --- |
| `sensewear_haptic` | DRV2605 | Haptics |
| `sensewear_ppg` | MAX30101 | PPG / heart rate |
| `sensewear_temperature` | MAX30208 | Skin temperature |
| `sensewear_touch` | MTCH6102 | Capacitive touch |

All daughter devices share one regulated rail (TPSM83102 / `VDD_DAUGHTER`), so
**at most one shield may be enabled at a time** — the build fails if two
regulator-dependent devices are active. The exact rule and the cases where
multiple shields could coexist are described in [BUILD.md](BUILD.md).

### Shared infrastructure

The device drivers build on a small set of in-house subsystems, all under the
board `drivers/` tree and enabled automatically (`default y`, or `select`ed by
the drivers that need them):

| Subsystem | Symbol | Purpose |
| --- | --- | --- |
| System I²C wrapper | `CONFIG_SENSEWEAR_SYS_I2C` | Ownership-locking wrapper over the Nordic I²C controller (`bus = <&sys_i2c>`) |
| System SPI wrapper | `CONFIG_SENSEWEAR_SYS_SPI` | Ownership-locking wrapper over the Nordic SPI controller (`bus = <&sys_spi>`) |
| Daughter-connector arbiter | `CONFIG_SENSEWEAR_DAUGHTER_IF_DRIVER` | Runtime arbitration of shared connector GPIO lines |
| Device event manager | `CONFIG_SENSEWEAR_DEVICE_DRIVER_EVENTS` | Shared queue + thread for device interrupt events |

### Libraries

| Library | Where | Enabled when |
| --- | --- | --- |
| BHY2 Sensor API (Bosch, vendored) | `libs/BHY2-Sensor-API/` | `CONFIG_SENSEWEAR_BHI360_DRIVER` (the IMU driver uses it) |
| Zephyr subsystems | `prj.conf` | BLE (`CONFIG_BT`), LittleFS over disk-access on the M95P (`CONFIG_FILE_SYSTEM_LITTLEFS`, `CONFIG_DISK_ACCESS`, `CONFIG_SENSEWEAR_M95P_DISK`), POSIX file API (`CONFIG_POSIX_API`), logging/shell |

Vendored libraries are aggregated into the `libs` interface target by
`libs/libs.cmake` and linked only when the consuming driver is enabled. The M95P
EEPROM is additionally exposed as a LittleFS filesystem at `/eeprom` through a
local Zephyr-module disk extension (`config/cmake/zephyr_module/`); see
[ORGANIZATION.md](ORGANIZATION.md).

### Testing

Each device driver has a standalone on-target bring-up test under
`tests/drivers/` (base-board) or `tests/shields/` (shield-scoped). A test is
enabled with its `CONFIG_SENSEWEAR_TEST_<DRIVER>_DRIVER` Kconfig symbol
(`default n`); enabling one replaces the application `main()` with the test's,
and exactly one may be on at a time. Shield tests additionally require the
matching `SHIELD=` so the driver and its devicetree node are present.

Because selection is Kconfig rather than CMake options, the symbols are passed on
the configure line (`-DCONFIG_…=y`) and forwarded into Kconfig — ready-made
combinations are provided as presets in `CMakePresets.json`. The full list of
which CMake options must be set for each test, how they reach Kconfig, and the
preset/`west` invocations is in [BUILD.md](BUILD.md#tests-selection-and-how-it-reaches-kconfig).
