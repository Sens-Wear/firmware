# SensWear Firmware

Zephyr-based firmware for the SensWear wearable platform (board
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
| BQ25180 | Battery charger / power path | I²C | `CONFIG_SENSWEAR_BQ25180_DRIVER` |
| BQ27427 | Fuel gauge | I²C | `CONFIG_SENSWEAR_BQ27427_DRIVER` |
| BHI360 | IMU | SPI | `CONFIG_SENSWEAR_BHI360_DRIVER` |
| LP5562 | LED controller | I²C | `CONFIG_SENSWEAR_LP5562_DRIVER` |
| M95P | Serial-page EEPROM | SPI | `CONFIG_SENSWEAR_M95P_DRIVER` |
| TPSM83102 | Buck regulator (daughter rail) | I²C | `CONFIG_SENSWEAR_TPSM83102_DRIVER` |

### Daughter-board devices (shields)

Optional daughter hardware is modeled as Zephyr **shields** under
`boards/shields/`. A shield is selected with `SHIELD=senswear_<name>` (or
`--shield`), which sets `CONFIG_SHIELD_SENSWEAR_<NAME>`; only then is the
shield's driver compiled (into `shield_drivers`) and its devicetree overlay
applied. Selecting a shield also drives the `SENSWEAR_DAUGHTER_BOARD` Kconfig
choice, which selects the haptic, PPG, or touch GATT service. The temperature
service follows its shield independently and can accompany PPG.

| Shield | Device | Function |
| --- | --- | --- |
| `senswear_haptic` | DRV2605 | Haptics |
| `senswear_ppg` | MAX30101 | PPG / heart rate |
| `senswear_temperature` | MAX30208 | Skin temperature |
| `senswear_touch` | MTCH6102 | Capacitive touch |

The daughter connector has one regulated rail (TPSM83102 / `VDD_DAUGHTER`), so
the build permits at most one device declaring ownership of that regulator.
The temperature shield declares no regulator ownership and can accompany PPG;
the PPG application preset enables both shields. The exact rule is described
in [BUILD.md](BUILD.md).

### Shared infrastructure

The device drivers build on a small set of in-house subsystems, all under the
board `drivers/` tree and enabled automatically (`default y`, or `select`ed by
the drivers that need them):

| Subsystem | Symbol | Purpose |
| --- | --- | --- |
| System I²C wrapper | `CONFIG_SENSWEAR_SYS_I2C` | Ownership-locking wrapper over the Nordic I²C controller (`bus = <&sys_i2c>`) |
| System SPI wrapper | `CONFIG_SENSWEAR_SYS_SPI` | Ownership-locking wrapper over the Nordic SPI controller (`bus = <&sys_spi>`) |
| Daughter-connector arbiter | `CONFIG_SENSWEAR_DAUGHTER_IF_DRIVER` | Runtime arbitration of shared connector GPIO lines |
| Device event manager | `CONFIG_SENSWEAR_DEVICE_DRIVER_EVENTS` | Shared queue + thread for device interrupt events |

### Libraries

| Library | Where | Enabled when |
| --- | --- | --- |
| BHY2 Sensor API (Bosch, vendored) | `libs/BHY2-Sensor-API/` | `CONFIG_SENSWEAR_BHI360_DRIVER` (the IMU driver uses it) |
| Zephyr subsystems | `prj.conf` | BLE (`CONFIG_BT`), LittleFS over disk-access on the M95P (`CONFIG_FILE_SYSTEM_LITTLEFS`, `CONFIG_DISK_ACCESS`, `CONFIG_SENSWEAR_M95P_DISK`), POSIX file API (`CONFIG_POSIX_API`), logging/shell |

Vendored libraries are aggregated into the `libs` interface target by
`libs/libs.cmake` and linked only when the consuming driver is enabled. The M95P
EEPROM is additionally exposed as a LittleFS filesystem at `/eeprom` through a
local Zephyr-module disk extension (`config/cmake/zephyr_module/`); see
[ORGANIZATION.md](ORGANIZATION.md).

### Testing

Each device driver has a standalone on-target bring-up test under
`tests/drivers/` (base-board) or `tests/shields/` (shield-scoped). A test is
enabled with its `CONFIG_SENSWEAR_TEST_<DRIVER>_DRIVER` Kconfig symbol
(`default n`); enabling one replaces the application `main()` with the test's,
and exactly one may be on at a time. Shield tests additionally require the
matching `SHIELD=` so the driver and its devicetree node are present.

Because selection is Kconfig rather than CMake options, the symbols are passed on
the configure line (`-DCONFIG_…=y`) and forwarded into Kconfig — ready-made
combinations are provided as presets in `CMakePresets.json`. The full list of
which CMake options must be set for each test, how they reach Kconfig, and the
preset/`west` invocations is in [BUILD.md](BUILD.md#tests-selection-and-how-it-reaches-kconfig).

## Flashing

In VS Code, flashing and debugging run through the hybrid nRF Connect + CMake
Tools flow described in [SETUP.md](SETUP.md#vs-code-extensions-hybrid-nrf-connect--cmake-tools-flow).
From the command line, flash a build directory with west:

```sh
west flash -d build/app/base
```

## Bluetooth

On boot the firmware starts connectable advertising and registers its GATT
services (the LED and IMU services, plus the selected daughter board's service).
The advertised device name is set with `CONFIG_BT_DEVICE_NAME` in `prj.conf`.

Clients can read the actual application version from the standard Device
Information service (`180a` / Firmware Revision String `2a26`). The root
[`VERSION`](VERSION) file defines the release version; the returned string is
generated by Zephyr when the image is built. A separate read-only capabilities
service reports compiled shield and supported feature masks so clients can
disable unsupported sensor/actuator pages. These masks describe the firmware
configuration, not physical daughter-board detection. See the
[metadata wire format and hardware validation procedure](src/bluetooth/services/device_info/README.md).

## Storage and filesystem

The M95P EEPROM is registered with Zephyr's disk-access subsystem and mounted as
a LittleFS filesystem at `/eeprom`. The mount happens automatically at boot
(`CONFIG_SENSWEAR_M95P_DISK_AUTOMOUNT`); application code can also mount or
unmount it explicitly with `m95p_fs_mount()` / `m95p_fs_unmount()`.

`prj.conf` enables `CONFIG_POSIX_API`, so the filesystem is reachable through
three layers:

- Zephyr's native filesystem API (`fs_open`/`fs_read`/`fs_write`).
- POSIX file operations (`open`/`read`/`write`/`close`).
- The C standard-library stdio API (`fopen`/`fread`/`fwrite`/`fclose`), which the
  libc retargets onto the POSIX calls above.

`CONFIG_ZVFS_OPEN_MAX` bounds the number of simultaneously open file descriptors.
The M95P bring-up test (`tests/drivers/main_test_m95p.c`) demonstrates the POSIX
file API by round-tripping a text file and a binary file against `/eeprom`.

## Logging

Logging is enabled over UART in immediate mode; adjust verbosity with
`CONFIG_LOG_DEFAULT_LEVEL` in `prj.conf`. A shell is available over the same
UART.
