# Source tree organization

This document describes where firmware components belong and how they enter the
Zephyr build.

## Top-level build and configuration

| Path | Purpose |
| --- | --- |
| `CMakeLists.txt` | Application build entry point. Registers the board, shields, devicetree bindings, and the local Zephyr module before loading Zephyr. It then links the board drivers, third-party libraries, and either the normal application or one driver test. |
| `Kconfig` | SenseWear application options, board-driver enable symbols, daughter-board selection, M95P disk options, and driver-test selections. |
| `prj.conf` | Default Zephyr configuration for the firmware image. |
| `sysbuild.conf` | Sysbuild configuration. |
| `CMakePresets.json` | Reproducible application and driver-test configure presets. |
| `SETUP.md` | Local toolchain, build, and debugging setup. |

Driver presence is controlled by `CONFIG_SENSEWEAR_<device>_DRIVER`. The
per-driver CMake files consume these resolved Kconfig symbols; there is no
second set of CMake driver options.

## Board definition and base-board drivers

The base-board implementation is under:

```text
boards/SenseraTechnologies/SensWear/
├── SensWear_nrf54l15_*.dts
├── SensWear_nrf54l15_*_defconfig
├── dts/bindings/
└── drivers/
```

- Board DTS, pin control, metadata, and default configuration describe the
  nRF54L15 targets and permanently fitted hardware.
- `dts/bindings/` contains bindings owned by the SenseWear board integration.
- `drivers/` contains drivers for devices physically owned by the base board:
  shared buses, charger, fuel gauge, IMU, LED controller, EEPROM, and
  regulator.

The driver tree is organized by subsystem:

```text
drivers/
├── bus/             # sys_i2c and sys_spi shared-bus wrappers
├── charger/         # BQ25180
├── gauge/           # BQ27427
├── imu/             # BHI360
├── led_controller/  # LP5562
├── memory/          # M95P and its disk-access adapter
└── regulator/       # TPSM83102
```

`drivers/board_drivers.cmake` includes each subsystem's
`<type>_drivers.cmake`. Those files append enabled sources and include paths to
the `board_drivers` interface library. `board_drivers.cmake` is itself pulled in
by the board-level aggregator `boards/drivers.cmake` (see below), alongside the
shared common infrastructure and the shield drivers.

## Board-level driver aggregation

`boards/drivers.cmake` is the single entry point the top-level `CMakeLists.txt`
includes to collect all driver code, as three INTERFACE libraries linked into
Zephyr's `app` target:

- `common_drivers` — shared infrastructure (below), used by both board and
  shield drivers;
- `board_drivers` — the base-board device drivers;
- `shield_drivers` — the selected daughter-board (shield) drivers.

Keeping the shared `common/` include in this aggregator means neither the board
nor the shield aggregate has to reach across trees to pull it in.

## Shared driver infrastructure

Infrastructure used by both the base-board drivers and the shield drivers lives
at the top of the boards tree, so it belongs to neither in particular:

```text
boards/common/
├── device_driver_events/         # shared device event manager (queue + consumer)
├── device_manager/               # message contract + device manager (zbus streams, opt-in)
├── generate_device_driver_dts_ids.py  # generates <LABEL>_DEVICE_ID at configure time
└── common_drivers.cmake          # builds the `common_drivers` INTERFACE library
```

`common_drivers.cmake` collects these into the `common_drivers` interface
library and additionally generates `device_driver_dts_ids.h` at configure time.
Because `common_drivers` is linked into `app` alongside `board_drivers` and
`shield_drivers`, both board and shield sources see these headers when compiled.

## Daughter boards

Optional daughter-board hardware is described as Zephyr shields under:

```text
boards/shields/
├── sensewear_haptic/
├── sensewear_ppg/
├── sensewear_temperature/
└── sensewear_touch/
```

Each shield owns its overlay, shield metadata, and Kconfig defaults. Selecting a
shield enables the corresponding `SENSEWEAR_DAUGHTER_*` choice. `src/app.cmake`
then adds only that daughter board's application bridge, Bluetooth service, and
device-facing source files.

## Application sources

```text
src/
├── main.c
├── app/
├── bluetooth/services/
├── drivers/
└── app.cmake
```

- `main.c` owns application startup.
- `app/` contains orchestration and bridges between hardware and Bluetooth
  services.
- `bluetooth/services/` contains custom GATT service implementations.
- `drivers/` contains application-level adapters and optional daughter-board
  drivers. Permanently fitted base-board drivers belong under the board tree
  instead.
- `app.cmake` creates the `app_src` interface library and selects sources from
  the resolved daughter-board Kconfig choice.

## Local Zephyr module extension

The project extends Zephyr's own CMake libraries through:

```text
config/cmake/zephyr_module/
├── zephyr/module.yml
├── CMakeLists.txt
└── drivers/disk/CMakeLists.txt
```

The top-level `CMakeLists.txt` appends this directory to
`EXTRA_ZEPHYR_MODULES` before `find_package(Zephyr)`. Zephyr discovers
`zephyr/module.yml` and later evaluates the module's root `CMakeLists.txt` after
Kconfig has resolved the `CONFIG_*` symbols.

The disk extension is loaded only when both
`CONFIG_SENSEWEAR_M95P_DRIVER` and `CONFIG_SENSEWEAR_M95P_DISK` are enabled.
Its `zephyr_library_amend()` call selects Zephyr's existing `drivers__disk`
library and marks it as allowed to remain empty. This is intentional because
the M95P disk backend is compiled through `board_drivers`, while
`CONFIG_DISK_ACCESS` still causes Zephyr to create the in-tree disk-driver
library.

When `CONFIG_FILE_SYSTEM_LITTLEFS` is set, the same adapter
(`memory/m95p/m95p_disk.c`) layers a LittleFS filesystem on the disk and, with
`CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT`, mounts it at `/eeprom` during system
init; `m95p_fs_mount()` / `m95p_fs_unmount()` expose the same operation to
application code. `prj.conf` enables `CONFIG_POSIX_API` (sized by
`CONFIG_ZVFS_OPEN_MAX`) so the mount is reachable not only through Zephyr's
native `fs_*` API but also through POSIX file calls (`open`/`read`/`write`) and
the C standard-library stdio API (`fopen`/`fread`/`fwrite`), which the libc
retargets onto those POSIX calls.

New extensions to Zephyr-owned libraries should follow the same mirrored path
structure, for example `drivers/<subsystem>/CMakeLists.txt`, under this module.

## Third-party libraries

Vendored dependencies are stored under `libs/`. `libs/libs.cmake` aggregates
their interface targets into the `libs` target. The BHY2 sensor API is currently
integrated through `libs/bhy2-sensors.cmake`.

## Driver bring-up tests

`tests/drivers/` contains one standalone `main_test_<device>.c` per base-board
driver. Enabling one `CONFIG_SENSEWEAR_TEST_<device>_DRIVER` symbol replaces
the normal application sources with that test. `tests/drivers/tests.cmake`
rejects configurations that select more than one test.

## Development configuration

`config/` contains repository-managed development configuration:

- `config/cmake/` contains Zephyr/CMake integration.
- `config/scripts/` contains portable environment wrappers.
- `config/openocd-*.cfg` contains probe and target configurations.

Machine-specific tool paths belong in the ignored `.env` file, as described in
`SETUP.md`.

## Generated files

`build/`, `.cache/`, and the root `compile_commands.json` symlink are generated
artifacts. Source files and persistent configuration must not be added under
the build tree.
