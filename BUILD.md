# Build system

How the SenseWear firmware is assembled by CMake and Kconfig: how board
drivers, shield drivers, and third-party libraries are gated, what the build
validates, what each shield turns on, when more than one shield may be enabled,
and how the bring-up tests are selected.

See `ORGANIZATION.md` for the source-tree layout and `SETUP.md` for toolchain
setup; this document covers the build *mechanics*.

## One source of truth: Kconfig drives CMake

Driver and test selection lives entirely in **Kconfig**. There is no parallel
set of CMake `option()`s. Two reasons:

- **Single source of truth.** Whether a driver is compiled, and whether a test
  replaces the app, is decided by one `CONFIG_*` symbol that also flows into the
  C preprocessor, devicetree gating, and `prj.conf` overrides.
- **Sysbuild propagation.** Under sysbuild only Kconfig symbols are forwarded
  from the top-level build (and therefore from a CMake preset) down into this
  image. A CMake `option()` declared here would never receive the override, so
  the selection *must* be a `CONFIG_*` symbol.

### Why CMake can read `CONFIG_*` symbols

`CMakeLists.txt` calls `find_package(Zephyr ...)` early. That runs Kconfig and
exposes every resolved `CONFIG_*` symbol as a CMake variable. All of the gating,
counting, and validation logic described below runs *after* that call, so it
sees fully resolved Kconfig values and the parsed devicetree.

A few things must be set up *before* `find_package(Zephyr)` because Zephyr
consumes them during configuration:

- `BOARD_ROOT`, `SHIELD_ROOT`, `DTS_ROOT`, `EXTRA_ZEPHYR_MODULES`.
- The `sensewear_haptic` shield ships its own DRV2605 binding, so its
  `dts/bindings` directory is appended to `DTS_ROOT` — but only when the shield
  is actually selected. Since Kconfig has not run yet, this keys off the raw
  `SHIELD` **input** variable (command line / preset / `$ENV{SHIELD}`), not
  `CONFIG_SHIELD_SENSEWEAR_HAPTIC`:

  ```cmake
  if("${SHIELD}" MATCHES "sensewear_haptic" OR "$ENV{SHIELD}" MATCHES "sensewear_haptic")
      list(APPEND DTS_ROOT .../boards/shields/sensewear_haptic)
  endif()
  ```

## Build graph

The application target `app` (created by Zephyr) is linked against four
`INTERFACE` libraries. The driver libraries are gathered by one board-level
aggregator, `boards/drivers.cmake`, which the top-level `CMakeLists.txt`
includes; it in turn includes each library's own aggregator CMake file:

| Interface library | Aggregator | Contents |
| --- | --- | --- |
| `common_drivers` | `boards/common/common_drivers.cmake` | Infrastructure shared by board and shield drivers (event manager, message contract, generated device IDs) |
| `board_drivers` | `boards/SenseraTechnologies/SensWear/drivers/board_drivers.cmake` | Drivers for permanently-fitted base-board devices |
| `shield_drivers` | `boards/shields/shields.cmake` | Drivers shipped by the enabled daughter-board shield(s) |
| `libs` | `libs/libs.cmake` | Vendored third-party code (BHY2 sensor API) |

```
app
├── common_drivers  (boards/drivers.cmake ⇒ boards/common/common_drivers.cmake; always compiled)
├── board_drivers   (per-type <type>_drivers.cmake, gated on CONFIG_SENSEWEAR_*_DRIVER)
├── shield_drivers  (per-shield shield_drivers.cmake, gated on CONFIG_SHIELD_SENSEWEAR_*)
├── libs            (gated on the driver that needs them, e.g. BHY2 ⇐ BHI360)
└── app_src OR test_<driver>   (mutually exclusive — see Tests)
```

### Board drivers

`board_drivers.cmake` includes one `<type>_drivers.cmake` per subsystem (bus,
daughter_if, charger, gauge, imu, led_controller, memory, regulator). Each
appends its enabled sources / include dirs to `BOARD_DRIVER_SOURCES` and
`BOARD_DRIVER_INCLUDE_DIRS`, gating directly on the driver's Kconfig symbol:

```cmake
if(CONFIG_SENSEWEAR_BQ25180_DRIVER)
    file(GLOB _bq25180_src CONFIGURE_DEPENDS .../bq25180/*.c)
    list(APPEND BOARD_DRIVER_SOURCES ${_bq25180_src})
    list(APPEND BOARD_DRIVER_INCLUDE_DIRS .../bq25180)
endif()
```

The base-board driver symbols are all `default y` (see `Kconfig`):

| Symbol | Driver | Depends on |
| --- | --- | --- |
| `CONFIG_SENSEWEAR_BQ25180_DRIVER` | BQ25180 charger | `SENSEWEAR_SYS_I2C`; selects `…_DEVICE_DRIVER_EVENTS` |
| `CONFIG_SENSEWEAR_BQ27427_DRIVER` | BQ27427 fuel gauge | `SENSEWEAR_SYS_I2C`; selects `…_DEVICE_DRIVER_EVENTS` |
| `CONFIG_SENSEWEAR_BHI360_DRIVER` | BHI360 IMU | `SPI` |
| `CONFIG_SENSEWEAR_LP5562_DRIVER` | LP5562 LED controller | `SENSEWEAR_SYS_I2C` |
| `CONFIG_SENSEWEAR_M95P_DRIVER` | M95P EEPROM | `SPI` |
| `CONFIG_SENSEWEAR_TPSM83102_DRIVER` | TPSM83102 regulator | `SENSEWEAR_SYS_I2C` |
| `CONFIG_SENSEWEAR_DAUGHTER_IF_DRIVER` | Daughter-connector GPIO arbiter | `GPIO` |

The shared-bus wrappers (`SENSEWEAR_SYS_I2C`, `SENSEWEAR_SYS_SPI`) and the
device event manager (`SENSEWEAR_DEVICE_DRIVER_EVENTS`) are infrastructure the
drivers above depend on or `select`. The `boards/common/` subsystem is always
compiled and additionally runs `generate_device_driver_dts_ids.py` at configure
time against the merged `zephyr.dts` to emit `device_driver_dts_ids.h`.

### Shield drivers

`shields.cmake` includes a shield's `drivers/shield_drivers.cmake` only when
that shield's `CONFIG_SHIELD_SENSEWEAR_*` symbol is set, then collects the
results into the `shield_drivers` interface library:

```cmake
if(CONFIG_SHIELD_SENSEWEAR_HAPTIC)
    include(.../sensewear_haptic/drivers/shield_drivers.cmake)
endif()
# ...PPG, TEMPERATURE, TOUCH likewise
```

`CONFIG_SHIELD_SENSEWEAR_*` is itself `def_bool $(shields_list_contains,...)` in
each shield's `Kconfig.shield`, i.e. it is true exactly when the shield name was
passed via `SHIELD=` / `--shield`.

The haptic shield's DRV2605 source is additionally gated on
`CONFIG_SENSEWEAR_DRV2605_DRIVER` (its own shield-scoped Kconfig symbol, bound
to the `sensewear,drv2605` devicetree compatible). The other shields compile
their single driver source unconditionally once the shield is selected.

### Third-party libraries

`libs.cmake` links a vendored library into the `libs` aggregate only when the
driver that needs it is enabled. Today that is the BHY2 sensor API, pulled in
when `CONFIG_SENSEWEAR_BHI360_DRIVER` is set. `libs` is always linked into `app`
(and into `app_src`) because the IMU driver uses BHY2 by default.

## What each shield enables

Each shield is a Zephyr shield under `boards/shields/sensewear_<name>/` with
three pieces:

- `Kconfig.shield` — declares `CONFIG_SHIELD_SENSEWEAR_<NAME>`.
- `Kconfig.defconfig` — `default`s the Kconfig symbols the shield needs (only
  while its `SHIELD_*` symbol is set).
- `sensewear_<name>.overlay` — devicetree nodes for the daughter device on the
  shared `&sys_i2c_peripheral` bus.
- `drivers/shield_drivers.cmake` — the shield's driver source(s).

Selecting a shield also drives the `SENSEWEAR_DAUGHTER_BOARD` choice in `Kconfig`
(e.g. `SHIELD_SENSEWEAR_PPG` ⇒ `default SENSEWEAR_DAUGHTER_PPG`), which is what
`src/app.cmake` keys on to add that daughter board's BLE bridge and GATT
service sources.

| Shield | Daughter device | `Kconfig.defconfig` turns on | Shield driver source |
| --- | --- | --- | --- |
| `sensewear_haptic` | DRV2605 haptics (I²C `0x5a`) | `I2C`, `SYS_I2C`, `GPIO`, `TPSM83102_DRIVER`, `DAUGHTER_IF_DRIVER`, `DEVICE_DRIVER_EVENTS`, + `drv2605` Kconfig | `haptic/drv2605/drv2605.c` |
| `sensewear_ppg` | MAX30101 PPG (I²C `0x57`) | `I2C`, `SYS_I2C`, `GPIO`, `SENSOR`, `REGULATOR`, `TPSM83102_DRIVER`, `DAUGHTER_IF_DRIVER`, `DEVICE_DRIVER_EVENTS` | `ppg/max30101/max30101.c` |
| `sensewear_temperature` | MAX30208 temperature (I²C `0x50`) | `I2C`, `SYS_I2C`, `GPIO`, `SENSOR`, `TPSM83102_DRIVER` | `temperature/max30208/max30208.c` |
| `sensewear_touch` | MTCH6102 touch (I²C `0x25`) | `I2C`, `SYS_I2C`, `GPIO`, `SENSOR`, `REGULATOR`, `TPSM83102_DRIVER`, `DAUGHTER_IF_DRIVER`, `DEVICE_DRIVER_EVENTS` | `touch/mtch6102/mtch6102.c` |

## What the build validates

All checks run in the top-level `CMakeLists.txt` after `find_package(Zephyr)`:

1. **At most one device on the shared daughter rail.** The daughter connector
   exposes a single regulated rail (TPSM83102 / `VDD_DAUGHTER`). CMake walks the
   `sys_i2c_peripheral` bus in the parsed devicetree and counts the `okay`
   children whose `vin-supply` resolves to the `tpsm83102` node. **More than one
   is a `FATAL_ERROR`.** The surviving count is published to the application as
   the `SENSEWEAR_NUM_SHIELDS` compile definition.

2. **Exactly one bring-up test.** A board-driver test and a shield test both
   defining `main()` is a `FATAL_ERROR`. Within `tests/drivers/tests.cmake` and
   `tests/shields/tests.cmake`, selecting more than one test of the same kind —
   or, for board tests, `TEST_DRIVERS` on with none selected — is also fatal.

The driver subsystems do **not** perform consistency checks; presence is purely
the `if(CONFIG_…)` gate, so Kconfig's own `depends on` / `select` rules are the
only constraint there.

## When may more than one shield be enabled?

`SHIELD=` can technically take a semicolon-separated list, and the build counts
shields rather than forbidding multiples outright. The real constraint is
validation check #1: **at most one enabled `okay` device may draw on the shared
`tpsm83102` rail.**

Every current shield's daughter device declares `vin-supply = <&tpsm83102>`, so
**any two SenseWear shields enabled together trip the fatal regulator check** —
in practice exactly one daughter shield at a time. The
`SENSEWEAR_DAUGHTER_BOARD` Kconfig `choice` reinforces this on the application
side: only one `SENSEWEAR_DAUGHTER_*` (and thus one set of bridge/service
sources) can be selected.

Two shields could only legally coexist if at most one of them enabled a
regulator-dependent device on that rail (e.g. a temperature sensor shield which 
is powered independently, or with its device left `disabled`).

## Tests: selection and how it reaches Kconfig

Bring-up tests live in `tests/drivers/` (base-board drivers) and
`tests/shields/` (shield-scoped drivers). Each `main_test_<driver>.c` defines
its own `main()` and replaces the application: when any test is on, `src/app.cmake`
(`app_src`) is **not** built, and the test's `main()` is linked into `app`
instead.

### The flow

1. Each test has a Kconfig symbol `CONFIG_SENSEWEAR_TEST_<DRIVER>_DRIVER`
   (all `default n`). Shield tests additionally `depends on` their shield:
   - `CONFIG_SENSEWEAR_TEST_DRV2605_DRIVER` depends on `SENSEWEAR_DRV2605_DRIVER`
     (so the `sensewear_haptic` shield must be present).
   - `CONFIG_SENSEWEAR_TEST_MAX30101_DRIVER` depends on `SHIELD_SENSEWEAR_PPG`.
   - `CONFIG_SENSEWEAR_TEST_MAX30208_DRIVER` depends on
     `SHIELD_SENSEWEAR_TEMPERATURE`.
2. `CMakeLists.txt` reads those resolved symbols and derives two CMake flags:
   - `TEST_DRIVERS` — on if any `BQ25180/BQ27427/BHI360/LP5562/M95P/TPSM83102`
     test symbol is set.
   - `TEST_SHIELDS` — on if any `DRV2605/MAX30101/MAX30208` test symbol is set.
3. If either flag is on, the matching `tests/.../tests.cmake` is included; it
   creates a `test_<driver>` INTERFACE library from
   `main_test_<driver>.c` and links it into `app`. Otherwise `src/app.cmake` is
   included for a normal build.

### Which CMake options must be enabled, and how they reach Kconfig

Because selection is Kconfig (not CMake `option()`s), you enable a test by
setting its `CONFIG_*` on the configure command — CMake forwards `-DCONFIG_*`
into Kconfig, and (under sysbuild) down to this image:

| To build… | Must set | Plus |
| --- | --- | --- |
| a board-driver test | `CONFIG_SENSEWEAR_TEST_<DRIVER>_DRIVER=y` | — |
| the DRV2605 shield test | `CONFIG_SENSEWEAR_TEST_DRV2605_DRIVER=y` | `SHIELD=sensewear_haptic` |
| the MAX30101 shield test | `CONFIG_SENSEWEAR_TEST_MAX30101_DRIVER=y` | `SHIELD=sensewear_ppg` |
| the MAX30208 shield test | `CONFIG_SENSEWEAR_TEST_MAX30208_DRIVER=y` | `SHIELD=sensewear_temperature` |

A shield test needs **both** the test symbol and `SHIELD=` so the driver source,
its Kconfig symbol, and its devicetree node are all present.

### The easy path: presets

`CMakePresets.json` bundles each test's symbol (and the shield, for shield
tests) into a ready-made preset:

```sh
cmake --preset test_max30101        # sets SHIELD=sensewear_ppg + CONFIG_…_MAX30101_DRIVER=y
cmake --build --preset test_max30101
```

Equivalent manual invocation:

```sh
west build -b SensWear/nrf54l15/cpuapp -- \
    -DSHIELD=sensewear_ppg \
    -DCONFIG_SENSEWEAR_TEST_MAX30101_DRIVER=y
```

Enable **exactly one** bring-up test across `tests/drivers` and
`tests/shields`; the build fails fast (validation #2) if two are on.
