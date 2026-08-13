# AGENTS.md

## Project overview

This repository contains the SensWear embedded firmware for the Nordic nRF54L15, built with
Zephyr/Nordic tooling, CMake, Kconfig, devicetree, and Ninja. It owns the hardware drivers,
device-manager streams, application behavior, and Bluetooth GATT wire protocol.

Read `README.md`, `SETUP.md`, and `BUILD.md` before changing build structure or board support.
`BUILD.md` explains how Kconfig controls source selection and why shield/test validation exists.

## Repository map

- `src/app/`: application entry points and orchestration.
- `src/bluetooth/`: GATT services and the authoritative BLE payload definitions.
- `src/drivers/`: application-level sensor and actuator drivers.
- `boards/SenseraTechnologies/SensWear/`: base-board devicetree, pin control, and board drivers.
- `boards/shields/`: haptic, PPG, temperature, and touch shield definitions and drivers.
- `boards/common/`: common buses, device manager, and device-driver event infrastructure.
- `tests/drivers/`, `tests/shields/`, `tests/device_manager/`: hardware bring-up targets.
- `libs/`: vendored third-party code, especially Bosch's BHY2 Sensor API.
- `config/`: debugger and tool wrapper configuration.
- `tools/`: project utilities.

Do not casually edit vendored files under `libs/BHY2-Sensor-API/`; keep such changes isolated
and explain why the upstream code must differ.

## Environment and setup

- Use the Nordic/Zephyr environment documented in `SETUP.md`.
- Copy `.env.example` to `.env` for local tool paths. `.env` is local configuration: do not
  commit it, expose it in logs, or replace portable project configuration with machine paths.
- Required variables include `ZEPHYR_BASE` and `NCS_TOOLCHAIN_ROOT`.
- Use an out-of-source directory under `build/`. Do not commit generated build products,
  `compile_commands.json`, logs, or flashed binaries unless explicitly requested.

## Build commands

Configure and build the base application:

```sh
cmake --preset senswear_nrf54l15_cpuapp_base
cmake --build --preset senswear_nrf54l15_cpuapp_base
```

Shield application presets:

```sh
cmake --preset senswear_nrf54l15_cpuapp_haptic
cmake --preset senswear_nrf54l15_cpuapp_ppg
cmake --preset senswear_nrf54l15_cpuapp_temperature
cmake --preset senswear_nrf54l15_cpuapp_touch
```

Build the same named preset with `cmake --build --preset <name>`. The PPG application preset
also enables the temperature shield because that daughter board carries the MAX30208.

Flash only when the user requests hardware programming and the target/build directory is
unambiguous:

```sh
west flash -d <build-directory>
```

Flashing is an external hardware mutation. Never infer the probe or target from stale output.

## Tests

Tests are firmware images that usually require the matching physical board. Use the presets in
`CMakePresets.json`, for example:

```sh
cmake --preset test_max30101
cmake --build --preset test_max30101
```

Available presets cover base drivers, shield drivers, RTC, and device-manager variants. Enable
exactly one bring-up test across `tests/drivers` and `tests/shields`; CMake intentionally fails
if multiple mutually exclusive tests are selected. A shield test requires both its Kconfig test
symbol and `SHIELD=`, so prefer the preset instead of reproducing flags manually.

For a source-only change, build the narrowest affected application/test preset. For changes to
common code, Kconfig, devicetree, CMake, device manager, or BLE, build the base app plus every
affected shield/test configuration. State clearly when validation was compile-only because no
physical hardware was available.

## Code style

- C and header files follow the root `.clang-format`: tabs, width 4, K&R attached braces,
  100-column limit, left-aligned pointers, and unsorted includes.
- Run `clang-format` only on files you intentionally changed; avoid formatting vendored code or
  unrelated files.
- Keep public declarations in headers and implementation details `static` in C files.
- Check every Zephyr/device API return value unless failure is provably impossible.
- Avoid unbounded work, blocking waits, allocation, or logging in interrupt context.
- Preserve explicit units and signedness in names and types (`*_ms`, `*_us`, `uint16_t`, etc.).
- Keep Kconfig, CMake source gating, devicetree compatibles, overlays, and documentation aligned.

## Bluetooth protocol rules

`src/bluetooth/` is the source of truth for UUIDs, characteristic properties, binary layouts,
units, enum values, timing, and validation. Treat all payloads as versioned public interfaces.

When changing a BLE interface:

1. Update the service implementation and any device-manager producer/consumer.
2. Verify exact packed size, endianness, padding, signedness, timestamp epoch/unit, and GATT
   properties.
3. Update both SDK repositories' UUID registries, typed parsers/encoders, module APIs, tests,
   examples, versions when appropriate, and README wire-format documentation.
4. Update the mobile app's vendored TypeScript SDK package and affected screen behavior.
5. Call out compatibility breaks explicitly; do not silently reuse a UUID with a new layout.

Related repositories are commonly checked out at:

- `../SDKs/Python`
- `../SDKs/TypeScript`
- `../Hardware/hardware-v1`
- `C:\SenswearMobileApp`

Verify that those paths exist before touching them. Do not modify another repository unless the
task places it in scope.

## Safety and generated files

- Do not change pin assignments, power sequencing, regulator settings, charging behavior, flash
  partitions, or sensor register values without checking the schematic/datasheet implications.
- Do not claim a hardware test passed when only compilation succeeded.
- Preserve unrelated user changes; this repository may contain active hardware bring-up work.
- Never commit secrets, private keys, serial numbers, local debugger paths, or device captures
  containing sensitive data.

## Completion checklist

- The relevant application/test presets configure and build.
- New behavior has a focused test target or a documented hardware validation procedure.
- Kconfig/CMake/devicetree changes are consistent.
- BLE changes are synchronized across SDKs, mobile app, tests, examples, and docs.
- `git diff --check` is clean and generated artifacts are not included accidentally.
