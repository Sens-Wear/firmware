# Local tool paths

See `ORGANIZATION.md` for source-tree ownership and the local Zephyr module
integration under `config/cmake/zephyr_module/`.

The checked-in VS Code and CMake configuration avoids machine-specific SDK
paths. Copy `.env.example` to `.env` on each development machine and adjust the
paths:

```sh
cp .env.example .env
```

`.env` is git-ignored. It defines the local toolchain locations:

| Variable              | Meaning                                                        |
| --------------------- | ------------------------------------------------------------- |
| `NCS_TOOLCHAIN_ROOT`  | NCS toolchain bundle (contains `bin/` and `opt/zephyr-sdk`).  |
| `ZEPHYR_BASE`         | Zephyr tree inside the installed NCS version.                 |
| `ZEPHYR_GDB`          | Zephyr SDK GDB; the wrappers derive matching `objdump`/`nm`.  |
| `OPENOCD`             | OpenOCD executable (flash/debug).                             |
| `OPENOCD_SCRIPTS`     | OpenOCD scripts directory (`-s`).                             |
| `JLINK_GDB_SERVER`    | Optional SEGGER GDB server path when it is not already on PATH.|

On Windows, use the same variable names with the local Windows paths. If the
toolchain is installed somewhere else, only the variable values should change,
not the repository files.

VS Code does not automatically load `.env` into `${env:...}` substitutions.
The checked-in OpenOCD, GDB, objdump, and nm wrappers read `.env` directly for
flash/debug tasks. VS Code uses the `.sh` wrappers on macOS/Linux and the
`.cmd` wrappers on Windows. For CMake, clangd, and other extension settings,
use one of these
workflows:

```sh
./config/scripts/code-with-env.sh
```

On Windows:

```bat
config\scripts\code-with-env.cmd
```

If VS Code is already running, quit it first so the new window inherits the
environment from the launcher.

Or install `direnv` and a VS Code direnv extension, then run:

```sh
direnv allow
```

## Building

The board, board root, and toolchain come from `CMakePresets.json`, which reads
the `.env` variables. With the environment loaded:

```sh
cmake --preset senswear_nrf54l15_cpuapp
cmake --build build
```

The active board is `SensWear/nrf54l15/cpuapp`; the build output lands in
`build/firmware/` (the sysbuild image domain matches the CMake `project()` name).

For one-off local CMake overrides, create `CMakeUserPresets.json`; it is
ignored by git.

## Debugging from VS Code

The Run and Debug selector provides three launch configurations:

- `CMSIS-DAP: flash and debug nRF54L15`
- `ST-Link/V2: flash and debug nRF54L15`
- `J-Link: flash and debug nRF54L15`

The OpenOCD profiles use probe-specific adapter files and a shared nRF54L15
target file under `config/`. They obtain the active build directory from the
CMake Tools extension and flash `firmware/zephyr/zephyr.hex` from that build
before Cortex-Debug attaches. The debugger uses the matching
`firmware/zephyr/zephyr.elf`; this also supports the driver-test presets under
`build/tests/`. The J-Link profile programs the selected ELF through the SEGGER
GDB server.

The selected OpenOCD installation must provide:

```text
interface/cmsis-dap.cfg
interface/stlink.cfg
target/nordic/nrf54l.cfg
```
