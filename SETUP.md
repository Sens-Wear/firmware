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

## VS Code extensions (hybrid nRF Connect + CMake Tools flow)

Development in VS Code uses two extensions together, and **both must be
installed**:

- **nRF Connect for VS Code** (`nordic-semiconductor.nrf-connect`) — provides the
  NCS toolchain/SDK integration and owns the *build configuration* that compiles
  the image, plus the debug bindings in `.vscode/settings.json`
  (`nrf-connect.debugging.bindings`).
- **CMake Tools** (`ms-vscode.cmake-tools`) — drives configure/build from
  `CMakePresets.json` (`cmake.useCMakePresets` is `always`) and exposes the
  *active preset's build directory*, which the flash and debug launchers consume.
  The workspace reads `cmake.cmakePath` from `CODEX_CMAKE_PATH`, which the
  repo's `config/scripts/code-with-env.*` launchers export to the platform-
  correct wrapper path.

The two are coupled through the CMake **configure preset** you select:

1. **Select the CMake configure preset** (CMake Tools status bar, or Command
   Palette → *CMake: Select Configure Preset*). This chooses the board and
   Kconfig options and, crucially, fixes the build directory. The flash tasks and
   debug launches in `.vscode/tasks.json` and `.vscode/launch.json` resolve that
   directory through the `cmake.buildDirectory` command
   (`${input:cmakeBuildDirectory}`), so **flashing and debugging only work once a
   preset is selected**.
2. **Build with the nRF Connect build configuration that matches that preset.**
   The build output must land in the same build directory the selected preset
   uses; if the nRF Connect build configuration and the CMake preset disagree,
   the flash/debug step programs a stale or missing image.

In short: pick the CMake preset first, build with the matching nRF Connect
configuration, then flash/debug — all three refer to the same build directory.
The command-line flow below is an alternative that does not need either
extension.

## Building

In VS Code, build through the **nRF Connect build configuration** that matches
your selected CMake preset (see the hybrid-flow section above); this is the
recommended path and keeps the build directory in sync with the flash/debug
launchers.

The command line below is the equivalent extension-free path. The board, board
root, and toolchain come from `CMakePresets.json`, which reads the `.env`
variables. With the environment loaded:

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

The `CMSIS-DAP` configuration drives any CMSIS-DAP-class probe through OpenOCD's
`cmsis-dap` interface driver, so any debugger exposing a CMSIS-DAP interface
works with it. For example, the Raspberry Pi Pico debugger (Debugprobe) is a
CMSIS-DAP probe and is supported through this launch configuration.

> **Warning — ST-Link/V2 and 1.8 V targets:** the nRF54L15 runs its SWD lines at
> 1.8 V. Standard ST-Link/V2 probes do not support 1.8 V targets, so the
> `ST-Link/V2` configuration will not communicate with the board on such probes.
> Do not use this option unless your probe is a variant that explicitly supports
> 1.8 V-level targets; otherwise use the CMSIS-DAP or J-Link configuration.

These require a CMake configure preset to be selected in CMake Tools (see the
hybrid-flow section above): the profiles obtain the active build directory from
the CMake Tools extension. The OpenOCD profiles use probe-specific adapter files
and a shared nRF54L15 target file under `config/`, and flash
`firmware/zephyr/zephyr.hex` from that build before Cortex-Debug attaches. The debugger uses the matching
`firmware/zephyr/zephyr.elf`; this also supports the driver-test presets under
`build/tests/`. The J-Link profile programs the selected ELF through the SEGGER
GDB server.

The selected OpenOCD installation must provide:

```text
interface/cmsis-dap.cfg
interface/stlink.cfg
target/nordic/nrf54l.cfg
```
