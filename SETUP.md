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
The checked-in OpenOCD, GDB, objdump, and nm wrappers read `.env` directly.
Each wrapper is available as `.sh` for macOS/Linux, `.cmd` for Command Prompt,
and `.ps1` for PowerShell (except where a platform has no corresponding tool).
For CMake, clangd, and other extension settings, use one of these workflows:

```sh
./config/scripts/code-with-env.sh
```

On Windows:

```bat
config\scripts\code-with-env.cmd
```

Or from PowerShell:

```powershell
.\config\scripts\code-with-env.ps1
```

The other PowerShell wrappers use the same calling convention and forward all
arguments to the configured tool, for example:

```powershell
.\config\scripts\openocd-with-env.ps1 --version
.\config\scripts\gdb-with-env.ps1 --version
```

PowerShell execution policy must permit local scripts. On a machine using the
default `Restricted` policy, enable them for the current PowerShell process
only, then run the wrapper:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\config\scripts\openocd-with-env.ps1 --version
```

This process-scoped setting disappears when that PowerShell window closes. If
an organization manages execution policy, follow its approved configuration.

> **Cortex-Debug:** `serverpath` and `gdbPath` require executable files, so do
> not point those properties at either a `.cmd` or `.ps1` wrapper. Launch VS
> Code through `code-with-env.*`, then use `${env:OPENOCD}` and
> `${env:ZEPHYR_GDB}` in `launch.json`. When using GDB directly, retain
> `debuggerArgs: ["-iex", "set remotetimeout 60"]` so image programming does
> not exceed GDB's default remote timeout.

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
cmake --preset senswear_nrf54l15_cpuapp_base
cmake --build --preset senswear_nrf54l15_cpuapp_base
```

The active board is `SensWear/nrf54l15/cpuapp`; the shield-less ("base") build
output lands in `build/app/base/`, alongside the per-shield builds in
`build/app/<shield>/`. Under sysbuild the image is nested one level deeper, in a
directory named after the application directory — `build/app/base/firmware/` for
a checkout in `firmware/` — because sysbuild derives the image (domain) name
from that directory's basename, not from the CMake `project()` name.

For one-off local CMake overrides, create `CMakeUserPresets.json`; it is
ignored by git.

## Debugging from VS Code

The Run and Debug selector provides three launch configurations:

- `CMSIS-DAP: flash and debug SensWear nRF54L15 (cpuapp), base (no shield)`
- `ST-Link/V2: flash and debug SensWear nRF54L15 (cpuapp), base (no shield)`
- `J-Link/OpenOCD: flash and debug SensWear nRF54L15 (cpuapp), base (no shield)`

(plus one CMSIS-DAP configuration per shield application and per bring-up test)

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
`<image>/zephyr/zephyr.hex` from that build before Cortex-Debug attaches. The debugger uses the matching
`<image>/zephyr/zephyr.elf`; this also supports the driver-test presets under
`build/tests/`. `<image>` is the sysbuild image directory, named after the
application directory — the launch configurations spell it
`${workspaceFolderBasename}` so a checkout under any directory name resolves
correctly without editing this file. The J-Link profile programs the selected ELF through the SEGGER
GDB server.

The selected OpenOCD installation must provide:

```text
interface/cmsis-dap.cfg
interface/stlink.cfg
target/nordic/nrf54l.cfg
```

## Debugging from the command line with west

`board.cmake` registers three west runners: `nrfutil` (the default for
`west flash`), `jlink` (the default for `west debug`), and `openocd`. The
OpenOCD runner reuses the same adapter and target configs as the launch
configurations, so it programs RRAM through the `nrf54l-load` proc rather than
OpenOCD's flash-bank path, which does not exist for this part:

```sh
west flash -d build/app/base --runner openocd
west debug -d build/app/base --runner openocd
```

`west attach`, `west debugserver`, and `west rtt` accept the same runner. The
OpenOCD binary comes from `config/scripts/openocd-with-env.sh`, which reads
`OPENOCD`/`OPENOCD_SCRIPTS` from `.env` at run time — the runner is not bound to
whichever `openocd` happens to be on `PATH`.

Unlike the launch configurations, which offer one entry per probe, west binds
the adapter config at configure time. It defaults to CMSIS-DAP; select another
probe when configuring the build directory:

```sh
cmake --preset senswear_nrf54l15_cpuapp_base -DSENSWEAR_OPENOCD_INTERFACE=stlink-v2
```

Under sysbuild, CMake cache variables are not forwarded to the image, so export
`SENSWEAR_OPENOCD_INTERFACE` in the environment instead. The value names a
`config/openocd-<value>.cfg` adapter file, and configuration fails if that file
does not exist.
