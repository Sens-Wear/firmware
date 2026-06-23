# Driver bring-up tests

Standalone, on-target bring-up tests — one `main_test_<driver>.c` per board
driver. Each file defines its own `main()` and exercises a single driver:
initialise, configure, then poll/print state every 2 seconds over the console.

| File | Driver | What it does |
|------|--------|--------------|
| `main_test_bq25180.c`  | BQ25180 charger    | init → default Li-Po/USB config → print charger state every 2 s |
| `main_test_bq27427.c`  | BQ27427 fuel gauge | init → config (450 mAh) → print battery state every 2 s |
| `main_test_bhi360.c`   | BHI360 IMU         | register callbacks → start streaming → print latest quat/lacc every 2 s |
| `main_test_lp5562.c`   | LP5562 LED ctrl    | init → configure → toggle LED 0 red on/off every 2 s |
| `main_test_m95p.c`     | M95P EEPROM        | init → JEDEC/geometry → mount LittleFS → POSIX text + binary file round-trips → walk + list the FS tree → print status every 2 s |
| `main_test_tpsm83102.c`| TPSM83102 regulator| placeholder (driver has no public API yet) |

## Building a test

Tests are selected with **Kconfig symbols** (all off by default, so a normal
build is unaffected). They are Kconfig — not CMake options — on purpose: under
sysbuild only Kconfig symbols are forwarded from the top-level build (and thus
from a CMake preset) down into the application image, so a plain CMake option
would never receive the override.

- `CONFIG_SENSEWEAR_TEST_<DRIVER>_DRIVER` — build that driver's test, e.g.
  `CONFIG_SENSEWEAR_TEST_M95P_DRIVER`.

Enable exactly **one** test (each defines its own `main()`; the build fails
fast if two are on). When a test is enabled, the application main
(`src/main.c`) is omitted and the test's `main()` takes its place. The easiest
way is the matching CMake preset (e.g. `test_m95p`); equivalently:

```sh
west build -b SensWear/nrf54l15/cpuapp -- -DCONFIG_SENSEWEAR_TEST_M95P_DRIVER=y
```

Wiring: the top `CMakeLists.txt` reads the `CONFIG_SENSEWEAR_TEST_*` symbols
(available as CMake variables after `find_package(Zephyr)`), derives the
internal `TEST_DRIVERS` flag, and includes `tests/drivers/tests.cmake` when it
is on; that file creates a `test_<driver>` INTERFACE library per enabled test
and links it into `app`.

The relevant driver must be enabled via its `CONFIG_SENSEWEAR_<DRIVER>_DRIVER`
Kconfig symbol (all enabled by default). That symbol is the single source of
truth: the per-type cmakes under `boards/.../drivers` gate their sources on it
directly.

## Filesystem / POSIX dependency (M95P)

`main_test_m95p.c` exercises the EEPROM through the LittleFS filesystem mounted
at `/eeprom` using POSIX file operations (`open`/`read`/`write`/`close`). This
relies on the storage and POSIX options already set in `prj.conf` — chiefly
`CONFIG_FILE_SYSTEM_LITTLEFS`, `CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT`, and
`CONFIG_POSIX_API` (sized by `CONFIG_ZVFS_OPEN_MAX`) — which route the POSIX
calls onto the mounted filesystem. See the "Storage and Filesystem" section of
the top-level `README.rst`.

The `test_m95p` preset also overrides `CONFIG_SHELL=n` and `CONFIG_I2C_SHELL=n`
(the rest of the firmware keeps the shell). The shell's serial backend and the
UART console both drive the same UART, so with the shell enabled every
`printk`/log line is emitted twice — once by the console and once re-rendered by
the shell prompt. Disabling the shell leaves a single console owner, so this
test's output appears once. Logging stays on (`CONFIG_LOG_BACKEND_UART`); with
the shell gone, its duplicate `CONFIG_SHELL_LOG_BACKEND` is dropped too.
