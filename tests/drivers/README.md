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
| `main_test_m95p.c`     | M95P EEPROM        | init → JEDEC/geometry → page write/read round-trip → print status every 2 s |
| `main_test_tpsm83102.c`| TPSM83102 regulator| placeholder (driver has no public API yet) |

## Building a test

Tests are selected with CMake options (all **OFF** by default, so a normal
build is unaffected):

- `TEST_<DRIVER>_DRIVER` — build that driver's test, e.g. `TEST_M95P_DRIVER`.
- `TEST_DRIVERS` — umbrella switch; enabling any `TEST_<DRIVER>_DRIVER` turns
  it on automatically. You normally never set this directly.

Enable exactly **one** test (each defines its own `main()`; the build fails
fast if two are on). When a test is enabled, the application main
(`src/main.c`) is omitted and the test's `main()` takes its place:

```sh
west build -b SensWear/nrf54l15/cpuapp -- -DTEST_M95P_DRIVER=ON
# or, reconfiguring an existing build dir:
cmake -DTEST_M95P_DRIVER=ON build/firmware && cmake --build build --target firmware
```

Wiring: the top `CMakeLists.txt` defines the options and includes
`tests/drivers/tests.cmake` when `TEST_DRIVERS` is on; that file creates a
`test_<driver>` INTERFACE library per enabled test and links it into `app`.

The relevant driver must be enabled on **both** sides (they are by default):
`-DHAVE_<DRIVER>_DRIVER=ON` and `CONFIG_SENSEWEAR_<DRIVER>_DRIVER=y` — the build
rejects a mismatch (see the guard in `CMakeLists.txt`).
