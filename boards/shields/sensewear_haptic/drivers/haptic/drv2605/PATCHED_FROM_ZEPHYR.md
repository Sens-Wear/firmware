# SenseWear DRV2605 Driver Provenance

This directory contains a local copy of Zephyr's DRV2605 haptics driver so the
SenseWear firmware can patch it without modifying the installed NCS tree.

Copied source baseline:

- NCS: `v3.3.0`
- Zephyr base: `/Users/yusein/Tools/nordic/ncs/v3.3.0/zephyr`
- Zephyr version reported by the local build: `4.3.99`

Copied files:

- `zephyr/drivers/haptics/drv2605.c` -> `drv2605.c`
- `zephyr/include/zephyr/drivers/haptics/drv2605.h` -> `drv2605.h`
- `zephyr/drivers/haptics/Kconfig.drv2605` -> `Kconfig.drv2605`
- `zephyr/dts/bindings/haptics/ti,drv2605.yaml` -> `ti,drv2605.yaml`

Current integration state:

- `haptic_drv2605.c` and `haptic_drv2605.h` are the existing SenseWear wrapper
  used by the application today.
- `drv2605.c` and `drv2605.h` are the copied Zephyr driver files for the next
  patch stage. They are intentionally not compiled by `shield_drivers.cmake`
  yet.

Local changes from the copied Zephyr baseline:

- Register access is routed through `sys_i2c` instead of Zephyr's direct
  `i2c_dt_spec` helpers.
- High-level driver operations acquire and fully release SYS_I2C ownership.
- Register-sequence helpers that run under an existing SYS_I2C ownership scope
  use the `_owned` suffix to make the locking contract explicit.
- RTP playback has per-device atomic active and stop-request flags so external
  stop requests can be observed by the worker.
- The driver posts playback events through the SenseWear device-driver event
  queue using `DRV2605_DEVICE_DTS_ID`: `Starting`, `Stopped`,
  `PlaybackActive`, and `Error`. `PlaybackActive` is emitted once per second
  from a `k_timer` expiry through the ISR-safe post path.
- The enable pin is mandatory and claimed as `daughter_if_gpio_0` instead of
  using a devicetree `en-gpios` property.
- If `daughter_if_gpio_0` is already owned, the driver logs an error asking the
  user to check enabled daughter boards and GPIO0 use, then asserts.

When patching the copied driver, keep Zephyr's original copyright and SPDX
headers, and document behavior changes here.
