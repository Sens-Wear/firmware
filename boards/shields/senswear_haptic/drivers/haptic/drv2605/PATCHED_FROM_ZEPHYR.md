# SensWear DRV2605 Driver Provenance

This directory contains a local copy of Zephyr's DRV2605 haptics driver so the
SensWear firmware can patch it without modifying the installed NCS tree.

Copied source baseline:

- NCS: `v3.3.0`
- Zephyr base: `/Users/yusein/Tools/nordic/ncs/v3.3.0/zephyr`
- Zephyr version reported by the local build: `4.3.99`

Copied files:

- `zephyr/drivers/haptics/drv2605.c` -> `drv2605.c`
- `zephyr/include/zephyr/drivers/haptics/drv2605.h` -> `drv2605.h`
- `zephyr/drivers/haptics/Kconfig.drv2605` -> `Kconfig.drv2605`
- `zephyr/dts/bindings/haptics/ti,drv2605.yaml` ->
  `../../../dts/bindings/haptic/senswear,drv2605.yaml`

Current integration state:

- `drv2605.c` and `drv2605.h` are the active haptic driver. They are compiled
  into the shield driver library when `CONFIG_SENSWEAR_DRV2605_DRIVER` is set
  (`boards/shields/senswear_haptic/drivers/shield_drivers.cmake`).
- The driver binds to the namespaced compatible `senswear,drv2605` so it
  replaces, rather than collides with, Zephyr's upstream `ti,drv2605` driver,
  binding, and `HAPTICS_DRV2605` Kconfig symbol. With no `ti,drv2605` node the
  upstream `DT_HAS_TI_DRV2605_ENABLED` is false and the upstream driver is not
  built.

Build wiring:

- The binding lives at
  `boards/shields/senswear_haptic/dts/bindings/haptic/senswear,drv2605.yaml`.
  The shield directory is registered as a `DTS_ROOT` in the top-level
  `CMakeLists.txt` so that binding is discovered.
- `Kconfig.drv2605` defines `SENSWEAR_DRV2605_DRIVER` (depends on
  `DT_HAS_SENSWEAR_DRV2605_ENABLED`, selects `HAPTICS`, `I2C`, `GPIO`,
  `REGULATOR`, `SENSWEAR_SYS_I2C`, `SENSWEAR_DEVICE_DRIVER_EVENTS`). It is
  `rsource`d from the shield `Kconfig.defconfig`.
- The patched driver posts events with `DRV2605_DEVICE_DTS_ID`. That macro is
  produced at configure time by
  `boards/common/generate_device_driver_dts_ids.py`
  into the generated `device_driver_dts_ids.h`, keyed off the devicetree node
  label. The haptic node is labelled `drv2605` in
  `boards/shields/senswear_haptic/senswear_haptic.overlay`, so the macro
  resolves in the SensWear board build.

Local changes from the copied Zephyr baseline:

`drv2605.c`

- `DT_DRV_COMPAT` is `senswear_drv2605` (the namespaced compatible), not
  `ti_drv2605`, so this driver owns the node instead of Zephyr's upstream one.
- Register access is routed through `sys_i2c` (`sys_i2c_write`,
  `sys_i2c_write_read`, `sys_i2c_lock` / `sys_i2c_release`) instead of Zephyr's
  direct `i2c_dt_spec` helpers; `struct drv2605_config::i2c` is a
  `sys_i2c_dt_spec`.
- The dev-level operations (`drv2605_haptic_config`, `drv2605_start_output`,
  `drv2605_stop_output`, `drv2605_pm_action`, `drv2605_init`) acquire SYS_I2C
  ownership on entry and fully release it before returning, preserving the
  release result so a release failure is not lost.
- The `drv2605_haptic_config_*` source helpers (audio, pwm/analog, rtp, rom)
  run under the ownership scope established by their caller and do not lock the
  bus themselves; this contract is stated in a comment above them.
- The enable pin is no longer a devicetree `en-gpios` property. It is mandatory
  and claimed at init from the daughter-board GPIO arbiter as `daughter_if_GPIO0`
  (`daughter_if_gpio_claim`). The claimed pin is held as a pointer in
  `struct drv2605_data::en_gpio` and released on init failure and on
  `PM_DEVICE_ACTION_TURN_OFF`. If GPIO0 is already owned, the driver logs an
  error pointing at daughter-board GPIO0 use and asserts.
- RTP playback gained per-device atomic state: `rtp_active`,
  `rtp_stop_requested`, and `rtp_active_seconds`. `drv2605_start_output` uses
  `atomic_cas` to reject a second RTP start with `-EBUSY`; the RTP work handler
  honours `rtp_stop_requested` so an external stop is observed mid-stream; and
  `drv2605_stop_output` requests the stop, cancels the work, and uses
  `rtp_worker_will_post_stopped` to avoid emitting a duplicate `Stopped` event.
- The driver reports lifecycle through the SensWear device-driver event queue
  with `DRV2605_DEVICE_DTS_ID`: `Starting`, `Stopped`, `PlaybackActive`, and
  `Error`. `PlaybackActive` is emitted once per second from a `k_timer` expiry
  through the ISR-safe post path and carries elapsed RTP seconds; `Error`
  carries the positive errno; the lifecycle events carry the active
  `drv2605_mode`.
- The driver distinguishes DRV2605 from DRV2605L at runtime and calculates the
  rated-voltage and overdrive-clamp registers with the detected variant's
  formulas. LRA instances also provide their nominal resonant frequency so the
  driver can seed `DRIVE_TIME` correctly.
- LRA initialization uses closed-loop auto-resonance and performs auto
  calibration after applying the actuator voltage and feedback parameters.
- RTP uses unsigned, unidirectional closed-loop input (`0` is off, `255` is full
  scale), while ROM playback restores the bidirectional format expected by the
  on-chip LRA library.
- The RTP worker locks and releases SYS_I2C for each register write instead of
  holding the shared bus while it sleeps between frames.
- The driver owns its supply rail. The `vin-supply` regulator is resolved from
  devicetree into `struct drv2605_config::regulator` and cached on the driver
  object (`struct drv2605_data::regulator`). The board rail is fixed at 3.6 V
  (`DRV2605_SUPPLY_VOLTAGE_UV`). `drv2605_supply_init` runs MAX30101-style
  power-up checks at init (regulator readiness, and that an already-enabled
  shared rail already sits at the fixed voltage). `drv2605_init` then sets and
  enables the rail (`drv2605_supply_on`, idempotent via `supply_enabled`) before
  probe and calibration; `drv2605_start_output` idempotently ensures it remains
  on. `PM_DEVICE_ACTION_TURN_OFF` disables it (`drv2605_supply_off`). The
  regulator transport locks the shared bus itself, so these run outside the
  device's SYS_I2C ownership scope. This selects `REGULATOR` in
  `Kconfig.drv2605`.

`drv2605.h`

- Added `enum drv2605_event_type`
  (`Starting`/`Stopped`/`PlaybackActive`/`Error`/`Count`) and the
  `drv2605_event_name()` helper, documenting the per-event `v_param` meaning.
- Added `drv2605_rtp_is_active()` so a caller that owns the RTP buffers passed to
  `drv2605_haptic_config()` can serialise patterns and avoid reusing those
  buffers while the async worker is still streaming from them.
`senswear,drv2605.yaml` (renamed from the copied `ti,drv2605.yaml`)

- The `compatible` is `senswear,drv2605` instead of `ti,drv2605`.
- The actuator properties describe LRA rated RMS voltage, peak clamp voltage,
  and nominal resonant frequency (`vib-resonant-hz`) rather than relying on
  generic motor defaults.
- Removed the `en-gpios` property: the enable pin is owned by the driver via the
  daughter_if GPIO arbiter and is not described in devicetree.
- The `vin-supply` regulator phandle is consumed by the driver to own the rail
  (fixed 3.6 V); see the `drv2605.c` supply notes above.

`Kconfig.drv2605`

- Defines `SENSWEAR_DRV2605_DRIVER` (depends on `DT_HAS_SENSWEAR_DRV2605_ENABLED`)
  instead of reusing Zephyr's upstream `HAPTICS_DRV2605` symbol, and selects the
  SensWear infrastructure the patched driver needs.

When patching the copied driver, keep Zephyr's original copyright and SPDX
headers, and document behavior changes here.
