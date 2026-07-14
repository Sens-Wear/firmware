# Device manager bring-up test

Standalone bring-up test for the SenseWear device manager.

- Source: `main_test_device_manager.c`
- Build presets: `test_device_manager`, `test_device_manager_haptic`,
  `test_device_manager_ppg`, `test_device_manager_temperature`,
  `test_device_manager_touch`
- Kconfig symbol: `CONFIG_SENSEWEAR_TEST_DEVICE_MANAGER`

Enable exactly one bring-up test at a time. The test replaces `src/main.c`
when `TEST_DRIVERS` is on and exercises the device manager zbus pipeline.
Optional devices are discovered through `device_manager_init()` and only ready
streams are subscribed. Disabled drivers, absent gauges, and unplugged shield
devices are skipped rather than configured by the test. The TPSM83102 regulator
status stream is subscribed when the devicetree regulator device is ready.
The device-manager presets disable `CONFIG_SENSEWEAR_BQ27427_DRIVER` by default
because the fuel gauge requires a battery to be present.
