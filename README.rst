SenseWear Firmware
==================

Overview
********

Firmware for the SensWear wearable platform based on the nRF54L15. The app is
built on Zephyr and brings up BLE along with a set of sensors, actuators, and
power peripherals used on the board.

Supported Hardware
******************

- Board: ``SensWear/nrf54l15/cpuapp`` (application core)
- Vendor: SenseraTechnologies

Features
********

- BLE peripheral advertising with custom LED and IMU services.
- Sensor drivers: BHI360 IMU, FDC1004 pressure, and one selected daughter
  board sensor: MAX30208 temperature, MAX30101 PPG, or MTCH6102 touch.
- Actuators: LP5562 LED controller and optional DRV2605 haptics daughter
  board.
- Power and storage: BQ25180 charger, TPSM83102 regulator, and the M95P
  EEPROM exposed as a LittleFS filesystem reachable through the C standard
  library and POSIX file APIs.
- UART logging and shell enabled by default.

Project Layout
**************

- ``ORGANIZATION.md``: Detailed source-tree ownership and build integration.
- ``src/main.c``: Application entry point, BLE startup, service bridges.
- ``src/app``: BLE bridge logic for LED and IMU data.
- ``src/bluetooth/services``: Custom GATT services.
- ``src/drivers``: Application adapters and optional daughter-board drivers.
- ``boards/SenseraTechnologies/SensWear``: Board definition, bindings, and
  base-board drivers.
- ``config/cmake/zephyr_module``: Conditional extensions to Zephyr libraries.
- ``prj.conf``: Zephyr configuration for this firmware.

Building
********

From the firmware root:

.. code-block:: console

   west build -p always -b SensWear/nrf54l15/cpuapp --shield sensewear_touch -d build/touch .
   west build -p always -b SensWear/nrf54l15/cpuapp --shield sensewear_ppg -d build/ppg .
   west build -p always -b SensWear/nrf54l15/cpuapp --shield sensewear_temperature -d build/temperature .
   west build -p always -b SensWear/nrf54l15/cpuapp --shield sensewear_haptic -d build/haptic .

Each build includes only the selected daughter board DTS nodes, driver, bridge,
and BLE service. For base-board bring-up without a daughter board, omit
``--shield``.

If you need to rebuild an existing build directory:

.. code-block:: console

   west build -p always -b SensWear/nrf54l15/cpuapp --shield sensewear_touch -d build/touch .

Flashing
********

Use your preferred nRF54L15 programming probe and:

.. code-block:: console

   west flash -d build/touch

Bluetooth
*********

The device name is set via ``CONFIG_BT_DEVICE_NAME`` in ``prj.conf``
(default: ``"Sense Wear"``). On boot, the firmware starts connectable
advertising and registers the LED and IMU GATT services.

Storage and Filesystem
**********************

The M95P EEPROM is registered with Zephyr's disk-access subsystem and mounted
as a LittleFS filesystem at ``/eeprom``. The mount happens automatically at
boot (``CONFIG_SENSEWEAR_M95P_DISK_AUTOMOUNT``); application code can also mount
or unmount it explicitly with ``m95p_fs_mount()`` / ``m95p_fs_unmount()``.

``prj.conf`` enables ``CONFIG_POSIX_API`` so the filesystem can be used through
several layers:

- Zephyr's native filesystem API (``fs_open``/``fs_read``/``fs_write``).
- POSIX file operations (``open``/``read``/``write``/``close``).
- The C standard-library stdio API (``fopen``/``fread``/``fwrite``/``fclose``),
  which the libc retargets onto the POSIX calls above.

``CONFIG_ZVFS_OPEN_MAX`` bounds the number of simultaneously open file
descriptors. The M95P bring-up test (``tests/drivers/main_test_m95p.c``)
demonstrates the POSIX file API by round-tripping a text file and a binary file
against ``/eeprom``.

Logging
*******

Logging is enabled over UART with immediate mode. You can adjust verbosity in
``prj.conf`` via ``CONFIG_LOG_DEFAULT_LEVEL``.
