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
- Sensor drivers: BHI360 IMU, FDC1004 pressure, MAX30208 temperature,
  MAX30101 PPG, MTCH6102 touch.
- Actuators: DRV2605 haptics, LP5562 LED controller.
- Power and storage: BQ25180 charger, TPSM83102 regulator, M95P EEPROM.
- UART logging and shell enabled by default.

Project Layout
**************

- ``src/main.c``: Application entry point, BLE startup, service bridges.
- ``src/app``: BLE bridge logic for LED and IMU data.
- ``src/bluetooth/services``: Custom GATT services.
- ``src/drivers``: Board-specific sensor, actuator, power, and memory drivers.
- ``boards/SenseraTechnologies/SensWear``: Board definition and device tree.
- ``prj.conf``: Zephyr configuration for this firmware.

Building
********

From the firmware root:

.. code-block:: console

   west build -b SensWear/nrf54l15/cpuapp -d build/senswear .

If you need a pristine build:

.. code-block:: console

   west build -p always -b SensWear/nrf54l15/cpuapp -d build/senswear .

Flashing
********

Use your preferred nRF54L15 programming probe and:

.. code-block:: console

   west flash -d build/senswear

Bluetooth
*********

The device name is set via ``CONFIG_BT_DEVICE_NAME`` in ``prj.conf``
(default: ``"Sense Wear"``). On boot, the firmware starts connectable
advertising and registers the LED and IMU GATT services.

Logging
*******

Logging is enabled over UART with immediate mode. You can adjust verbosity in
``prj.conf`` via ``CONFIG_LOG_DEFAULT_LEVEL``.
