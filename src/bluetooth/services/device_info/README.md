# Firmware identity and capabilities

The application registers these read-only services before sensor bring-up. Values
describe the running firmware image and stay unchanged until it is replaced.
They do **not** detect a physically attached daughter board or guarantee that a
compiled sensor has initialized successfully.

## Firmware version

| Service | Characteristic | Properties | Value |
| --- | --- | --- | --- |
| Device Information `180a` | Firmware Revision String `2a26` | Read | UTF-8 application version, without a NUL terminator |

The repository-root `VERSION` file is the sole source for release numbering.
Zephyr generates `APP_VERSION_EXTENDED_STRING`, which is returned directly;
with the initial version this is `0.1.0+0`. The format is
`major.minor.patch[-extra]+tweak`. Update `VERSION` for subsequent firmware
releases, then rebuild. This is the application version, independent of the
Nordic SDK and Zephyr kernel versions. The application's DIS implementation
must not be combined with Zephyr's optional `CONFIG_BT_DIS` service.

## Compiled capabilities, schema 1

Service: `9b8e0001-6b7d-4e9f-9b0d-2d7f6e5a4c30`.

Characteristic: `9b8e0002-6b7d-4e9f-9b0d-2d7f6e5a4c30`, **Read only** (no write,
notification, or CCC descriptor). The value is exactly 9 bytes, with no padding:

| Offset | Size | Type | Meaning |
| --- | --- | --- | --- |
| 0 | 1 | unsigned byte | Schema version, `1` |
| 1 | 4 | unsigned 32-bit little-endian | Compiled shield mask |
| 5 | 4 | unsigned 32-bit little-endian | Supported feature mask |

These are flags, not enums containing a single selected value. The PPG app preset
includes both the PPG and temperature shields because its daughter board has
both sensors. A client must not infer shield selection solely from discovered
service UUIDs; core GATT services can remain present with disabled drivers.

| Shield | Bit | Mask | Build gate |
| --- | --- | --- | --- |
| Haptic | 0 | `0x01` | `CONFIG_SHIELD_SENSWEAR_HAPTIC` |
| PPG | 1 | `0x02` | `CONFIG_SHIELD_SENSWEAR_PPG` |
| Temperature | 2 | `0x04` | `CONFIG_SHIELD_SENSWEAR_TEMPERATURE` |
| Touch | 3 | `0x08` | `CONFIG_SHIELD_SENSWEAR_TOUCH` |

All feature bits require `CONFIG_SENSWEAR_DEVICE_MANAGER`. The remaining gates
follow `services.cmake` and the device-manager producers/actuators:

| Feature | Bit | Mask | Additional build gates |
| --- | --- | --- | --- |
| IMU | 0 | `0x01` | BHI360 driver |
| LED | 1 | `0x02` | LP5562 driver |
| Haptic | 2 | `0x04` | Haptic daughter selection, haptic shield, DRV2605 driver |
| PPG | 3 | `0x08` | PPG daughter selection and PPG shield |
| Temperature | 4 | `0x10` | Temperature shield and RTC driver (periodic sampling) |
| Touch | 5 | `0x20` | Touch daughter selection and touch shield |
| Battery | 6 | `0x40` | BQ27427 fuel-gauge driver |
| Time | 7 | `0x80` | RTC driver |

Unassigned bits are reserved and sent as zero; clients ignore unknown bits but
retain the raw masks. Clients reject lengths other than 9 and unsupported
schema versions rather than decoding them as schema 1. Cache metadata only for
the current connection and read again after reconnecting/reflashing.

This is an additive interface: all existing UUIDs and payloads are unchanged.
Older firmware lacks one or both services; clients display version/shield
information as unavailable instead of inventing values. Only explicit feature
bits authorize navigation to a capability in clients that require support
metadata. A future incompatible capabilities layout must use a new schema
version; a changed characteristic contract must use a new UUID.

## Validation

Build all application presets listed in `CMakePresets.json` (base, haptic, PPG,
temperature, touch). Existing standalone driver/device-manager tests do not
link `app_src` or these GATT services. Compilation validates the generated
application version, GATT declarations, and explicit payload boundaries.

After explicitly authorized programming of the matching physical board, use
the SensWear SDK or a GATT browser to perform these hardware checks:

1. Discover Device Information and the capabilities service. Read `2a26` and
   verify it matches the built image's generated application version, including
   extra/tweak fields, and contains no terminating NUL.
2. Read the capabilities characteristic. For default driver configurations,
   compare the whole byte value against this table:

   | Application preset suffix | Shields | Features | Expected bytes (hex) |
   | --- | --- | --- | --- |
   | `base` | `0x00` | `0xc3` | `01 00 00 00 00 c3 00 00 00` |
   | `haptic` | `0x01` | `0xc7` | `01 01 00 00 00 c7 00 00 00` |
   | `ppg` | `0x06` | `0xdb` | `01 06 00 00 00 db 00 00 00` |
   | `temperature` | `0x04` | `0xd3` | `01 04 00 00 00 d3 00 00 00` |
   | `touch` | `0x08` | `0xe3` | `01 08 00 00 00 e3 00 00 00` |

3. Verify writes are rejected. Read with an offset or a short buffer using an
   ATT test client; Zephyr's `bt_gatt_attr_read` must return the corresponding
   suffix/fragment and reject offsets past the end.
4. Rebuild with one core driver disabled (for example BHI360). Its feature bit
   must clear even when its core service UUID remains discoverable. Rebuild PPG
   shields with a different daughter service selection; the PPG feature bit must
   clear when the PPG service is omitted even though its shield bit remains set.
5. Reconnect the mobile app to each image. Check the firmware version and
   compiled shield listing in Settings, and disabled/enabled Home navigation.
   Compare SDK decoding with the raw bytes above. Disconnect or switch devices
   and verify the previous device's metadata is not reused.

Compilation alone does not validate BLE reads, physical shield presence, or
sensor operation; those require this hardware procedure.
