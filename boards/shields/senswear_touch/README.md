# SensWear linear touch shield

The touch board has 15 electrodes in one physical row. The MTCH6102 supplies
per-channel measurements; the firmware computes a one-dimensional position and
gestures from those measurements. Its built-in X/Y decoder cannot represent this
board geometry reliably.

## Hardware mapping

With the connector on the left and the electrode surface facing you, X increases
away from the connector. The pad pitch is 3 mm. The second and third electrodes
are connected to RX2 and RX1, respectively; channel number alone is not physical
order.

| Physical pad, connector to tip | MTCH6102 channel | X at pad center |
| --- | --- | --- |
| 1 | RX0 | 0 |
| 2 | RX2 | 64 |
| 3 | RX1 | 128 |
| 4 | RX3 | 192 |
| 5 | RX4 | 256 |
| 6 | RX5 | 320 |
| 7 | RX6 | 384 |
| 8 | RX7 | 448 |
| 9 | RX8 | 512 |
| 10 | RX9 | 576 |
| 11 | RX10 | 640 |
| 12 | RX11 | 704 |
| 13 | RX12 | 768 |
| 14 | RX13 | 832 |
| 15 | RX14 | 896 |

This order was checked against both files in the adjacent hardware repository:

- `../Hardware/hardware-v1/SensWear-V1R1-Touch/Output/SensWear_Touch_Documentation.PDF`,
  schematic on page 2: Touch pads 1, 2, 3 connect to CH_0, CH_2, CH_1; pads
  4 through 15 connect to CH_3 through CH_14. Each CH_n connects to RXn.
- `../Hardware/hardware-v1/SensWear-V1R1-Touch/PCB/SensWear-V1R1-Touch.PcbDoc`:
  the `Pads6` records for component `Touch`, joined to `Nets6`, place pads
  1 through 15 in increasing PCB X at constant Y. Pad 1 is at
  X=1289.3701 mil and pad 15 at X=2942.9134 mil; successive centers are
  118.1102 mil (3 mm) apart. The connector is at X=1059.0551 mil.

The paths above are relative to the firmware repository root. Hardware files were
inspected without modification.

## Supply and logic levels

The touch driver requests **1.8 V** on the daughter regulator. In the touch
schematic above, U70 VDD (pin 17) uses `V_REG_IO`, but R70 pulls RESET (pin 26)
to `1V8`. The main-board schematic pulls SYS_I2C SDA/SCL to `1V8` through
R15/R16. SYNC connects directly to the MCU's 1.8 V GPIO domain.

The previous 2.8 V setting was incompatible with these input levels:
[MTCH6102 tables 18-1 and 18-2](https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/40001750A.pdf)
specify a 1.8-3.6 V supply, RESET high at least `0.8 * VDD`, and I2C high at
least `0.7 * VDD`. At 2.8 V those thresholds are 2.24 V and 1.96 V, both above
the 1.8 V pullups. This can prevent the controller from acknowledging its
address before any touch configuration is written.

PCB net checks confirm that the main-board `V_REG_IO` rail feeds the regulator
output capacitors and daughter connector only; on the touch shield it feeds
U70 and C70. The main-board LED controller, IMU, memory and MCU use other rails.
This setting applies to the touch shield, not other daughter boards.

The regulator setting is not a voltage measurement. Since 1.8 V is the
controller's specified minimum, measure VDD at U70/C70, including ripple and
load transients, and RESET/SDA/SCL/SYNC levels during hardware qualification.
If supply tolerance takes VDD below its minimum, hardware power/level matching
needs correction; raising the daughter voltage alone also raises SYNC into
the MCU's lower-voltage domain.

## Acquisition and decoding

The controller remains configured for 12 X channels plus 3 Y channels, in full
mode. This enables measurement of RX0 through RX14. Changing the counts to 15+0
is not a supported way to enable a slider: the
[MTCH6102 datasheet](https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/40001750A.pdf),
section 6.2, specifies at least three channels per axis and at most 15 total;
section 15 describes correction of invalid configuration values.

The driver reads the 15 `SENSORVALUES` bytes at 0x80 through 0x8E after the SYNC
falling edge. This avoids depending on the controller's two-dimensional touch
decision to discover activity on an electrode. Section 5.3 identifies SYNC as a
frame signal for host decoding. A 100 ms fallback services acquisition when a
frame notification is missed. ISR notifications are coalesced and acquisition
work is bounded. Decoded events own immutable samples from an eight-entry
pool; consumers release each sample after dispatch. Temporary bus failures
are retried on subsequent frames, and mixed frames are discarded.

The software decoder applies touch thresholds, hysteresis and debounce, then
uses the physical channel order to interpolate X. X spans 0 through 896 with
64 units per electrode pitch. The legacy Y field is always zero. The nominal
distance from the first electrode center is `x * 3 / 64` mm; this is a conversion
using PCB pitch, not a claim of calibrated physical accuracy. Finger size,
overlay and electrode response affect interpolation and edge behavior.

Click, double-click, hold, left/right swipe and swipe-and-hold gestures are
decoded in software using the existing gesture codes. Left means toward the
connector; right means toward the tip. Vertical gestures are not generated.
Host timing defaults are 500 ms for a hold, 250 ms for the double-tap window,
and 320 ms stationary after a swipe for swipe-and-hold. A single tap is delayed
until its double-tap window expires. These timings use monotonic time rather
than the hardware gesture timing/angle registers.

The controller's Raw ADC mode is not used: section 7 describes selection of one
channel through `MODECON`, with no interrupt output in that mode.

A failed startup probe does not permanently prevent configuration: the next
configuration attempt retries initialization and preserves the original I2C
error if the device still does not respond. This allows the application's
existing bring-up retry loop to recover after a transient probe failure.

Stopping acquisition places the MTCH6102 in standby while keeping its supply
enabled. Restarting resumes measurement without deliberately removing power
from a device attached to the shared I2C bus.

## Bluetooth behavior and compatibility

The existing UUIDs and payload layouts are retained: 13-byte packed position,
10-byte packed gesture, and 16-byte raw state including its existing padding. The position fields
now describe the complete linear strip: X is a host-decoded coordinate and Y is
reserved as zero. `gesture_state` uses the existing numeric gesture namespace,
but its value is now produced by the software decoder.

The raw `touch_state` byte remains a controller diagnostic. Its hardware TCH bit
can disagree with the host-decoded `touched` field on this one-dimensional board;
applications should use `touched` for contact state. The characteristic named
"raw touch data" carries decoded position and status, not 15 electrode ADC
measurements.

BLE notifications use a bounded queue serviced separately from acquisition.
When a slow client or transport congestion exhausts capacity, samples, including
gesture notifications, may be dropped. These streams are best-effort and do not
provide lossless recording. Touch transport overload does not wait on the device-event dispatch thread.
Other BLE services currently retain their existing notification paths; combined
stream testing is needed if subscribing to additional sensor services.

## Hardware validation procedure

Compilation and synthetic decoder checks do not establish touch performance on
a physical board. Run these checks on the intended assembled board and overlay;
record firmware version, configuration, observed failures and duration.

If initialization fails, the `test_mtch6102` image prints the original error,
reads the daughter regulator's CONTROL1/VOUT/CONTROL2 registers (expected
`0x01/0x20/0x05` for this board configuration), and scans non-reserved I2C
addresses using one-byte reads. This diagnostic only runs in the isolated
bring-up test and can consume pending peripheral status. It does not change
device addresses or supply settings. A correct regulator readback does not
prove voltage reaches the touch chip: measure C70 VDD and the RESET side of
R70 relative to GND if address 0x25 still fails to acknowledge. Use the board's
configured UART speed, 921600 baud, for this test.

1. Start untouched, then press and release each of the 15 electrodes. Confirm
   `touched` changes on every electrode, including the first three and last three.
2. Slide slowly in both directions across the entire strip. Confirm monotonic X,
   Y=0, correct direction, and no discontinuity across pads 2/3 or pads 12/13
   (the controller's X/Y grouping boundary). Check both ends and neighboring-pad
   interpolation; verify nominal pad-center coordinates against the table.
3. Exercise click, double-click, hold, and left/right swipes and swipe-and-hold
   near both ends and across the grouping boundary. Check that vertical gesture
   codes are absent.
4. Stream position, gesture and raw touch notifications together for at least
   30 minutes while repeatedly exercising the strip. Confirm acquisition and
   subsequent gestures continue; inspect logs for queue drops and I2C errors.
5. Repeat with a slow BLE client, disconnect/reconnect cycles, notification
   subscription changes, and repeated sampling stop/start. Dropped notifications
   under overload are allowed; persistent loss of acquisition is not.
6. Under a controlled hardware test setup, inject a temporary I2C failure or bus
   contention. Verify errors are bounded, sampling retries after the fault clears,
   and later touch events still arrive without rebooting. Do not power-cycle the
   shared rail as an unqualified recovery action.

These are required hardware checks, not a record of completed testing.
