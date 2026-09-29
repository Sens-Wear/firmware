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

Release hysteresis applies within one electrode pitch of the current contact.
A more distant pad must reach its full activation threshold before taking over.
This prevents a weak residual signal at the opposite end from prolonging a
contact or moving its coordinate across the strip after release.

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

## Build-time enclosure selection

Select one option in `prj.conf` or an application configuration fragment:

```ini
# Ring enclosure (default): X threshold 16, Y threshold 6
CONFIG_SENSWEAR_TOUCH_RING_ENCLOSURE=y
```

For directly exposed electrodes, replace that line with:

```ini
# Without enclosure: original X threshold 55, Y threshold 40
CONFIG_SENSWEAR_TOUCH_NO_ENCLOSURE=y
```

These are mutually exclusive Kconfig choices under **SensWear firmware > Touch
electrode enclosure** in menuconfig, available only with the touch shield.
The choice applies to both the controller registers and the host 1D decoder.
It changes only the two detection thresholds; filtering, debounce, baseline
timing, hysteresis and gesture behavior keep their current settings. The
without-enclosure option does not restore the entire historical configuration.
X/Y here name the controller's two RX banks; output remains 1D with Y=0.

You can also select the option when configuring the touch application:

```sh
cmake --fresh --preset senswear_nrf54l15_cpuapp_touch -DCONFIG_SENSWEAR_TOUCH_NO_ENCLOSURE=y
cmake --build --preset senswear_nrf54l15_cpuapp_touch
```

To select the ring again, use `-DCONFIG_SENSWEAR_TOUCH_RING_ENCLOSURE=y` with
the same fresh configure command. `--fresh` removes a previous CMake-cached
selection; keep only the desired choice in any configuration fragments too.
The same options work with `test_mtch6102` and `test_device_manager_touch`.
Rebuild and flash to apply a changed choice; there is no new BLE setting.

## Enclosed-overlay calibration

**Calibration remains incomplete.** Measurements with the resin enclosure have
not established a profile that reliably detects all pads, holds, releases and
gestures. The restored source profile below is provisional and **not
production-qualified**. Calibrate with the final enclosure fitted;
record its material/thickness, board and firmware revision, power source,
configuration, and the location of each touch. Keep the same support and hand
position between comparisons.

Use the isolated `test_mtch6102` image, built with:

```sh
cmake --preset test_mtch6102
cmake --build --preset test_mtch6102
```

The debugger mailbox below exposes measurements and three bounded commands only
in this test image. It is absent from the production application and BLE
protocol. Use the matching ELF to resolve symbols and a debugger capable of
reading/writing RAM while the CPU runs; halting acquisition changes the
experiment. Programming the test image requires an explicitly selected board
and authorization to flash it.

1. Leave the enclosed sensing surface untouched for five seconds. Issue the
   baseline command with no finger on or near the strip, wait for success and a
   fresh frame, then record at least 60 seconds of untouched data. Record startup
   transients separately; a quiet final interval does not erase an early false touch.
2. In physical pad order, press each pad for one second and release for two
   seconds, three times each. Include gentle contact. Record actual contact/lift
   times as well as capture phase names; changing a label does not prove a finger
   moved. Retain RX channel order; use the mapping above for analysis.
3. Hold the weakest responding pads for at least 90 seconds and try a slow approach.
   Compare the initial and sustained signal, raw value, and baseline. If the
   signal fades while the raw value remains shifted and the baseline follows
   it, investigate baseline absorption before declaring that pad insensitive.
4. Select each bank's threshold above its worst untouched noise plus hysteresis
   and a measured margin, but below its weakest sustained touch. If these ranges
   overlap, there is no defensible threshold from that capture. Change one
   setting at a time, reconfigure untouched, and repeat the quiet/touch capture.
5. Check 60 seconds untouched, long holds, release, both sweep directions, both
   ends, and the RX11/RX12 boundary. Then run the hardware validation below on
   the normal application, including simultaneous BLE streams.

Threshold changes through the mailbox update both controller and host decoder.
Start with the existing scan count and filters, recording their controller
readback after CFG. Check saturation, noise and frame cadence for each change.
Compensation is reported but not changed by this interface. `SENSORVALUES` are
the host decoder's signal inputs; keep `RAWVALUES` and `BASEVALUES` separately
and label their signed difference as a diagnostic, not a documented exact
conversion to `SENSORVALUES`.

[Microchip DS40001750A, sections 11–15](https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/40001750A.pdf)
describes scan summation, filtering, compensation, and baseline adaptation.
It describes threshold crossings postponing automatic baseline updates;
subthreshold contact can be absorbed. Do not assume the host's 1D `touched`
state controls the controller's baselining. Positive/negative baseline filters
limit each update's step:
larger limits permit faster adaptation. BS forces a new baseline regardless of
those limits; CFG also baselines the sensor. Never force either while touched.
Compensation uses a coefficient divided by 64, with zero bypassing compensation.
Do not issue `CMD.NV`: the
[official MTCH6102 errata](https://ww1.microchip.com/downloads/en/DeviceDoc/MTCH6102-Errata-80000801A.pdf)
documents a one-write limitation for non-default NVM parameters in firmware 2.0.
Keep trials volatile and put accepted settings in the host firmware configuration.

### Findings from the enclosed-board trials

With the default ring-enclosure choice, the normal application uses the measured
C5 trial profile:

| Setting | Value |
| --- | --- |
| X / Y bank touch threshold | 16 / 6 |
| Hysteresis | 3 |
| Release / contact debounce | 2 / 2 frames |
| Scan count | 6 |
| Acquisition filter | IIR, strength 3 |
| ACTIVEPERIOD / IDLEPERIOD | Both 645 (`0x0285`); about 19.4 ms per frame observed |
| BASEINTERVAL | 200; about 3.9 s between observed updates |
| Positive / negative baseline step limits | 1 / 1 |

This profile improved quiet operation and sustained observed holds beyond 40 seconds.
Phase labels were changed manually and do not independently timestamp the actual
finger lift, so they do not establish an exact maximum hold duration. The sweep
trial remained fragmented and did not establish reliable full-strip gestures;
release ghosts also occurred across the trials. It does not constitute a
complete 15-pad pass. The power cost of the faster idle cadence has not been
measured. Start with hands away from the strip because startup CFG establishes
the baseline; settings are reapplied from firmware and no CMD.NV is used.

The September 29, 2026 normal-application check ran simultaneous BLE position,
raw-state and gesture subscriptions for about eight minutes after correcting
the separate BHI360 FIFO parser stall (see
[`LOCAL_CHANGES.md`](../../../libs/BHY2-Sensor-API/LOCAL_CHANGES.md)). Acquisition
continued, sampling stop/resume succeeded, X reached 0 and 896, and Y remained
zero. Click, hold and both swipe codes were received, but sweeps were fragmented
and no double-click was captured. More than three minutes of fresh probe samples
after the final observed release were untouched. These checks preceded the
spatial hysteresis correction, which passed host tests and removed distant
residual jumps in recorded-signal replay; it has not had a further user touch test.
The later 12/6-threshold trial was not retained. C5 remains the selected enclosure
setting; further physical tuning was stopped at the user's request.

Captures on controller firmware 2.5 showed baseline absorption: RAWVALUES stayed
elevated during a held contact while BASEVALUES followed them and SENSORVALUES
fell below detection. Stronger filtering and a faster idle cadence improved
quiet measurements, but did not establish reliable weak-pad gestures and long
holds. Scan counts 8 and 12 produced worse quiet noise in these trials. Lowering
the Y-bank threshold without the faster cadence also produced false contacts.
No final noise margin or complete 15-pad pass has been established.

The baseline interval also needs verification. Measured changes to the baseline
vector occurred at approximately these acquisition intervals:

| Requested BASEINTERVAL | Observed frames between updates |
| --- | --- |
| 10 | 12 |
| 40 | 42 |
| 80 | 82 |
| 200 | 202 |
| 2000 (`0x07D0`) | 210, consistent with low byte 208 plus two |

The last observation suggests that the high byte was ineffective on this device;
controller register readback is still pending because the subsequent diagnostic
startup failed to obtain an I2C acknowledgement. This is an observed discrepancy,
not a documented 8-bit limit. At the measured roughly 19.4 ms frame cadence,
the 2000 trial updated about every 4.07 seconds, rather than about 39 seconds.
Compare requested bytes, post-CFG readback and measured BASEVALUES changes before
relying on a longer interval. The cited datasheet and errata do not describe this
low-byte behavior.

Offline RAWVALUES decoder experiments were rejected: freezing a host baseline
preserved holds but retained offsets after lift, causing stuck contacts. Tested
drift-following alternatives caused position migration or lost holds. No software
RAWVALUES baseline helper is included in the normal application. Captures skip
some acquisitions, so offline replay can expose failures but cannot establish a
hardware pass.

### Debugger mailbox

`tests/shields/main_test_mtch6102.c` defines two global arrays of 32-bit words:
`mtch6102_calibration_snapshot[80]` and `mtch6102_calibration_command[12]`.
Word index `i` is at the symbol address plus `4*i` bytes; values are little-endian.
Resolve the addresses from the running image's symbols each session. No host
script or C structure layout is needed to use this interface.

| Snapshot word index | Meaning |
| --- | --- |
| 0 | Magic `0x53435432`, set at startup; does not prove acquisition is ready |
| 1 | Sequence-lock revision: odd while updating, even when complete |
| 2 | Snapshot result, interpreted as signed 32-bit errno; zero means valid |
| 3 | Acquisition sequence, incremented on SYNC falling edges, wraps at 32 bits |
| 4 | Acquisition uptime in milliseconds, low 32 bits, wraps after about 49.7 days |
| 5–6 | Host `touched` flag and slider X |
| 7 | Y, reserved zero |
| 8 | Host gesture in MTCH6102 byte encoding |
| 9 | Native controller TOUCHSTATE diagnostic byte |
| 10–24 | SENSORVALUES RX0–RX14, one unsigned byte value per word |
| 25–39 | RAWVALUES RX0–RX14, each controller byte pair decoded little-endian into a word |
| 40–54 | BASEVALUES RX0–RX14, decoded the same way |
| 55–58 | SENSORCOMP RX0–RX14 read back at configuration: four bytes per word, lowest RX in bits 7:0; word 58's high byte is unused |
| 59–67 | Configuration RAM 0x20–0x43 read back after CFG: four bytes per word, lowest register address in bits 7:0 |
| 68–69 | Startup probe diagnostics: signed read errno and byte value for TPSM83102 CONTROL1 (0x02) |
| 70–71 | Startup probe diagnostics: signed read errno and byte value for TPSM83102 VOUT (0x03) |
| 72–73 | Startup probe diagnostics: signed read errno and byte value for TPSM83102 CONTROL2 (0x05) |
| 74–77 | Startup I2C ACK bitmap: address `a` sets bit `a % 32` in word `74 + a / 32` |
| 78–79 | Reserved; ignore |

For example, word 59 contains registers 0x20–0x23, and word 67 contains
0x40–0x43. BASEINTERVALL/H (0x2D/0x2E) are the middle two bytes of word 62;
`(snapshot[62] >> 8) & 0xffff` gives their combined register value. These bytes
are cached after configuration, not reread on every frame. Use this layout only
with the matching image and magic; older calibration images used different sizes.

For acquisition data, first require command word 10 to be 4. Read snapshot word
1, then the whole array, then word 1 again. Accept the copy only if
all three revision values agree and are even, magic matches, and signed word 2
is zero. Nonzero status leaves old payload fields in place: do not use them as
a new measurement. The mailbox refreshes about every 20 ms but may repeat a
cached acquisition; deduplicate using word 3 and retain word 4. It is not a
lossless recorder, and the native TCH bit does not replace host `touched`.

Words 68–77 are separate startup-failure diagnostics, outside the frame sequence
lock. Read them after the UART reports `Probe diagnostics complete`. A regulator
value is usable only if that register read completed with errno zero; zero-filled
words before the diagnostic runs are not successful measurements. The address
scan uses one-byte reads at addresses 0x08–0x77. A set bit records an ACK; a clear
bit alone does not distinguish NACK from an address not reached if scanning
stopped early. Check the UART errors as well. These words do not measure voltage.

| Command word index | Meaning |
| --- | --- |
| 0 | Host request ID; write this **last**, using a new nonzero ID |
| 1 | Opcode: 1 = force baseline, 2 = acquisition settings, 3 = debounce/baseline/idle settings |
| 2–7 | Opcode-specific parameters below |
| 8 | Firmware result, signed 32-bit errno; zero means success |
| 9 | Firmware acknowledgement: last completed request ID |
| 10 | Firmware startup stage: 1 = initializing, 2 = configuring, 3 = starting, 4 = calibration loop ready |
| 11 | Firmware startup retry index: 0 = initial attempt, 1–2 = retries; read-only |

| Parameter word | Opcode 2 | Opcode 3 |
| --- | --- | --- |
| 2 | X threshold, 1–255 | Release debounce, 1–10 frames |
| 3 | Y threshold, 1–255 | Contact debounce, 1–10 frames |
| 4 | Hysteresis, zero to less than both thresholds | Positive baseline step limit, 1–255 |
| 5 | Scan count, accepted test range 1–32 | Negative baseline step limit, 1–255 |
| 6 | Requested baseline interval, 1–65535; see observed limitation above | IDLEPERIOD register value, 1–65535; ACTIVEPERIOD is preserved |
| 7 | 0 preserves filtering; 1–3 selects IIR with that strength | Ignored |

These are interface validation ranges, not guarantees of useful sensor behavior.
Invalid parameters are rejected before stopping acquisition. Other configuration
fields remain as previously set. IIR strengths 1, 2 and 3 weight new data by 1/2,
1/4 and 1/8 respectively; stronger filtering also slows the response to a touch.

Initialization permits three attempts, with 250 ms between retries. On startup
failure, command word 8 records the error and the stage remains where it failed.
Wait for stage 4 and valid snapshots. Keep only one request outstanding: write
the opcode/parameters, then the new request ID, and wait until word 9 equals
that ID before reading result word 8 or sending another command. Do not write
firmware-owned result, acknowledgement, stage, retry index, or snapshot words.
Record the requested parameters together with their post-CFG readback for each trial.

Acknowledgement precedes the next snapshot refresh, so an old valid snapshot
may still be visible at that instant. After success, note its revision and wait
for a different even revision with result zero and a fresh acquisition sequence
before recording the next measurement interval.

Opcode 1 ignores parameter words 2–7. Its driver uses a 250 ms polling deadline
plus bounded lock and I2C transport latency, invalidates cached contact/gestures,
and leaves snapshots invalid until a subsequent acquisition.
Opcodes 2 and 3 stop acquisition, apply the candidate with CFG, and restart; on
configuration failure it attempts to restore the previous configuration. Check
the result and fresh frames rather than assuming recovery succeeded. All three
commands require hands off the strip. A failed request remains acknowledged
with a negative result; retry only with a new request ID. Reboot discards these
volatile changes and reloads the image's defaults.

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
