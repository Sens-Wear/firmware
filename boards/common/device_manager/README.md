# SensWear Device Manager

The device manager is the single point that turns raw, per-driver device activity
into stable, application-facing data streams. Each SensWear device driver
(`bhi360`, `bq25180`, `bq27427`, `max30101`, `max30208`, `mtch6102`, `tpsm83102`,
`rtc`) posts low-level events onto one shared queue. The device manager owns the
one thread that drains that queue, drives each driver's interrupt handling, and
re-publishes the decoded results as typed [zbus](https://docs.zephyrproject.org/latest/services/zbus/index.html)
messages — one channel per message type.

Application code never talks to the drivers' event queue directly and never
parses raw FIFO/register data. It subscribes to the streams it cares about and
receives fully decoded, timestamped messages.

- **Header / API:** [`device_manager.h`](device_manager.h)
- **Message contract:** [`device_driver_messages.h`](device_driver_messages.h)
- **Implementation:** [`device_manager.c`](device_manager.c)
- **Build gate:** `CONFIG_SENSWEAR_DEVICE_MANAGER`

---

## 1. Design overview

```
   device drivers ──▶ device_driver_events queue ──▶ device manager (sole publisher)
    (post events)          (shared, MPSC)                 │  one consumer thread
                                                          │  publish by value
              ┌───────────────┬───────────────┬──────────┴────────────┐
          IMU streams      PPG stream      touch streams          battery / charger
              │               │               │                    / regulator ...
        BLE manager      storage / log    app / LED-haptic         app / power logic
        (subscriber)     (subscriber)     (listener/subscriber)    (subscriber)
```

Key properties:

- **Single publisher.** The device manager is the only writer of every stream
  channel. Publishing happens by value from one thread, so subscribers never see
  a torn or concurrently mutated message.
- **One message type per stream.** A stream carries exactly one message struct.
  Filtering is done by the framework (pick the stream you want), not by the
  consumer (inspecting a tagged union on a firehose).
- **Decode once, fan out.** Each driver event is decoded a single time by a
  per-device *translator* and published to the matching stream. Any number of
  subscribers then observe it.
- **Driver-agnostic contract.** Consumers depend only on
  `device_driver_messages.h`, never on individual driver headers. Swapping or
  reconfiguring a driver does not ripple into consumers.

### Streams and their producing devices

| Stream (`device_manager_stream_*`) | Message type          | Producing device      | Build gate                          |
| ---------------------------------- | --------------------- | --------------------- | ----------------------------------- |
| `ImuQuaternion`                    | `imu_quaternion_msg_t`| BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `ImuAccel`                         | `imu_accel_msg_t`     | BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `ImuGyro`                          | `imu_gyro_msg_t`      | BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `ImuPedometer`                     | `imu_pedometer_msg_t` | BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `ImuGesture`                       | `imu_gesture_msg_t`   | BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `ImuActivity`                      | `imu_activity_msg_t`  | BHI360                | `CONFIG_SENSWEAR_BHI360_DRIVER`    |
| `Ppg`                              | `ppg_msg_t`           | MAX30101              | `CONFIG_SHIELD_SENSWEAR_PPG`       |
| `Temperature`                      | `temperature_msg_t`   | MAX30208              | `CONFIG_SHIELD_SENSWEAR_TEMPERATURE` |
| `Touch`                            | `touch_msg_t`         | MTCH6102              | `CONFIG_SHIELD_SENSWEAR_TOUCH`     |
| `TouchGesture`                     | `touch_gesture_msg_t` | MTCH6102              | `CONFIG_SHIELD_SENSWEAR_TOUCH`     |
| `Battery`                          | `battery_msg_t`       | BQ27427 (fuel gauge)  | `CONFIG_SENSWEAR_BQ27427_DRIVER`   |
| `Charger`                          | `charger_msg_t`       | BQ25180 (charger)     | `CONFIG_SENSWEAR_BQ25180_DRIVER`   |
| `Regulator`                        | `regulator_msg_t`     | TPSM83102 (regulator) | `CONFIG_SENSWEAR_TPSM83102_DRIVER` |

A stream whose device is not built is simply never valid; registering an
observer for it returns an error rather than crashing.

---

## 2. Execution and threading model

The device manager runs **one consumer thread** (priority 7, 2 KB stack). Its
loop is:

```
forever:
    ev = device_driver_event_wait()          # block on the shared queue
    dispatch(ev):
        select translator by ev.device_id
        translator(ev):
            if ev is a raw INT/timer event:
                drive the driver's handler (decodes, delivers IMU batches now)
            else:
                build the contract message and publish it by value
```

Two event shapes flow through the queue:

1. **Raw interrupt / timer events** (e.g. `bhi360_event_Irq`, `mtch6102_Irq`,
   `max30208_TimerIrq`). The translator calls that driver's handler
   (`bhi360_irq_handler_with_batch_callback()`, `mtch6102_irq_handler()`,
   `max30208_get_samples()`, …) **from the consumer thread**, which reads the
   hardware. Most decoded events are queued for a later loop iteration, but
   high-rate IMU batches are translated immediately, before another queued IRQ
   can reset the driver's sample cache. BHI360 low-rate and meta-events still
   use the queue.
2. **Queued decoded data events** (e.g. `max30208_event_SampleReady`).
   The translator maps them to the contract message and publishes.

This is why interrupt handling is safe to do work in: GPIO/timer callbacks in
the drivers only *post* an event from ISR context; the actual bus I/O and
decoding happen in the manager's thread.

### Publishing is best-effort

`device_manager_publish()` uses `zbus_chan_pub()` with a short timeout
(`DEVICE_MANAGER_PUB_TIMEOUT_MS`, 10 ms). If a channel cannot be acquired in
time the publish is dropped and a warning is logged. **Stream delivery is not a
guaranteed transport** — do not rely on receiving every sample for correctness;
rely on it for monitoring, UI, and logging.

---

## 3. Operating assumptions

Read these before building on the device manager.

- **Consume from your own thread.** Prefer a zbus **message subscriber**
  (`ZBUS_MSG_SUBSCRIBER_DEFINE`) or a **subscriber** (`ZBUS_SUBSCRIBER_DEFINE`)
  drained by your own thread. A **listener** runs synchronously in the device
  manager's publisher thread — it must not block, sleep, or do heavy work, or it
  stalls every other stream. Use listeners only for tiny, non-blocking reactions
  (e.g. toggling an LED).
- **Register after the device is ready.** A stream accepts observers only once
  its producing device is ready (see §4). Register streams after configuring
  their device and before/after `device_manager_start()` — but note that
  `device_manager_start()` refuses to start if a stream already has observers
  whose device is not ready.
- **Runtime observers must be enabled.** Registration uses zbus runtime
  observers. Without that Kconfig support, `device_manager_stream_register()`
  returns `-ENOTSUP`.
- **Timestamps are Unix microseconds, and need the RTC set.** Message
  `timestamp` fields are microseconds since the Unix epoch, sourced from the RTC
  facade. Call `device_manager_set_rtc_time()` (or otherwise set the RTC) early,
  or timestamps will be relative to an unset clock. The regulator message has no
  device timestamp; the manager stamps it at translation time (0 when the RTC
  driver is not built).
- **Optional devices are best-effort.** `device_manager_init()` logs and skips
  any device that fails to come up and always returns 0; the manager still
  serves every other device. Absent shields and disabled drivers are simply not
  supported streams.
- **One shared event queue.** The manager depends on the shared
  `device_driver_events` queue being initialized before `device_manager_start()`
  (otherwise start returns `-ENODEV`).

---

## 4. Lifecycle

The expected bring-up order:

```
 1. (optional) app-level device init/config for devices the manager cannot init
 2. device_manager_init()                 # run every supported driver's init
 3. device_manager_config(cfg)            # apply high-level knobs (IMU streaming,
    and/or device_manager_configure_device(dev, native)   #  cadences) / native cfg
 4. device_manager_stream_register(...)   # subscribe ready streams
 5. device_manager_start()                # spawn the consumer thread
 6. consume messages from your thread
```

### `device_manager_init()`

Runs each supported device's driver init. Failures are logged and skipped;
returns 0 always. **Does not** start the consumer thread.

### `device_manager_config()` / `device_manager_configure_device()`

Two configuration surfaces:

- **`device_manager_config(const struct device_manager_config_t*)`** — the
  high-level aggregate. It enables/stops IMU physical streams and PPG sampling
  at their requested FIFO cadences, and arms the RTC minute alarm when any
  minute-driven cadence is requested. Passing `NULL` selects the SensWear defaults. It rejects up
  front (`-ENODEV`, no state change) if a requested feature's device is not
  ready.
- **`device_manager_configure_device(device, const void* native_cfg)`** —
  configures one device through its *native* driver config struct (e.g.
  `struct bhi360_config_t`, `struct max30101_config_t`). `NULL` selects that
  driver's defaults. On success the device's streams become eligible for
  observers. The **TPSM83102 regulator has no configuration surface** and returns
  `-EINVAL` here.

### Device readiness

A stream is *ready* (accepts observers) when it exists and its producing device
is ready:

- For most devices, readiness means the driver has been initialized **and**
  configured, as reported by the driver's `*_is_ready()`.
- The **TPSM83102 regulator** is devicetree-instantiated with no init/config
  step; it is ready as soon as its Zephyr device is ready
  (`device_is_ready()`).

`device_manager_stream_valid()` and `device_manager_stream_ready()` currently
report the same predicate.

### `device_manager_start()`

Spawns the consumer thread. Idempotent (a second call is a no-op). Returns
`-ENODEV` if the shared event queue is not initialized, or if a stream already
has registered observers but its producing device is not ready.

---

## 5. Configuration reference

### Aggregate config (`struct device_manager_config_t`)

| Field                        | Meaning                                          | Default |
| ---------------------------- | ------------------------------------------------ | ------- |
| `imu.phy_streams_enabled`    | Stream quaternion/accel/gyro at high rate        | `false` |
| `imu.drain_period_ms`        | BHI360 FIFO drain-timer period (ms)              | `100`   |
| `ppg.sampling_enabled`       | Acquire multi-LED wrist-HR samples               | `false` |
| `ppg.per_sample_irq`         | Interrupt per sample instead of FIFO batch       | `false` |
| `gauge.update_period_min`    | Fuel-gauge refresh cadence, minutes (0 disables) | `5`     |
| `temperature.update_period_min` | Temperature sample cadence, minutes (0 disables) | `1`  |

Fetch the defaults with `device_manager_get_default_config()`. Fields for
devices that are not built are ignored.

### Runtime setters

These change one knob after configuration without re-applying the whole config:

- `device_manager_set_imu_drain_period(uint32_t ms)`
- `device_manager_set_gauge_update_period(uint16_t minutes)`
- `device_manager_set_body_temperature_update_period(uint16_t minutes)`
- `device_manager_set_rtc_time(time_t unix_seconds)`
- `device_manager_set_led_color(struct device_manager_led_color_t color)`

### Periodic (RTC-minute-driven) updates

Some devices refresh on a wall-clock cadence rather than their own interrupt.
When any minute cadence is non-zero, the manager arms the RTC minute alarm. On
each `rtc_event_MinuteAlarm` it counts minutes and, when a device's period
elapses:

- **Fuel gauge:** calls `bq27427_update_state()`, publishing a fresh `Battery`
  message.
- **Temperature:** triggers a one-shot MAX30208 conversion on the configured
  device. The driver's independent sampling timer does not need to be running;
  the RTC alarm provides the cadence.

---

## 6. Message contract

Every message lives in `device_driver_messages.h`. All carry a `time_t
timestamp` in microseconds since the Unix epoch. A few examples:

```c
struct battery_msg_t {
    time_t   timestamp;
    int32_t  voltage_mv;
    int32_t  average_current_ma;      /* signed: negative on discharge */
    int32_t  state_of_charge_dpct;    /* tenths of a percent */
    int32_t  remaining_capacity_mah;
    bool     learning_in_progress;
    /* ... */
};

struct regulator_msg_t {
    time_t   timestamp;   /* stamped by the manager (event carries none) */
    uint32_t vout_uv;     /* output-voltage setpoint, microvolts */
    bool     enabled;     /* rail on/off */
};
```

A tagged envelope, `struct device_msg_t`, can carry any message on a single
channel (`type` selects the valid `payload` member, `device_id` names the
producer). The per-type streams are the normal path; the envelope exists for
consumers that want one unified channel.

---

## 7. Example usage

### 7.1 Bring-up + subscribe + consume

```c
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include "device_manager.h"

/* A message subscriber with a queue of 8; drained by our own thread. */
ZBUS_MSG_SUBSCRIBER_DEFINE(app_sub);

/* One buffer big enough for any stream message we subscribe to. */
union any_msg {
    struct battery_msg_t   battery;
    struct charger_msg_t   charger;
    struct regulator_msg_t regulator;
    struct temperature_msg_t temperature;
};

static const enum device_manager_stream_type wanted[] = {
    device_manager_stream_Battery,
    device_manager_stream_Charger,
    device_manager_stream_Regulator,
    device_manager_stream_Temperature,
};

int main(void)
{
    /* Set the wall clock so timestamps and minute cadences are meaningful. */
    (void)device_manager_set_rtc_time((time_t)1767225600); /* 2026-01-01T00:00:00Z */

    /* Run every supported driver's init (failures are skipped). */
    if (device_manager_init() != 0) {
        return 0;
    }

    /* Apply the high-level config: no IMU streaming, gauge every 5 min,
     * temperature every 1 min. NULL would select the same defaults. */
    struct device_manager_config_t cfg;
    device_manager_get_default_config(&cfg);
    cfg.imu.phy_streams_enabled = false;
    (void)device_manager_config(&cfg);

    /* Register only the streams whose device is ready. */
    for (size_t i = 0; i < ARRAY_SIZE(wanted); i++) {
        if (device_manager_stream_ready(wanted[i])) {
            int ret = device_manager_stream_register(wanted[i], &app_sub, K_MSEC(100));
            if (ret != 0) {
                printk("register stream %d failed: %d\n", (int)wanted[i], ret);
            }
        }
    }

    /* Start the consumer thread now that observers are registered. */
    if (device_manager_start() != 0) {
        return 0;
    }

    /* Consume. zbus copies the message into our queue; we read it by value. */
    const struct zbus_channel *chan;
    union any_msg msg;

    while (zbus_sub_wait_msg(&app_sub, &chan, &msg, K_FOREVER) == 0) {
        switch (device_manager_stream_from_channel(chan)) {
        case device_manager_stream_Battery:
            printk("battery: soc=%d.%d%% v=%dmV\n",
                   msg.battery.state_of_charge_dpct / 10,
                   msg.battery.state_of_charge_dpct % 10,
                   msg.battery.voltage_mv);
            break;
        case device_manager_stream_Charger:
            printk("charger: pg=%d charging=%d fault=%d\n",
                   msg.charger.power_good, msg.charger.charging, msg.charger.fault);
            break;
        case device_manager_stream_Regulator:
            printk("regulator: enabled=%d vout=%uuV\n",
                   msg.regulator.enabled, msg.regulator.vout_uv);
            break;
        case device_manager_stream_Temperature:
            printk("temp: %d m°C\n", msg.temperature.temperature_mdeg_c);
            break;
        default:
            break;
        }
    }
    return 0;
}
```

### 7.2 Configuring a device through its native driver config

```c
/* Start high-rate IMU streaming with BHI360 defaults, then subscribe. */
struct bhi360_config_t imu_cfg;
device_manager_get_default_device_config(device_manager_device_Bhi360, &imu_cfg);
/* ... adjust imu_cfg as needed ... */

int ret = device_manager_configure_device(device_manager_device_Bhi360, &imu_cfg);
if (ret == 0 && device_manager_stream_ready(device_manager_stream_ImuQuaternion)) {
    device_manager_stream_register(device_manager_stream_ImuQuaternion, &app_sub, K_MSEC(100));
}
```

### 7.3 Monitoring the regulator rail

The regulator is a passive, config-less producer. Once its devicetree device is
ready, just register for the stream:

```c
if (device_manager_stream_ready(device_manager_stream_Regulator)) {
    device_manager_stream_register(device_manager_stream_Regulator, &app_sub, K_MSEC(100));
}
/* You then receive a regulator_msg_t on every enable/disable and VOUT change. */
```

### 7.4 Haptic control (haptic shield only)

Available when `CONFIG_SHIELD_SENSWEAR_HAPTIC` is set. The manager owns the
playback buffer, so caller arrays need not outlive the call.

```c
/* Simple buzz: amplitude 0-255 held for a duration. */
(void)device_manager_haptic_vibrate(200, 300);   /* 300 ms */

/* Play a ROM waveform sequence from the DRV2605 LRA library (1-123),
 * terminated by the first zero entry. */
static const uint8_t click[] = { 1 };
(void)device_manager_haptic_play_rom(click, ARRAY_SIZE(click));

/* Query "is anything playing" (RTP stream or ROM sequence). */
if (device_manager_haptic_is_active()) {
    /* ... */
}

(void)device_manager_haptic_stop();
```

---

## 8. API summary

| Function                                        | Purpose                                            |
| ----------------------------------------------- | -------------------------------------------------- |
| `device_manager_init`                           | Init all supported devices (skips failures)        |
| `device_manager_get_default_config`             | Fill aggregate config with defaults                |
| `device_manager_config`                         | Apply aggregate config (IMU streaming, cadences)   |
| `device_manager_get_default_device_config`      | Fill one device's native config with defaults      |
| `device_manager_configure_device`              | Configure one device via its native driver config  |
| `device_manager_stream_valid` / `_ready`        | Query whether a stream exists / accepts observers  |
| `device_manager_stream_register` / `_unregister`| Add / remove a zbus observer on a stream           |
| `device_manager_stream_from_channel`            | Map a received channel pointer back to its stream  |
| `device_manager_start`                          | Spawn the consumer thread                          |
| `device_manager_set_imu_drain_period`           | Change IMU FIFO-drain period at runtime            |
| `device_manager_set_gauge_update_period`        | Change fuel-gauge cadence (minutes)                |
| `device_manager_set_body_temperature_update_period` | Change temperature cadence (minutes)           |
| `device_manager_set_rtc_time`                   | Set wall-clock time through the RTC driver         |
| `device_manager_set_led_color`                  | Set the 24-bit RGB indicator color (LP5562 only)   |
| `device_manager_haptic_*`                       | Haptic playback control (haptic shield only)       |

See [`device_manager.h`](device_manager.h) for full per-function contracts,
return codes, and preconditions.

---

## 9. Build and testing

- Enable the module with `CONFIG_SENSWEAR_DEVICE_MANAGER`. Per-device
  translators compile only when their driver's Kconfig is set, so the module is
  safe to build in any device/shield combination.
- A standalone bring-up test lives in
  [`tests/device_manager/`](../../../tests/device_manager/) with presets
  `test_device_manager`, `test_device_manager_haptic`, `test_device_manager_ppg`,
  `test_device_manager_temperature`, and `test_device_manager_touch`. It drives
  the full pipeline: init, configure, subscribe ready streams, and print decoded
  messages.
- To validate high-rate IMU streaming on hardware, build and flash the base
  application, enable IMU streaming, and capture at least one 10-minute IMU-only
  run and one 10-minute IMU+PPG run. Compare per-channel sensor-production,
  device-manager-publication, GATT-submission, and receiver counts using a
  profiling build; check the device log for `IMU * batch count mismatch` and
  `* sample dropped` warnings. A compile-only test cannot establish that sample
  delivery is lossless on a physical board.
