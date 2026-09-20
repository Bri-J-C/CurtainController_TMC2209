# CurtainController TMC2209

ESP32-C3 based smart curtain controller with TMC2209 stepper driver, UART control, StallGuard4 sensorless homing, and Home Assistant integration via MQTT.

**Firmware version: v5.5**

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | ESP32-C3 Super Mini (Nologo) |
| Stepper driver | TMC2209 (UART mode, StallGuard4) |
| Motor | NEMA17 or similar stepper |

### Pin Assignment

Two wiring profiles ship in the firmware, selectable on the setup page along with fully custom
assignments. The `current` profile is below; `legacy` (STEP 10, DIR 6, EN 0, DIAG 7, TX 21, RX 20)
matches boards built to the original layout.

| Function | GPIO | Notes |
|----------|------|-------|
| STEP | 6 | Step pulse output |
| DIR | 5 | Direction control |
| ENABLE | 20 | Driver enable (active LOW) |
| DIAG | 21 | StallGuard interrupt input (active HIGH) |
| TMC_TX | 7 | UART transmit to the TMC2209 UART pin |
| TMC_RX | 10 | UART receive from the TMC2209 UART pin |
| STATUS_LED | 8 | Onboard LED (active LOW) |
| RESET_BUTTON | 9 | Config portal / factory reset button |

GPIO 8 and 9 are strapping pins on the ESP32-C3 — they are used here for LED and button but avoided at boot by the firmware initialization order.

### TMC2209 UART Wiring

```
ESP32-C3                TMC2209
--------                -------
GPIO 7  (TX) ---------> UART
GPIO 10 (RX) <--------- UART
```

Single-wire half-duplex UART: TX and RX share the driver's UART pin.

A series resistor in the TX leg is required, so the driver can pull the line low against the ESP32's
idling-high TX output while it replies. Some modules have one built in on one of their two UART pads —
if yours does not, or you are on the pad without it, fit a 1K resistor in the TX leg. Without it you get
the request echoed back and no reply, which `tmcdiag` reports directly.

No address jumper is needed — leaving MS1 and MS2 unconnected selects address 0.

See [HARDWARE.md](HARDWARE.md) for the full wiring diagram, bill of materials and mounting notes.

---

## Features

- **Silent operation** via TMC2209 StealthChop (spread-spectrum PWM)
- **Sensorless homing** using StallGuard4 — no end-stop switches required
- **Auto-calibration** — finds the full travel range automatically and stores it; back-off margins set precise, repeatable endpoints
- **Direction invert** — swap open/close direction via WebSerial command, MQTT switch entity, or setup page; persisted to NVS
- **UART motor control** — current, microsteps, and stall threshold configurable at runtime without recompiling; motor current comes from the UART setting alone, not the module's VREF trimpot
- **Home Assistant auto-discovery** — cover entity plus number/select/switch entities for all tunable parameters
- **MQTT control** — open, close, stop, position (0–100%), and settings topics
- **WebSerial console** at `/webserial` — full command interface over browser
- **Web setup page** at `/setup` — dark theme UI; configure WiFi, MQTT, hostname, and motor parameters without reflashing; root `/` redirects here automatically
- **WiFiManager** — captive portal on first boot or button hold for WiFi provisioning
- **OTA updates** — ArduinoOTA over port 3232
- **Motor auto-sleep** — driver disabled after configurable inactivity timeout
- **Hardware-timed stepping** — step pulses come from a hardware timer ISR with accel/decel ramps, so WiFi/MQTT activity can't cause step jitter (which StallGuard reads as load)
- **Position persistence** — current position written to NVS every second while moving and on stop
- **Structured logging** — four log levels (ERROR/WARN/INFO/DEBUG) with subsystem tags
- **WiFi reconnection** — automatic reconnect with restart fallback after 30s timeout
- **TMC2209 error monitoring** — overtemperature and short-circuit detection with automatic driver reset
- **Bulk WebSocket output** — `ws_send_bulk()` sends full status/config/help blocks as a single WebSocket message for instant display
- **Single-source version** — `#define FW_VERSION` propagates the version string to HA discovery and the setup page footer

---

## Dependencies

Install these libraries via the Arduino Library Manager or `arduino-cli lib install`:

| Library | Purpose |
|---------|---------|
| WiFiManager | Captive portal WiFi provisioning |
| PubSubClient | MQTT client |
| ArduinoJson | MQTT discovery payload serialization |
| TMCStepper | TMC2209 UART register access |
| ESPAsyncWebServer | Async HTTP server for `/setup` and WebSerial |
| ArduinoOTA | Over-the-air firmware updates |

The ESP32-C3 Arduino core (espressif/arduino-esp32) is required. Install via the Arduino Boards Manager or arduino-cli.

---

## Build

### Flash a prebuilt binary

Download `firmware-merged.bin` from the
[latest release](https://github.com/Bri-J-C/CurtainController_TMC2209/releases/latest) and write it
over USB:

```bash
esptool --chip esp32c3 -p /dev/ttyUSB0 write-flash 0x0 firmware-merged.bin
```

The merged image contains the bootloader, partition table and firmware, and includes the larger app
partition layout below. It writes the whole flash, so saved WiFi credentials and settings are erased.
To keep existing settings, build from source and upload over OTA instead.

### Optional: larger app partitions

`partitions.csv` in the sketch folder replaces the stock layout (1.25 MB per OTA slot plus 1.5 MB of
unused SPIFFS) with 1.9 MB per OTA slot. The firmware fits either way — 98% full on the stock layout,
65% with this one — so it's optional, but it leaves room to grow.

- Arduino CLI picks the file up automatically; add `--build-property upload.maximum_size=1966080` so the
  size check uses the bigger slot.
- **A partition change only applies over USB.** OTA writes into the existing slot and can't relayout flash.
- Settings survive: the NVS region keeps the same address.
- To skip it, delete `partitions.csv` and keep builds under 1.25 MB.

### Arduino CLI

```bash
# Install the board package (once)
arduino-cli core install esp32:esp32

# Compile
arduino-cli compile \
  --fqbn esp32:esp32:nologo_esp32c3_super_mini \
  --build-property upload.maximum_size=1966080 \
  CurtainController_TMC2209

# Upload (replace /dev/ttyUSB0 with your port)
arduino-cli upload \
  --fqbn esp32:esp32:nologo_esp32c3_super_mini \
  --port /dev/ttyUSB0 \
  CurtainController_TMC2209
```

### Arduino IDE

Select board: **ESP32C3 Dev Module** (or Nologo ESP32-C3 Super Mini if available in your core version). No special partition scheme is required.

---

## First-Use Setup

1. **Flash the firmware** using the build instructions above.
2. **Power on** the device. On first boot (no saved WiFi), it starts the config portal automatically.
3. **Connect to the AP** named `CurtainSetup` (password: `12345678`).
4. **Fill in the captive portal form** with your WiFi credentials, MQTT server IP, MQTT port, username, password, and MQTT root topic. The default root topic is `home/room/curtains`.
5. **Save** — the device connects to your network and restarts.
6. **Open the setup page** at `http://<device-ip>/setup` (or just `http://<device-ip>/` — the root redirects there) to refine motor settings (current, microsteps, stall sensitivity).
7. **Run calibration** via WebSerial or MQTT to detect the curtain travel range automatically.

---

## Button Hold Actions

Hold the RESET_BUTTON (GPIO 9) while the device is running:

| Hold Duration | Action |
|---------------|--------|
| 3 – 5 seconds | Opens WiFi config portal (LED solid on during window) |
| 10 – 13 seconds | Factory reset — clears all NVS settings and WiFi config, then restarts |

Holding at boot for 3+ seconds also forces the config portal to open immediately, before any WiFi connection is attempted.

---

## Web Interfaces

| URL | Purpose |
|-----|---------|
| `http://<device-ip>/` | Redirects to `/setup` |
| `http://<device-ip>/setup` | Tabbed configuration — Network, Motor, System. Includes the wiring profile and per-pin GPIO assignment, which are deliberately not exposed over MQTT or the console. Saves to NVS and reboots. |
| `http://<device-ip>/webserial` | Browser-based serial console. Full command interface. |

The setup page uses a dark theme with cyan-purple gradient styling. The device also registers via mDNS as `<hostname>.local` (HTTP and Arduino OTA services).

---

## WebSerial Commands

Connect to `http://<device-ip>/webserial` or open a serial monitor at 115200 baud.

### Movement

| Command | Description |
|---------|-------------|
| `open` | Move to fully open position |
| `close` | Move to fully closed position |
| `stop` | Stop movement or cancel calibration |
| `<0-100>` | Move to position as a percentage (e.g. `50` moves to midpoint) |

### Settings

| Command | Description |
|---------|-------------|
| `speed <rpm>` | Shaft speed, 10–300 RPM. Default: 75. The step interval is derived from this and the microstep setting |
| `current <mA>` | RMS motor current (100–2000 mA). Default: 800 |
| `microsteps <n>` | Microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256). Default: 16. Position and travel range are rescaled with it, and shaft speed is held constant |
| `sensitivity <level>` | Stall sensitivity: `extra_low`, `low`, `medium`, `high`, `max`, or `custom <0-255>` |
| `backoff [end n]` | Show both end clearances, or set one: `backoff open 300`, `backoff close 20`. Default: 15 each |
| `invert` | Toggle open/close direction (persisted to NVS) |
| `sleep <ms>` | Motor idle timeout in ms before driver disables (0 = never). Default: 30000 |
| `travelsteps <n>` | Override total travel range in steps (1–500000) |

### Calibration

| Command | Description |
|---------|-------------|
| `calibrate` | Run sensorless calibration — finds closed and open endpoints automatically |
| `motortest [sec]` | Run motor for N seconds (default 5) using the same stall detector as calibration. Reports the free-running SG range, stall events with peak scores, UART read errors, and a suggested `sensitivity custom N`. `stop` aborts it |

### Diagnostics

| Command | Description |
|---------|-------------|
| `status` | Current position, motor state, MQTT state, TMC2209 live registers |
| `tmcdiag` | Probe the UART link: address scan, echo/reply split, write counter, and a decode of the driver's live registers |
| `config` | Full configuration dump (hostname, IP, MQTT, all motor settings) |
| `verbose` | Toggle StallGuard debug output during movement |
| `loglevel <level>` | Set log level: `error`, `warn`, `info`, `debug` |
| `ledon` / `ledoff` | Manual LED control |

### System

| Command | Description |
|---------|-------------|
| `setposition <n>` | Override the position counter without moving (use with care) |
| `hadiscovery` | Force-republish all Home Assistant MQTT discovery payloads |
| `restart` | Reboot the device |
| `help` | Show command reference |

---

## MQTT Topics

All topics are derived from the configured MQTT root topic (default: `home/room/curtains`).

| Topic | Direction | Payload | Description |
|-------|-----------|---------|-------------|
| `<root>/cmd` | Subscribe | `open` / `close` / `stop` / `0-100` | Movement commands and position set |
| `<root>/status` | Publish | `open` / `opening` / `closing` / `closed` / `stopped` | Cover state |
| `<root>/position` | Publish | `0`–`100` | Current position percentage |
| `<root>/availability` | Publish | `online` / `offline` | LWT availability |
| `<root>/calibrate` | Subscribe | `press` | Trigger calibration |
| `<root>/speed_rpm/set` | Subscribe | `10`–`300` | Set shaft speed (RPM) |
| `<root>/speed_rpm/state` | Publish | integer | Current shaft speed |
| `<root>/current/set` | Subscribe | `100`–`2000` | Set motor current (mA) |
| `<root>/current/state` | Publish | integer | Current motor current |
| `<root>/backoff_close/set` | Subscribe | `1`–`2000` | Set the closed-end clearance (full steps) |
| `<root>/backoff_close/state` | Publish | integer | Current closed-end clearance |
| `<root>/backoff_open/set` | Subscribe | `1`–`2000` | Set the open-end clearance (full steps) |
| `<root>/backoff_open/state` | Publish | integer | Current open-end clearance |
| `<root>/stallthreshold/set` | Subscribe | `extra_low` / `low` / `medium` / `high` / `max` | Set stall sensitivity |
| `<root>/stallthreshold/state` | Publish | sensitivity name | Current stall sensitivity |
| `<root>/microsteps/set` | Subscribe | `1`–`256` | Set microstep resolution |
| `<root>/microsteps/state` | Publish | integer | Current microstep setting |
| `<root>/invert/set` | Subscribe | `ON` / `OFF` | Set direction inversion |
| `<root>/invert/state` | Publish | `ON` / `OFF` | Current direction inversion state |

All published topics use retained messages. Availability uses a MQTT LWT (Last Will and Testament) so Home Assistant marks the device offline immediately on disconnect.

Retained command messages delivered within 2 seconds of subscribing are silently ignored to prevent stale commands from executing on reconnect.

---

## Home Assistant Integration

The device publishes MQTT auto-discovery payloads on first connect (and on `hadiscovery` command). The following entities are created automatically under the device `<hostname>`:

| Entity | Type | Description |
|--------|------|-------------|
| Cover | `cover` (device_class: curtain) | Open/close/stop/position control |
| Calibrate | `button` | Triggers sensorless calibration |
| Speed | `number` (10–300 RPM, step 5) | Shaft speed |
| Motor Current | `number` (100–2000 mA, step 100) | RMS current limit |
| Stall Sensitivity | `select` (extra_low / low / medium / high / max) | StallGuard sensitivity preset |
| Microsteps | `select` (1–256, powers of 2) | Microstep resolution |
| Back-off Close | `number` (1–2000 full steps, step 5) | Clearance kept at the closed end |
| Back-off Open | `number` (1–2000 full steps, step 5) | Clearance kept at the open end |
| Invert Direction | `switch` | Swap open/close direction |

The cover entity uses `set_position_topic` pointing to the command topic, so HA position slider commands send a bare percentage number directly.

---

## Speed and microsteps

Speed is stored as shaft RPM, and the step interval is derived from it and the current microstep
setting. Changing microsteps therefore changes smoothness only — the curtain keeps moving at the same
speed, and StallGuard tuning stays valid. Position and travel range are counted in microsteps, so both
are rescaled when the resolution changes.

StallGuard needs roughly 1 rev/s or more to read load reliably (60 RPM on a 200-step motor), and
StealthChop's automatic tuning expects 60–300 RPM. Calibrate somewhere in that band.

---

## Stall Sensitivity System

StallGuard4 reports a motor load value (`SG_RESULT`, 0–1023). A stall is detected when `SG_RESULT` drops below `SGTHRS × 2`. The sensitivity presets map to these SGTHRS values:

| Preset | SGTHRS | Stall triggers when SG < |
|--------|--------|--------------------------|
| `extra_low` | 5 | 10 |
| `low` | 15 | 30 |
| `medium` | 30 | 60 |
| `high` | 60 | 120 |
| `max` | 100 | 200 |

### How a stall is detected

- The step ISR samples DIAG once per full step (the rate StallGuard updates). A full step counts as stalled if DIAG pulsed since the last sample or is still high.
- A leaky score goes +1 per stalled full step and −1 per clean one; a stall is confirmed at **4**. Isolated spikes decay away; a real stall accumulates.
- Detection is blanked during the accel ramp (~300 ms) plus 200 ms at cruise speed, while StealthChop's current regulation settles and SG_RESULT is meaningless.

Higher sensitivity catches lighter stalls (useful for lightweight curtains or lower current settings). Lower sensitivity ignores friction and minor resistance (useful for heavier curtains or if calibration stops prematurely).

The `sensitivity_name()` reverse-mapping uses boundary thresholds: ≤8 → `extra_low`, ≤20 → `low`, ≤45 → `medium`, ≤80 → `high`, ≤120 → `max`.

**Tuning workflow:**

1. Run `motortest 10` with the curtain free to move (away from the ends). Watch the SG values and stall score.
2. Apply the suggested `sensitivity custom N` from the results (puts the stall line at ~60% of the lowest free-running SG).
3. Re-run `motortest` and apply resistance to the shaft; confirm `STALL!` appears.
4. If calibration stops before reaching the end, lower the sensitivity. If it grinds at the end without stopping, raise it.
5. Run `calibrate` once the sensitivity is correct.

SG_RESULT depends on speed and current, so re-run `motortest` after changing `speed`, `current`, or `microsteps`.

During normal movement (non-calibration), stall events are logged in verbose mode but do not stop the motor. Stall detection only drives calibration endpoint detection.

---

## Calibration

Calibration uses StallGuard4 to find the mechanical travel limits without end-stop switches:

1. The motor drives toward the closed (minimum) position until a stall is detected.
2. The motor backs off by the configured margin (`backoff`, 15 full steps by default). This backed-off position is set as position 0 — the safe closed boundary.
3. The motor drives toward the open (maximum) position until a second stall is detected.
4. The motor backs off the same margin from the open wall. The usable travel range is the total steps driven minus this open back-off.
5. The resulting travel range (in steps) is saved to NVS and HA discovery is re-published.

If the measured travel is implausibly short (under 4× the back-off), calibration aborts instead of saving — that's almost always a false stall.

Both back-off margins are accounted for in the stored `steps_per_revolution` so position 0% and 100% reliably stop before the mechanical limits.

Trigger via WebSerial (`calibrate`), MQTT (`<root>/calibrate` with payload `press`), or the HA Calibrate button entity.

---

## Direction Invert

If your curtain moves in the wrong direction (open closes it, close opens it), use the invert feature:

- **WebSerial**: `invert` — toggles and persists immediately
- **HA switch**: `switch.<hostname>_invert` — toggle from the Home Assistant UI
- **MQTT**: publish `ON` or `OFF` to `<root>/invert/set`

Inversion is implemented via the TMC2209 `shaft` register bit, so it affects the driver-level step direction without changing any wiring or logic. The setting is stored in NVS and survives reboots.

---

## Logging

Log output is sent to both the hardware serial port (115200 baud) and the WebSerial browser console. Format:

```
[LEVEL] [SUBSYSTEM] message
```

| Level | When used |
|-------|-----------|
| `ERROR` | Hardware failures, UART errors, movement timeouts |
| `WARN` | MQTT reconnection, TMC2209 temperature pre-warning, open load |
| `INFO` | Normal operational events (connects, moves, calibration steps) |
| `DEBUG` | StallGuard register values, MQTT payload details |

The log level persists across reboots (stored in NVS). Set with `loglevel <level>` via WebSerial or serial.

Subsystem tags used: `BOOT`, `TMC`, `MOTOR`, `CAL`, `MQTT`, `WIFI`, `BTN`, `CMD`, `NVS`, `OTA`, `SYS`.

---

## OTA Updates

ArduinoOTA listens on port 3232. The hostname is set to the configured device hostname. To set an OTA password, enter it in the `/setup` page under "OTA Password" or in the WiFiManager config portal.

```bash
arduino-cli upload \
  --fqbn esp32:esp32:nologo_esp32c3_super_mini \
  --port <device-ip>:3232 \
  --upload-field password=<ota-password> \
  CurtainController_TMC2209
```

Or use the Arduino IDE's **Sketch > Upload Using Programmer** after selecting the network port.

---

## Troubleshooting

### TMC2209 not responding

Check UART wiring: GPIO 7 (TX) and GPIO 10 (RX) both go to the driver's UART pin. If the module has no series resistor on that pin, add a 1K resistor in the TX leg — without it, TX fights the driver during replies and corrupts reads. `motortest` reports UART read errors.

### Calibration stops too early

The motor stalls before reaching the physical end. Lower the sensitivity: `sensitivity extra_low` or `sensitivity low`. Run `motortest 10` to see the load values during free movement and confirm you have clearance above the stall threshold.

### Curtain moves the wrong direction

Use `invert` in WebSerial, the Invert Direction switch in Home Assistant, or toggle `<root>/invert/set` via MQTT. No re-wiring or recompiling is needed.

### MQTT not connecting

Verify the MQTT server IP, port, username, and password in the setup page. The device uses exponential backoff (2s to 60s) between retries. The `config` command shows the currently configured MQTT server and user.

### Position drifts over time

Run `calibrate` to re-establish the travel range. If the motor skips steps under load, increase the motor current (`current <mA>`) or lower the speed (`speed <us>`).

---

## License

MIT
