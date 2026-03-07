# CurtainController TMC2209

ESP32-C3 based smart curtain controller with TMC2209 stepper driver, UART control, StallGuard4 sensorless homing, and Home Assistant integration via MQTT.

**Firmware version: v5.2**

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | ESP32-C3 Super Mini (Nologo) |
| Stepper driver | TMC2209 (UART mode, StallGuard4) |
| Motor | NEMA17 or similar stepper |

### Pin Assignment

| Function | GPIO | Notes |
|----------|------|-------|
| STEP | 10 | Step pulse output |
| DIR | 6 | Direction control |
| ENABLE | 0 | Driver enable (active LOW) |
| DIAG | 7 | StallGuard interrupt input (active HIGH) |
| TMC_RX | 20 | UART receive from TMC2209 PDN_UART |
| TMC_TX | 21 | UART transmit to TMC2209 PDN_UART (via 1K resistor) |
| STATUS_LED | 8 | Onboard LED (active LOW) |
| RESET_BUTTON | 9 | Config portal / factory reset button |

GPIO 8 and 9 are strapping pins on the ESP32-C3 — they are used here for LED and button but avoided at boot by the firmware initialization order.

### TMC2209 UART Wiring

```
ESP32-C3                TMC2209
--------                -------
GPIO 20 (RX) <--------- PDN_UART
GPIO 21 (TX) ---[1K]--> PDN_UART
```

Single-wire half-duplex UART: both RX and TX share the PDN_UART pin. The 1K resistor on TX prevents bus contention during driver responses. No address jumper is needed (driver address 0b00 is the default when MS1/MS2 are not pulled high).

---

## Features

- **Silent operation** via TMC2209 StealthChop (spread-spectrum PWM)
- **Sensorless homing** using StallGuard4 — no end-stop switches required
- **Auto-calibration** — finds the full travel range automatically and stores it
- **UART motor control** — current, microsteps, and stall threshold configurable at runtime without recompiling
- **Home Assistant auto-discovery** — cover entity plus number/select entities for all tunable parameters
- **MQTT control** — open, close, stop, position (0-100%), and settings topics
- **WebSerial console** at `/webserial` — full command interface over browser
- **Web setup page** at `/setup` — configure WiFi, MQTT, hostname, and motor parameters without reflashing
- **WiFiManager** — captive portal on first boot or button hold for WiFi provisioning
- **OTA updates** — ArduinoOTA over port 3232
- **Motor auto-sleep** — driver disabled after configurable inactivity timeout
- **Position persistence** — current position written to NVS every 50 steps and on stop
- **Structured logging** — four log levels (ERROR/WARN/INFO/DEBUG) with subsystem tags
- **WiFi reconnection** — automatic reconnect with restart fallback after 30s timeout
- **TMC2209 error monitoring** — overtemperature and short-circuit detection with automatic driver reset

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
| MycilaWebSerial | Browser-based serial console |
| ArduinoOTA | Over-the-air firmware updates |

The ESP32-C3 Arduino core (espressif/arduino-esp32) is required. Install via the Arduino Boards Manager or arduino-cli.

---

## Build

### Arduino CLI

```bash
# Install the board package (once)
arduino-cli core install esp32:esp32

# Compile
arduino-cli compile \
  --fqbn esp32:esp32:nologo_esp32c3_super_mini \
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
6. **Open the setup page** at `http://<device-ip>/setup` to refine motor settings (current, microsteps, stall threshold).
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
| `http://<device-ip>/setup` | Configure hostname, MQTT, and motor parameters. Saves to NVS and reboots. |
| `http://<device-ip>/webserial` | Browser-based serial console. Full command interface. |

The device also registers via mDNS as `<hostname>.local` (HTTP and Arduino OTA services).

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
| `speed <us>` | Step delay in microseconds (100–10000; lower = faster). Default: 2000 |
| `current <mA>` | RMS motor current (100–2000 mA). Default: 800 |
| `microsteps <n>` | Microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256). Default: 2 |
| `sensitivity <level>` | Stall sensitivity: `low`, `medium`, `high`, or `custom <0-255>` |
| `sleep <ms>` | Motor idle timeout in ms before driver disables (0 = never). Default: 30000 |
| `travelsteps <n>` | Override total travel range in steps (1–500000) |

### Calibration

| Command | Description |
|---------|-------------|
| `calibrate` | Run sensorless calibration — finds closed and open endpoints automatically |
| `motortest [sec]` | Run motor under load for N seconds (default 5) and report StallGuard values with a live load bar |

### Diagnostics

| Command | Description |
|---------|-------------|
| `status` | Current position, motor state, MQTT state, TMC2209 live registers |
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
| `<root>/speed/set` | Subscribe | `100`–`10000` | Set step delay (us) |
| `<root>/speed/state` | Publish | integer | Current step delay |
| `<root>/current/set` | Subscribe | `100`–`2000` | Set motor current (mA) |
| `<root>/current/state` | Publish | integer | Current motor current |
| `<root>/stallthreshold/set` | Subscribe | `low` / `medium` / `high` | Set stall sensitivity |
| `<root>/stallthreshold/state` | Publish | `low` / `medium` / `high` | Current stall sensitivity |
| `<root>/microsteps/set` | Subscribe | `1`–`256` | Set microstep resolution |
| `<root>/microsteps/state` | Publish | integer | Current microstep setting |

All published topics use retained messages. Availability uses a MQTT LWT (Last Will and Testament) so Home Assistant marks the device offline immediately on disconnect.

Retained command messages delivered within 2 seconds of subscribing are silently ignored to prevent stale commands from executing on reconnect.

---

## Home Assistant Integration

The device publishes MQTT auto-discovery payloads on first connect (and on `hadiscovery` command). The following entities are created automatically under the device `<hostname>`:

| Entity | Type | Description |
|--------|------|-------------|
| Cover | `cover` (device_class: curtain) | Open/close/stop/position control |
| Calibrate | `button` | Triggers sensorless calibration |
| Speed | `number` (100–10000 us, step 100) | Step delay / motor speed |
| Motor Current | `number` (100–2000 mA, step 100) | RMS current limit |
| Stall Sensitivity | `select` (low / medium / high) | StallGuard sensitivity preset |
| Microsteps | `select` (1–256, powers of 2) | Microstep resolution |

The cover entity uses `set_position_topic` pointing to the command topic, so HA position slider commands send a bare percentage number directly.

---

## Stall Sensitivity System

StallGuard4 reports a motor load value (`SG_RESULT`, 0–1023). A stall is detected when `SG_RESULT` drops below `SGTHRS × 2`. The sensitivity presets map to these SGTHRS values:

| Preset | SGTHRS | Stall triggers when SG < |
|--------|--------|--------------------------|
| `low` | 15 | 30 |
| `medium` | 30 | 60 |
| `high` | 60 | 120 |

Higher sensitivity catches lighter stalls (useful for lightweight curtains or lower current settings). Lower sensitivity ignores friction and minor resistance (useful for heavier curtains or if calibration stops prematurely).

**Tuning workflow:**

1. Run `motortest 10` with the curtain free to move. Observe the live load bar and SG values.
2. Manually apply resistance to the shaft and confirm `STALL!` appears.
3. If you see false stalls during free movement, use `sensitivity low`.
4. If calibration stops before reaching the end, also try `sensitivity low`.
5. Run `calibrate` once the sensitivity is correct.

During normal movement (non-calibration), stall events are logged in verbose mode but do not stop the motor. Stall detection only drives calibration endpoint detection.

---

## Calibration

Calibration uses StallGuard4 to find the mechanical travel limits without end-stop switches:

1. The motor drives toward the closed (minimum) position until a stall is detected.
2. Position is set to 0. The motor backs off 50 steps.
3. The motor drives toward the open (maximum) position until a second stall is detected.
4. A 2% margin is applied at each end to avoid running into the mechanical stops during normal operation.
5. The resulting travel range (in steps) is saved to NVS and HA discovery is re-published.

Trigger via WebSerial (`calibrate`), MQTT (`<root>/calibrate` with payload `press`), or the HA Calibrate button entity.

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

## License

MIT
