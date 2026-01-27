# CurtainController TMC2209

ESP32-C3 based curtain controller with TMC2209 stepper driver and StallGuard4 sensorless homing.

## Features

- **WiFi Control** - Control via MQTT and Home Assistant
- **TMC2209 Driver** - Silent StealthChop operation with UART control
- **StallGuard4** - Sensorless homing and stall detection via DIAG interrupt
- **Web Interface** - Configuration and WebSerial console
- **OTA Updates** - Over-the-air firmware updates
- **Auto-Calibration** - Automatic travel distance detection using StallGuard

## Hardware

- ESP32-C3 Super Mini (Nologo)
- TMC2209 Stepper Driver (UART mode)
- NEMA17 or similar stepper motor

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| STEP | 10 | Step pulse output |
| DIR | 6 | Direction control |
| ENABLE | 0 | Driver enable (active LOW) |
| DIAG | 7 | StallGuard stall detection input |
| TMC_RX | 20 | UART receive from TMC2209 |
| TMC_TX | 21 | UART transmit to TMC2209 (via 1K resistor) |
| STATUS_LED | 8 | Onboard LED (active LOW) |
| RESET_BUTTON | 9 | Config/reset button |

## TMC2209 UART Wiring

```
ESP32-C3                TMC2209
---------               -------
GPIO 20 (RX) <--------- PDN_UART
GPIO 21 (TX) ---[1K]--> PDN_UART
```

- Single-wire UART: Both RX and TX connect to PDN_UART pin
- 1K resistor on TX line prevents bus contention
- PDN_UART directly connected (no jumper needed on most modules)

## Commands

### Movement
| Command | Description |
|---------|-------------|
| `open` | Open curtain fully |
| `close` | Close curtain fully |
| `stop` | Stop movement |
| `position <n>` | Move to step position |

### Motor Settings
| Command | Description |
|---------|-------------|
| `speed <us>` | Set step delay (100-10000 us) |
| `steps <n>` | Set steps per revolution |
| `setposition <n>` | Reset position counter |
| `sleep <ms>` | Set motor sleep timeout |

### TMC2209 Settings
| Command | Description |
|---------|-------------|
| `microsteps <n>` | Set microsteps (1-256) |
| `current <mA>` | Set motor current (100-2000 mA) |
| `stallthreshold <n>` | Set stall sensitivity (0-255) |
| `calibrate` | Auto-calibrate using StallGuard |
| `verbose` | Toggle SG output during movement |
| `sgtest [sec]` | Monitor StallGuard values (default 5s) |

### System
| Command | Description |
|---------|-------------|
| `status` | Show current status and TMC2209 info |
| `config` | Show configuration |
| `hadiscovery` | Republish Home Assistant discovery |
| `restart` | Reboot device |
| `help` | Show all commands |

## StallGuard Stall Detection

Uses DIAG pin interrupt for efficient stall detection:
- DIAG pin triggers when SG_RESULT < SGTHRS × 2
- Higher threshold = more sensitive to stalls
- Use `sgtest` command to find optimal threshold:
  1. Run `sgtest 10` and note SG values during free movement
  2. Apply resistance to motor shaft and observe SG drop
  3. Set threshold using recommendations provided

## Web Interface

- **Setup Page**: `http://<device-ip>/setup` - Configure all settings
- **WebSerial**: `http://<device-ip>/webserial` - Serial console

## Home Assistant Integration

Auto-discovery via MQTT. Supports:
- Open/Close/Stop commands
- Position control (0-100%)
- Position reporting
- Availability status

## Button Functions

- **3-5 second hold**: Start WiFi configuration portal
- **10-13 second hold**: Factory reset (clears all settings)

## Libraries Required

- WiFiManager
- PubSubClient
- ArduinoJson
- TMCStepper
- ESPAsyncWebServer
- MycilaWebSerial

## License

MIT
