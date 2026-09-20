# Hardware Guide

## Bill of Materials

### Core components
- **ESP32-C3 Super Mini** development board
- **TMC2209 stepper driver module** (16-pin stepstick, UART mode, StallGuard4) — the board used here has `EN, MS1, MS2, PDN, USART, CLK, STEP, DIR` on one side, `VM, GND, A2, A1, B1, B2, VDD, GND` on the other, and `VREF, DIAG, INDEX` on the end
- **Bipolar stepper motor** (NEMA 17, 200 steps/revolution)
- **12–24 V power supply**, 2 A or more
- **Mini buck converter** (12–24 V to 5 V) for the ESP32
- **Perf board** 4 cm × 6 cm, with headers, pins and terminal blocks
- **100 µF capacitor** across the motor supply

### Curtain components
- **Curtains** and **low-friction curtain track** — friction is what StallGuard has to see past, so a smooth track matters
- **GT2 timing belt**, 6 mm
- **5 mm bore pulley** for the motor shaft, **5 mm idler pulley**, **belt tensioner**, **belt clamp**
- **Stepper motor mount**
- **Idler pulley mount**

## Wiring


### ESP32-C3 to TMC2209

| ESP32-C3 | TMC2209 |
|----------|---------|
| GPIO 5 | DIR |
| GPIO 6 | STEP |
| GPIO 7 | UART (TX) |
| GPIO 10 | UART (RX) |
| GPIO 20 | EN |
| GPIO 21 | DIAG |
| 3V3 | VDD (VIO) |
| GND | GND |

TX and RX share the single UART pin, with a 1K resistor in the TX leg. Some modules have one built into
one of their UART pads; without it in the path the driver cannot pull the line low to answer.

MS1 and MS2 stay unconnected, which selects driver address 0.

VDD must be 3.3 V, not 5 V. The driver drives the UART line at whatever VDD is, and 5 V on an ESP32 input pin damages it.

### Power

| From | To |
|------|-----|
| Power supply + | TMC2209 VM, buck converter IN+ |
| Power supply − | TMC2209 GND, buck converter IN− |
| Buck converter OUT+ | ESP32-C3 5V |
| Buck converter OUT− | ESP32-C3 GND |

A 100 µF capacitor goes across VM and GND, close to the driver.

All grounds are common: supply, buck converter, ESP32 and driver.

### Motor

| TMC2209 | Motor |
|---------|-------|
| A1, A2 | coil 1 |
| B1, B2 | coil 2 |

To find the pairs, check continuity between motor wires — two wires that beep together are one coil.

Never unplug the motor while the driver is powered. It can destroy the driver.

## Motor current

Current is set in software (`current <mA>`), not with the VREF potentiometer — in UART mode the pot does nothing.

Start around 600–1000 mA for a curtain. Higher isn't better: with too much current the motor out-muscles the belt, which then skips teeth at the end of travel instead of stalling, and StallGuard never sees the end. If calibration runs into the stops and grinds, lower the current before touching the sensitivity.

## Motor specifications

Typical NEMA 17:

- **Steps per revolution:** 200 (1.8° per step)
- **Voltage:** 12–24 V
- **Current per phase:** 1.0–2.0 A
- **Holding torque:** 40–60 Ncm
- **Coil resistance:** 2–4 Ω

## Heat

- **Driver:** warm is normal. If it's too hot to touch, lower the current. The firmware watches for overtemperature and resets the driver if it trips.
- **Motor:** also warm in normal use. The sleep timeout (`sleep <ms>`) drops the driver output when idle, which keeps it cooler.

## Mounting

- Mount the motor rigidly to the frame or wall, with the shaft aligned to the belt path.
- Rubber feet or cabinet bumpers under the mount cut down vibration noise.
- Keep cable runs clear of the belt and the curtain's travel.

## Safety

- Size wire for the motor current, and secure every connection.
- Make sure the curtain can't bind or jam along its travel.
- Ventilate the enclosure. Don't seal the driver in a closed box.
