# ESP32 ASCOM Alpaca Dome (Roll-off Roof) & Safety Controller

> **Fork with extensions:** light barrier as alternative park sensor, configurable pins
> and relay polarity, optional dedicated "open" output, park check enforced by the dome
> itself. See [Fork Extensions](#fork-extensions). Sketch folder: `ESP32_DOME_CONTROLLER/`.

A modern, wireless replacement for the classic Rolling Roof Computer Interface (RRCI),
built on a single ESP32 dev board and the ASCOM Alpaca standard. No Windows drivers,
no USB cables — just WiFi.

![ESP32 Setup](esp32_setup.jpg)

## Overview

This project upgrades the classic RRCI to the ASCOM Alpaca standard, replacing the
old USB-tethered Arduino solution with a completely wireless, OS-independent system
for roll-off roof control. A single relay drives the roof motor (open/close), and
magnetic reed switches report roof and mount park status.

The controller presents itself to your astronomy software (NINA, Voyager, SGP) as
two distinct Alpaca devices:

- **Dome Device** — Handles roof movement (Open/Close)
- **Safety Monitor** — Exposes the physical "Mount Parked" sensor state (reed park sensor only)

> **Important:** Dome Alpaca endpoints (Open/Close) check the park sensor before every
> move and reject the command with an error message if the mount is not parked. Only the
> Web UI override buttons skip this check.
>
> The Safety Monitor device (reed park sensor only) exposes the park sensor state. Imaging
> software usually uses a Safety Monitor to decide whether observing is safe (e.g. NINA
> sequences that loop "while safe") — a "mount parked" sensor reports unsafe during the
> whole session, so it is usually not suitable as the main Safety Monitor.

---

## Features

- **Alpaca Native** — No Windows ASCOM Platform required. Works on Linux (INDI),
  macOS, and Windows purely over WiFi
- **Integrated Safety** — Physical "Mount Parked" reed switch acts as a hardware
  interlock; automation software can reject commands if the mount is not parked
- **Dual Interface**
  - ASCOM/Alpaca REST API for automated imaging sessions
  - Mobile-friendly Web Dashboard hosted on the device (port 11111)
- **Manual Override** — Web UI includes override buttons to force roof movement
  during testing or emergencies, bypassing safety sensors
- **Low Cost** — Built for ~20€ using standard components from AliExpress

---

## Hardware Required (~20€)

| Component | Notes |
|---|---|
| ESP32 Development Board | Recommended: ESP32-WROOM-32U with external antenna for better range inside metal domes |
| ESP32 Expansion Board (38-pin) | For easy screw-terminal connections |
| 5V Relay Module (1-channel) | Triggers your existing roof motor controller |
| 3× Magnetic Reed Switches (NO) | Roof Open, Roof Closed, and Mount Parked detection |

---

## Pinout & Wiring

> All sensors use internal `INPUT_PULLUP`. Wire them to switch to **GND** when active
> (default; active-HIGH sensors are supported via `ROOF_INPUT_ACTIVE_LEVEL` /
> `PARK_INPUT_ACTIVE_LEVEL`, which switches the pin to `INPUT_PULLDOWN`).
> Default pinout (`config_default.h`); all pins can be changed in the configuration.

| Function | GPIO | Wiring |
|---|---|---|
| Relay Trigger | GPIO 26 | Connect to Relay IN — triggers the roof motor |
| Roof Closed Sensor | GPIO 32 | Sensor between Pin 32 and GND |
| Roof Open Sensor | GPIO 33 | Sensor between Pin 33 and GND |
| Mount Safe Sensor | GPIO 27 | Sensor between Pin 27 and GND |

---

## Configuration

Copy `ESP32_DOME_CONTROLLER/config_default.h` to `ESP32_DOME_CONTROLLER/config_local.h`
and edit the copy. `config.h` uses `config_local.h` if it exists; it is listed in
`.gitignore`, so your WiFi credentials and pinout are never committed.

- **CRITICAL:** Set `WIFI_SSID` and `WIFI_PASSWORD` to match your observatory's WiFi network
- Choose the park sensor (`PARK_SENSOR_TYPE`), pins, relay polarity and timings
- Default port: `11111`
- Static IP is not yet implemented in the sketch (see [TODO](#todo)); it is recommended
  to assign a static IP via your router's DHCP reservation using the ESP32's MAC address

---

## Fork Extensions

### Light barrier as park sensor (`PARK_SENSOR_BEAM`)

For mounts without space for a reed switch: an IR light barrier is placed so that an
unparked telescope interrupts the beam. Its receiver contact (closed = beam clear) goes to
`PARK_SAFE_PIN` and GND, its supply is switched by a relay on `PARK_POWER_PIN`.

Before every Open/Close the controller

1. reads the contact while the light barrier is unpowered — it must read "interrupted",
   otherwise the signal line is shorted (`PARK_BEAM_SELFTEST`),
2. powers the light barrier for `PARK_BEAM_SETTLE_MS` and reads the contact,
3. switches the light barrier off again and only then triggers the motor.

The light barrier is therefore lit for about one second per move and cannot disturb
exposures. Because it cannot be read between moves, no Safety Monitor device is offered in
this mode; use an external Safety Monitor (weather / cloud / rain sensor) in your software.

The light barrier is a sensor for the ESP32 only — do not wire it into the safety input of
the motor controller.

### Motor controller and relays

- `RELAY_PIN`: impulse on a single-button input (open / stop / close), as in the original.
- `OPEN_RELAY_PIN`: optional dedicated "open only" input, if the controller has one.
- `RELAY_ACTIVE_LEVEL`: `HIGH` or `LOW` depending on the relay board.
- `ROOF_INPUT_ACTIVE_LEVEL` / `PARK_INPUT_ACTIVE_LEVEL`: level of an active input
  (default `LOW` = contact to GND with pull-up; `HIGH` = active-high sensor with pull-down).

### Roof logic

- Roof contacts are debounced (`REED_DEBOUNCE_MS`).
- Abort sends a stop impulse (after `PULSE_GAP_MS`).
- Timeout, wrong direction (single-button input started between the end positions),
  both contacts active and unexpected movement are reported as `shutterError` with a
  reason, shown in the Web UI and returned to Alpaca clients.
- Open/Close are accepted from the error state, so the roof can be recovered remotely.

---

## Required Libraries (Arduino IDE)

- `ESPAsyncWebServer`
- `AsyncTCP`
- Standard ESP32 WiFi libraries (included with ESP32 board support package)

---

## Connecting to NINA / ASCOM

Alpaca UDP discovery is disabled to conserve resources for the web server.
**You must add the device manually:**

### Dome Device

1. Open **ASCOM Diagnostics**
2. Go to **Device > Choose and Connect to Device**
3. Select **Dome** as Device Type → **Choose** → **Alpaca** → **Create Alpaca Driver**
4. Enter a name (e.g. `ESP32 Dome`) and click **OK**
5. Enter the ESP32's IP address and port `11111`
6. Click **OK**, then select your newly created device and click **Connect** to test

### Safety Monitor

Repeat the steps above, selecting **Safety Monitor** as the Device Type in step 3.

---

## Web Dashboard

Access `http://[YOUR_ESP32_IP]:11111` in any browser.

![Web UI](esp32_UI.jpg) ![Alpaca Dome](esp32_alpaca.jpg) ![Alpaca Safety](esp32_alpaca_safety.jpg)

- **Standard Controls** — Respect the Mount Safe sensor state
- **Override Controls** — Bypass sensors (use with caution!)
- **Status Panel** — Real-time feedback for Roof Open, Roof Closed, and Park Sensor

---

## ⚠️ Safety Warning

**USE AT YOUR OWN RISK.**

Remote observatory control carries a real risk of equipment collision. The "Mount Safe"
sensor interlock depends entirely on correct physical sensor placement and wiring.
Always maintain a backup method (e.g. a physical power cutoff) to kill power to the
roof motor in the event of a software or network failure.

---

## TODO

- [ ] Static IP configuration option in the sketch (currently requires router-side
  DHCP reservation by MAC address)

---

## Contributing

Contributions are welcome! Feel free to open issues, submit pull requests, or suggest
improvements. Please keep code readable and comment any non-obvious logic.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Author

**Robert Zibreg** — original author and maintainer.
