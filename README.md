# ESP32 Alpaca Roll-off Roof Controller

ESP32 firmware that drives a roll-off roof through the push-button input of a single-button
gate controller (open → stop → close → stop) and exposes it via ASCOM Alpaca as a **Dome**
and a **Safety Monitor** (mount parked).

## Park detection (light barrier)

The parked mount blocks a light barrier; its relay contact (NO) is the park signal.

- "Parked" counts after the beam has been blocked for 10 s without interruption
- "Not parked" takes effect immediately
- After a reboot the 10 s confirmation starts again
- If the beam clears while the roof is closing, a fault latches
- Many light barriers also close the contact when sender or receiver lose power; with a single
  contact this looks like "parked"

## Roof drive

- One relay pulse per command (open → stop → close → stop)
- The firmware tracks what the next pulse does; after an abort only the reverse command is accepted
- Abort only pulses while the roof is moving

## Features

- Alpaca Dome + Safety Monitor on port `11111`, UDP discovery on `32227`
- Roof commands are rejected unless the mount is parked
- Faults latch (both end switches active, park lost while closing) until reset in the web UI
- Web UI with status, open/close, confirmed overrides and fault reset
- Firmware update from the browser (`/update`, only with roof closed and mount parked)
- WiFi reconnect, reboot after 10 min without WiFi (never while moving), task watchdog

## Hardware

- Waveshare ESP32-S3-Relay-6CH (ESP32-S3-WROOM-1U-N16)
- Roof controller with a single push-button input (open → stop → close → stop)
- 2 end switches (NO) for the roof positions
- Light barrier with relay output (NO) for the park position

| Function | GPIO | Wiring |
|---|---|---|
| Relay | 1 | Relay CH1 → push-button input of the roof controller |
| Roof open switch | 35 | COM → GND, NO → GPIO |
| Roof closed switch | 36 | COM → GND, NO → GPIO |
| Park light barrier | 37 | Relay output: COM → GND, NO → GPIO |

Pins are set in the `PIN DEFINITIONS` section of the sketch.

## Setup (Arduino IDE)

1. Install the ESP32 board package (core 3.x) and the libraries **ESP Async WebServer** and **Async TCP**
   (both by ESP32Async; older forks do not compile with core 3.x) and **ArduinoJson**.
2. Copy `arduino_secrets.h.example` to `arduino_secrets.h` and enter your WiFi
   (the file is gitignored; the sketch does not compile without it).
3. Board settings: **ESP32S3 Dev Module**, Flash Size **16MB**, PSRAM **Disabled**,
   USB CDC On Boot **Enabled**, Partition Scheme **16M Flash (3MB APP/9.9MB FATFS)**.
4. Flash via USB once. Later updates: *Sketch → Export Compiled Binary*, then upload the
   `.bin` from `build/` at `http://<ip>:11111/update`.

Further settings (timeouts, hostname, discovery, time zone) are at the top of the sketch.

## Usage

- Web UI: `http://<ip>:11111/`
- Diagnostics: `http://<ip>:11111/diag` (event log with NTP time, survives reboots but not power loss; WiFi, heap, last Alpaca request)
- NINA finds Dome and Safety Monitor via Alpaca discovery in the same network.

## Safety

Use at your own risk. The park interlock is only as good as the sensor placement and wiring.
Keep a way to cut power to the roof motor independently of this controller.

## License

MIT, see [LICENSE](LICENSE). Based on
[ESP32-ALPACA-DOME](https://github.com/rzibreg/ESP32-ALPACA-DOME) by Robert Žibreg.
