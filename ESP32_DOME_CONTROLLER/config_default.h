/*
  config_default.h - Default configuration (pinout of the original project:
  ESP32 DevKit + relay module, reed switch as park sensor).

  Do not put personal settings here. Copy this file to config_local.h in the same
  folder and edit the copy: config.h uses config_local.h instead of this file if it
  exists, and .gitignore keeps it (WiFi credentials, local pinout) out of the repository.
*/
#pragma once

// ========= WIFI / NETWORK =========
#define WIFI_SSID     "SSID"
#define WIFI_PASSWORD "PASSWORD"

#define DEFAULT_ALPACA_PORT    11111
#define WIFI_CHECK_INTERVAL     30000UL
#define WIFI_RETRY_INTERVAL     5000UL

// ========= PARK SENSOR =========
// PARK_SENSOR_REED: reed/limit switch at the mount, read continuously (original project).
//                   Also exposed as Alpaca Safety Monitor.
// PARK_SENSOR_BEAM: IR light barrier whose beam is interrupted by an unparked telescope.
//                   Powered via PARK_POWER_PIN only for a short check before each move,
//                   so its IR light cannot disturb exposures. No Safety Monitor device.
//                   (A permanently powered light barrier is just a contact: use REED.)
#define PARK_SENSOR_TYPE  PARK_SENSOR_REED

// ========= PINS (-1 = not used) =========
// Inputs: wire contacts between pin and GND (active LOW, internal pull-up). For sensors
// with an active-HIGH output set the *_INPUT_ACTIVE_LEVEL below to HIGH (pull-down).
//
// Examples:          ESP32 DevKit + relay modules   Waveshare ESP32-S3-Relay-6CH
//                    (original project)
//   RELAY_PIN              26                         1  (relay CH1)
//   OPEN_RELAY_PIN         -1                        -1  (or 41 = relay CH3)
//   PARK_POWER_PIN         -1 / e.g. 25               2  (relay CH2)
//   ROOF_OPEN_PIN          33                         4  (header H1 pin 15)
//   ROOF_CLOSED_PIN        32                         5  (header H1 pin 14)
//   PARK_SAFE_PIN          27                         6  (header H1 pin 12)
#define RELAY_PIN       26    // motor impulse; single-button input = open / stop / close
#define OPEN_RELAY_PIN  -1    // optional dedicated "open only" input of the motor controller
#define PARK_POWER_PIN  -1    // PARK_SENSOR_BEAM only: relay that powers the light barrier
#define ROOF_OPEN_PIN   33    // roof fully open contact
#define ROOF_CLOSED_PIN 32    // roof fully closed contact
#define PARK_SAFE_PIN   27    // park sensor contact, closed = parked (beam clear)

#define RELAY_ACTIVE_LEVEL  HIGH  // LOW for relay modules that switch on with a LOW input
#define ROOF_INPUT_ACTIVE_LEVEL  LOW  // level of ROOF_OPEN_PIN / ROOF_CLOSED_PIN when the contact is active
#define PARK_INPUT_ACTIVE_LEVEL  LOW  // level of PARK_SAFE_PIN when parked (beam clear)

// ========= TIMING =========
#define MOTOR_PULSE_MS      1000UL   // impulse length
#define PULSE_GAP_MS        1000UL   // minimum pause between two impulses (abort = stop impulse)
#define TRAVEL_TIMEOUT_MS   60000UL  // measure your travel time, set to travel time + ~15 s
#define REED_DEBOUNCE_MS    100      // roof contacts must be stable this long (EMI)

// PARK_SENSOR_BEAM only
#define PARK_BEAM_SETTLE_MS 1000UL   // light barrier powered this long before it is read
// 1: while unpowered the contact must read "interrupted", otherwise a shorted cable would
// always report "parked". Set 0 only if the receiver contact is closed while unpowered.
#define PARK_BEAM_SELFTEST  1
