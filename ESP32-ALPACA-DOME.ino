/*
  ESP32 Alpaca Roll-off Roof Controller
 
  Drives a roll-off roof through the push-button input of a single-button controller
  (open -> stop -> close -> stop) and exposes it via ASCOM Alpaca as Dome and Safety Monitor.
  The park position comes from a light barrier. Setup: see README.md.
*/
 
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h> 
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <Preferences.h>
#include <inttypes.h>
#include <Update.h>
#include <esp_task_wdt.h>
 
// ========= USER SETTINGS =========
// SECRET_SSID / SECRET_PASS: copy arduino_secrets.h.example to arduino_secrets.h (ignored by git).
// Deliberately no fallback: a build without real credentials would lose WiFi after an OTA update.
#include "arduino_secrets.h"
#define WIFI_HOSTNAME          "esp32-dome"
 
#define ENABLE_DISCOVERY       true     // Alpaca UDP discovery (NINA finds the device without an IP)
 
#define DEFAULT_ALPACA_PORT    11111
#define ALPACA_DISCOVERY_PORT  32227
#define WIFI_RETRY_INTERVAL     30000UL  // while disconnected: reconnect attempt this often
#define WIFI_REBOOT_AFTER_MS    600000UL // still no WiFi after 10 min: reboot (only if the roof is idle)
#define WDT_TIMEOUT_S           10       // loop() must run at least this often, otherwise the chip resets
#define RELAY_PULSE_MS          1000UL
#define RELAY_ACTIVE_LEVEL      HIGH     // Waveshare ESP32-S3-Relay-6CH: HIGH = relay on
#define ROOF_TIMEOUT_MS         20000UL  // roof needs ~5 s; no end switch after this = position unknown
#define DEBOUNCE_MS             500UL    // an end switch level counts only after being stable this long
#define PARK_CONFIRM_MS         10000UL  // light barrier must stay blocked this long before "parked" counts
#define STOP_ON_PARK_LOSS       false    // send a stop pulse if park is lost while closing (test first!)
#define OVERRIDE_TOKEN_TTL_MS   15000UL
 
// ========= ALPACA ERROR CODES =========
#define ERR_NOT_IMPLEMENTED    0x400
#define ERR_NOT_CONNECTED      0x407
#define ERR_INVALID_OPERATION  0x40B
 
// ========= PIN DEFINITIONS =========
// Set these for your board. Inputs are active LOW (switch to GND, internal pull-up).
// Waveshare ESP32-S3-Relay-6CH: relay 1 (CH1), open 35, closed 36, park 37.
//   Avoid GPIO26-32 (SPI flash) and GPIO45/46 (strapping pins, relay CH5/CH6);
//   GPIO35-37 are only free on modules without octal PSRAM (N16, not N16R8).
#define RELAY_PIN       1   // relay CH1 -> push-button input of the roof controller
#define ROOF_OPEN_PIN   35
#define ROOF_CLOSED_PIN 36
#define PARK_SAFE_PIN   37  // park light barrier relay: COM to GND, NO to GPIO37
 
// ========= STATE =========
enum ShutterState {
  SHUTTER_OPEN = 0,
  SHUTTER_CLOSED = 1,
  SHUTTER_OPENING = 2,
  SHUTTER_CLOSING = 3,
  SHUTTER_ERROR = 4
};
 
// What the next relay pulse does on the single-button controller (open -> stop -> close -> stop)
enum NextPulse { PULSE_UNKNOWN, PULSE_OPENS, PULSE_CLOSES };
 
ShutterState currentShutterState = SHUTTER_ERROR; // unknown until the end switches are read
NextPulse nextPulse = PULSE_UNKNOWN;
bool isSlewing = false;
unsigned long lastCommandTime = 0;
bool parkGuarded = false;          // closing started with confirmed park: losing it is a fault
bool roofLost = false;             // latched fault, cleared only by resetRoofFault() or an override
const char* roofErrorReason = "";  // why the position is unknown or the fault latched
unsigned long faultAt = 0;
bool domeConnected = false;
bool safetyConnected = false;
bool relayActive = false;
unsigned long relayOnTime = 0;
uint32_t overrideToken = 0;
bool overrideTokenForClose = false;
unsigned long overrideTokenIssuedAt = 0;
// Guards roof state: HTTP handlers (async_tcp task) and loop() run concurrently
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
 
// Debounced switch input (active = switch closed to GND). Updated in loop() only.
struct DebouncedSwitch {
  uint8_t pin;
  unsigned long settleMs;    // raw level must be stable this long before it counts
  bool releaseImmediately;   // true: going inactive is accepted without waiting (fail-safe)
  volatile bool active;      // debounced level, read by HTTP handlers
  bool raw;                  // last raw reading
  unsigned long rawSince;    // when raw last changed
};
DebouncedSwitch roofOpenSw   = { ROOF_OPEN_PIN,   DEBOUNCE_MS,     false, false, false, 0 };
DebouncedSwitch roofClosedSw = { ROOF_CLOSED_PIN, DEBOUNCE_MS,     false, false, false, 0 };
// Park light barrier: the parked mount blocks the beam and closes COM-NO. Many barriers also
// close COM-NO when sender or receiver lose power, so with a single contact an outage looks like PARKED.
DebouncedSwitch parkSw       = { PARK_SAFE_PIN,   PARK_CONFIRM_MS, true,  false, false, 0 };
 
volatile uint32_t serverTransactionID = 0;
unsigned long lastWifiCheck = 0;
bool wifiDown = false;
unsigned long wifiDownSince = 0;
volatile bool restartPending = false;
unsigned long restartRequestedAt = 0;
volatile bool otaInProgress = false;
volatile unsigned long otaLastChunkAt = 0;
bool otaRejected = false;
 
int currentAlpacaPort = DEFAULT_ALPACA_PORT;
 
// ========= OBJECTS =========
AsyncWebServer *server = NULL;
AsyncUDP udp;
Preferences preferences;
 
// ========= HTML (Minified) =========
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8" /><meta name="viewport" content="width=device-width, initial-scale=1" /><title>ESP32 Dome Control</title><style> *, *::before, *::after { box-sizing: border-box; } body, html { margin: 0; padding: 0; height: 100%; background-color: #121212; color: #eee; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Ubuntu, sans-serif; display: flex; justify-content: center; align-items: flex-start; padding: 1rem 0.5rem; user-select: none; } main { width: 100%; max-width: 400px; background: #1c1c1c; border: 1px solid #333; box-shadow: inset 0 0 8px #222, 0 0 16px #0a84ff33; padding: 16px 20px; display: flex; flex-direction: column; gap: 1rem; color: #ddd; } header { text-align: center; } header h1 { font-weight: 600; font-size: 1.4rem; letter-spacing: 0.12em; margin-bottom: 6px; color: #0af; } header p { font-size: 0.8rem; color: #555; margin-top: -4px; letter-spacing: 0.08em; } #domestatus { margin-top: 8px; font-weight: 700; text-transform: uppercase; font-size: 1.2rem; letter-spacing: 0.12em; padding: 6px 0; border-top: 1px solid #333; border-bottom: 1px solid #333; color: #0af; } .status-open { color: #6cf98b; } .status-closed { color: #5c98fc; } .status-opening, .status-closing { color: #ffcc00; } .status-unknown { color: #ff5252; } .status-error { color: #ff5252; } section { display: flex; flex-direction: column; gap: 0.6rem; } .sensor { display: flex; justify-content: space-between; font-family: monospace; font-size: 0.95rem; padding: 4px 0; border-bottom: 1px solid #2a2a2a; } .sensor:last-child { border-bottom: none; } .sensor-name { color: #999; text-transform: uppercase; letter-spacing: 0.07em; font-weight: 600; } .sensor-value { font-weight: 700; text-transform: capitalize; } .trig { color: #5ce35c; } .notrig { color: #666; } .error { color: #ff4f4f; } .safe-status { text-align: center; font-weight: 700; text-transform: uppercase; letter-spacing: 0.12em; padding: 6px 0; user-select: none; } .safe-status.safe { color: #6cf98b; border-top: 1px solid #3a7d3a; border-bottom: 1px solid #3a7d3a; } .safe-status.notSafe { color: #ff5252; border-top: 1px solid #7d3a3a; border-bottom: 1px solid #7d3a3a; } button { width: 100%; padding: 14px 12px; background: #222; border: none; color: #ddd; font-size: 1rem; font-weight: 600; letter-spacing: 0.1em; text-transform: uppercase; cursor: pointer; transition: background 0.15s ease; user-select: none; border-left: 3px solid transparent; outline-offset: 2px; } button:focus-visible { outline: 2px solid #0af; outline-offset: 3px; } button:active:not(:disabled) { background: #333; } button:disabled { background: #111; color: #555; cursor: default; border-left: 3px solid #444; } #sopenbtn { border-left-color: #6cf98b; margin-bottom: 10px; } #sclosebtn { border-left-color: #5c98fc; } .override-btn { font-size: 0.84rem; font-weight: 500; padding: 10px 12px; background: #111; color: #999; border-left: 3px solid transparent; } .override-btn:enabled:hover { color: #0af; border-left-color: #0af; background: #191919; } .override-btn:disabled { color: #333; border-left-color: #333; cursor: default; background: #111; } .override-wrap { display: flex; justify-content: space-between; gap: 10px; margin-top: 0.6rem; } .override-wrap div { flex: 1; } .note { font-size: 0.8rem; color: #555; user-select: none; padding-top: 0.6rem; border-top: 1px solid #333; line-height: 1.3; letter-spacing: 0.02em; } </style></head><body><main role="main"><header><h1>ESP32 Dome Control</h1><p>Remote roof controller</p><div id="domestatus" class="status-error">Loading...</div></header><section><div id="safetystatus" class="safe-status notSafe">Loading...</div></section><section><button id="sopenbtn" onclick="postCmd('/sopen')">Open Dome</button><button id="sclosebtn" onclick="postCmd('/sclose')">Close Dome</button></section><section class="override-wrap"><div><button id="openbtn" onclick="confirmOverride('/open')" class="override-btn">Override Open</button></div><div><button id="closebtn" onclick="confirmOverride('/close')" class="override-btn">Override Close</button></div></section><section><div class="sensor"><div class="sensor-name">Roof Open</div><div id="roofopen" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Roof Closed</div><div id="roofclosed" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Park Light Barrier</div><div id="parksensor" class="sensor-value error">Loading</div></div><div class="sensor"><div class="sensor-name">Last Reset</div><div id="resetreason" class="sensor-value notrig">Loading</div></div></section><div class="note"><strong>Note:</strong> Safe dome open/close buttons require the mount to be parked. Override commands ignore the park sensor and work regardless. <a href="/update" style="color:#555">Firmware update</a></div><section><div class="sensor"><span class="sensor-name">Fault</span><span id="faulttext" class="sensor-value">-</span></div><button id="resetfault" onclick="resetFault()" class="override-btn">Reset Fault</button></section></main><script> async function confirmOverride(url) { const closing = url === '/close'; const msg = closing ? 'OVERRIDE CLOSE\n\nMount park confirmation is bypassed. The roof can collide with your telescope.\n\nProceed with override close?' : 'OVERRIDE OPEN\n\nMount park confirmation is bypassed. Check roof clearance and mount position.\n\nProceed with override open?'; if (!window.confirm(msg)) return; try { const armed = await fetch('/override/arm?direction=' + (closing ? 'close' : 'open'), {method:'POST',cache:'no-store'}); if (!armed.ok) throw new Error(await armed.text()); const data = await armed.json(); const res = await fetch(url + '?token=' + encodeURIComponent(data.token), {method:'POST',cache:'no-store'}); if (!res.ok) throw new Error(await res.text()); updateStatus(); } catch (e) { alert('Override rejected: ' + e.message); } } async function resetFault() { if (!window.confirm('Reset the fault latch after checking the roof and sensor positions?')) return; const r = await fetch('/fault/reset', {method:'POST'}); if (!r.ok) alert('Reset rejected: ' + await r.text()); updateStatus(); } function postCmd(url) { fetch(url, { method: 'POST' }) .then(async response => { if (!response.ok) alert('Command rejected: ' + await response.text()); }) .catch(error => console.error('Error:', error)); } function updateStatus() { fetch('/status', {cache: 'no-store'}) .then(res => res.json()) .then(data => { const domeStatusEl = document.getElementById('domestatus'); domeStatusEl.textContent = data.domeStatus || 'ERROR'; domeStatusEl.className = 'status-' + ((data.domeStatus || '').toLowerCase()); const safeEl = document.getElementById('safetystatus'); safeEl.textContent = data.safetyStatus || 'Error'; safeEl.className = data.safetySafe ? 'safe-status safe' : 'safe-status notSafe'; const roofOpenEl = document.getElementById('roofopen'); roofOpenEl.textContent = data.roofOpen ? 'Triggered' : 'Not triggered'; roofOpenEl.className = 'sensor-value ' + (data.roofOpen ? 'trig' : 'notrig'); const roofClosedEl = document.getElementById('roofclosed'); roofClosedEl.textContent = data.roofClosed ? 'Triggered' : 'Not triggered'; roofClosedEl.className = 'sensor-value ' + (data.roofClosed ? 'trig' : 'notrig'); const parkEl = document.getElementById('parksensor'); parkEl.textContent = data.parkConfirmed ? 'Scope parked' : (data.parkBlocked ? 'Verifying (' + (data.parkProgressMs / 1000).toFixed(1) + '/10 s)' : 'Scope NOT parked'); document.getElementById('faulttext').textContent = data.fault || 'none'; document.getElementById('resetreason').textContent = data.resetReason + ', up ' + Math.floor(data.uptimeS / 60) + ' min'; document.getElementById('resetfault').disabled = !data.faultLatched; parkEl.className = 'sensor-value ' + (data.parkConfirmed ? 'trig' : 'error'); document.getElementById('sopenbtn').disabled = !!data.roofOpen || !data.safetySafe; document.getElementById('sclosebtn').disabled = !!data.roofClosed || !data.safetySafe; document.getElementById('openbtn').disabled = !!data.roofOpen; document.getElementById('closebtn').disabled = !!data.roofClosed; }) .catch(() => { document.getElementById('domestatus').textContent = 'ERROR (Conn)'; }); } setInterval(updateStatus, 2000); window.addEventListener('load', updateStatus); </script></body></html>)rawliteral";
 
const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><body><form action="/setup/save" method="POST" style="background:#222;color:#eee;padding:20px;font-family:sans-serif">
<h1>Setup</h1><label>TCP</label><input name="tcp" value="%TCP%"><br><br>
<button>Save & Reboot</button></form></body></html>
)rawliteral";
 
const char UPDATE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><body style="background:#222;color:#eee;padding:20px;font-family:sans-serif">
<h1>Firmware Update</h1><p>Running build: %BUILD%</p>
<form action="/update" method="POST" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin"><br><br>
<button>Upload & Reboot</button></form></body></html>
)rawliteral";
 
// ========= UTILS =========
void initSwitch(DebouncedSwitch &s) {
  s.raw = (digitalRead(s.pin) == LOW);
  // Fail-safe inputs start inactive and have to settle first (no PARKED state across a reboot)
  s.active = s.releaseImmediately ? false : s.raw;
  s.rawSince = millis();
}
 
void updateSwitch(DebouncedSwitch &s) {
  bool now = (digitalRead(s.pin) == LOW);
  if (now != s.raw) { s.raw = now; s.rawSince = millis(); }
  if (s.active == s.raw) return;
  if ((s.releaseImmediately && !s.raw) || (millis() - s.rawSince >= s.settleMs)) s.active = s.raw;
}
 
bool bothEndSwitches() { return roofOpenSw.active && roofClosedSw.active; }
 
const char* domeStatusStr() {
  if (roofLost || bothEndSwitches()) return "ERROR";
  if (isSlewing) return (currentShutterState == SHUTTER_OPENING) ? "OPENING" : "CLOSING";
  if (roofOpenSw.active)   return "OPEN";
  if (roofClosedSw.active) return "CLOSED";
  return "UNKNOWN";
}
 
bool isSystemSafe() { return parkSw.active && !roofLost && !bothEndSwitches(); }
 
// A reboot must not interrupt a roof movement or clear a latched fault
bool rebootSafe() { return !isSlewing && !relayActive && !roofLost; }
 
const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "Power on";
    case ESP_RST_EXT:      return "External reset";
    case ESP_RST_SW:       return "Software restart";
    case ESP_RST_PANIC:    return "Crash (panic)";
    case ESP_RST_INT_WDT:  return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog";
    case ESP_RST_WDT:      return "Other watchdog";
    case ESP_RST_BROWNOUT: return "Brownout";
    default:               return "Other";
  }
}
 
// An aborted upload (client gone) never calls the completion handler, so expire after 10 s idle
bool otaActive() {
  if (!otaInProgress) return false;
  unsigned long last = otaLastChunkAt;   // read before millis(): written by the upload handler
  return millis() - last < 10000UL;
}
 
// ========= STATE LOGIC (all helpers below: caller holds stateMux) =========
bool triggerRelay() {
  if (relayActive) return false;
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL);
  relayActive = true;
  relayOnTime = millis();
  return true;
}
 
// Real fault: stays latched until resetRoofFault() or an override
void latchRoofFault(const char* why) {
  if (roofLost) return;   // preserve the first cause
  roofLost = true; roofErrorReason = why; faultAt = millis();
  currentShutterState = SHUTTER_ERROR; isSlewing = false;
}
 
// Position unknown without a real fault: cleared automatically at an end switch
void setPositionUnknown(const char* why, NextPulse next) {
  currentShutterState = SHUTTER_ERROR; isSlewing = false;
  roofErrorReason = why; nextPulse = next;
}
 
void setEndPosition(bool open) {
  currentShutterState = open ? SHUTTER_OPEN : SHUTTER_CLOSED;
  nextPulse = open ? PULSE_CLOSES : PULSE_OPENS;
  isSlewing = false; roofErrorReason = "";
}
 
// ========= ROOF COMMANDS =========
// Returns NULL if the roof was started, otherwise the reason for the rejection.
// Overrides (enforceSafety=false) bypass only the park check and a latched fault,
// never a movement conflict or a pulse that is known to drive the other way.
const char* startRoof(bool open, bool enforceSafety) {
  if (otaActive()) return "Firmware update in progress";   // never start the motor right before an OTA reboot
  if (enforceSafety && roofLost) return "Fault latched, reset required";
  if (enforceSafety && !parkSw.active) return "Mount not parked";
  const char* err = NULL;
  portENTER_CRITICAL(&stateMux);
  bool atTarget = open ? roofOpenSw.active : roofClosedSw.active;
  NextPulse wanted = open ? PULSE_OPENS : PULSE_CLOSES;
  if (isSlewing || relayActive)                             err = "Roof moving or relay busy";
  else if (bothEndSwitches())                               err = "Both limit switches active";
  else if (atTarget)                                        err = open ? "Already open" : "Already closed";
  else if (nextPulse != wanted && nextPulse != PULSE_UNKNOWN) err = open ? "Next pulse would close the roof" : "Next pulse would open the roof";
  else if (nextPulse == PULSE_UNKNOWN && enforceSafety)     err = "Roof position unknown, use override";
  else {
    triggerRelay();
    roofLost = false; roofErrorReason = "";
    currentShutterState = open ? SHUTTER_OPENING : SHUTTER_CLOSING;
    isSlewing = true; lastCommandTime = millis();
    parkGuarded = !open && parkSw.active;
  }
  portEXIT_CRITICAL(&stateMux);
  return err;
}
 
// Single-button controller: a pulse at standstill would start the motor, so only pulse while moving.
// Returns false if the start pulse is still active and the stop pulse cannot be sent yet.
bool abortRoof() {
  bool ok = true;
  portENTER_CRITICAL(&stateMux);
  if (isSlewing) {
    if (relayActive) ok = false;
    else {
      // The drive reverses after a stop, so the next pulse goes the other way
      NextPulse next = (currentShutterState == SHUTTER_OPENING) ? PULSE_CLOSES : PULSE_OPENS;
      triggerRelay();
      setPositionUnknown("Aborted between end switches", next);
    }
  }
  portEXIT_CRITICAL(&stateMux);
  return ok;
}
 
bool resetRoofFault() {
  bool ok = false;
  portENTER_CRITICAL(&stateMux);
  // Only at a stable, unambiguous end position, PARKED, and with no pulse active
  bool stable = millis() - roofOpenSw.rawSince >= 2000UL && millis() - roofClosedSw.rawSince >= 2000UL;
  if (roofLost && !isSlewing && !relayActive && parkSw.active && stable &&
      roofOpenSw.active != roofClosedSw.active && millis() - faultAt >= 5000UL) {
    roofLost = false;
    setEndPosition(roofOpenSw.active);
    ok = true;
  }
  portEXIT_CRITICAL(&stateMux);
  return ok;
}
 
void updateRoofState() {
  updateSwitch(roofOpenSw);
  updateSwitch(roofClosedSw);
  updateSwitch(parkSw);
  portENTER_CRITICAL(&stateMux);
  // Elapsed-time comparison is rollover-safe (millis() wraps after ~49.7 days)
  if (relayActive && (millis() - relayOnTime >= RELAY_PULSE_MS)) { digitalWrite(RELAY_PIN, !RELAY_ACTIVE_LEVEL); relayActive = false; }
  if (bothEndSwitches()) latchRoofFault("Both roof limit switches active");
  if (roofLost) { portEXIT_CRITICAL(&stateMux); return; }
  if (isSlewing) {
    if (currentShutterState == SHUTTER_CLOSING && parkGuarded && !parkSw.active) {
      // Software stop only: relies on the controller stopping on the next pulse
      bool stopped = STOP_ON_PARK_LOSS && triggerRelay();
      nextPulse = stopped ? PULSE_OPENS : PULSE_UNKNOWN;
      latchRoofFault("Lost park confirmation during closing");
    }
    else if (currentShutterState == SHUTTER_OPENING && roofOpenSw.active)   setEndPosition(true);
    else if (currentShutterState == SHUTTER_CLOSING && roofClosedSw.active) setEndPosition(false);
    else if (millis() - lastCommandTime > ROOF_TIMEOUT_MS) setPositionUnknown("Roof travel timeout, motor state unknown", PULSE_UNKNOWN);
  }
  // While idle the end switches are the source of truth (covers reboot and manual movement)
  else if (roofOpenSw.active || roofClosedSw.active) setEndPosition(roofOpenSw.active);
  else if (currentShutterState == SHUTTER_OPEN || currentShutterState == SHUTTER_CLOSED)
    setPositionUnknown("Roof left end position without command", PULSE_UNKNOWN);
  portEXIT_CRITICAL(&stateMux);
}
 
// Serial output is not allowed inside the critical section, so log changes from loop()
void logStateChanges() {
  static const char* loggedWhy = "";
  static const char* loggedStatus = "";
  static bool loggedRelay = false;
  static bool loggedPark = false;
 
  const char* why = roofErrorReason;
  if (why != loggedWhy) { loggedWhy = why; if (*why) Serial.printf("[ROOF] %s\n", why); }
  const char* status = domeStatusStr();   // returns string literals, so comparing pointers is enough
  if (status != loggedStatus) { loggedStatus = status; Serial.printf("[STATE] %s\n", status); }
  bool relay = relayActive;
  if (relay != loggedRelay) { loggedRelay = relay; Serial.println(relay ? "[RELAY] pulse on" : "[RELAY] pulse off"); }
  bool park = parkSw.active;
  if (park != loggedPark) { loggedPark = park; Serial.println(park ? "[PARK] confirmed" : "[PARK] not parked"); }
}
 
// ========= ID Helper =========
// Alpaca: parameter names are case-insensitive (query string and form body)
bool alpacaArg(AsyncWebServerRequest *r, const char* name, String &value) {
  for (size_t i = 0; i < r->params(); i++) {
    const AsyncWebParameter* p = r->getParam(i);
    if (p->name().equalsIgnoreCase(name)) { value = p->value(); return true; }
  }
  return false;
}
 
// Echoed back unchanged; 0 if the client sent none (toInt() would overflow above 2^31)
uint32_t getClientID(AsyncWebServerRequest *r) {
  String v;
  return alpacaArg(r, "ClientTransactionID", v) ? (uint32_t)strtoul(v.c_str(), nullptr, 10) : 0;
}
 
// ========= ALPACA RESPONSES =========
void sendJson(AsyncWebServerRequest *r, const char* json) {
    r->send(r->beginResponse(200, "application/json", json));
}
 
// Response for methods without a Value (errorNumber 0 = success)
void sendAlpacaResult(AsyncWebServerRequest *r, int errorNumber, const char* msg) {
    char buf[192];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%" PRIu32 ",\"ServerTransactionID\":%" PRIu32 ",\"ErrorNumber\":%d,\"ErrorMessage\":\"%s\"}", getClientID(r), serverTransactionID++, errorNumber, msg);
    sendJson(r, buf);
}
 
// Successful response with a Value; valueJson is already JSON (number, bool, quoted string, array, object)
void sendAlpacaValue(AsyncWebServerRequest *r, const char* valueJson) {
    char buf[384];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%" PRIu32 ",\"ServerTransactionID\":%" PRIu32 ",\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", valueJson, getClientID(r), serverTransactionID++);
    sendJson(r, buf);
}
 
void sendAlpacaBool(AsyncWebServerRequest *r, bool value) { sendAlpacaValue(r, value ? "true" : "false"); }
 
void sendAlpacaInt(AsyncWebServerRequest *r, int value) {
    char v[12]; snprintf(v, sizeof(v), "%d", value);
    sendAlpacaValue(r, v);
}
 
// Only for fixed strings without quotes or backslashes
void sendAlpacaString(AsyncWebServerRequest *r, const char* value) {
    char v[64]; snprintf(v, sizeof(v), "\"%s\"", value);
    sendAlpacaValue(r, v);
}
 
// ASCOM: state and motion members must fail with NotConnected while disconnected
bool rejectIfDomeDisconnected(AsyncWebServerRequest *r) {
    if (domeConnected) return false;
    sendAlpacaResult(r, ERR_NOT_CONNECTED, "Not connected");
    return true;
}
 
// PUT Connected: the value is case-insensitive; a missing or invalid value is HTTP 400 (Alpaca spec)
void putConnected(AsyncWebServerRequest *r, bool &connected) {
    String v;
    if (!alpacaArg(r, "Connected", v) || !(v.equalsIgnoreCase("true") || v.equalsIgnoreCase("false"))) {
      r->send(400, "text/plain", "Connected must be true or false");
      return;
    }
    connected = v.equalsIgnoreCase("true");
    sendAlpacaResult(r, 0, "");
}
 
// UI commands: 200 on success, 409 with the rejection reason otherwise
void sendCmdResult(AsyncWebServerRequest *r, const char* err, const char* okMsg) {
  r->send(err ? 409 : 200, "text/plain", err ? err : okMsg);
}
 
// One-use, action-bound, short-lived token prevents bare POST /open or /close.
// This is an accidental-operation guard, NOT authentication or access control.
bool consumeOverrideToken(AsyncWebServerRequest *r, bool forClose) {
  uint32_t token = (uint32_t)strtoul(r->arg("token").c_str(), nullptr, 10);
  bool valid = overrideToken && token == overrideToken && overrideTokenForClose == forClose &&
               millis() - overrideTokenIssuedAt <= OVERRIDE_TOKEN_TTL_MS;
  overrideToken = 0; // consume on every attempt
  return valid;
}
 
// ========= HANDLERS =========
void registerHandlers() {
 
  // 1. WEB UI
  server->on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", INDEX_HTML); });
 
  // 2. INTERNAL STATUS API (Web UI Polling)
  server->on("/status", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();
    root["domeStatus"] = domeStatusStr();
    root["safetyStatus"] = isSystemSafe() ? "SAFE" : "UNSAFE";
    root["safetySafe"] = isSystemSafe();
    root["parkBlocked"] = parkSw.raw;
    root["parkProgressMs"] = (parkSw.raw && !parkSw.active) ? min(PARK_CONFIRM_MS, millis() - parkSw.rawSince) : 0UL;
    root["fault"] = roofErrorReason;
    root["faultLatched"] = roofLost;
    root["roofOpen"] = (bool)roofOpenSw.active;
    root["roofClosed"] = (bool)roofClosedSw.active;
    root["parkConfirmed"] = (bool)parkSw.active;
    root["resetReason"] = resetReasonStr();
    root["uptimeS"] = millis() / 1000;
    resp->setLength(); r->send(resp);
  });
 
  // 3. UI COMMANDS
  server->on("/sopen", HTTP_POST, [](AsyncWebServerRequest *r){ sendCmdResult(r, startRoof(true, true), "Open started"); });
  server->on("/sclose", HTTP_POST, [](AsyncWebServerRequest *r){ sendCmdResult(r, startRoof(false, true), "Close started"); });
  server->on("/override/arm", HTTP_POST, [](AsyncWebServerRequest *r){
    String dir = r->arg("direction");
    if (dir != "open" && dir != "close") { r->send(400, "text/plain", "Unknown direction"); return; }
    overrideToken = esp_random(); if (overrideToken == 0) overrideToken = 1;
    overrideTokenForClose = (dir == "close"); overrideTokenIssuedAt = millis();
    r->send(200, "application/json", String("{\"token\":") + String(overrideToken) + "}");
  });
  server->on("/open", HTTP_POST, [](AsyncWebServerRequest *r){
    if (!consumeOverrideToken(r, false)) { r->send(403, "text/plain", "Override confirmation required"); return; }
    sendCmdResult(r, startRoof(true, false), "Override open started");
  });
  server->on("/close", HTTP_POST, [](AsyncWebServerRequest *r){
    if (!consumeOverrideToken(r, true)) { r->send(403, "text/plain", "Override confirmation required"); return; }
    sendCmdResult(r, startRoof(false, false), "Override close started");
  });
  server->on("/fault/reset", HTTP_POST, [](AsyncWebServerRequest *r){
    bool ok = resetRoofFault();
    r->send(ok ? 200 : 409, "text/plain", ok ? "Fault reset" : "Reset requires stable endpoint and confirmed park");
  });
 
  // 4. SETUP
  server->on("/setup", HTTP_GET, [](AsyncWebServerRequest *r){
    String h = FPSTR(SETUP_HTML);
    h.replace("%TCP%", String(currentAlpacaPort));
    r->send(200, "text/html", h);
  });
  server->on("/setup/save", HTTP_POST, [](AsyncWebServerRequest *r){
    long port = r->hasArg("tcp") ? r->arg("tcp").toInt() : 0;   // toInt() returns 0 for garbage
    if (port < 1 || port > 65535) { r->send(400, "text/plain", "Invalid port (1-65535)"); return; }
    preferences.putInt("alpaca_tcp", (int)port);
    r->send(200, "text/plain", "Saved. Rebooting...");
    // No delay() here: it would block the async_tcp task. loop() restarts after the response is out.
    restartRequestedAt = millis(); restartPending = true;
  });
 
  // 4b. FIRMWARE UPDATE (OTA)
  server->on("/update", HTTP_GET, [](AsyncWebServerRequest *r){
    String h = FPSTR(UPDATE_HTML);
    h.replace("%BUILD%", __DATE__ " " __TIME__);
    r->send(200, "text/html", h);
  });
  server->on("/update", HTTP_POST,
    // Called once the whole upload has been received
    [](AsyncWebServerRequest *r){
      if (otaRejected)        { r->send(409, "text/plain", "Rejected: roof must be closed and mount parked."); return; }
      if (Update.hasError())  { otaInProgress = false; r->send(500, "text/plain", String("Update failed: ") + Update.errorString()); return; }
      r->send(200, "text/plain", "Update OK. Rebooting...");
      restartRequestedAt = millis(); restartPending = true;   // otaInProgress stays set until reboot
    },
    // Called per chunk of the uploaded file
    [](AsyncWebServerRequest *r, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
      if (index == 0) {
        // Only update in the safe rest position: a reboot then cannot interrupt anything
        otaRejected = !(currentShutterState == SHUTTER_CLOSED && !isSlewing && isSystemSafe());
        if (otaRejected) return;
        if (Update.isRunning()) Update.abort();   // leftover from an aborted upload
        otaInProgress = true; otaLastChunkAt = millis();
        Serial.printf("OTA start: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) { Update.printError(Serial); return; }
      }
      if (otaRejected || !Update.isRunning()) return;
      otaLastChunkAt = millis();
      if (Update.write(data, len) != len) { Update.printError(Serial); return; }
      // end(true) verifies the image and only then marks it bootable; on error the old firmware stays active
      if (final && !Update.end(true)) Update.printError(Serial);
    });
 
  // 5. ALPACA MANAGEMENT
  server->on("/management/apiversions", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaValue(r, "[1]"); });
  server->on("/management/v1/description", HTTP_GET, [](AsyncWebServerRequest *r){
    sendAlpacaValue(r, "{\"ServerName\":\"ESP32 Observatory\",\"Manufacturer\":\"ESP32-ALPACA-DOME\",\"ManufacturerVersion\":\"1.0\",\"Location\":\"Observatory\"}");
  });
  server->on("/management/v1/configureddevices", HTTP_GET, [](AsyncWebServerRequest *r){
    sendAlpacaValue(r, "[{\"DeviceName\":\"ESP32 Dome\",\"DeviceType\":\"Dome\",\"DeviceNumber\":0,\"UniqueID\":\"ESP32-DOME\"},"
                        "{\"DeviceName\":\"ESP32 Safety\",\"DeviceType\":\"SafetyMonitor\",\"DeviceNumber\":0,\"UniqueID\":\"ESP32-SAFE\"}]");
  });
 
  // 6. ALPACA DOME (Device 0)
  server->on("/api/v1/dome/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, domeConnected); });
  server->on("/api/v1/dome/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ putConnected(r, domeConnected); });
  server->on("/api/v1/dome/0/name", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "ESP32 Dome"); });
  server->on("/api/v1/dome/0/description", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "Roll-off Roof"); });
  server->on("/api/v1/dome/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "ESP32 Async Driver"); });
  server->on("/api/v1/dome/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "1.0"); });
  server->on("/api/v1/dome/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaInt(r, 2); });
  server->on("/api/v1/dome/0/supportedactions", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaValue(r, "[]"); });
 
  // State members fail with NotConnected while disconnected
  server->on("/api/v1/dome/0/shutterstatus", HTTP_GET, [](AsyncWebServerRequest *r){ if (!rejectIfDomeDisconnected(r)) sendAlpacaInt(r, (int)currentShutterState); });
  server->on("/api/v1/dome/0/slewing", HTTP_GET, [](AsyncWebServerRequest *r){ if (!rejectIfDomeDisconnected(r)) sendAlpacaBool(r, isSlewing); });
  server->on("/api/v1/dome/0/athome", HTTP_GET, [](AsyncWebServerRequest *r){ if (!rejectIfDomeDisconnected(r)) sendAlpacaBool(r, currentShutterState == SHUTTER_CLOSED); });
  server->on("/api/v1/dome/0/atpark", HTTP_GET, [](AsyncWebServerRequest *r){ if (!rejectIfDomeDisconnected(r)) sendAlpacaBool(r, currentShutterState == SHUTTER_CLOSED); });
 
  // Roll-off roof: only the shutter can be controlled
  server->on("/api/v1/dome/0/cansetshutter", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, true); });
  server->on("/api/v1/dome/0/cansetaltitude", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/cansetazimuth", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/cansetpark", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/canfindhome", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/canpark", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/canslave", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/cansyncazimuth", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/slaved", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, false); });
  server->on("/api/v1/dome/0/slaved", HTTP_PUT, [](AsyncWebServerRequest *r){ sendAlpacaResult(r, ERR_NOT_IMPLEMENTED, "Cannot Slave"); });
  server->on("/api/v1/dome/0/altitude", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaResult(r, ERR_NOT_IMPLEMENTED, "Property not implemented"); });
  server->on("/api/v1/dome/0/azimuth", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaResult(r, ERR_NOT_IMPLEMENTED, "Property not implemented"); });
 
  server->on("/api/v1/dome/0/openshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    if (rejectIfDomeDisconnected(r)) return;
    ShutterState s = currentShutterState;
    // Already open/opening: no-op (repeated commands from NINA must not fail)
    if (s == SHUTTER_OPEN || s == SHUTTER_OPENING) { sendAlpacaResult(r, 0, ""); return; }
    const char* err = startRoof(true, true);
    sendAlpacaResult(r, err ? ERR_INVALID_OPERATION : 0, err ? err : "");
  });
 
  server->on("/api/v1/dome/0/closeshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    if (rejectIfDomeDisconnected(r)) return;
    ShutterState s = currentShutterState;
    // Already closed/closing: no-op (repeated commands from NINA must not fail)
    if (s == SHUTTER_CLOSED || s == SHUTTER_CLOSING) { sendAlpacaResult(r, 0, ""); return; }
    // From ERROR this only works if the next pulse is known to close (e.g. after aborting an opening)
    const char* err = startRoof(false, true);
    sendAlpacaResult(r, err ? ERR_INVALID_OPERATION : 0, err ? err : "");
  });
 
  server->on("/api/v1/dome/0/abortslew", HTTP_PUT, [](AsyncWebServerRequest *r){
    if (rejectIfDomeDisconnected(r)) return;
    // Start pulse still active: the stop pulse cannot be sent yet
    if (abortRoof()) sendAlpacaResult(r, 0, "");
    else             sendAlpacaResult(r, ERR_INVALID_OPERATION, "Relay busy, retry");
  });
 
  auto notImpl = [](AsyncWebServerRequest *r){ sendAlpacaResult(r, ERR_NOT_IMPLEMENTED, "Not Implemented"); };
  server->on("/api/v1/dome/0/findhome", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/park", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/setpark", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoaltitude", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoazimuth", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/synctoazimuth", HTTP_PUT, notImpl);
 
  // 7. ALPACA SAFETY (Device 0)
  server->on("/api/v1/safetymonitor/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, safetyConnected); });
  server->on("/api/v1/safetymonitor/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ putConnected(r, safetyConnected); });
  // ASCOM: IsSafe must return false (not an error) while disconnected
  server->on("/api/v1/safetymonitor/0/issafe", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaBool(r, safetyConnected && isSystemSafe()); });
  server->on("/api/v1/safetymonitor/0/name", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "ESP32 Safety"); });
  server->on("/api/v1/safetymonitor/0/description", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "ESP32 Safety Monitor"); });
  server->on("/api/v1/safetymonitor/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "ESP32 Async Driver"); });
  server->on("/api/v1/safetymonitor/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaString(r, "1.0"); });
  server->on("/api/v1/safetymonitor/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){ sendAlpacaInt(r, 1); });
 
  server->onNotFound([](AsyncWebServerRequest *r){ sendAlpacaResult(r, ERR_NOT_IMPLEMENTED, "Not Implemented"); });
}
 
// ========= SETUP & LOOP =========
 
void initWiFi() {
  WiFi.persistent(false);   // credentials come from the sketch, no need to rewrite flash on every boot
  WiFi.setHostname(WIFI_HOSTNAME);   // before mode()/begin(), otherwise the default name is used
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  Serial.print("Connecting to WiFi");
}
 
// Alpaca discovery: answer "alpacadiscovery1" broadcasts with the HTTP port.
// The reply must carry ONLY AlpacaPort; extra fields make NINA drop the device (ESP32-Rolloff).
void startDiscovery() {
#if ENABLE_DISCOVERY
  udp.close();   // re-armed after every WiFi reconnect
  if (!udp.listen(ALPACA_DISCOVERY_PORT)) { Serial.println("Alpaca discovery: listen failed"); return; }
  udp.onPacket([](AsyncUDPPacket packet) {
    if (packet.length() < 16 || strncmp((const char*)packet.data(), "alpacadiscovery1", 16) != 0) return;
    char resp[32];
    snprintf(resp, sizeof(resp), "{\"AlpacaPort\":%d}", currentAlpacaPort);
    packet.print(resp);
  });
#endif
}
 
void handleWiFi() {
  unsigned long now = millis();
 
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiDown) {
      Serial.println("\nWiFi Connected!");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      wifiDown = false;
      startDiscovery();
    }
    return;
  }
  // Boot without a connection counts as down too
  if (!wifiDown) { wifiDown = true; wifiDownSince = now; lastWifiCheck = now; Serial.println("WiFi down"); }
  if (now - lastWifiCheck >= WIFI_RETRY_INTERVAL) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastWifiCheck = now;
  }
  if (now - wifiDownSince >= WIFI_REBOOT_AFTER_MS && rebootSafe()) {
    Serial.println("WiFi down too long, rebooting");
    ESP.restart();
  }
}
 
// Hardware task watchdog for loop(); the TWDT API differs between core 2.x and 3.x
void initWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  // Core 3.x usually initialises the TWDT already: reconfigure in that case
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   // watch the loop task
}
 
void setup() {
  Serial.begin(115200);
  Serial.printf("\nESP32 Dome Controller, build %s %s, reset reason: %s\n", __DATE__, __TIME__, resetReasonStr());
 
  digitalWrite(RELAY_PIN, !RELAY_ACTIVE_LEVEL); // set OFF level before enabling the output
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROOF_OPEN_PIN, INPUT_PULLUP);
  pinMode(ROOF_CLOSED_PIN, INPUT_PULLUP);
  pinMode(PARK_SAFE_PIN, INPUT_PULLUP);
  delay(10); // let pull-ups settle before the first sensor read
 
  // Do not assume CLOSED at boot: the roof may be open after a reset
  initSwitch(roofOpenSw);
  initSwitch(roofClosedSw);
  initSwitch(parkSw);
  portENTER_CRITICAL(&stateMux);
  if (bothEndSwitches()) latchRoofFault("Both roof limit switches active at boot");
  else if (roofOpenSw.active || roofClosedSw.active) setEndPosition(roofOpenSw.active);
  else setPositionUnknown("Roof position unknown at boot", PULSE_UNKNOWN);
  portEXIT_CRITICAL(&stateMux);
 
  preferences.begin("alpaca", false);
  currentAlpacaPort = preferences.getInt("alpaca_tcp", DEFAULT_ALPACA_PORT);
 
  // Initial Connect
  initWiFi();
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) { 
    delay(200); Serial.print("."); 
  }
  Serial.println("");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
 
  server = new AsyncWebServer(currentAlpacaPort);
  registerHandlers();
  server->begin();
  if (WiFi.status() == WL_CONNECTED) startDiscovery();   // otherwise started by handleWiFi() on connect
 
  initWatchdog();   // armed last, so the blocking WiFi wait above cannot trip it
}
 
void loop() {
  esp_task_wdt_reset();
  updateRoofState();
  logStateChanges();
  handleWiFi();
  if (restartPending && (millis() - restartRequestedAt > 1000)) ESP.restart();
}
 
