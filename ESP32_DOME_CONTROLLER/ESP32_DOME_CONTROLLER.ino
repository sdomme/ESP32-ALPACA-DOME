/*
  OPTIMIZED ESP32 DOME CONTROLLER (REV 27 - Pure TCP / No Discovery)
  - REMOVED: AsyncUDP (Discovery) to free resources and prevent stack conflicts.
  - OPTIMIZATION: Pure TCP focus for maximum polling speed.
  - CONFIG: Requires manual IP entry in NINA (Discovery disabled).
 
  EXTENSIONS:
  - All pins, relay polarity and timings configurable (config_default.h / config_local.h).
  - Park sensor: reed switch (original) OR IR light barrier. The light barrier is powered
    only for a short check before each move, so its IR light cannot disturb exposures.
  - Optional dedicated "open" input for motor controllers that offer one.
  - Debounced roof contacts, abort with stop impulse, wrong-direction / timeout detection,
    error text returned to Alpaca clients and shown in the Web UI.
*/
 
#include <WiFi.h>
// #include <AsyncUDP.h> // REMOVED: Discovery disabled
#include <ESPAsyncWebServer.h> 
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <mutex>
#include "config.h"   // WiFi, pins, park sensor type, timing
 
#if PARK_SENSOR_TYPE == PARK_SENSOR_BEAM && PARK_POWER_PIN < 0
#error "PARK_SENSOR_BEAM needs PARK_POWER_PIN (use PARK_SENSOR_REED for a permanently powered light barrier)"
#endif
 
#define RELAY_ON   (RELAY_ACTIVE_LEVEL)
#define RELAY_OFF  (!(RELAY_ACTIVE_LEVEL))
// Active-LOW inputs get a pull-up, active-HIGH inputs a pull-down: an open contact is inactive
#define INPUT_MODE(activeLevel)  ((activeLevel) == LOW ? INPUT_PULLUP : INPUT_PULLDOWN)
 
// ========= USER SETTINGS (see config.h) =========
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
 
// ========= STATE =========
enum ShutterState {
  SHUTTER_OPEN = 0,
  SHUTTER_CLOSED = 1,
  SHUTTER_OPENING = 2,
  SHUTTER_CLOSING = 3,
  SHUTTER_ERROR = 4
};
 
ShutterState currentShutterState = SHUTTER_ERROR;  // set from roof contacts in setup()
bool isSlewing = false;
unsigned long lastCommandTime = 0;
bool domeConnected = false;
bool safetyConnected = false;
bool relayActive = false;
int relayPin = RELAY_PIN;              // relay of the running impulse
unsigned long relayOnTime = 0;
unsigned long relayOffTime = 0;        // end of last impulse
bool startedOpen = false;              // roof contacts when the move started
bool startedClosed = false;
bool abortPending = false;
const char* lastError = "";            // shown in Web UI and returned to Alpaca
std::mutex roofMutex;                  // Web/Alpaca handlers run in the AsyncTCP task, not in loop()
 
volatile int serverTransactionID = 0;
unsigned long lastWifiCheck = 0;
bool wifiConnecting = false;
 
int currentAlpacaPort = DEFAULT_ALPACA_PORT;
 
// ========= OBJECTS =========
AsyncWebServer *server = NULL;
// AsyncUDP udp; // REMOVED
Preferences preferences;
 
// ========= HTML (Minified) =========
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8" /><meta name="viewport" content="width=device-width, initial-scale=1" /><title>ESP32 Dome Control</title><style> *, *::before, *::after { box-sizing: border-box; } body, html { margin: 0; padding: 0; height: 100%; background-color: #121212; color: #eee; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Ubuntu, sans-serif; display: flex; justify-content: center; align-items: flex-start; padding: 1rem 0.5rem; user-select: none; } main { width: 100%; max-width: 400px; background: #1c1c1c; border: 1px solid #333; box-shadow: inset 0 0 8px #222, 0 0 16px #0a84ff33; padding: 16px 20px; display: flex; flex-direction: column; gap: 1rem; color: #ddd; } header { text-align: center; } header h1 { font-weight: 600; font-size: 1.4rem; letter-spacing: 0.12em; margin-bottom: 6px; color: #0af; } header p { font-size: 0.8rem; color: #555; margin-top: -4px; letter-spacing: 0.08em; } #domestatus { margin-top: 8px; font-weight: 700; text-transform: uppercase; font-size: 1.2rem; letter-spacing: 0.12em; padding: 6px 0; border-top: 1px solid #333; border-bottom: 1px solid #333; color: #0af; } .status-open { color: #6cf98b; } .status-closed { color: #5c98fc; } .status-moving { color: #ffcc00; } .status-error { color: #ff5252; } section { display: flex; flex-direction: column; gap: 0.6rem; } .sensor { display: flex; justify-content: space-between; font-family: monospace; font-size: 0.95rem; padding: 4px 0; border-bottom: 1px solid #2a2a2a; } .sensor:last-child { border-bottom: none; } .sensor-name { color: #999; text-transform: uppercase; letter-spacing: 0.07em; font-weight: 600; } .sensor-value { font-weight: 700; text-transform: capitalize; } .trig { color: #5ce35c; } .notrig { color: #666; } .error { color: #ff4f4f; } .safe-status { text-align: center; font-weight: 700; text-transform: uppercase; letter-spacing: 0.12em; padding: 6px 0; user-select: none; } .safe-status.safe { color: #6cf98b; border-top: 1px solid #3a7d3a; border-bottom: 1px solid #3a7d3a; } .safe-status.notSafe { color: #ff5252; border-top: 1px solid #7d3a3a; border-bottom: 1px solid #7d3a3a; } button { width: 100%; padding: 14px 12px; background: #222; border: none; color: #ddd; font-size: 1rem; font-weight: 600; letter-spacing: 0.1em; text-transform: uppercase; cursor: pointer; transition: background 0.15s ease; user-select: none; border-left: 3px solid transparent; outline-offset: 2px; } button:focus-visible { outline: 2px solid #0af; outline-offset: 3px; } button:active:not(:disabled) { background: #333; } button:disabled { background: #111; color: #555; cursor: default; border-left: 3px solid #444; } #sopenbtn { border-left-color: #6cf98b; margin-bottom: 10px; } #sclosebtn { border-left-color: #5c98fc; } .override-btn { font-size: 0.84rem; font-weight: 500; padding: 10px 12px; background: #111; color: #999; border-left: 3px solid transparent; } .override-btn:enabled:hover { color: #0af; border-left-color: #0af; background: #191919; } .override-btn:disabled { color: #333; border-left-color: #333; cursor: default; background: #111; } .override-wrap { display: flex; justify-content: space-between; gap: 10px; margin-top: 0.6rem; } .override-wrap div { flex: 1; } .note { font-size: 0.8rem; color: #555; user-select: none; padding-top: 0.6rem; border-top: 1px solid #333; line-height: 1.3; letter-spacing: 0.02em; } </style></head><body><main role="main"><header><h1>ESP32 Dome Control</h1><p>Remote roof controller</p><div id="domestatus" class="status-error">Loading...</div><p id="lasterror"></p></header><section><div id="safetystatus" class="safe-status notSafe">Loading...</div></section><section><button id="sopenbtn" onclick="postCmd('/sopen')">Open Dome</button><button id="sclosebtn" onclick="postCmd('/sclose')">Close Dome</button><button id="stopbtn" onclick="postCmd('/abort')" class="override-btn">Stop</button></section><section class="override-wrap"><div><button id="openbtn" onclick="postCmd('/open')" class="override-btn">Override Open</button></div><div><button id="closebtn" onclick="postCmd('/close')" class="override-btn">Override Close</button></div></section><section><div class="sensor"><div class="sensor-name">Roof Open</div><div id="roofopen" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Roof Closed</div><div id="roofclosed" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Park Sensor</div><div id="parksensor" class="sensor-value error">Loading</div></div></section><div class="note"><strong>Note:</strong> Safe dome open/close buttons require the mount to be parked (park sensor is checked before every move). Override commands ignore the park sensor and work regardless.</div></main><script> function postCmd(url) { fetch(url, { method: 'POST' }) .then(response => { if (!response.ok) response.text().then(t => alert(t || 'Command failed')); }) .catch(error => console.error('Error:', error)); } function updateStatus() { fetch('/status', {cache: 'no-store'}) .then(res => res.json()) .then(data => { const domeStatusEl = document.getElementById('domestatus'); domeStatusEl.textContent = data.domeStatus || 'ERROR'; domeStatusEl.className = 'status-' + ((data.domeStatus || '').toLowerCase()); document.getElementById('lasterror').textContent = data.lastError || ''; const live = !!data.parkLive; const safeEl = document.getElementById('safetystatus'); safeEl.textContent = data.safetyStatus || 'Error'; safeEl.className = (!live || data.safetySafe) ? 'safe-status safe' : 'safe-status notSafe'; const roofOpenEl = document.getElementById('roofopen'); roofOpenEl.textContent = data.roofOpen ? 'Triggered' : 'Not triggered'; roofOpenEl.className = 'sensor-value ' + (data.roofOpen ? 'trig' : 'notrig'); const roofClosedEl = document.getElementById('roofclosed'); roofClosedEl.textContent = data.roofClosed ? 'Triggered' : 'Not triggered'; roofClosedEl.className = 'sensor-value ' + (data.roofClosed ? 'trig' : 'notrig'); const parkEl = document.getElementById('parksensor'); parkEl.textContent = !live ? 'Checked on move' : data.safetySafe ? 'Scope parked' : 'Scope NOT parked'; parkEl.className = 'sensor-value ' + (!live ? 'notrig' : data.safetySafe ? 'trig' : 'error'); const busy = !!data.slewing; const blocked = live && !data.safetySafe; document.getElementById('sopenbtn').disabled = !!data.roofOpen || busy || blocked; document.getElementById('sclosebtn').disabled = !!data.roofClosed || busy || blocked; document.getElementById('openbtn').disabled = !!data.roofOpen || busy; document.getElementById('closebtn').disabled = !!data.roofClosed || busy; document.getElementById('stopbtn').disabled = !busy; }) .catch(() => { document.getElementById('domestatus').textContent = 'ERROR (Conn)'; }); } setInterval(updateStatus, 1000); window.addEventListener('load', updateStatus); </script></body></html>)rawliteral";
 
const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><body><form action="/setup/save" method="POST" style="background:#222;color:#eee;padding:20px;font-family:sans-serif">
<h1>Setup</h1><label>TCP</label><input name="tcp" value="%TCP%"><br><br>
<button>Save & Reboot</button></form></body></html>
)rawliteral";
 
// ========= ROOF CONTACTS =========
// Debounced roof position contact (long cables near a motor pick up EMI)
struct Reed { uint8_t pin; bool active; bool raw; unsigned long changed; };
Reed reedOpen   = { ROOF_OPEN_PIN,   false, false, 0 };
Reed reedClosed = { ROOF_CLOSED_PIN, false, false, 0 };
 
void reedBegin(Reed &r) {
  pinMode(r.pin, INPUT_MODE(ROOF_INPUT_ACTIVE_LEVEL));
  delay(5);
  r.raw = r.active = (digitalRead(r.pin) == ROOF_INPUT_ACTIVE_LEVEL);
  r.changed = millis();
}
 
void reedUpdate(Reed &r, unsigned long now) {
  bool raw = (digitalRead(r.pin) == ROOF_INPUT_ACTIVE_LEVEL);
  if (raw != r.raw) { r.raw = raw; r.changed = now; }
  else if (raw != r.active && now - r.changed >= REED_DEBOUNCE_MS) r.active = raw;
}
 
// ========= PARK SENSOR =========
// PARK_SAFE_PIN at PARK_INPUT_ACTIVE_LEVEL = parked. Reed switch: magnet at park position.
// Light barrier: receiver NC contact, closed while the beam is not interrupted.
 
bool isSystemSafe() { return (digitalRead(PARK_SAFE_PIN) == PARK_INPUT_ACTIVE_LEVEL); }
 
// Debounced read: parked only if 10 samples in a row read active
bool parkContactClosed() {
  for (int i = 0; i < 10; i++) {
    if (!isSystemSafe()) return false;
    delay(10);
  }
  return true;
}
 
// Called before every move. Returns nullptr if parked, otherwise the reason.
// PARK_SENSOR_BEAM blocks the calling handler for ~1.2 s, which Alpaca clients tolerate.
const char* checkPark() {
#if PARK_SENSOR_TYPE == PARK_SENSOR_BEAM
#if PARK_BEAM_SELFTEST
  // Unpowered receiver must report "interrupted", else the signal line is shorted
  if (parkContactClosed()) return "Park sensor fault: light barrier reads clear while unpowered";
#endif
  digitalWrite(PARK_POWER_PIN, RELAY_ON);
  delay(PARK_BEAM_SETTLE_MS);
  bool parked = parkContactClosed();
  digitalWrite(PARK_POWER_PIN, RELAY_OFF);
  return parked ? nullptr : "Mount not parked: light barrier interrupted";
#else
  return parkContactClosed() ? nullptr : "Mount not parked";
#endif
}
 
// ========= UTILS =========
const char* domeStatusStr() {
  switch (currentShutterState) {
    case SHUTTER_OPEN:    return "OPEN";
    case SHUTTER_CLOSED:  return "CLOSED";
    case SHUTTER_OPENING: return "OPENING";
    case SHUTTER_CLOSING: return "CLOSING";
    default:              return "ERROR";
  }
}
 
ShutterState stateFromReeds() {
  if (reedOpen.active && !reedClosed.active) return SHUTTER_OPEN;
  if (reedClosed.active && !reedOpen.active) return SHUTTER_CLOSED;
  return SHUTTER_ERROR;
}
 
void triggerRelay(int pin) {
  relayPin    = pin;
  digitalWrite(pin, RELAY_ON);
  relayActive = true;
  relayOnTime = millis();
}
 
// Caller holds roofMutex
void finishMove(ShutterState s, const char* err) {
  currentShutterState = s;
  isSlewing    = false;
  abortPending = false;
  lastError    = err;
}
 
// ========= STATE LOGIC =========
// Returns nullptr if accepted, otherwise the reason (sent to Alpaca client / Web UI).
// Accepted from any idle state incl. ERROR, so the roof can be recovered remotely.
const char* roofCommand(bool open, bool enforceSafety) {
  {
    std::lock_guard<std::mutex> lock(roofMutex);
    if (isSlewing || relayActive || millis() - relayOffTime < PULSE_GAP_MS) return "Roof is moving - abort first";
    if (reedOpen.active && reedClosed.active) { lastError = "Both roof contacts active"; return lastError; }
    if (open ? reedOpen.active : reedClosed.active) {
      // Already there: no-op (ASCOM), also clears a latched ERROR
      finishMove(open ? SHUTTER_OPEN : SHUTTER_CLOSED, "");
      return nullptr;
    }
  }
  if (enforceSafety) {
    const char* err = checkPark();
    if (err) { std::lock_guard<std::mutex> lock(roofMutex); lastError = err; return err; }
  }
  std::lock_guard<std::mutex> lock(roofMutex);
  startedOpen   = reedOpen.active;
  startedClosed = reedClosed.active;
  triggerRelay((open && OPEN_RELAY_PIN >= 0) ? OPEN_RELAY_PIN : RELAY_PIN);
  currentShutterState = open ? SHUTTER_OPENING : SHUTTER_CLOSING;
  isSlewing = true; lastCommandTime = millis();
  abortPending = false; lastError = "";
  return nullptr;
}
 
const char* roofOpen(bool enforceSafety)  { return roofCommand(true,  enforceSafety); }
const char* roofClose(bool enforceSafety) { return roofCommand(false, enforceSafety); }
 
void roofAbort() {
  std::lock_guard<std::mutex> lock(roofMutex);
  if (isSlewing) abortPending = true;   // stop impulse is sent from loop()
}
 
void updateRoofState() {
  unsigned long now = millis();
  reedUpdate(reedOpen, now);
  reedUpdate(reedClosed, now);
  bool ro = reedOpen.active;
  bool rc = reedClosed.active;
 
  std::lock_guard<std::mutex> lock(roofMutex);
  if (relayActive && now - relayOnTime >= MOTOR_PULSE_MS) {
    digitalWrite(relayPin, RELAY_OFF);
    relayActive  = false;
    relayOffTime = now;
  }
 
  if (isSlewing) {
    bool opening = (currentShutterState == SHUTTER_OPENING);
    if (ro && rc) {
      finishMove(SHUTTER_ERROR, "Both roof contacts active");
    } else if (ro && !startedOpen) {
      // Reached an end position: the wanted one, or the other one (single-button input
      // started from an intermediate position and went the wrong way)
      finishMove(opening ? SHUTTER_OPEN : SHUTTER_ERROR, opening ? "" : "Wrong direction: roof is open");
    } else if (rc && !startedClosed) {
      finishMove(opening ? SHUTTER_ERROR : SHUTTER_CLOSED, opening ? "Wrong direction: roof is closed" : "");
    } else if (abortPending && !relayActive && now - relayOffTime >= PULSE_GAP_MS) {
      triggerRelay(RELAY_PIN);   // impulse while moving = stop
      finishMove(stateFromReeds(), "Aborted");
    } else if (now - lastCommandTime > TRAVEL_TIMEOUT_MS) {
      finishMove(SHUTTER_ERROR, "Timeout: end position not reached");
    }
  } else if (ro && rc) {
    finishMove(SHUTTER_ERROR, "Both roof contacts active");
  } else if ((currentShutterState == SHUTTER_OPEN && !ro) || (currentShutterState == SHUTTER_CLOSED && !rc)) {
    finishMove(SHUTTER_ERROR, "Roof moved without command");
  }
}
 
// ========= ID Helper =========
uint32_t getClientID(AsyncWebServerRequest *r) {
  if(r->hasArg("ClientTransactionID")) return r->arg("ClientTransactionID").toInt();
  if(r->hasArg("ClientID")) return r->arg("ClientID").toInt();
  return 0;
}
 
// ========= TCP Optimization Helper =========
void sendFastResponse(AsyncWebServerRequest *r, const char* jsonStr) {
    AsyncWebServerResponse *response = r->beginResponse(200, "application/json", jsonStr);
    //response->addHeader("Connection", "keep-alive"); // FORCE KEEP ALIVE
    r->send(response);
}
 
// Alpaca method response; errors as InvalidOperation (0x40B)
void sendCommandResult(AsyncWebServerRequest *r, const char* err) {
    char buf[192];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":%d,\"ErrorMessage\":\"%s\"}",
             getClientID(r), serverTransactionID++, err ? 0x40B : 0, err ? err : "");
    sendFastResponse(r, buf);
}
 
// Web UI command response: 409 + reason shows an alert in the browser
void sendUiResult(AsyncWebServerRequest *r, const char* err) {
    if (err) r->send(409, "text/plain", err); else r->send(200);
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
#if PARK_SENSOR_TYPE == PARK_SENSOR_BEAM
    root["parkLive"] = false;                // light barrier is unpowered between moves
    root["safetyStatus"] = "CHECKED ON MOVE";
#else
    root["parkLive"] = true;
    root["safetyStatus"] = isSystemSafe() ? "SAFE" : "UNSAFE";
    root["safetySafe"] = isSystemSafe();
#endif
    root["roofOpen"] = reedOpen.active;
    root["roofClosed"] = reedClosed.active;
    root["slewing"] = isSlewing;
    root["lastError"] = lastError;
    resp->setLength(); r->send(resp);
  });
 
  // 3. UI COMMANDS
  server->on("/sopen", HTTP_POST, [](AsyncWebServerRequest *r){ sendUiResult(r, roofOpen(true)); });
  server->on("/sclose", HTTP_POST, [](AsyncWebServerRequest *r){ sendUiResult(r, roofClose(true)); });
  server->on("/open", HTTP_POST, [](AsyncWebServerRequest *r){ sendUiResult(r, roofOpen(false)); });
  server->on("/close", HTTP_POST, [](AsyncWebServerRequest *r){ sendUiResult(r, roofClose(false)); });
  server->on("/abort", HTTP_POST, [](AsyncWebServerRequest *r){ roofAbort(); r->send(200); });
 
  // 4. SETUP
  server->on("/setup", HTTP_GET, [](AsyncWebServerRequest *r){
    String h = FPSTR(SETUP_HTML);
    h.replace("%TCP%", String(currentAlpacaPort));
    r->send(200, "text/html", h);
  });
  server->on("/setup/save", HTTP_POST, [](AsyncWebServerRequest *r){
    if(r->hasArg("tcp")) {
      preferences.putInt("alpaca_tcp", r->arg("tcp").toInt());
      r->send(200, "text/plain", "Saved. Rebooting...");
      delay(1000); ESP.restart();
    }
  });
 
  // 5. ALPACA MANAGEMENT
  server->on("/management/apiversions", HTTP_GET, [](AsyncWebServerRequest *r){ 
    sendFastResponse(r, "{\"Value\":[1],\"ClientTransactionID\":0,\"ServerTransactionID\":0,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); 
  });
 
  server->on("/management/v1/configureddevices", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();
    JsonArray arr = root.createNestedArray("Value");
    JsonObject d1 = arr.createNestedObject(); d1["DeviceName"]="ESP32 Dome"; d1["DeviceType"]="Dome"; d1["DeviceNumber"]=0; d1["UniqueID"]="ESP32-DOME";
#if PARK_SENSOR_TYPE == PARK_SENSOR_REED
    JsonObject d2 = arr.createNestedObject(); d2["DeviceName"]="ESP32 Safety"; d2["DeviceType"]="SafetyMonitor"; d2["DeviceNumber"]=0; d2["UniqueID"]="ESP32-SAFE";
#endif
    root["ClientTransactionID"] = getClientID(r);
    root["ServerTransactionID"] = serverTransactionID++;
    root["ErrorNumber"]=0; 
    root["ErrorMessage"]=""; 
    resp->setLength(); r->send(resp);
  });
 
  server->on("/management/v1/description", HTTP_GET, [](AsyncWebServerRequest *r){
      char buf[192];
      snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Observatory\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
      sendFastResponse(r, buf);
  });
 
  // 6. ALPACA DOME (Device 0)
 
  server->on("/api/v1/dome/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             domeConnected ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ 
    if(r->hasArg("Connected")) domeConnected = (r->arg("Connected") == "true" || r->arg("Connected") == "True");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/name", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Dome\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/description", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"Roll-off Roof\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Async Driver\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"1.0\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":2,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/shutterstatus", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%d,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", (int)currentShutterState, getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slewing", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", isSlewing?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/altitude", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":0.0,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/azimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":0.0,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/athome", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", currentShutterState==SHUTTER_CLOSED?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/atpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", currentShutterState==SHUTTER_CLOSED?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/cansetshutter", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":true,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slaved", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slaved", HTTP_PUT, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Cannot Slave\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetaltitude", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetazimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canfindhome", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canslave", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansyncazimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  // Park sensor is checked inside roofOpen/roofClose; the client gets the reason as error
  server->on("/api/v1/dome/0/openshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    sendCommandResult(r, roofOpen(true));
  });
 
  server->on("/api/v1/dome/0/closeshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    sendCommandResult(r, roofClose(true));
  });
 
  server->on("/api/v1/dome/0/abortslew", HTTP_PUT, [](AsyncWebServerRequest *r){
      roofAbort();
      sendCommandResult(r, nullptr);
  });
 
  auto notImpl = [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Implemented\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  };
  server->on("/api/v1/dome/0/findhome", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/park", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/setpark", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoaltitude", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoazimuth", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/synctoazimuth", HTTP_PUT, notImpl);
 
  server->on("/api/v1/dome/0/supportedactions", HTTP_GET, [](AsyncWebServerRequest *r){ 
      char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":[],\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
      sendFastResponse(r, buf);
  });
 
  // 7. ALPACA SAFETY (Device 0) - needs a continuously readable park sensor
#if PARK_SENSOR_TYPE == PARK_SENSOR_REED
 
  server->on("/api/v1/safetymonitor/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             safetyConnected ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ 
    if(r->hasArg("Connected")) safetyConnected = (r->arg("Connected") == "true" || r->arg("Connected") == "True");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/safetymonitor/0/issafe", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             isSystemSafe() ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/safetymonitor/0/name", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Safety\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/description", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Safety Monitor\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Async Driver\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"1.0\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":1,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
#endif
 
  server->onNotFound([](AsyncWebServerRequest *r){ 
    char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Implemented\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
}
 
// ========= SETUP & LOOP =========
 
void initWiFi() {
  WiFi.mode(WIFI_STA); 
  WiFi.setSleep(false); 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
}
 
void handleWiFi() {
  unsigned long now = millis();
 
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnecting && (now - lastWifiCheck > WIFI_RETRY_INTERVAL)) {
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      lastWifiCheck = now;
      wifiConnecting = true;
    }
  } else {
    if (wifiConnecting) {
      Serial.println("\nWiFi Connected!");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      wifiConnecting = false;
    }
  }
}
 
void setup() {
  setCpuFrequencyMhz(240); 
  Serial.begin(115200);
 
  // Relays off first, then inputs; initial position from the roof contacts
  const int relays[] = { RELAY_PIN, OPEN_RELAY_PIN, PARK_SENSOR_TYPE == PARK_SENSOR_BEAM ? PARK_POWER_PIN : -1 };
  for (int pin : relays) {
    if (pin < 0) continue;
    digitalWrite(pin, RELAY_OFF);
    pinMode(pin, OUTPUT);
  }
  reedBegin(reedOpen);
  reedBegin(reedClosed);
  pinMode(PARK_SAFE_PIN, INPUT_MODE(PARK_INPUT_ACTIVE_LEVEL));
  currentShutterState = stateFromReeds();
  if (currentShutterState == SHUTTER_ERROR) lastError = "Position unknown after boot";
 
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
}
 
void loop() {
  updateRoofState();
  handleWiFi(); 
 
  // static unsigned long lastDiag = 0;
  // unsigned long now = millis();
  // if (now - lastDiag > 30000) {
  //   lastDiag = now;
  //   Serial.printf("[Diag] WiFi: %d, RSSI: %d, Free heap: %u, IP: %s\n",
  //    WiFi.status(), WiFi.RSSI(), ESP.getFreeHeap(), WiFi.localIP().toString().c_str());
  // }
}
 
 