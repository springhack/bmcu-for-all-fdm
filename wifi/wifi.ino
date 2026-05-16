#include <Arduino.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {

constexpr int kBmcuRxPin = A3;
constexpr int kBmcuTxPin = A4;
constexpr int kActionLedPin = 8;
constexpr bool kActionLedActiveLow = true;
constexpr uint32_t kBmcuBaudRate = 115200;
constexpr size_t kMaxLineLength = 512;
constexpr uint32_t kWifiConnectTimeoutMs = 15000;
constexpr uint32_t kActionLedBlinkMs = 250;
constexpr uint32_t kStateRequestIntervalMs = 100;
constexpr char kHostname[] = "bmcu";
constexpr char kWifiNamespace[] = "wifi";
constexpr char kWifiSsidKey[] = "ssid";
constexpr char kWifiPassKey[] = "pass";

WebServer server(80);
HardwareSerial bmcuSerial(1);
Preferences preferences;
String usbLineBuffer;
String bmcuLineBuffer;
String wifiSsid;
String wifiPassword;
String bmcuStateJson = "{\"loaded\":-1,\"channels\":[{\"inserted\":0,\"buffer\":\"IDLE\",\"sw1\":0,\"sw2\":0,\"status\":\"#000000\",\"online\":\"#000000\"},{\"inserted\":0,\"buffer\":\"IDLE\",\"sw1\":0,\"sw2\":0,\"status\":\"#000000\",\"online\":\"#000000\"},{\"inserted\":0,\"buffer\":\"IDLE\",\"sw1\":0,\"sw2\":0,\"status\":\"#000000\",\"online\":\"#000000\"},{\"inserted\":0,\"buffer\":\"IDLE\",\"sw1\":0,\"sw2\":0,\"status\":\"#000000\",\"online\":\"#000000\"}]}";
bool bmcuBusy = false;
bool serverStarted = false;
bool mdnsStarted = false;
uint32_t lastStateMs = 0;
uint32_t lastActionLedToggleMs = 0;
uint32_t lastStateRequestMs = 0;
bool actionLedOn = true;

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UFB</title>
<style>
:root{color-scheme:light dark;--bg:#f6f7f9;--fg:#15171a;--muted:#6b7280;--line:#dfe3e8;--panel:#ffffff;--soft:#eef1f5;--ok:#0f9f6e;--accent:#2563eb;--danger:#be185d;--shadow:0 16px 40px rgba(16,24,40,.08)}
@media(prefers-color-scheme:dark){:root{--bg:#0f1115;--fg:#eef2f6;--muted:#9aa4b2;--line:#2a3039;--panel:#171b22;--soft:#202633;--ok:#34d399;--accent:#60a5fa;--danger:#f472b6;--shadow:0 16px 40px rgba(0,0,0,.35)}}
*{box-sizing:border-box}body{min-height:100svh;margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,"Liberation Mono",monospace;letter-spacing:0;display:grid;place-items:center}
.wrap{width:min(980px,100%);padding:24px 16px}
header{display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:18px}
h1{margin:0;font-size:28px;font-weight:720}.sub{color:var(--muted);font-size:13px;margin-top:2px}.pill{border:1px solid var(--line);border-radius:999px;padding:7px 11px;background:var(--panel);color:var(--muted);white-space:nowrap}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;box-shadow:var(--shadow);padding:14px;min-width:0}
.top{display:flex;justify-content:space-between;align-items:flex-start;gap:8px;margin-bottom:16px}.name{font-size:18px;font-weight:700}.state{height:18px;font-size:12px;color:var(--muted);margin-top:2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.mode{width:50px;text-align:center;font-variant-numeric:tabular-nums;font-size:12px;color:var(--muted);background:var(--soft);border-radius:999px;padding:4px 0;white-space:nowrap}
.mode.push{color:var(--accent)}.mode.pull{color:var(--danger)}.mode.idle{color:var(--ok)}
.lights{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}.light{border:1px solid var(--line);border-radius:8px;padding:9px;background:var(--bg)}.label{font-size:11px;color:var(--muted);margin-bottom:7px}.sw{height:28px;border-radius:6px;border:1px solid rgba(0,0,0,.16);box-shadow:inset 0 0 0 1px rgba(255,255,255,.18)}
.switches{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:14px}.switch{height:30px;display:grid;place-items:center}.toggle{position:relative;width:58px;height:26px;border-radius:999px;background:var(--soft);border:1px solid var(--line);overflow:hidden;transition:background .16s,border-color .16s}.knob{position:absolute;left:2px;top:2px;width:20px;height:20px;border-radius:50%;background:var(--panel);box-shadow:0 1px 4px rgba(0,0,0,.28);transition:transform .16s}.switch.on .toggle{background:color-mix(in srgb,var(--ok) 34%,var(--soft));border-color:color-mix(in srgb,var(--ok) 55%,var(--line))}.switch.on .knob{transform:translateX(32px)}
.actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}button{appearance:none;border:0;border-radius:8px;padding:11px 10px;color:white;font:inherit;font-weight:700;cursor:pointer;background:var(--accent);min-width:0}button.out{background:var(--danger)}button:disabled{cursor:not-allowed;filter:saturate(.35);opacity:.55}
.toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:var(--fg);color:var(--bg);border-radius:999px;padding:9px 14px;opacity:0;pointer-events:none;transition:opacity .18s,transform .18s;font-size:13px}.toast.show{opacity:1;transform:translateX(-50%) translateY(-4px)}
@media(max-width:780px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}h1{font-size:24px}}
@media(max-width:460px){body{display:block}.wrap{padding:18px 12px 28px}.grid{grid-template-columns:1fr}header{align-items:flex-start}.pill{font-size:12px}}
</style>
</head>
<body>
<div class="wrap">
  <header><div><h1>UFB</h1><div class="sub">Universal Filament Buffer base on BMCU</div></div><div id="status" class="pill">INIT</div></header>
  <main id="grid" class="grid"></main>
</div>
<div id="toast" class="toast"></div>
<script>
const grid=document.getElementById('grid'),statusEl=document.getElementById('status'),toast=document.getElementById('toast');
let busy=false,loaded=-1;
const names=['Channel 1','Channel 2','Channel 3','Channel 4'];
function showToast(t){toast.textContent=t;toast.classList.add('show');setTimeout(()=>toast.classList.remove('show'),1400)}
function bufferMode(value){
  const text=(value||'IDLE').toString().toUpperCase();
  if(text==='PUSH')return {text:'PUSH',cls:'push'};
  if(text==='PULL')return {text:'PULL',cls:'pull'};
  return {text:'IDLE',cls:'idle'};
}
function card(i,c){
  const inserted=!!c.inserted, active=loaded===i;
  const mode=bufferMode(c.buffer);
  return `<section class="card"><div class="top"><div><div class="name">${names[i]}</div><div class="state">${inserted?'Connected':'Not connected'}${active?' · Loaded':''}</div></div><div class="mode ${mode.cls}">${mode.text}</div></div>
  <div class="lights"><div class="light"><div class="label">Online LED</div><div class="sw" style="background:${c.online||'#000'}"></div></div><div class="light"><div class="label">Status LED</div><div class="sw" style="background:${c.status||'#000'}"></div></div></div>
  <div class="switches"><div class="switch ${c.sw1?'on':''}"><span class="toggle"><span class="knob"></span></span></div><div class="switch ${c.sw2?'on':''}"><span class="toggle"><span class="knob"></span></span></div></div>
  <div class="actions"><button ${busy?'disabled':''} onclick="act('input',${i})">Load</button><button class="out" ${busy?'disabled':''} onclick="act('output',${i})">Unload</button></div></section>`;
}
async function refresh(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});
    const s=await r.json();
    busy=!!s.busy;loaded=s.loaded??-1;
    statusEl.textContent=s.stale?'WAIT':(busy?'BUSY':'IDLE');
    grid.innerHTML=(s.channels||[]).map((c,i)=>card(i,c)).join('');
  }catch(e){statusEl.textContent='OFFL'}
}
async function act(cmd,ch){
  try{
    const r=await fetch(`/api/action?cmd=${cmd}&ch=${ch}`,{method:'POST'});
    const t=(await r.text()).trim();
    showToast(t==='OK'?'Sent':t);
    refresh();
  }catch(e){showToast('Request failed')}
}
refresh();setInterval(refresh,900);
</script>
</body>
</html>
)HTML";

bool parseChannelId(const String& value, uint8_t& channelId) {
  if (value.length() != 1) return false;
  const char ch = value.charAt(0);
  if (ch < '0' || ch > '3') return false;
  channelId = static_cast<uint8_t>(ch - '0');
  return true;
}

bool extractChannelId(const String& uri, const char* prefix, uint8_t& channelId) {
  const String prefixString(prefix);
  if (!uri.startsWith(prefixString)) return false;
  const String suffix = uri.substring(prefixString.length());
  if (suffix.indexOf('/') >= 0) return false;
  return parseChannelId(suffix, channelId);
}

String normalizeCommand(String command) {
  command.trim();
  command.toUpperCase();
  return command;
}

String extractCommandKeyword(const String& line) {
  const int spaceIndex = line.indexOf(' ');
  return normalizeCommand(spaceIndex < 0 ? line : line.substring(0, spaceIndex));
}

bool parseBmcuActionCommand(const String& line, String& command, uint8_t& channelId) {
  const int spaceIndex = line.indexOf(' ');
  if (spaceIndex <= 0) return false;
  const String verb = normalizeCommand(line.substring(0, spaceIndex));
  if (verb != "INPUT" && verb != "OUTPUT") return false;
  if (!parseChannelId(line.substring(spaceIndex + 1), channelId)) return false;
  command = verb;
  return true;
}

bool parseWifiCommand(const String& line, String& ssid, String& password) {
  const int spaceIndex = line.indexOf(' ');
  if (spaceIndex <= 0 || extractCommandKeyword(line) != "WIFI") return false;
  const String credentials = line.substring(spaceIndex + 1);
  const int separatorIndex = credentials.indexOf('/');
  if (separatorIndex <= 0 || separatorIndex >= credentials.length() - 1) return false;
  ssid = credentials.substring(0, separatorIndex);
  password = credentials.substring(separatorIndex + 1);
  ssid.trim();
  password.trim();
  return !ssid.isEmpty() && !password.isEmpty();
}

void sendPlainText(const char* body) {
  server.send(200, "text/plain; charset=utf-8", body);
}

void writeActionLed(bool on) {
  actionLedOn = on;
  digitalWrite(kActionLedPin, (kActionLedActiveLow ? !on : on) ? HIGH : LOW);
}

void initActionLed() {
  pinMode(kActionLedPin, OUTPUT);
  writeActionLed(true);
  lastActionLedToggleMs = millis();
}

void updateActionLed() {
  const uint32_t nowMs = millis();
  if (!bmcuBusy) {
    if (!actionLedOn) writeActionLed(true);
    lastActionLedToggleMs = nowMs;
    return;
  }

  if (nowMs - lastActionLedToggleMs >= kActionLedBlinkMs) {
    writeActionLed(!actionLedOn);
    lastActionLedToggleMs = nowMs;
  }
}

void consumeBmcuLine(String line) {
  line.trim();
  if (line.isEmpty()) return;

  if (line == "DONE") {
    bmcuBusy = false;
    return;
  }

  if (line.startsWith("STATE ")) {
    bmcuStateJson = line.substring(6);
    lastStateMs = millis();
  }
}

void pollBmcuResponses() {
  while (bmcuSerial.available() > 0) {
    const char ch = static_cast<char>(bmcuSerial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!bmcuLineBuffer.isEmpty()) {
        const String line = bmcuLineBuffer;
        bmcuLineBuffer = "";
        consumeBmcuLine(line);
      }
      continue;
    }
    if (bmcuLineBuffer.length() < kMaxLineLength) bmcuLineBuffer += ch;
  }
}

void startBmcuAction(const String& command, uint8_t channelId) {
  bmcuSerial.printf("%s %u\n", command.c_str(), channelId);
  bmcuBusy = true;
}

void requestBmcuState() {
  const uint32_t nowMs = millis();
  if (nowMs - lastStateRequestMs < kStateRequestIntervalMs) return;
  lastStateRequestMs = nowMs;
  bmcuSerial.print("STATE\n");
}

void handleIndex() {
  server.send_P(200, "text/html; charset=utf-8", kIndexHtml);
}

void handleApiState() {
  pollBmcuResponses();
  String body = "{\"busy\":";
  body += bmcuBusy ? "true" : "false";
  body += ",\"stale\":";
  body += (lastStateMs == 0 || millis() - lastStateMs > 3000) ? "true" : "false";
  body += ",";
  body += bmcuStateJson.substring(1);
  server.send(200, "application/json; charset=utf-8", body);
}

void handleApiAction() {
  if (server.method() != HTTP_POST) {
    sendPlainText("METHOD_NOT_ALLOWED");
    return;
  }

  const String cmd = normalizeCommand(server.arg("cmd"));
  uint8_t channelId = 0;
  if ((cmd != "INPUT" && cmd != "OUTPUT") || !parseChannelId(server.arg("ch"), channelId)) {
    sendPlainText("INVALID_ACTION");
    return;
  }

  if (bmcuBusy) {
    sendPlainText("BUSY");
    return;
  }

  startBmcuAction(cmd, channelId);
  sendPlainText("OK");
}

void handleActionRequest(const char* command, const char* prefix) {
  uint8_t channelId = 0;
  if (!extractChannelId(server.uri(), prefix, channelId)) {
    sendPlainText("INVALID_CHANNEL");
    return;
  }
  if (bmcuBusy) {
    sendPlainText("BUSY");
    return;
  }
  startBmcuAction(command, channelId);
  sendPlainText("OK");
}

void handleDynamicRequest() {
  if (server.method() != HTTP_GET) {
    sendPlainText("METHOD_NOT_ALLOWED");
    return;
  }
  const String uri = server.uri();
  if (uri.startsWith("/input/")) {
    handleActionRequest("INPUT", "/input/");
    return;
  }
  if (uri.startsWith("/output/")) {
    handleActionRequest("OUTPUT", "/output/");
    return;
  }
  sendPlainText("NOT_FOUND");
}

void startMdns() {
  if (mdnsStarted) return;
  if (MDNS.begin(kHostname)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.println("mDNS started: bmcu.local");
  } else {
    Serial.println("mDNS start failed");
  }
}

void ensureServerStarted() {
  if (serverStarted) return;
  server.on("/", HTTP_GET, handleIndex);
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/action", HTTP_POST, handleApiAction);
  server.on("/state", HTTP_GET, handleApiState);
  server.onNotFound(handleDynamicRequest);
  server.begin();
  serverStarted = true;
  Serial.println("HTTP server started");
}

bool loadWifiConfig() {
  wifiSsid = preferences.getString(kWifiSsidKey, "");
  wifiPassword = preferences.getString(kWifiPassKey, "");
  return !wifiSsid.isEmpty() && !wifiPassword.isEmpty();
}

void saveWifiConfig(const String& ssid, const String& password) {
  preferences.putString(kWifiSsidKey, ssid);
  preferences.putString(kWifiPassKey, password);
  wifiSsid = ssid;
  wifiPassword = password;
}

bool connectWifi(const String& ssid, const String& password) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(kHostname);
  WiFi.setAutoReconnect(true);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.printf("Connecting to Wi-Fi SSID: %s\n", ssid.c_str());
  const uint32_t startMs = millis();
  while (millis() - startMs < kWifiConnectTimeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      ensureServerStarted();
      startMdns();
      Serial.print("Wi-Fi connected. IP: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(250);
  }

  Serial.println("Wi-Fi connect timeout");
  return false;
}

void connectWifiIfConfigured() {
  if (!loadWifiConfig()) {
    Serial.println("No saved Wi-Fi config");
    return;
  }
  connectWifi(wifiSsid, wifiPassword);
}

void handleUsbCommand(const String& rawLine) {
  String line = rawLine;
  line.trim();
  if (line.isEmpty()) return;

  const String keyword = extractCommandKeyword(line);
  String command;
  uint8_t channelId = 0;
  if (parseBmcuActionCommand(line, command, channelId)) {
    if (bmcuBusy) {
      Serial.println("BUSY");
      return;
    }
    startBmcuAction(command, channelId);
    return;
  }

  if (keyword == "STATE") {
    pollBmcuResponses();
    Serial.println(bmcuStateJson);
    return;
  }

  String ssid;
  String password;
  if (parseWifiCommand(line, ssid, password)) {
    saveWifiConfig(ssid, password);
    const bool connected = connectWifi(ssid, password);
    Serial.println(connected ? "WIFI_SAVED" : "WIFI_SAVED_CONNECT_FAILED");
    return;
  }

  if (keyword == "INPUT" || keyword == "OUTPUT") {
    Serial.println("INVALID_BMCU_COMMAND");
    return;
  }
  if (keyword == "WIFI") {
    Serial.println("INVALID_WIFI_COMMAND");
    return;
  }
  Serial.println("UNKNOWN_COMMAND");
}

void pollUsbCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!usbLineBuffer.isEmpty()) {
        const String line = usbLineBuffer;
        usbLineBuffer = "";
        handleUsbCommand(line);
      }
      continue;
    }
    if (usbLineBuffer.length() < kMaxLineLength) usbLineBuffer += ch;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 3000) {
    delay(10);
  }
  delay(100);
  initActionLed();

  bmcuSerial.begin(kBmcuBaudRate, SERIAL_8N1, kBmcuRxPin, kBmcuTxPin);
  Serial.println("BMCU UART initialized");

  preferences.begin(kWifiNamespace, false);
  connectWifiIfConfigured();
}

void loop() {
  pollUsbCommands();
  requestBmcuState();
  pollBmcuResponses();
  if (serverStarted) server.handleClient();
  updateActionLed();
}
