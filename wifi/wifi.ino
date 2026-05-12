#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {

constexpr int kBmcuRxPin = 16;
constexpr int kBmcuTxPin = 17;
constexpr uint32_t kBmcuBaudRate = 115200;
constexpr size_t kMaxBmcuLineLength = 128;
constexpr uint32_t kWifiConnectTimeoutMs = 15000;
constexpr char kWifiNamespace[] = "wifi";
constexpr char kWifiSsidKey[] = "ssid";
constexpr char kWifiPassKey[] = "pass";

WebServer server(80);
Preferences preferences;
String usbLineBuffer;
String bmcuLineBuffer;
String wifiSsid;
String wifiPassword;
bool bmcuBusy = false;
bool serverStarted = false;

bool parseChannelId(const String& value, uint8_t& channelId) {
  if (value.length() != 1) {
    return false;
  }

  const char ch = value.charAt(0);
  if (ch < '0' || ch > '3') {
    return false;
  }

  channelId = static_cast<uint8_t>(ch - '0');
  return true;
}

bool extractChannelId(const String& uri, const char* prefix, uint8_t& channelId) {
  const String prefixString(prefix);
  if (!uri.startsWith(prefixString)) {
    return false;
  }

  const String suffix = uri.substring(prefixString.length());
  if (suffix.indexOf('/') >= 0) {
    return false;
  }

  return parseChannelId(suffix, channelId);
}

String normalizeCommand(String command) {
  command.trim();
  command.toUpperCase();
  return command;
}

String extractCommandKeyword(const String& line) {
  const int spaceIndex = line.indexOf(' ');
  if (spaceIndex < 0) {
    return normalizeCommand(line);
  }

  return normalizeCommand(line.substring(0, spaceIndex));
}

bool parseBmcuActionCommand(const String& line, String& command, uint8_t& channelId) {
  const int spaceIndex = line.indexOf(' ');
  if (spaceIndex <= 0) {
    return false;
  }

  const String verb = normalizeCommand(line.substring(0, spaceIndex));
  if (verb != "INPUT" && verb != "OUTPUT") {
    return false;
  }

  const String channelText = line.substring(spaceIndex + 1);
  if (!parseChannelId(channelText, channelId)) {
    return false;
  }

  command = verb;
  return true;
}

bool parseWifiCommand(const String& line, String& ssid, String& password) {
  const int spaceIndex = line.indexOf(' ');
  if (spaceIndex <= 0 || extractCommandKeyword(line) != "WIFI") {
    return false;
  }

  const String credentials = line.substring(spaceIndex + 1);
  const int separatorIndex = credentials.indexOf('/');
  if (separatorIndex <= 0 || separatorIndex >= credentials.length() - 1) {
    return false;
  }

  ssid = credentials.substring(0, separatorIndex);
  password = credentials.substring(separatorIndex + 1);
  ssid.trim();
  password.trim();
  return !ssid.isEmpty() && !password.isEmpty();
}

void sendPlainText(const char* body) {
  server.send(200, "text/plain; charset=utf-8", body);
}

void startBmcuAction(const String& command, uint8_t channelId) {
  Serial2.printf("%s %u\n", command.c_str(), channelId);
  bmcuBusy = true;
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

void handleStateRequest() {
  sendPlainText(bmcuBusy ? "BUSY" : "IDLE");
}

void ensureServerStarted() {
  if (serverStarted) {
    return;
  }

  server.on("/state", HTTP_GET, handleStateRequest);
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
  WiFi.setAutoReconnect(true);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.printf("Connecting to Wi-Fi SSID: %s\n", ssid.c_str());
  const uint32_t startMs = millis();
  while (millis() - startMs < kWifiConnectTimeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      ensureServerStarted();
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
  if (line.isEmpty()) {
    return;
  }

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

  String ssid;
  String password;
  if (parseWifiCommand(line, ssid, password)) {
    saveWifiConfig(ssid, password);
    const bool connected = connectWifi(ssid, password);
    if (connected) {
      Serial.println("WIFI_SAVED");
    } else {
      Serial.println("WIFI_SAVED_CONNECT_FAILED");
    }
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

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      if (usbLineBuffer.isEmpty()) {
        continue;
      }

      const String line = usbLineBuffer;
      usbLineBuffer = "";
      handleUsbCommand(line);
      continue;
    }

    if (usbLineBuffer.length() < kMaxBmcuLineLength) {
      usbLineBuffer += ch;
    }
  }
}

// Read line-based BMCU responses, forward them to USB serial, and clear busy on DONE.
void pollBmcuResponses() {
  while (Serial2.available() > 0) {
    const char ch = static_cast<char>(Serial2.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      if (bmcuLineBuffer.isEmpty()) {
        continue;
      }

      String line = bmcuLineBuffer;
      bmcuLineBuffer = "";
      line.trim();

      Serial.println(line);
      if (line == "DONE") {
        bmcuBusy = false;
      }
      continue;
    }

    if (bmcuLineBuffer.length() < kMaxBmcuLineLength) {
      bmcuLineBuffer += ch;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.begin(kBmcuBaudRate, SERIAL_8N1, kBmcuRxPin, kBmcuTxPin);
  Serial.println("UART2 initialized");

  preferences.begin(kWifiNamespace, false);
  connectWifiIfConfigured();
}

void loop() {
  pollUsbCommands();
  if (serverStarted) {
    server.handleClient();
  }
  pollBmcuResponses();
}
