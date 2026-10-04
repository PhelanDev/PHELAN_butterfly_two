#include "web_server.h"
#include "flight.h"
#include "servo_sys.h"
#include "web_ui.h"
#include <ESPmDNS.h>
#include <WiFi.h>

namespace WebServer {
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool wifiActive = false;
bool webControlEnabled = false;
volatile unsigned long lastWsPacketTime = 0;
unsigned long lastTelemSendTime = 0;
volatile int8_t wsArmRequest = 0;
constexpr unsigned long WS_FAILSAFE_MS = 2000;
constexpr unsigned long TELEM_INTERVAL_MS = 200;

static int16_t webChannels[16] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                                  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (client->client())
      client->client()->setNoDelay(true);
  } else if (type == WS_EVT_DISCONNECT) {
    if (ws.count() == 0)
      webControlEnabled = false;
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == sizeof(FlightPacket) &&
        data[0] == 0xAA) {
      FlightPacket *pkt = (FlightPacket *)data;
      lastWsPacketTime = millis();

      webChannels[Config::CH_THROTTLE] =
          constrain(1000 + (int16_t)pkt->throttle * 10, 1000, 2000);
      webChannels[Config::CH_YAW] =
          constrain(1500 + (int16_t)pkt->yaw * 5, 1000, 2000);
      webChannels[Config::CH_ELEVATOR] =
          constrain(1500 + (int16_t)pkt->pitch * 5, 1000, 2000);
      webChannels[Config::CH_STEERING] =
          constrain(1500 + (int16_t)pkt->roll * 5, 1000, 2000);

      wsArmRequest = (pkt->flags & 0x01) ? 1 : 0;
      webChannels[Config::CH_KILL_SWITCH] = (pkt->flags & 0x02) ? 1900 : 1000;

      if (pkt->flags & 0x04) {
        webChannels[Config::CH_ELEVATOR] = 1100;
      }

      webControlEnabled = true;
    } else if (info->opcode == WS_TEXT && len >= 4 &&
               memcmp(data, "PING", 4) == 0) {
      client->text("PONG");
    }
  }
}

void sendTelemetry() {
  if (ws.count() == 0)
    return;

  TelemetryPacket tpkt;
  tpkt.header = 0xBB;
  uint8_t modeId = 0;
  switch (Flight::state.mode) {
  case FlightMode::CONNECTED:
    modeId = 1;
    break;
  case FlightMode::PERCHING:
    modeId = 2;
    break;
  case FlightMode::FAILSAFE:
    modeId = 3;
    break;
  default:
    modeId = 0;
    break;
  }
  tpkt.flags = (Flight::state.isArmed ? 1 : 0) | (modeId << 1);
  tpkt.offset = (int8_t)constrain((int)Flight::state.currentOffset, -127, 127);
  tpkt.servoL = (uint8_t)constrain(ServoSystem::state.lastPosL, 0, 255);
  tpkt.servoR = (uint8_t)constrain(ServoSystem::state.lastPosR, 0, 255);
  tpkt.cpuTemp = 42;
  tpkt.cpuMhz = 160;
  tpkt.uptimeSec = (uint16_t)(millis() / 1000);
  tpkt.timeH = 0;
  tpkt.timeM = 0;
  tpkt.timeS = 0;

  ws.binaryAll((uint8_t *)&tpkt, sizeof(tpkt));
}

bool isConnected() {
  unsigned long now = millis();
  bool linkLost =
      (lastWsPacketTime > 0 && (now - lastWsPacketTime > WS_FAILSAFE_MS));
  return !linkLost && webControlEnabled;
}

void init() {
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.printf("[BOOT] MAC: %s | SSID: %s\n", WiFi.macAddress().c_str(),
                Config::WIFI_STA_SSID);

  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("[RF SCAN] Scanning for: %s ...\n", Config::WIFI_STA_SSID);

  int n = WiFi.scanNetworks(false, false, false, 300);
  int bestChannel = 0;
  uint8_t bestBSSID[6] = {0};
  int bestRSSI = -999;
  bool foundRouter = false;
  String bestSSID = Config::WIFI_STA_SSID;
  int countCh[14] = {0};

  for (int i = 0; i < n; i++) {
    int ch = WiFi.channel(i);
    if (ch >= 1 && ch <= 13)
      countCh[ch]++;
    String curSsid = WiFi.SSID(i);
    if (curSsid == Config::WIFI_STA_SSID || curSsid == "Phong12-5G" ||
        curSsid == "Phong12") {
      if (WiFi.RSSI(i) > bestRSSI) {
        bestRSSI = WiFi.RSSI(i);
        bestChannel = ch;
        bestSSID = curSsid;
        memcpy(bestBSSID, WiFi.BSSID(i), 6);
        foundRouter = true;
      }
    }
  }

  Serial.printf("[RF AUDIT] Ch 1: %d | Ch 6: %d | Ch 11: %d\n", countCh[1],
                countCh[6], countCh[11]);

  if (foundRouter) {
    Serial.printf("[ROUTER] Found %s on Ch %d (%d dBm)\n", bestSSID.c_str(),
                  bestChannel, bestRSSI);
    WiFi.begin(bestSSID.c_str(), Config::WIFI_STA_PASS, bestChannel, bestBSSID);
  } else {
    WiFi.begin(Config::WIFI_STA_SSID, Config::WIFI_STA_PASS);
  }
  WiFi.scanDelete();

  WiFi.setAutoReconnect(true);

  Serial.print("[WiFi] Connecting ");
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
    if (millis() - startWifi > 10000) {
      Serial.printf("\n[WiFi] Retry %s (MAC: %s)...\n[WiFi] Connecting ",
                    Config::WIFI_STA_SSID, WiFi.macAddress().c_str());
      startWifi = millis();
      WiFi.begin(Config::WIFI_STA_SSID, Config::WIFI_STA_PASS);
    }
  }

  digitalWrite(Config::LED_PIN, LOW);
  Serial.printf("\n[WiFi] Connected! IP: http://%s | MAC: %s | Ch: %d\n",
                WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str(),
                WiFi.channel());

  if (MDNS.begin(Config::MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] http://%s.local\n", Config::MDNS_HOST);
  } else {
    Serial.println("[mDNS] Failed to start");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  wifiActive = true;
}

void handleLoop() {
  unsigned long now = millis();

  bool linkLost =
      (lastWsPacketTime > 0 && (now - lastWsPacketTime > WS_FAILSAFE_MS));
  if (linkLost && webControlEnabled) {
    webControlEnabled = false;
    wsArmRequest = 0;
  }

  bool snapshotConnected = !linkLost && webControlEnabled;
  bool killActive =
      webChannels[Config::CH_KILL_SWITCH] > Config::KILL_SWITCH_THRESHOLD;
  bool armRequested = (wsArmRequest == 1);

  uint8_t flags = 0;
  if (snapshotConnected)
    flags |= 0x01;
  if (killActive)
    flags |= 0x02;
  if (armRequested)
    flags |= 0x04;

  publishControlSnapshot(
      webChannels[Config::CH_STEERING], webChannels[Config::CH_ELEVATOR],
      webChannels[Config::CH_THROTTLE], webChannels[Config::CH_YAW], flags);

  if (now - lastTelemSendTime >= TELEM_INTERVAL_MS) {
    lastTelemSendTime = now;
    sendTelemetry();
    ws.cleanupClients();
  }
}
}
