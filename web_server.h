#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "config.h"

namespace WebServer {
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern bool wifiActive;
extern bool webControlEnabled;

void init();
bool isConnected();
void handleLoop();
void sendTelemetry();
}
