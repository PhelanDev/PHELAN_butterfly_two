#include "config.h"
#include "flight.h"
#include "servo_sys.h"
#include "web_server.h"

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 1500)) {
    delay(10);
  }
  delay(100);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pinMode(Config::LED_PIN, OUTPUT);
  digitalWrite(Config::LED_PIN, HIGH);

  ServoSystem::state.lastPosL = ServoSystem::state.centerAngle;
  ServoSystem::state.lastPosR = ServoSystem::state.centerAngle;

  WebServer::init();

  xTaskCreatePinnedToCore(flightCoreTask, "flightCore", 4096, NULL, 5, NULL, 0);
}

void loop() {
  unsigned long now = millis();

  WebServer::handleLoop();

  if (!WebServer::isConnected()) {
    digitalWrite(Config::LED_PIN, (now / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(Config::LED_PIN, Flight::state.isArmed
                                      ? HIGH
                                      : ((now / 1000) % 2 == 0 ? HIGH : LOW));
  }

  delay(2);
}