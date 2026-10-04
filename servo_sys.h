#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

namespace ServoSystem {
struct Control {
  Servo left;
  Servo right;
  static constexpr int centerAngle = 90;
  int lastPosL = centerAngle;
  int lastPosR = centerAngle;
  int32_t filteredL = 90 << Config::SERVO_EMA_ALPHA_SHIFT;
  int32_t filteredR = 90 << Config::SERVO_EMA_ALPHA_SHIFT;
  bool isAttached = false;
};

extern Control state;

void attachAll();
void detachAll();
void update(int targetL, int targetR, bool bypassFilter = false);
}
