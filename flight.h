#pragma once

#include <Arduino.h>
#include <atomic>
#include "config.h"

namespace Flight {
struct State {
  FlightMode mode = FlightMode::INIT;
  uint16_t phase = 0;
  float currentOffset = 0.0f;
  bool isPerching = false;
  int perchAngleLevel = Config::CH_MIN;
  bool phaseNeedsResync = true;
  bool isFlapUp = true;
  unsigned long failsafeEnterMs = 0;
  bool wasFlapping = false;
  bool wasPerching = false;
  bool isFlapping = false;
  bool isArmed = false;
};

extern State state;

void updateCurrentOffsetTriangle(int throttle, unsigned long dt_us);
void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us);
void updateCurrentOffsetGlide(unsigned long dt_us);
void processFailsafe(unsigned long dt_us);
}

void publishControlSnapshot(int16_t roll, int16_t pitch, int16_t throttle,
                            int16_t yaw, uint8_t flags);
ControlSnapshot readControlSnapshot();

void processFlightLogic(unsigned long dt_us, int steering, int elevator,
                        int throttle, int yaw, bool connected);

void flightCoreTask(void *pvParameters);
