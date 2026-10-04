#include "flight.h"
#include "servo_sys.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Flight {
State state;

static const int16_t PROGMEM tri_LUT[256] = {
    0,     16,   31,   47,   63,   78,   94,   109,  125,  141,  156,  172,
    188,   203,  219,  234,  250,  266,  281,  297,  313,  328,  344,  359,
    375,   391,  406,  422,  438,  453,  469,  484,  500,  516,  531,  547,
    563,   578,  594,  609,  625,  641,  656,  672,  688,  703,  719,  734,
    750,   766,  781,  797,  813,  828,  844,  859,  875,  891,  906,  922,
    938,   953,  969,  984,  1000, 984,  969,  953,  938,  922,  906,  891,
    875,   859,  844,  828,  813,  797,  781,  766,  750,  734,  719,  703,
    688,   672,  656,  641,  625,  609,  594,  578,  563,  547,  531,  516,
    500,   484,  469,  453,  438,  422,  406,  391,  375,  359,  344,  328,
    313,   297,  281,  266,  250,  234,  219,  203,  188,  172,  156,  141,
    125,   109,  94,   78,   63,   47,   31,   16,   0,    -16,  -31,  -47,
    -63,   -78,  -94,  -109, -125, -141, -156, -172, -188, -203, -219, -234,
    -250,  -266, -281, -297, -313, -328, -344, -359, -375, -391, -406, -422,
    -438,  -453, -469, -484, -500, -516, -531, -547, -563, -578, -594, -609,
    -625,  -641, -656, -672, -688, -703, -719, -734, -750, -766, -781, -797,
    -813,  -828, -844, -859, -875, -891, -906, -922, -938, -953, -969, -984,
    -1000, -984, -969, -953, -938, -922, -906, -891, -875, -859, -844, -828,
    -813,  -797, -781, -766, -750, -734, -719, -703, -688, -672, -656, -641,
    -625,  -609, -594, -578, -563, -547, -531, -516, -500, -484, -469, -453,
    -438,  -422, -406, -391, -375, -359, -344, -328, -313, -297, -281, -266,
    -250,  -234, -219, -203, -188, -172, -156, -141, -125, -109, -94,  -78,
    -63,   -47,  -31,  -16};

static const uint16_t PROGMEM atri_LUT[256] = {
    0,     64,    128,   193,   257,   321,   386,   450,   514,   579,   643,
    707,   772,   836,   900,   965,   1029,  1093,  1158,  1222,  1286,  1351,
    1415,  1479,  1543,  1608,  1672,  1736,  1801,  1865,  1929,  1994,  2058,
    2122,  2187,  2251,  2315,  2380,  2444,  2508,  2572,  2637,  2701,  2765,
    2830,  2894,  2958,  3023,  3087,  3151,  3216,  3280,  3344,  3409,  3473,
    3537,  3601,  3666,  3730,  3794,  3859,  3923,  3987,  4052,  4116,  4180,
    4245,  4309,  4373,  4437,  4502,  4566,  4630,  4695,  4759,  4823,  4888,
    4952,  5016,  5081,  5145,  5209,  5273,  5338,  5402,  5466,  5531,  5595,
    5659,  5724,  5788,  5852,  5917,  5981,  6045,  6109,  6174,  6238,  6302,
    6367,  6431,  6495,  6560,  6624,  6688,  6753,  6817,  6881,  6945,  7010,
    7074,  7138,  7203,  7267,  7331,  7396,  7460,  7524,  7589,  7653,  7717,
    7781,  7846,  7910,  7974,  8039,  8103,  8167,  8232,  8296,  8360,  8425,
    8489,  8553,  8617,  8682,  8746,  8810,  8875,  8939,  9003,  9068,  9132,
    9196,  9261,  9325,  9389,  9453,  9518,  9582,  9646,  9711,  9775,  9839,
    9904,  9968,  10032, 10097, 10161, 10225, 10289, 10354, 10418, 10482, 10547,
    10611, 10675, 10740, 10804, 10868, 10933, 10997, 11061, 11125, 11190, 11254,
    11318, 11383, 11447, 11511, 11576, 11640, 11704, 11769, 11833, 11897, 11961,
    12026, 12090, 12154, 12219, 12283, 12347, 12412, 12476, 12540, 12605, 12669,
    733,   12797, 12862, 12926, 12990, 13055, 13119, 13183, 13248, 13312, 13376,
    13441, 13505, 13569, 13633, 13698, 13762, 13826, 13891, 13955, 14019, 14084,
    14148, 14212, 14277, 14341, 14405, 14469, 14534, 14598, 14662, 14727, 14791,
    14855, 14920, 14984, 15048, 15113, 15177, 15241, 15305, 15370, 15434, 15498,
    15563, 15627, 15691, 15756, 15820, 15884, 15949, 16013, 16077, 16141, 16206,
    16270, 16334, 16384};

void updateCurrentOffsetTriangle(int throttle, unsigned long dt_us) {
  if (state.phaseNeedsResync) {
    int32_t ratioScaled =
        (int32_t)(state.currentOffset * 1000.0f / Config::FLAP_AMP);
    ratioScaled = constrain(ratioScaled, -1000, 1000);
    bool negate = (ratioScaled < 0);
    int32_t abs_ratio = negate ? -ratioScaled : ratioScaled;
    int index = (abs_ratio * 255) / 1000;
    uint16_t approxPhase = atri_LUT[index];
    int32_t signedPhase = negate ? -approxPhase : approxPhase;
    if (!state.isFlapUp)
      signedPhase = 32768 - signedPhase;
    state.phase = (uint16_t)signedPhase;
    state.phaseNeedsResync = false;
  }

  throttle = constrain(throttle, Config::THROTTLE_IDLE_MAX, Config::CH_MAX);
  float dynamicAmp =
      55.0f - ((float)(throttle - Config::THROTTLE_IDLE_MAX) /
               (float)(Config::CH_MAX - Config::THROTTLE_IDLE_MAX)) *
                  20.0f;
  uint32_t speed = Config::FLAP_SPEED_MIN +
                   (uint32_t)(throttle - Config::THROTTLE_IDLE_MAX) *
                       (Config::FLAP_SPEED_MAX - Config::FLAP_SPEED_MIN) /
                       (Config::CH_MAX - Config::THROTTLE_IDLE_MAX);

  if (dt_us > 0) {
    uint32_t phaseDelta = ((uint64_t)speed * dt_us) / 1000000;
    state.phase += phaseDelta;
  }

  float oldOffset = state.currentOffset;
  state.currentOffset =
      ((float)tri_LUT[state.phase >> 8] * dynamicAmp) / 1000.0f;
  float delta = state.currentOffset - oldOffset;
  if (delta > 0.1f)
    state.isFlapUp = true;
  else if (delta < -0.1f)
    state.isFlapUp = false;
}

void updateCurrentOffsetPerch(int perchAngleLevel, unsigned long dt_us) {
  if (state.phaseNeedsResync)
    state.phaseNeedsResync = false;
  float maxUpAngle =
      Config::PERCH_BASE_UP_ANGLE +
      ((perchAngleLevel - Config::CH_MIN) * Config::PERCH_MULT) / 1000.0f;
  constexpr float maxOffset = Config::SERVO_MAX_ANGLE -
                              ServoSystem::Control::centerAngle -
                              Config::PERCH_HEADROOM_DEG;
  float target = constrain(maxUpAngle, 0.0f, maxOffset);

  if (dt_us > 0) {
    float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
    if (state.currentOffset < target) {
      state.currentOffset += step;
      if (state.currentOffset > target)
        state.currentOffset = target;
    } else if (state.currentOffset > target) {
      state.currentOffset -= step;
      if (state.currentOffset < target)
        state.currentOffset = target;
    }
  } else {
    state.currentOffset = target;
  }
}

void updateCurrentOffsetGlide(unsigned long dt_us) {
  if (state.currentOffset == 0)
    return;
  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
  if (state.currentOffset > 0) {
    state.currentOffset -= step;
    if (state.currentOffset < 0)
      state.currentOffset = 0;
  } else {
    state.currentOffset += step;
    if (state.currentOffset > 0)
      state.currentOffset = 0;
  }
}

void processFailsafe(unsigned long dt_us) {
  state.isPerching = false;
  unsigned long lostMs = millis() - state.failsafeEnterMs;
  if (lostMs > Config::RX_HARD_FAILSAFE_MS) {
    state.isArmed = false;
  }
  if (!state.isArmed) {
    ServoSystem::detachAll();
    return;
  }

  float step = (Config::GLIDE_SERVO_SPEED_DPS * (float)dt_us) / 1000000.0f;
  if (state.currentOffset < Config::FAILSAFE_DIHEDRAL) {
    state.currentOffset += step;
    if (state.currentOffset > Config::FAILSAFE_DIHEDRAL)
      state.currentOffset = Config::FAILSAFE_DIHEDRAL;
  } else if (state.currentOffset > Config::FAILSAFE_DIHEDRAL) {
    state.currentOffset -= step;
    if (state.currentOffset < Config::FAILSAFE_DIHEDRAL)
      state.currentOffset = Config::FAILSAFE_DIHEDRAL;
  }

  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + (int)state.currentOffset,
        ServoSystem::state.centerAngle - (int)state.currentOffset, false);
  }
  state.phaseNeedsResync = true;
}
}

static void handleFailsafeMode(unsigned long dt_us, bool connected,
                               unsigned long currentMillis) {
  if (connected) {
    Flight::state.failsafeEnterMs = 0;
    Flight::state.mode =
        Flight::state.isArmed ? FlightMode::CONNECTED : FlightMode::INIT;
    return;
  }
  if (Flight::state.failsafeEnterMs == 0) {
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
  }
  Flight::processFailsafe(dt_us);
}

static void handleInitMode(unsigned long dt_us, bool connected,
                           unsigned long currentMillis) {
  if (!connected) {
    Flight::state.mode = FlightMode::FAILSAFE;
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
    return;
  }
  Flight::state.failsafeEnterMs = 0;
  Flight::updateCurrentOffsetGlide(dt_us);
  if (ServoSystem::state.isAttached) {
    ServoSystem::update(
        ServoSystem::state.centerAngle + (int)Flight::state.currentOffset,
        ServoSystem::state.centerAngle - (int)Flight::state.currentOffset,
        false);
  }
}

static void handleModeTransition(int elevator, int throttle) {
  Flight::state.isFlapping = (throttle >= Config::THROTTLE_IDLE_MAX);
  if (Flight::state.isFlapping && !Flight::state.wasFlapping) {
    Flight::state.phaseNeedsResync = true;
  }
  Flight::state.wasFlapping = Flight::state.isFlapping;

  if (throttle < Config::THROTTLE_IDLE_MAX) {
    if (elevator < Config::PERCH_ENTER_ELEV)
      Flight::state.isPerching = true;
    else if (elevator > Config::PERCH_EXIT_ELEV) {
      Flight::state.isPerching = false;
      Flight::state.perchAngleLevel = Config::CH_MIN;
    }
    Flight::state.mode =
        Flight::state.isPerching ? FlightMode::PERCHING : FlightMode::CONNECTED;
  } else {
    Flight::state.isPerching = false;
    Flight::state.perchAngleLevel = Config::CH_MIN;
    Flight::state.mode = FlightMode::CONNECTED;
  }

  if (Flight::state.mode == FlightMode::PERCHING &&
      !Flight::state.wasPerching) {
    Flight::state.phaseNeedsResync = true;
  }
  Flight::state.wasPerching = (Flight::state.mode == FlightMode::PERCHING);
}

static void computeServoTargets(unsigned long dt_us, int steering, int elevator,
                                int throttle) {
  int steerValue = (abs(steering - Config::CH_CENTER) > Config::STICK_DEADBAND)
                       ? Config::mapTrim15(steering)
                       : 0;
  int elevOffset = 0;

  if (Flight::state.mode != FlightMode::PERCHING) {
    elevOffset = (abs(elevator - Config::CH_CENTER) > Config::STICK_DEADBAND)
                     ? Config::mapTrim15(elevator)
                     : 0;
  }

  if (Flight::state.mode == FlightMode::PERCHING) {
    int currentPull =
        constrain(Config::CH_MIN + (Config::CH_CENTER - elevator) * 2,
                  Config::CH_MIN, Config::CH_MAX);
    Flight::state.perchAngleLevel = currentPull;
    Flight::updateCurrentOffsetPerch(Flight::state.perchAngleLevel, dt_us);
  } else {
    if (throttle < Config::THROTTLE_IDLE_MAX)
      Flight::updateCurrentOffsetGlide(dt_us);
    else
      Flight::updateCurrentOffsetTriangle(throttle, dt_us);
  }

  int offsetInt = (int)Flight::state.currentOffset;
  ServoSystem::update(
      ServoSystem::state.centerAngle + offsetInt + elevOffset + steerValue,
      ServoSystem::state.centerAngle - offsetInt - elevOffset + steerValue,
      Flight::state.isFlapping);
}

static void handleConnectedMode(unsigned long dt_us, bool connected,
                                unsigned long currentMillis, int steering,
                                int elevator, int throttle) {
  if (!connected) {
    Flight::state.mode = FlightMode::FAILSAFE;
    Flight::state.failsafeEnterMs = currentMillis;
    if (Flight::state.failsafeEnterMs == 0)
      Flight::state.failsafeEnterMs = 1;
    return;
  }
  Flight::state.failsafeEnterMs = 0;
  handleModeTransition(elevator, throttle);
  computeServoTargets(dt_us, steering, elevator, throttle);
}

void processFlightLogic(unsigned long dt_us, int steering, int elevator,
                        int throttle, int yaw, bool connected) {
  unsigned long currentMillis = millis();
  switch (Flight::state.mode) {
  case FlightMode::FAILSAFE:
    handleFailsafeMode(dt_us, connected, currentMillis);
    break;
  case FlightMode::INIT:
    handleInitMode(dt_us, connected, currentMillis);
    break;
  case FlightMode::PERCHING:
  case FlightMode::CONNECTED:
    handleConnectedMode(dt_us, connected, currentMillis, steering, elevator,
                        throttle);
    break;
  }
}

static ControlSnapshot g_cmd_buffer[2];
static std::atomic<uint8_t> g_cmd_index{0};

void publishControlSnapshot(int16_t roll, int16_t pitch,
                            int16_t throttle, int16_t yaw,
                            uint8_t flags) {
  uint8_t next = 1 - g_cmd_index.load(std::memory_order_relaxed);
  g_cmd_buffer[next].ch[0] = roll;
  g_cmd_buffer[next].ch[1] = pitch;
  g_cmd_buffer[next].ch[2] = throttle;
  g_cmd_buffer[next].ch[3] = yaw;
  g_cmd_buffer[next].flags = flags;
  g_cmd_index.store(next, std::memory_order_release);
}

ControlSnapshot readControlSnapshot() {
  uint8_t idx = g_cmd_index.load(std::memory_order_acquire);
  return g_cmd_buffer[idx];
}

void flightCoreTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  unsigned long lastUs = micros();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    unsigned long nowUs = micros();
    unsigned long dt_us = nowUs - lastUs;
    if (dt_us == 0 || dt_us > 100000)
      dt_us = Config::LOOP_DT_US;
    lastUs = nowUs;

    ControlSnapshot cmd = readControlSnapshot();
    bool snapshotConnected = (cmd.flags & 0x01) != 0;
    bool killActive = (cmd.flags & 0x02) != 0;
    bool reqArm = (cmd.flags & 0x04) != 0;

    static uint8_t killDebounce = 0;
    if (killActive) {
      if (killDebounce < 10)
        killDebounce++;
    } else {
      killDebounce = 0;
    }

    if (killDebounce >= 5) {
      Flight::state.isArmed = false;
      Flight::state.isPerching = false;
      Flight::state.isFlapping = false;
      Flight::state.wasFlapping = false;
      Flight::state.wasPerching = false;
      Flight::state.phaseNeedsResync = true;
      Flight::state.mode = FlightMode::INIT;
      Flight::state.failsafeEnterMs = 0;
      ServoSystem::detachAll();
    } else {
      bool throttleIdle =
          cmd.ch[Config::CH_THROTTLE] < Config::THROTTLE_IDLE_MAX;

      if (reqArm && !Flight::state.isArmed && snapshotConnected) {
        if (throttleIdle) {
          Flight::state.isArmed = true;
          Flight::state.phaseNeedsResync = true;
          Flight::state.mode = FlightMode::CONNECTED;
          Flight::state.isPerching = false;
          Flight::state.perchAngleLevel = Config::CH_MIN;
          ServoSystem::state.lastPosL = ServoSystem::Control::centerAngle;
          ServoSystem::state.lastPosR = ServoSystem::Control::centerAngle;
          ServoSystem::attachAll();
        }
      } else if (!reqArm && Flight::state.isArmed) {
        if (throttleIdle) {
          Flight::state.isArmed = false;
          ServoSystem::detachAll();
          Flight::state.mode = FlightMode::INIT;
        }
      }

      processFlightLogic(dt_us, cmd.ch[0], cmd.ch[1], cmd.ch[2], cmd.ch[3],
                         snapshotConnected);
    }
  }
}
