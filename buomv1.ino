#include "esp_wifi.h"
#include "hal/brownout_ll.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct alignas(4) ControlSnapshot {
  int16_t ch[4];
  uint8_t flags;
};

ControlSnapshot readControlSnapshot();
void publishControlSnapshot(int16_t roll, int16_t pitch, int16_t throttle,
                            int16_t yaw, uint8_t flags);

__attribute__((constructor)) static void disable_bod_early() {
  brownout_ll_bod_enable(false);
  brownout_ll_intr_enable(false);
}

namespace Config {
constexpr uint8_t SERVO_L_PIN = 5;
constexpr uint8_t SERVO_R_PIN = 6;
constexpr uint8_t LED_PIN = 8;

// Wi-Fi credentials — change these to match your network.
// Leave SSID empty ("") to always run in SoftAP mode.
constexpr char WIFI_STA_SSID[] = "YourWiFiSSID";
constexpr char WIFI_STA_PASS[] = "YourWiFiPassword";
constexpr char MDNS_HOST[] = "phelan";

constexpr int SERVO_HZ = 300;
constexpr int SERVO_CENTER = 90;
constexpr int SERVO_MIN_ANGLE = 20;
constexpr int SERVO_MAX_ANGLE = 160;
constexpr int SERVO_EMA_ALPHA_SHIFT = 1;
constexpr bool REVERSE_SERVO_L = false;
constexpr bool REVERSE_SERVO_R = false;

constexpr int THROTTLE_IDLE_MAX = 1050;
constexpr unsigned long LOOP_DT_US = 3333;
constexpr int FLAP_AMP = 65;
constexpr uint32_t FLAP_SPEED_MIN = 157286;
constexpr uint32_t FLAP_SPEED_MAX = 262144;
constexpr int PERCH_MULT = 31;
constexpr int PERCH_BASE_UP_ANGLE = 30;
constexpr int FAILSAFE_DIHEDRAL = 15;
constexpr int GLIDE_SERVO_SPEED_DPS = 1570;

constexpr int16_t PERCH_ENTER_ELEV = 1300;
constexpr int16_t PERCH_EXIT_ELEV = 1700;
constexpr int16_t STICK_DEADBAND = 10;
constexpr unsigned long RX_HARD_FAILSAFE_MS = 3000;

constexpr int16_t CH_CENTER = 1500;
constexpr int16_t CH_MIN = 1000;
constexpr int16_t CH_MAX = 2000;

constexpr uint8_t CH_STEERING = 0;
constexpr uint8_t CH_ELEVATOR = 1;
constexpr uint8_t CH_THROTTLE = 2;
constexpr uint8_t CH_YAW = 3;
constexpr uint8_t CH_KILL_SWITCH = 4;
constexpr int16_t KILL_SWITCH_THRESHOLD = 1700;
constexpr int PERCH_HEADROOM_DEG = 15;

inline int mapTrim15(int x) {
  int d = x - CH_CENTER;
  if (abs(d) <= STICK_DEADBAND)
    return 0;
  int s = (d > 0) ? (d - STICK_DEADBAND) : (d + STICK_DEADBAND);
  return constrain(s / 30, -15, 15);
}
}

#pragma pack(push, 1)
struct FlightPacket {
  uint8_t header;
  uint8_t flags;
  int8_t throttle;
  int8_t yaw;
  int8_t pitch;
  int8_t roll;
};

struct TelemetryPacket {
  uint8_t header;
  uint8_t flags;
  int8_t offset;
  uint8_t servoL;
  uint8_t servoR;
  uint8_t cpuTemp;
  uint8_t cpuMhz;
  uint16_t uptimeSec;
  uint8_t timeH;
  uint8_t timeM;
  uint8_t timeS;
};
#pragma pack(pop)

enum class FlightMode { INIT, CONNECTED, PERCHING, FAILSAFE };

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

Control state;

void attachAll() {
  if (!state.isAttached) {
    state.left.setPeriodHertz(Config::SERVO_HZ);
    state.right.setPeriodHertz(Config::SERVO_HZ);
    state.left.attach(Config::SERVO_L_PIN, 500, 2500);
    state.right.attach(Config::SERVO_R_PIN, 500, 2500);
    int outL =
        Config::REVERSE_SERVO_L ? (180 - state.lastPosL) : state.lastPosL;
    int outR =
        Config::REVERSE_SERVO_R ? (180 - state.lastPosR) : state.lastPosR;
    state.left.write(outL);
    state.right.write(outR);
    state.filteredL = (int32_t)state.lastPosL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)state.lastPosR << Config::SERVO_EMA_ALPHA_SHIFT;
    state.isAttached = true;
  }
}

void detachAll() {
  if (state.isAttached) {
    state.left.detach();
    state.right.detach();
    state.isAttached = false;
  }
}

void update(int targetL, int targetR, bool bypassFilter = false) {
  if (!state.isAttached)
    return;
  targetL =
      constrain(targetL, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);
  targetR =
      constrain(targetR, Config::SERVO_MIN_ANGLE, Config::SERVO_MAX_ANGLE);

  if (bypassFilter) {
    state.filteredL = (int32_t)targetL << Config::SERVO_EMA_ALPHA_SHIFT;
    state.filteredR = (int32_t)targetR << Config::SERVO_EMA_ALPHA_SHIFT;
  } else {
    state.filteredL +=
        targetL - (state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT);
    state.filteredR +=
        targetR - (state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT);
  }

  targetL = state.filteredL >> Config::SERVO_EMA_ALPHA_SHIFT;
  targetR = state.filteredR >> Config::SERVO_EMA_ALPHA_SHIFT;

  if (abs(targetL - state.lastPosL) >= 2) {
    int outL = Config::REVERSE_SERVO_L ? (180 - targetL) : targetL;
    state.left.write(outL);
    state.lastPosL = targetL;
  }
  if (abs(targetR - state.lastPosR) >= 2) {
    int outR = Config::REVERSE_SERVO_R ? (180 - targetR) : targetR;
    state.right.write(outR);
    state.lastPosR = targetR;
  }
}
}

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

void handleFailsafeMode(unsigned long dt_us, bool connected,
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

void handleInitMode(unsigned long dt_us, bool connected,
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

void handleModeTransition(int elevator, int throttle) {
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

void computeServoTargets(unsigned long dt_us, int steering, int elevator,
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

void handleConnectedMode(unsigned long dt_us, bool connected,
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

inline void publishControlSnapshot(int16_t roll, int16_t pitch,
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

inline ControlSnapshot readControlSnapshot() {
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

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool wifiActive = false;
bool webControlEnabled = false;
volatile unsigned long lastWsPacketTime = 0;
unsigned long lastTelemSendTime = 0;
volatile int8_t wsArmRequest = 0;
constexpr unsigned long WS_FAILSAFE_MS = 2000;
constexpr unsigned long TELEM_INTERVAL_MS = 200;

int16_t webChannels[16] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>Phelan GCS - Tactical Flight Deck</title>
<style>
:root {
  --bg-deep: #06080b;
  --bg-panel: #0c1017;
  --bg-card: rgba(18, 24, 34, 0.85);
  --border-dim: rgba(255, 255, 255, 0.08);
  --border-glow: rgba(59, 130, 246, 0.4);
  --border-active: #3b82f6;

  --text-white: #f8fafc;
  --text-gray: #718096;
  --text-muted: #4a5568;

  --accent-green: #10b981;
  --accent-green-glow: rgba(16, 185, 129, 0.4);
  --accent-red: #ef4444;
  --accent-red-glow: rgba(239, 68, 68, 0.4);
  --accent-amber: #f59e0b;
  --accent-amber-glow: rgba(245, 158, 11, 0.4);
  --accent-cyan: #06b6d4;

  --font-mono: ui-monospace, SFMono-Regular, "JetBrains Mono", Menlo, Consolas, monospace;
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
}

* {
  box-sizing: border-box;
  margin: 0;
  padding: 0;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

html, body {
  width: 100vw;
  height: 100vh;
  background: var(--bg-deep);
  background-image: 
    radial-gradient(circle at 50% 50%, rgba(16, 185, 129, 0.03) 0%, transparent 60%),
    linear-gradient(rgba(255, 255, 255, 0.015) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 255, 255, 0.015) 1px, transparent 1px);
  background-size: 100% 100%, 32px 32px, 32px 32px;
  color: var(--text-white);
  font-family: var(--font-sans);
  overflow: hidden;
  position: fixed;
  inset: 0;
}

#portrait-guard {
  display: none;
  position: fixed;
  inset: 0;
  background: #06080b;
  z-index: 9999;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 16px;
  padding: 24px;
  text-align: center;
}
.rot-svg {
  width: 52px;
  height: 52px;
  stroke: var(--accent-green);
  animation: rotAnim 2.5s infinite ease-in-out;
}
@keyframes rotAnim {
  0%, 100% { transform: rotate(0deg); }
  35%, 65% { transform: rotate(-90deg); }
}
.rot-msg {
  font-family: var(--font-mono);
  font-size: 14px;
  font-weight: 700;
  color: var(--text-white);
  letter-spacing: 2px;
}
@media (orientation: portrait) {
  #portrait-guard { display: flex !important; }
}

#interlock-toast {
  position: fixed;
  top: 48px;
  left: 50%;
  transform: translateX(-50%) translateY(-20px);
  background: rgba(239, 68, 68, 0.95);
  border: 1px solid #f87171;
  color: #fff;
  font-family: var(--font-mono);
  font-weight: 700;
  font-size: 12px;
  padding: 6px 14px;
  border-radius: 6px;
  box-shadow: 0 4px 16px rgba(239, 68, 68, 0.5);
  pointer-events: none;
  opacity: 0;
  transition: all 0.25s cubic-bezier(0.16, 1, 0.3, 1);
  z-index: 500;
  letter-spacing: 0.5px;
}
#interlock-toast.show {
  opacity: 1;
  transform: translateX(-50%) translateY(0);
}

#failsafe-banner {
  display: none;
  position: fixed;
  top: 44px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(220, 38, 38, 0.95);
  border: 1px solid #fca5a5;
  color: #fff;
  font-family: var(--font-mono);
  font-size: 11px;
  font-weight: 800;
  letter-spacing: 1.5px;
  padding: 4px 16px;
  border-radius: 20px;
  box-shadow: 0 0 16px rgba(239, 68, 68, 0.7);
  z-index: 600;
}

.header-bar {
  height: 38px;
  background: rgba(10, 13, 18, 0.92);
  backdrop-filter: blur(12px);
  border-bottom: 1px solid var(--border-dim);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  position: relative;
  z-index: 100;
}

.target-badge {
  display: flex;
  align-items: center;
  gap: 8px;
  background: var(--bg-card);
  border: 1px solid var(--border-dim);
  padding: 3px 10px;
  border-radius: 5px;
  font-family: var(--font-mono);
  font-size: 11px;
}
.target-label { color: var(--text-gray); font-size: 10px; }
.target-id-val { color: var(--text-white); font-weight: 700; letter-spacing: 0.5px; }
.badge-state {
  font-size: 10px;
  font-weight: 700;
  padding: 2px 6px;
  border-radius: 4px;
}
.badge-linked { color: var(--accent-green); background: rgba(16, 185, 129, 0.15); }
.badge-disc { color: var(--accent-red); background: rgba(239, 68, 68, 0.15); }

.brand-center-group {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  display: flex;
  align-items: center;
  gap: 8px;
  font-family: var(--font-mono);
  font-size: 13px;
  font-weight: 800;
  letter-spacing: 2px;
  color: #fff;
  pointer-events: none;
}
.brand-icon {
  width: 18px;
  height: 18px;
  fill: var(--accent-green);
  filter: drop-shadow(0 0 6px var(--accent-green-glow));
}

.header-right {
  display: flex;
  align-items: center;
  gap: 8px;
}
.btn-fs {
  background: var(--bg-card);
  border: 1px solid var(--border-dim);
  color: var(--text-gray);
  border-radius: 5px;
  padding: 4px 10px;
  font-size: 10px;
  font-family: var(--font-mono);
  cursor: pointer;
  letter-spacing: 1px;
  transition: all 0.2s;
}
.btn-fs:hover { color: var(--text-white); border-color: var(--border-active); }

.main-layout {
  display: grid;
  grid-template-columns: 1fr clamp(230px, 28vw, 300px) 1fr;
  height: calc(100vh - 38px);
  width: 100vw;
  padding: 4px 12px 6px 12px;
  gap: 12px;
  align-items: center;
  box-sizing: border-box;
}

.joystick-col {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  position: relative;
  height: 100%;
}

.joystick-wrapper {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
}

.joystick-touch-zone {
  --size: min(290px, 32vw, calc(100vh - 90px));
  --max-dist: calc(var(--size) * 0.36);
  width: var(--size);
  height: var(--size);
  background: radial-gradient(circle, #151a23 0%, #0d1117 70%, #080a0e 100%);
  border: 2px solid var(--border-dim);
  border-radius: 50%;
  position: relative;
  touch-action: none;
  cursor: crosshair;
  box-shadow: 
    inset 0 0 24px rgba(0, 0, 0, 0.9),
    0 6px 24px rgba(0, 0, 0, 0.6),
    0 0 0 1px rgba(255, 255, 255, 0.05);
}

.joystick-radar-rings {
  position: absolute;
  inset: 0;
  border-radius: 50%;
  pointer-events: none;
}
.joystick-radar-rings::before {
  content: "";
  position: absolute;
  inset: 18%;
  border-radius: 50%;
  border: 1px dashed rgba(255, 255, 255, 0.08);
}
.joystick-radar-rings::after {
  content: "";
  position: absolute;
  inset: 36%;
  border-radius: 50%;
  border: 1px dashed rgba(255, 255, 255, 0.06);
}

.joystick-touch-zone::before,
.joystick-touch-zone::after {
  content: "";
  position: absolute;
  background: rgba(255, 255, 255, 0.08);
  pointer-events: none;
}
.joystick-touch-zone::before { left: 50%; top: 8%; bottom: 8%; width: 1px; transform: translateX(-50%); }
.joystick-touch-zone::after { top: 50%; left: 8%; right: 8%; height: 1px; transform: translateY(-50%); }

.throttle-graduations {
  position: absolute;
  inset: 0;
  pointer-events: none;
}
.throttle-baseline {
  position: absolute;
  left: 20%;
  right: 20%;
  top: calc(50% + var(--max-dist));
  height: 2px;
  background: var(--accent-green);
  box-shadow: 0 0 8px var(--accent-green-glow);
}
.throttle-mark {
  position: absolute;
  left: calc(50% + 36px);
  transform: translateY(-50%);
  font-family: var(--font-mono);
  font-size: 10px;
  font-weight: 700;
  color: var(--text-gray);
  pointer-events: none;
  letter-spacing: 0.5px;
}
.throttle-mark.mark-100 { top: calc(50% - var(--max-dist)); }
.throttle-mark.mark-50  { top: 50%; }
.throttle-mark.mark-0   { top: calc(50% + var(--max-dist)); color: var(--accent-green); }

.joystick-knob {
  width: 48px;
  height: 48px;
  border-radius: 50%;
  background: radial-gradient(circle at 35% 35%, #2a3342 0%, #151a23 70%, #0d1015 100%);
  border: 2px solid rgba(255, 255, 255, 0.85);
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  pointer-events: none;
  box-shadow: 
    0 6px 18px rgba(0, 0, 0, 0.7),
    0 0 10px rgba(255, 255, 255, 0.2),
    inset 0 2px 3px rgba(255, 255, 255, 0.5);
  transition: transform 0.04s ease-out;
}
.joystick-knob-dot {
  width: 8px;
  height: 8px;
  background: var(--accent-cyan);
  box-shadow: 0 0 8px var(--accent-cyan);
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
}

.joystick-readout-card {
  display: flex;
  align-items: center;
  gap: 12px;
  background: var(--bg-card);
  border: 1px solid var(--border-dim);
  padding: 4px 14px;
  border-radius: 12px;
  font-family: var(--font-mono);
  font-size: 11px;
  color: var(--text-gray);
  box-shadow: 0 2px 8px rgba(0,0,0,0.4);
}
.joystick-readout-card b {
  color: var(--text-white);
  font-size: 12px;
}

.center-col {
  background: rgba(12, 16, 23, 0.92);
  border: 1px solid var(--border-dim);
  border-radius: 12px;
  display: flex;
  flex-direction: column;
  padding: 8px 10px;
  gap: 8px;
  box-shadow: 0 8px 28px rgba(0, 0, 0, 0.6);
  box-sizing: border-box;
}

.hud-telemetry-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 6px;
}
.hud-tile {
  background: rgba(18, 24, 34, 0.9);
  border: 1px solid var(--border-dim);
  border-radius: 8px;
  padding: 4px 8px;
  display: flex;
  flex-direction: column;
  gap: 2px;
}
.hud-tile-title {
  font-family: var(--font-mono);
  font-size: 9px;
  color: var(--text-gray);
  letter-spacing: 0.5px;
  display: flex;
  align-items: center;
  gap: 4px;
}
.hud-tile-val {
  font-family: var(--font-mono);
  font-size: 14px;
  font-weight: 800;
  letter-spacing: 0.5px;
  color: var(--text-white);
  line-height: 1.1;
}
.hud-tile-val.ping-good { color: var(--accent-green); }
.hud-tile-val.ping-warn { color: var(--accent-amber); }
.hud-tile-val.ping-bad  { color: var(--accent-red); }
.hud-tile-val.warn { color: var(--accent-amber); }
.hud-tile-val.alert { color: var(--accent-red); }

.action-buttons {
  display: flex;
  flex-direction: column;
  gap: 6px;
}
.action-row-split {
  display: grid;
  grid-template-columns: 1.25fr 1fr;
  gap: 6px;
}

.btn-action {
  height: 38px;
  border-radius: 8px;
  font-family: var(--font-mono);
  cursor: pointer;
  background: linear-gradient(180deg, #18202c 0%, #0e131a 100%);
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0 10px;
  border: 1px solid var(--border-dim);
  transition: all 0.15s ease;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.4);
}
.btn-action:active {
  transform: scale(0.97);
}

.btn-main-txt {
  font-size: 12px;
  font-weight: 800;
  letter-spacing: 1.5px;
  line-height: 1;
}

.btn-arm {
  border-color: rgba(16, 185, 129, 0.4);
  color: var(--accent-green);
}
.btn-arm.armed {
  background: linear-gradient(180deg, #10b981 0%, #059669 100%);
  color: #041a10;
  border-color: #34d399;
  box-shadow: 0 0 16px rgba(16, 185, 129, 0.6);
  font-weight: 900;
}

.btn-perch {
  border-color: rgba(245, 158, 11, 0.35);
  color: var(--accent-amber);
}
.btn-perch.active {
  background: linear-gradient(180deg, #f59e0b 0%, #d97706 100%);
  color: #1f1202;
  border-color: #fcd34d;
  box-shadow: 0 0 16px rgba(245, 158, 11, 0.6);
  font-weight: 900;
}

.btn-stop {
  height: 40px;
  background: linear-gradient(180deg, rgba(239, 68, 68, 0.2) 0%, rgba(185, 28, 28, 0.35) 100%);
  border-color: rgba(239, 68, 68, 0.6);
  color: #fca5a5;
  box-shadow: 0 0 12px rgba(239, 68, 68, 0.25);
}
.btn-stop:active {
  background: #dc2626;
  color: #fff;
}
</style>
</head>
<body>

<div id="portrait-guard">
  <svg class="rot-svg" viewBox="0 0 24 24" fill="none" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
    <rect x="5" y="2" width="14" height="20" rx="3"></rect>
    <line x1="12" y1="18" x2="12.01" y2="18"></line>
  </svg>
  <div class="rot-msg">XOAY NGANG ĐỂ ĐIỀU KHIỂN</div>
</div>

<div id="interlock-toast">CẢNH BÁO: HẠ TAY GA VỀ 0% ĐỂ ARM!</div>
<div id="failsafe-banner">FAILSAFE ACTIVATED - ĐÃ CẮT TẢI SERVO</div>

<header class="header-bar">
  <div class="target-badge">
    <span class="target-label">ID:</span>
    <b class="target-id-val">PHELAN-DECK</b>
    <span class="badge-state badge-disc" id="hdr-link-state">DISCONNECTED</span>
  </div>

  <div class="brand-center-group">
    <svg class="brand-icon" viewBox="0 0 24 24">
      <path d="M12 2C10.5 4 8 6 4 6c0 5 3 9 8 16 5-7 8-11 8-16-4 0-6.5-2-8-4z"/>
    </svg>
    PHELAN DECK
  </div>

  <div class="header-right">
    <button class="btn-fs" onclick="toggleFullScreen()" title="Toàn màn hình">FULLSCREEN</button>
  </div>
</header>

<main class="main-layout" id="cockpit-layout">
  <section class="joystick-col">
    <div class="joystick-wrapper">
      <div class="joystick-touch-zone" id="zone-l">
        <div class="joystick-radar-rings"></div>
        <div class="throttle-graduations">
          <div class="throttle-baseline"></div>
          <span class="throttle-mark mark-100">100%</span>
          <span class="throttle-mark mark-50">50%</span>
          <span class="throttle-mark mark-0">0%</span>
        </div>
        <div class="joystick-knob" id="knob-l">
          <div class="joystick-knob-dot"></div>
        </div>
      </div>
      <div class="joystick-readout-card">
        <span>THR: <b id="txt-thr">0%</b></span>
        <span>|</span>
        <span>YAW: <b id="txt-yaw">0%</b></span>
      </div>
    </div>
  </section>

  <section class="center-col">
    <div class="hud-telemetry-grid">
      <div class="hud-tile">
        <div class="hud-tile-title">⏱ UPTIME</div>
        <div class="hud-tile-val" id="time-uptime">00:00</div>
      </div>
      <div class="hud-tile">
        <div class="hud-tile-title">📶 PING RTT</div>
        <div class="hud-tile-val ping-good" id="val-ping">-- ms</div>
      </div>
      <div class="hud-tile">
        <div class="hud-tile-title">⚡ ARM TIME</div>
        <div class="hud-tile-val" id="time-armed">00:00</div>
      </div>
      <div class="hud-tile">
        <div class="hud-tile-title">🪽 FLIGHT TIME</div>
        <div class="hud-tile-val" id="time-flight">00:00</div>
      </div>
    </div>

    <div class="action-buttons">
      <div class="action-row-split">
        <button class="btn-action btn-arm ready" id="btn-arm" onclick="handleArmClick()">
          <span class="btn-main-txt" id="txt-arm-main">ARM</span>
        </button>
        <button class="btn-action btn-perch" id="btn-perch" onclick="togglePerchClick()">
          <span class="btn-main-txt" id="txt-perch-main">PERCH</span>
        </button>
      </div>

      <button class="btn-action btn-stop" id="btn-stop" onclick="handleStopClick()">
        <span class="btn-main-txt">STOP / EMERGENCY</span>
      </button>
    </div>
  </section>

  <section class="joystick-col">
    <div class="joystick-wrapper">
      <div class="joystick-touch-zone" id="zone-r">
        <div class="joystick-radar-rings"></div>
        <div class="joystick-knob" id="knob-r">
          <div class="joystick-knob-dot"></div>
        </div>
      </div>
      <div class="joystick-readout-card">
        <span>PIT: <b id="txt-pit">0%</b></span>
        <span>|</span>
        <span>ROL: <b id="txt-rol">0%</b></span>
      </div>
    </div>
  </section>
</main>

<script>
const App = {
  isArmed: false,
  isPerching: false,
  isFailsafe: false,
  thr: 0, yaw: 0, pit: 0, rol: 0,
  ws: null,
  wsConnected: false,
  armedStartTime: 0,
  lastPingTime: 0,
  lastHeartbeatTime: 0
};

let accumulatedFlightSeconds = 0;
let lastFlightTick = 0;

function toggleFullScreen() {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen().catch(() => {});
  }
}

function setKnobPosition(knob, x, y) {
  if (!knob) return;
  knob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
}

function resetJoysticksToIdle() {
  App.pit = 0; App.rol = 0; App.thr = 0; App.yaw = 0;
  const knobR = document.getElementById('knob-r');
  const knobL = document.getElementById('knob-l');
  const zoneL = document.getElementById('zone-l');
  if (knobR) setKnobPosition(knobR, 0, 0);
  if (zoneL && knobL) {
    const rect = zoneL.getBoundingClientRect();
    const maxDist = (rect.width / 2) * 0.72;
    setKnobPosition(knobL, 0, maxDist);
  }
  document.getElementById('txt-thr').textContent = '0%';
  document.getElementById('txt-yaw').textContent = '0%';
  document.getElementById('txt-pit').textContent = '0%';
  document.getElementById('txt-rol').textContent = '0%';
}

function initJoysticks() {
  const zoneL = document.getElementById('zone-l');
  const zoneR = document.getElementById('zone-r');
  const knobL = document.getElementById('knob-l');
  const knobR = document.getElementById('knob-r');
  const activeTouches = {};

  function getZoneMetrics(zone) {
    const rect = zone.getBoundingClientRect();
    const radius = rect.width / 2;
    const maxDist = radius * 0.72;
    return { rect, radius, maxDist };
  }

  function updateLeftKnobPosition() {
    const { maxDist } = getZoneMetrics(zoneL);
    const pixelX = (App.yaw / 100) * maxDist;
    const pixelY = maxDist - (App.thr / 100) * (2 * maxDist);
    setKnobPosition(knobL, pixelX, pixelY);
  }

  updateLeftKnobPosition();

  function handleMove(zone, clientX, clientY, isLeft) {
    const { rect, radius, maxDist } = getZoneMetrics(zone);
    const centerX = rect.left + radius;
    const centerY = rect.top + radius;
    let dx = clientX - centerX;
    let dy = clientY - centerY;

    if (isLeft) {
      dx = Math.max(-maxDist, Math.min(maxDist, dx));
      dy = Math.max(-maxDist, Math.min(maxDist, dy));
      const rawThr = Math.round(((maxDist - dy) / (2 * maxDist)) * 100);
      App.thr = Math.max(0, Math.min(100, rawThr));
      App.yaw = Math.round((dx / maxDist) * 100);

      setKnobPosition(knobL, dx, dy);
      document.getElementById('txt-thr').textContent = App.thr + '%';
      document.getElementById('txt-yaw').textContent = App.yaw + '%';
    } else {
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > maxDist) {
        dx = (dx / dist) * maxDist;
        dy = (dy / dist) * maxDist;
      }
      App.rol = Math.round((dx / maxDist) * 100);
      App.pit = Math.round((-dy / maxDist) * 100);

      setKnobPosition(knobR, dx, dy);
      document.getElementById('txt-rol').textContent = App.rol + '%';
      document.getElementById('txt-pit').textContent = App.pit + '%';
    }
    sendFlightPacket();
  }

  function handleRelease(isLeft) {
    if (isLeft) {
      App.yaw = 0;
      updateLeftKnobPosition();
      document.getElementById('txt-yaw').textContent = '0%';
    } else {
      App.pit = 0;
      App.rol = 0;
      setKnobPosition(knobR, 0, 0);
      document.getElementById('txt-pit').textContent = '0%';
      document.getElementById('txt-rol').textContent = '0%';
    }
    sendFlightPacket();
  }

  [ { zone: zoneL, isLeft: true }, { zone: zoneR, isLeft: false } ].forEach(({ zone, isLeft }) => {
    let isMouseDown = false;
    zone.addEventListener('mousedown', (e) => {
      isMouseDown = true;
      handleMove(zone, e.clientX, e.clientY, isLeft);
    });
    window.addEventListener('mousemove', (e) => {
      if (isMouseDown) handleMove(zone, e.clientX, e.clientY, isLeft);
    });
    window.addEventListener('mouseup', () => {
      if (isMouseDown) {
        isMouseDown = false;
        handleRelease(isLeft);
      }
    });

    zone.addEventListener('touchstart', (e) => {
      e.preventDefault();
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        activeTouches[t.identifier] = { isLeft, zone };
        handleMove(zone, t.clientX, t.clientY, isLeft);
      }
    }, { passive: false });

    zone.addEventListener('touchmove', (e) => {
      e.preventDefault();
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        if (activeTouches[t.identifier] && activeTouches[t.identifier].zone === zone) {
          handleMove(zone, t.clientX, t.clientY, isLeft);
        }
      }
    }, { passive: false });

    const onTouchEnd = (e) => {
      e.preventDefault();
      for (let i = 0; i < e.changedTouches.length; i++) {
        const t = e.changedTouches[i];
        if (activeTouches[t.identifier]) {
          const wasLeft = activeTouches[t.identifier].isLeft;
          delete activeTouches[t.identifier];
          handleRelease(wasLeft);
        }
      }
    };
    zone.addEventListener('touchend', onTouchEnd);
    zone.addEventListener('touchcancel', onTouchEnd);
  });

  window.addEventListener('resize', () => {
    updateLeftKnobPosition();
    setKnobPosition(knobR, 0, 0);
  });
}

function showInterlockToast(msg) {
  const toast = document.getElementById('interlock-toast');
  toast.textContent = msg;
  toast.classList.add('show');
  setTimeout(() => toast.classList.remove('show'), 2200);
}

function updateArmButtonUI(isArmed) {
  const btn = document.getElementById('btn-arm');
  const txtMain = document.getElementById('txt-arm-main');
  if (!btn) return;
  if (isArmed) {
    btn.classList.add('armed');
    if (txtMain) txtMain.textContent = 'DISARM';
  } else {
    btn.classList.remove('armed');
    if (txtMain) txtMain.textContent = 'ARM';
  }
}

function updatePerchButtonUI(isPerching) {
  const btn = document.getElementById('btn-perch');
  const txtMain = document.getElementById('txt-perch-main');
  if (!btn) return;
  if (isPerching) {
    btn.classList.add('active');
    if (txtMain) txtMain.textContent = 'PERCHING';
  } else {
    btn.classList.remove('active');
    if (txtMain) txtMain.textContent = 'PERCH';
  }
}

function handleArmClick() {
  if (!App.isArmed) {
    if (App.thr > 0) {
      showInterlockToast("CẢNH BÁO: HẠ TAY GA VỀ 0% ĐỂ ARM!");
      if (navigator.vibrate) navigator.vibrate([120, 60, 120]);
      return;
    }
    App.isArmed = true;
    App.armedStartTime = Date.now();
    updateArmButtonUI(true);
    if (navigator.vibrate) navigator.vibrate(60);
  } else {
    App.isArmed = false;
    App.armedStartTime = 0;
    lastFlightTick = 0;
    updateArmButtonUI(false);
    if (navigator.vibrate) navigator.vibrate(40);
  }
  sendFlightPacket();
}

function handleStopClick() {
  App.isArmed = false;
  App.isPerching = false;
  App.armedStartTime = 0;
  lastFlightTick = 0;
  if (navigator.vibrate) navigator.vibrate([100, 50, 150]);
  updateArmButtonUI(false);
  updatePerchButtonUI(false);
  resetJoysticksToIdle();
  sendFlightPacket(true);
}

function togglePerchClick() {
  App.isPerching = !App.isPerching;
  updatePerchButtonUI(App.isPerching);
  if (navigator.vibrate) navigator.vibrate(50);
  sendFlightPacket();
}

function formatTime(totalSec) {
  if (isNaN(totalSec) || totalSec < 0) totalSec = 0;
  const m = String(Math.floor(totalSec / 60)).padStart(2, '0');
  const s = String(Math.floor(totalSec % 60)).padStart(2, '0');
  return `${m}:${s}`;
}

setInterval(() => {
  const now = Date.now();

  const armedEl = document.getElementById('time-armed');
  if (App.isArmed && App.armedStartTime > 0) {
    const armedSec = Math.floor((now - App.armedStartTime) / 1000);
    armedEl.textContent = formatTime(armedSec);
    if (armedSec >= 480) armedEl.className = 'hud-tile-val alert';
    else if (armedSec >= 360) armedEl.className = 'hud-tile-val warn';
    else armedEl.className = 'hud-tile-val';
  } else {
    armedEl.textContent = "00:00";
    armedEl.className = 'hud-tile-val';
  }

  const flightEl = document.getElementById('time-flight');
  if (App.isArmed && App.thr > 0) {
    if (lastFlightTick > 0) {
      accumulatedFlightSeconds += (now - lastFlightTick) / 1000;
    }
    lastFlightTick = now;
    flightEl.textContent = formatTime(Math.floor(accumulatedFlightSeconds));
  } else {
    lastFlightTick = 0;
  }
}, 500);

function initWebSocket() {
  const wsUrl = `ws://${location.host}/ws`;
  App.ws = new WebSocket(wsUrl);
  App.ws.binaryType = 'arraybuffer';

  App.ws.onopen = () => {
    App.wsConnected = true;
    App.lastHeartbeatTime = performance.now();
    const badge = document.getElementById('hdr-link-state');
    badge.textContent = "LINKED";
    badge.className = "badge-state badge-linked";
  };

  App.ws.onclose = () => {
    App.wsConnected = false;
    const badge = document.getElementById('hdr-link-state');
    badge.textContent = "DISCONNECTED";
    badge.className = "badge-state badge-disc";
    const pingEl = document.getElementById('val-ping');
    if (pingEl) {
      pingEl.textContent = "-- ms";
      pingEl.className = "hud-tile-val ping-bad";
    }
    setTimeout(initWebSocket, 2000);
  };

  App.ws.onmessage = (event) => {
    App.lastHeartbeatTime = performance.now();
    if (typeof event.data === 'string') {
      if (event.data === "PONG") {
        const rtt = Math.round(performance.now() - App.lastPingTime);
        const pingEl = document.getElementById('val-ping');
        if (pingEl) {
          pingEl.textContent = rtt + ' ms';
          pingEl.className = 'hud-tile-val ' + (rtt < 50 ? 'ping-good' : (rtt < 120 ? 'ping-warn' : 'ping-bad'));
        }
      }
    } else if (event.data instanceof ArrayBuffer && event.data.byteLength === 12) {
      const view = new DataView(event.data);
      if (view.getUint8(0) === 0xBB) {
        const flags = view.getUint8(1);
        const armedFromHw = (flags & 0x01) !== 0;
        if (armedFromHw !== App.isArmed) {
          App.isArmed = armedFromHw;
          updateArmButtonUI(App.isArmed);
        }
        const uptime = view.getUint16(7, true);
        document.getElementById('time-uptime').textContent = formatTime(uptime);
      }
    }
  };
}

function triggerFailsafeAlert() {
  const banner = document.getElementById('failsafe-banner');
  banner.style.display = 'block';
  setTimeout(() => {
    banner.style.display = 'none';
  }, 3000);
}

function sendFlightPacket(isKill = false) {
  if (!App.ws || App.ws.readyState !== WebSocket.OPEN) return;
  if (App.ws.bufferedAmount > 32 && !isKill) return;

  const buf = new ArrayBuffer(6);
  const view = new DataView(buf);
  view.setUint8(0, 0xAA);

  let flags = 0;
  if (App.isArmed && !isKill) flags |= 0x01;
  if (isKill) flags |= 0x02;
  if (App.isPerching) flags |= 0x04;
  view.setUint8(1, flags);

  view.setInt8(2, App.thr);
  view.setInt8(3, App.yaw);
  view.setInt8(4, App.pit);
  view.setInt8(5, App.rol);

  App.ws.send(buf);
}

setInterval(() => {
  if (App.wsConnected) {
    sendFlightPacket();
  }
}, 25);

setInterval(() => {
  if (App.wsConnected) {
    App.lastPingTime = performance.now();
    App.ws.send("PING");
    if (performance.now() - App.lastHeartbeatTime > 2500 && App.isArmed) {
      triggerFailsafeAlert();
    }
  }
}, 500);

window.addEventListener('DOMContentLoaded', () => {
  initJoysticks();
  initWebSocket();
});
</script>
</body>
</html>
)rawhtml";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
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

  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.printf("[BOOT] MAC: %s | SSID: %s\n", WiFi.macAddress().c_str(), Config::WIFI_STA_SSID);

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

  Serial.printf("[RF AUDIT] Ch 1: %d | Ch 6: %d | Ch 11: %d\n", countCh[1], countCh[6], countCh[11]);

  if (foundRouter) {
    Serial.printf("[ROUTER] Found %s on Ch %d (%d dBm)\n", bestSSID.c_str(), bestChannel, bestRSSI);
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
                WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str(), WiFi.channel());

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
  xTaskCreatePinnedToCore(flightCoreTask, "flightCore", 4096, NULL, 5, NULL, 0);
}

void loop() {
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

  if (!snapshotConnected) {
    digitalWrite(Config::LED_PIN, (now / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(Config::LED_PIN, Flight::state.isArmed
                                      ? HIGH
                                      : ((now / 1000) % 2 == 0 ? HIGH : LOW));
  }

  if (now - lastTelemSendTime >= TELEM_INTERVAL_MS) {
    lastTelemSendTime = now;
    sendTelemetry();
    ws.cleanupClients();
  }

  delay(2);
}