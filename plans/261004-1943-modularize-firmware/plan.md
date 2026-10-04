# Modularize Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the monolithic `buomv1.ino` (1,801 lines) into clean, decoupled C++ modules adhering to Arduino IDE multi-file sketch standards and keeping code files under 200 lines.

**Architecture:** Split by domain responsibility:
1. `config.h`: Hardware pinouts, Wi-Fi configuration, constants, packet structs (`FlightPacket`, `TelemetryPacket`, `ControlSnapshot`), and `FlightMode` enum.
2. `servo_sys.h` & `servo_sys.cpp`: Low-level 300Hz PWM servo control, EMA filtering, attach/detach, and inversion logic (`ServoSystem`).
3. `flight.h` & `flight.cpp`: Aerodynamic flapping oscillation (triangle LUTs), dynamic amplitude scaling, perching glide transition, failsafe control, lock-free command buffer, and the FreeRTOS `flightCoreTask`.
4. `web_ui.h`: PROGMEM HTML/CSS/JS payload for the Tactical Flight Deck interface.
5. `web_server.h` & `web_server.cpp`: Wi-Fi STA/AP connection, mDNS, AsyncWebSocket server, telemetry broadcast, and link watchdog.
6. `buomv1.ino`: High-level entry point orchestrating `setup()` and `loop()`.

**Tech Stack:** C++17, ESP32 Arduino Core >= 2.0.0, FreeRTOS, ESP32Servo, ESPAsyncWebServer, AsyncTCP.

## Global Constraints
- Preserve all existing functionality, memory layout (`#pragma pack(push, 1)`), FreeRTOS task pinning, and timing parameters (`LOOP_DT_US = 3333`, `SERVO_HZ = 300`).
- Ensure no circular dependencies between headers.
- Keep each code file concise and under 200 lines where practical (apart from the embedded UI asset in `web_ui.h`).
- Compatible with Arduino IDE 2.x compile model (all `.cpp` in sketch root are compiled as compilation units).

## Review Focus
1. Double-buffering atomic synchronization: `publishControlSnapshot` and `readControlSnapshot` must maintain memory barrier semantics across modules.
2. Extern definitions vs declarations: `ServoSystem::state`, `Flight::state`, `server`, `ws` must be properly declared `extern` in `.h` and defined once in `.cpp`.
3. LUT placement: `tri_LUT` and `atri_LUT` with `PROGMEM` should be in `flight.cpp` to prevent multiple definition errors during linking.
4. Brownout protection: The constructor function `disable_bod_early()` must be retained early in compilation.
5. Wi-Fi credentials security: Keep placeholders `YourWiFiSSID` / `YourWiFiPassword` intact.

---

### Task 1: Create `config.h`
Extract constants, pin definitions, data packets, and shared types.

**Files:**
- Create: `d:/buomv1/config.h`

**Steps:**
- [ ] Create `config.h` with header guards `#pragma once`.
- [ ] Include necessary system headers (`<Arduino.h>`, `esp_wifi.h`, `hal/brownout_ll.h`, `soc/rtc_cntl_reg.h`, `soc/soc.h`).
- [ ] Define `disable_bod_early()` constructor.
- [ ] Define `namespace Config` with pins, frequencies, Wi-Fi placeholders, and `mapTrim15`.
- [ ] Define `FlightPacket`, `TelemetryPacket`, `ControlSnapshot`, and `FlightMode`.

---

### Task 2: Create `servo_sys.h` and `servo_sys.cpp`
Extract the `ServoSystem` namespace for servo management.

**Files:**
- Create: `d:/buomv1/servo_sys.h`
- Create: `d:/buomv1/servo_sys.cpp`

**Steps:**
- [ ] In `servo_sys.h`, declare `struct Control` and functions `attachAll()`, `detachAll()`, `update()`, with `extern Control state`.
- [ ] In `servo_sys.cpp`, implement `state`, `attachAll()`, `detachAll()`, and `update(int targetL, int targetR, bool bypassFilter)`.

---

### Task 3: Create `flight.h` and `flight.cpp`
Extract flapping aerodynamics, glide/perch state machine, lock-free command buffer, and FreeRTOS task.

**Files:**
- Create: `d:/buomv1/flight.h`
- Create: `d:/buomv1/flight.cpp`

**Steps:**
- [ ] In `flight.h`, declare `Flight::State`, control snapshot publication/read functions, and `flightCoreTask`.
- [ ] In `flight.cpp`, include `tri_LUT` and `atri_LUT` in PROGMEM, implement state machine transitions, LUT lookups, double buffer atomic operations, and `flightCoreTask`.

---

### Task 4: Create `web_ui.h`
Extract the PROGMEM HTML/CSS/JavaScript Tactical Flight Deck payload into a dedicated UI asset header.

**Files:**
- Create: `d:/buomv1/web_ui.h`

**Steps:**
- [ ] Move `const char PAGE_HTML[] PROGMEM = R"rawhtml(...);` into `web_ui.h`.

---

### Task 5: Create `web_server.h` and `web_server.cpp`
Extract AsyncWebServer, WebSocket handler, telemetry broadcaster, and Wi-Fi connection logic.

**Files:**
- Create: `d:/buomv1/web_server.h`
- Create: `d:/buomv1/web_server.cpp`

**Steps:**
- [ ] In `web_server.h`, declare `initNetworkAndServer()`, `handleNetworkLoop()`, `sendTelemetry()`.
- [ ] In `web_server.cpp`, define `server`, `ws`, `onWsEvent`, RF scan & connection logic, mDNS registration, and telemetry serialization.

---

### Task 6: Refactor `buomv1.ino`
Trim `buomv1.ino` down to the top-level lifecycle functions.

**Files:**
- Modify: `d:/buomv1/buomv1.ino`

**Steps:**
- [ ] Include `config.h`, `servo_sys.h`, `flight.h`, `web_server.h`.
- [ ] Simplify `setup()` to initialize peripherals, call `initNetworkAndServer()`, and launch `flightCoreTask`.
- [ ] Simplify `loop()` to call `handleNetworkLoop()` and drive LED indicators.

---

### Task 7: Verification and Code Quality Review
Verify compilation syntax, file sizes, and clean code standards.

**Files:**
- Review: all files in `d:/buomv1/`

**Steps:**
- [ ] Verify each file is correctly scoped and syntax-valid.
- [ ] Verify no symbol duplication.
- [ ] Update `README.md` project structure section to reflect the new modular layout.
