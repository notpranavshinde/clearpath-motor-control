/**********************************************************************
 * ClearPath SDSK (Step/Dir) — AccelStepper runner with runtime input
 * Mechanics: XL 20T pulley  →  0.1016 m/rev
 * Resolution: PPR = 200      →  STEPS_PER_M ≈ 1968.5039
 *
 * Wiring (single-ended):
 *   STEP+ (Black)  -> D2
 *   DIR+  (White)  -> D3
 *   EN+   (Blue)   -> D4  (through your physical switch, in series)
 *   STEP-/DIR-/EN- -> GND
 *
 * Good ranges / guards (tweak if needed):
 *   • Distance ≤ 2.50 m (tank limit; >2.5 m risks wave generator clash)
 *   • Accel    ≤ 10.0 m/s^2 (belt slip risk above this; ~5 is comfy)
 *   • Uno + AccelStepper ceiling ≈ 4400 steps/s:
 *       -> with PPR=200, speed ceiling ≈ 4400 / 1968.5 ≈ 2.24 m/s
 *********************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
#include <stdlib.h>   // strtod
#include <ctype.h>    // tolower
#include <math.h>     // sqrtf

/* ------------ UI helpers (Serial text formatting) ----------------- */
static inline void uiLine() {
  Serial.println(F("=============================================================="));
}

static inline void uiSubLine() {
  Serial.println(F("--------------------------------------------------------------"));
}

static inline void uiBanner() {
  Serial.println();
  uiLine();
  Serial.println(F(" ClearPath SDSK Runner  -  AccelStepper  -  PPR=200"));
  uiLine();
}

static inline void uiSection(const __FlashStringHelper* title) {
  Serial.println();
  uiSubLine();
  Serial.print(F(" >> ")); // visual cue for a new section
  Serial.println(title);
  uiSubLine();
}

static inline void uiBulletKV(const __FlashStringHelper* key, float value, uint8_t decimals, const __FlashStringHelper* unit) {
  Serial.print(F("  • "));
  Serial.print(key);
  Serial.print(F(": "));
  Serial.print(value, decimals);
  if (unit) { Serial.print(F(" ")); Serial.print(unit); }
  Serial.println();
}

/* ------------ Motor / mechanics constants (PPR=200) --------------- */
const int   PPR         = 200;
const float M_PER_REV   = 0.1016f;                 // 4.000 in = 0.1016 m
const float STEPS_PER_M = (float)PPR / M_PER_REV;  // ≈ 1968.5039

/* ------------ Safety / practical limits --------------------------- */
const float D_SAFE_MAX  = 2.50f;     // m
const float A_SAFE_MAX  = 10.0f;     // m/s^2  (recommend ~5 for no slip)
const float MCU_SPS_LIM = 4400.0f;   // steps/s (Uno+AccelStepper typical)
const float SPEED_MAX   = MCU_SPS_LIM / STEPS_PER_M; // ≈ 2.24 m/s

/* ------------ Pins ------------------------------------------------- */
constexpr byte STEP_PIN = 2;
constexpr byte DIR_PIN  = 3;
constexpr byte EN_PIN   = 4;

/* ------------ Driver ----------------------------------------------- */
AccelStepper axis(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ----------- Input helpers ---------------------------------------- */
float askFloat(const char* label, float deflt) {
  Serial.print(label); Serial.print(" ["); Serial.print(deflt, 4); Serial.print("]: ");
  String s;
  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        s.trim();
        if (s.length() == 0) { Serial.println(); return deflt; }
        char* endp = nullptr;
        double dv = strtod(s.c_str(), &endp);     // robust parse
        if (endp && *endp == '\0') { Serial.println(); return (float)dv; }
        Serial.print("  (bad number, try again) ");
        s = "";
      } else {
        s += c;
      }
    }
  }
}

bool askYesNo(const char* label, bool deflt) {
  Serial.print(label); Serial.print(deflt ? " [Y]/n: " : " y/[N]: ");
  while (true) {
    while (Serial.available()) {
      char c = (char)tolower(Serial.read());
      if (c == '\n' || c == '\r') { Serial.println(); return deflt; }
      if (c == 'y') { Serial.println(); return true; }
      if (c == 'n') { Serial.println(); return false; }
    }
  }
}

// Integer prompt (clamped). ENTER accepts default.
int askInt(const char* label, int deflt, int minVal, int maxVal) {
  Serial.print(label); Serial.print(" ["); Serial.print(deflt); Serial.print("]: ");
  String s;
  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        s.trim();
        if (s.length() == 0) { Serial.println(); return deflt; }
        char* endp = nullptr;
        long lv = strtol(s.c_str(), &endp, 10);
        if (endp && *endp == '\0') {
          if (lv < minVal) lv = minVal;
          if (lv > maxVal) lv = maxVal;
          Serial.println();
          return (int)lv;
        }
        Serial.print("  (bad integer, try again) ");
        s = "";
      } else {
        s += c;
      }
    }
  }
}

/* ---- Run one motion, print plan & result. Distance sign = direction. ---- */
void planAndRun(float dist_m, float speed_m_s, float accel_m_s2) {
  float D = dist_m;
  float V = fabs(speed_m_s);
  float A = fabs(accel_m_s2);

  // Clamp to safe / practical limits
  bool clamped = false;
  if (fabs(D) > D_SAFE_MAX) { D = (D > 0 ? +D_SAFE_MAX : -D_SAFE_MAX); clamped = true; }
  if (A > A_SAFE_MAX)       { A = A_SAFE_MAX; clamped = true; }
  if (V > SPEED_MAX)        { V = SPEED_MAX;  clamped = true; }
  if (A < 1e-3f)            { A = 1e-3f; } // avoid divide-by-zero

  // Convert to steps domain
  const long  move_steps = lround(D * STEPS_PER_M);
  const float speed_sps  = V * STEPS_PER_M;
  const float accel_sps2 = A * STEPS_PER_M;

  // Triangular vs trapezoid (min accel to reach V over D)
  const float A_min = (V > 0 && fabs(D) > 0) ? (V*V / fabs(D)) : 0.0f;
  const bool  willCruise = (A >= A_min);

  float t_pred;
  if (willCruise) {
    t_pred = fabs(D)/V + V/A;           // trapezoid total time
  } else {
    t_pred = 2.0f * sqrtf(fabs(D)/A);   // triangular total time
  }
  const float v_avg_pred = (fabs(D) > 0) ? (fabs(D)/t_pred) : 0.0f;

  // Plan printout (prettier)
  uiSection(F("PLAN"));
  if (clamped) Serial.println(F("NOTE: Inputs exceeded limits; values were clamped to safe ranges."));
  uiBulletKV(F("Distance"), D, 4, F("m"));
  uiBulletKV(F("Speed"),    V, 4, F("m/s"));
  uiBulletKV(F("Accel"),    A, 4, F("m/s^2"));
  Serial.print(F("  • Profile: ")); Serial.println(willCruise ? F("Trapezoid (will cruise)") : F("Triangular (accel-limited)"));
  uiBulletKV(F("Pred time"), t_pred, 3, F("s"));
  uiBulletKV(F("Pred avg"),  v_avg_pred, 3, F("m/s"));
  uiBulletKV(F("Steps/m"),   STEPS_PER_M, 3, nullptr);
  Serial.print(F("  • Move steps: ")); Serial.println(move_steps);
  uiBulletKV(F("Cmd speed"), speed_sps, 1, F("steps/s"));
  uiBulletKV(F("Cmd accel"), accel_sps2, 1, F("steps/s^2"));

  // Configure and run
  axis.setMaxSpeed(speed_sps > 1.0f ? speed_sps : 1.0f);
  axis.setAcceleration(accel_sps2 > 1.0f ? accel_sps2 : 1.0f);
  digitalWrite(DIR_PIN, (D >= 0) ? LOW : HIGH);

  digitalWrite(EN_PIN, HIGH);   // your physical ENABLE switch must be closed
  delay(5);

  unsigned long t0 = micros();
  axis.move(move_steps);
  while (axis.run()) { /* ramp → cruise → ramp-down */ }
  unsigned long dt_us = micros() - t0;

  digitalWrite(EN_PIN, LOW);

  // Result (prettier)
  uiSection(F("RESULT"));
  const float actual_sps = (fabs((float)move_steps)) / (dt_us / 1e6f);
  const float actual_mps = actual_sps / STEPS_PER_M;
  uiBulletKV(F("Elapsed"), dt_us / 1e6f, 4, F("s"));
  uiBulletKV(F("Avg speed"), actual_mps, 4, F("m/s"));
  Serial.println();
}

void setup() {
  pinMode(EN_PIN, OUTPUT);   digitalWrite(EN_PIN, LOW); // keep disabled during upload/reset
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT); digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {}

  uiBanner();
  Serial.print  (F(" Steps per meter: ")); Serial.println(STEPS_PER_M, 4);
  Serial.println(F(" Safe operating guidelines:"));
  Serial.println(F("  • Distance ≤ 2.50 m   (tank limit)"));
  Serial.println(F("  • Accel    ≤ 10.0 m/s^2 (belt slip risk; ~5 is comfy)"));
  Serial.print  (F("  • Speed ceiling ≈ ")); Serial.print(SPEED_MAX, 2); Serial.println(F(" m/s (Uno + AccelStepper)"));
  Serial.println();
  Serial.println(F(" Close the physical ENABLE switch when ready."));
  uiSubLine();
}

void loop() {
  // Prompts (ENTER accepts default)
  Serial.println();
  Serial.println(F("Enter motion parameters (press ENTER for defaults):"));
  float dist_m  = askFloat("  Distance (m, sign = direction)", 0.40f);
  float speed_m = askFloat("  Speed (m/s)",                      1.00f);
  float accel_m = askFloat("  Accel (m/s^2)",                    3.00f);

  int loops = askInt("Number of round-trips (0 = none)", 0, 0, 100);
  float dwell_s  = 0.0f;
  if (loops > 0) dwell_s = askFloat("Dwell between legs, seconds", 0.50f);

  if (loops == 0) {
    planAndRun(dist_m, speed_m, accel_m);
  } else {
    uiSection(F("BATCH RUN"));
    Serial.print(F("Starting ")); Serial.print(loops); Serial.println(F(" round-trip(s) (front + back)."));
    float d = fabs(dist_m);
    for (int k = 0; k < loops; ++k) {
      planAndRun(+d, speed_m, accel_m);
      if (dwell_s > 0) delay((unsigned long)(dwell_s * 1000.0f));
      planAndRun(-d, speed_m, accel_m);
      if (dwell_s > 0) delay((unsigned long)(dwell_s * 1000.0f));
    }
    Serial.println(F("Round-trips complete."));
  }

  uiSubLine();
  Serial.println(F("Press ENTER to set new values."));
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') break;
    }
  }
}
