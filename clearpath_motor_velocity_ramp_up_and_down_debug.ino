/**********************************************************************
 * ClearPath SDSK (Step/Dir) — XL 20T pulley (0.200" pitch)
 * 4.000 in/rev  →  1600 steps/in @ 6400 PPR
 *
 * CHANGE ONLY THESE (metric):
 *   SPEED_M_S   – desired linear speed (m/s)          (>0)
 *   DIST_M      – desired linear distance (m)         (sign = direction)
 *   ACCEL_M_S2  – desired linear accel (m/s^2)        (>0)
 *
 * Wiring (single-ended):
 *   STEP+ (Black)  -> D2
 *   DIR+  (White)  -> D3
 *   EN+   (Blue)   -> D4  (through your toggle switch, in series)
 *   STEP-/DIR-/EN- -> Arduino GND
 *
 * Use:
 *   1) Keep ENABLE switch OPEN while uploading.
 *   2) Close the switch, open Serial Monitor (115200), press ENTER to run.
 *********************************************************************/
#include <AccelStepper.h>

/* ================= USER TUNABLES (metric) ================= */
const float SPEED_M_S   = 0.10f;  // m/s
const float DIST_M      = 0.10f;  // m  (negative = reverse)
const float ACCEL_M_S2  = 0.30f;  // m/s^2
/* ========================================================== */

/* ---- Pulley / motor constants ---- */
const int   PPR          = 6400;          // ClearPath pulses per rev
const float IN_PER_REV   = 0.200f * 20.0f; // 4.000 in/rev (XL, 20T)
const float STEPS_PER_IN = PPR / IN_PER_REV;  // 1600 steps/in
const float M_TO_IN      = 39.37007874f;      // meters → inches

/* ---- Pins ---- */
constexpr byte STEP_PIN = 2;   // Black
constexpr byte DIR_PIN  = 3;   // White
constexpr byte EN_PIN   = 4;   // Blue (through your physical switch)

/* ---- Stepper driver ---- */
AccelStepper axis(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ---- Helper: run one trapezoid move and report actuals ---- */
void run_move_report(float speed_in_s, float dist_in, float accel_in_s2) {
  // Convert to steps domain
  const long  move_steps     = lround(dist_in * STEPS_PER_IN);
  const float req_sps        = fabs(speed_in_s * STEPS_PER_IN);   // requested steps/s
  const float accel_sps2     = max(1.0f, fabs(accel_in_s2 * STEPS_PER_IN)); // steps/s^2

  // Practical ceiling for Uno+AccelStepper. Adjust if you use a faster MCU.
  const float MCU_SPS_LIMIT  = 5000.0f;

  // Accel-limited peak speed: v^2 = a * d   (symmetric accel/decel)
  const float accel_peak_sps = sqrtf(accel_sps2 * fabs((float)move_steps));

  // Planner will use the MIN of requested, accel-limited, and MCU ceiling
  const float plan_sps       = min(req_sps, min(accel_peak_sps, MCU_SPS_LIMIT));

  axis.setMaxSpeed(plan_sps);
  axis.setAcceleration(accel_sps2);

  // Summary
  Serial.println(F("------- PLAN -------"));
  Serial.print(F("Distance (in): "));     Serial.println(dist_in, 4);
  Serial.print(F("Req speed  (in/s): ")); Serial.println(speed_in_s, 4);
  Serial.print(F("Req accel  (in/s^2): ")); Serial.println(accel_in_s2, 4);
  Serial.print(F("Move steps: "));        Serial.println(move_steps);
  Serial.print(F("Req   steps/s: "));     Serial.println(req_sps, 1);
  Serial.print(F("Accel steps/s: "));     Serial.println(accel_peak_sps, 1);
  Serial.print(F("MCU   steps/s: "));     Serial.println(MCU_SPS_LIMIT, 0);
  Serial.print(F("Using steps/s: "));     Serial.println(plan_sps, 1);

  // Energize & run
  digitalWrite(EN_PIN, HIGH);
  delay(5);

  unsigned long t0 = micros();
  axis.move(move_steps);
  while (axis.run()) { /* ramp → cruise → ramp-down */ }
  unsigned long dt_us = micros() - t0;

  digitalWrite(EN_PIN, LOW);

  // Compute actual average speed over the move
  float actual_sps   = (fabs((float)move_steps)) / (dt_us / 1e6f);
  float actual_in_s  = actual_sps / STEPS_PER_IN;

  Serial.println(F("------- RESULT -----"));
  Serial.print(F("Elapsed (ms): "));      Serial.println(dt_us / 1000.0f, 3);
  Serial.print(F("Actual steps/s: "));    Serial.println(actual_sps, 1);
  Serial.print(F("Actual in/s  : "));     Serial.println(actual_in_s, 4);
  Serial.println();
}

void setup() {
  pinMode(EN_PIN,   OUTPUT);  digitalWrite(EN_PIN, LOW); // keep disabled
  pinMode(STEP_PIN, OUTPUT);  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT);  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {}

  // Convert user knobs (metric) → inches
  const float speed_in_s = SPEED_M_S  * M_TO_IN;
  const float dist_in    = DIST_M     * M_TO_IN;
  const float accel_in_s2= ACCEL_M_S2 * M_TO_IN;

  Serial.println(F("\n=== ClearPath Linear Move (metric knobs → inches under the hood) ==="));
  Serial.print(F("in/rev: "));       Serial.println(IN_PER_REV, 3);
  Serial.print(F("steps/in: "));     Serial.println(STEPS_PER_IN, 3);
  Serial.print(F("DIST_M: "));       Serial.print(DIST_M, 4);      Serial.print(F("  ->  "));
  Serial.print(F("dist_in: "));      Serial.println(dist_in, 4);
  Serial.print(F("SPEED_M_S: "));    Serial.print(SPEED_M_S, 4);   Serial.print(F(" ->  "));
  Serial.print(F("speed_in_s: "));   Serial.println(speed_in_s, 4);
  Serial.print(F("ACCEL_M_S2: "));   Serial.print(ACCEL_M_S2, 4);  Serial.print(F(" ->  "));
  Serial.print(F("accel_in_s2: "));  Serial.println(accel_in_s2, 4);
  Serial.println(F("\nClose the ENABLE switch, then press <ENTER> to run."));
}

void loop() {
  if (!Serial.available()) return;
  while (Serial.available()) Serial.read();   // clear buffer

  // Convert user knobs (metric) → inches each run
  const float speed_in_s = SPEED_M_S  * M_TO_IN;
  const float dist_in    = DIST_M     * M_TO_IN;
  const float accel_in_s2= ACCEL_M_S2 * M_TO_IN;

  run_move_report(speed_in_s, dist_in, accel_in_s2);

  Serial.println(F("Press <ENTER> again to repeat with the same settings."));
}