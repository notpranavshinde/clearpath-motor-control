/**********************************************************************
 * ClearPath SDSK (Step/Dir) — XL 20T pulley (0.200" pitch)
 * 4.000 in/rev, 1600 steps/in @ 6400 PPR
 *
 * Change ONLY these three:
 *   SPEED_IN_S  – linear speed (inches/second, >0)
 *   DIST_IN     – linear distance (inches, SIGN = direction)
 *   ACCEL_IN_S2 – linear accel (inches/second^2, >0)
 *
 * Wiring (single-ended):
 *   STEP+ (Black)  -> D2
 *   DIR+  (White)  -> D3
 *   EN+   (Blue)   -> D4  (through your toggle switch in series)
 *   STEP-/DIR-/EN- -> Arduino GND
 *
 * Use: keep the ENABLE switch OPEN during uploads.
 *      Close the switch, open Serial Monitor, press ENTER to run.
 *********************************************************************/
#include <AccelStepper.h>

/* ================= USER TUNABLES ================= */
// Input your desired values in METRIC units here.
const float SPEED_M_S   = 0.1;   // m/s
const float DIST_M      = 0.25;  // m (negative = reverse)
const float ACCEL_M_S2  = 0.2;   // m/s^2

/* ---- Conversion from SI to Imperial ---- */
const float M_TO_IN = 39.3701f; // Conversion factor: meters to inches

// These values are calculated from the metric inputs above. Do not change them directly.
const float SPEED_IN_S  = SPEED_M_S * M_TO_IN;   // in/s
const float DIST_IN     = DIST_M * M_TO_IN;      // in
const float ACCEL_IN_S2 = ACCEL_M_S2 * M_TO_IN;  // in/s^2
/* ================================================= */

/* ---- Motor/pulley constants ---- */
const int   PPR          = 6400;     // ClearPath input resolution (pulses per rev)
const float IN_PER_REV   = 0.200f * 20.0f;  // 4.000 in/rev for XL-20T
const float STEPS_PER_IN = PPR / IN_PER_REV; // 1600 steps/in

/* ---- Pins ---- */
constexpr byte STEP_PIN = 2;   // Black
constexpr byte DIR_PIN  = 3;   // White
constexpr byte EN_PIN   = 4;   // Blue (through your physical switch)

/* ---- Stepper driver ---- */
AccelStepper axis(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ---- UI state ---- */
bool runRequested = false;  // set true when user presses ENTER

void setup() {
  // Keep motor disabled until we actually run
  pinMode(EN_PIN,   OUTPUT);  digitalWrite(EN_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT);  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  delay(50);

  // Derived (in steps units)
  const long  moveSteps    = lround(DIST_IN * STEPS_PER_IN);
  const float maxSpeed_sps = fabs(SPEED_IN_S  * STEPS_PER_IN);    // steps/s
  const float accel_sps2   = fabs(ACCEL_IN_S2 * STEPS_PER_IN);    // steps/s^2

  // Basic sanity
  if (maxSpeed_sps < 1.0f) {
    Serial.println(F("SPEED_IN_S too low. Bumping to 1 step/s."));
  }
  if (accel_sps2 < 1.0f) {
    Serial.println(F("ACCEL_IN_S2 too low. Bumping to 1 step/s^2."));
  }

  axis.setMaxSpeed(max(1.0f, maxSpeed_sps));
  axis.setAcceleration(max(1.0f, accel_sps2));

  // Pretty info header so you can verify math once
  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("  ClearPath Linear Move — units: inches"));
  Serial.println(F("============================================"));
  Serial.print(F("in/rev                : ")); Serial.println(IN_PER_REV, 3);
  Serial.print(F("steps/in              : ")); Serial.println(STEPS_PER_IN, 3);
  Serial.print(F("distance (in)         : ")); Serial.println(DIST_IN, 4);
  Serial.print(F("speed (in/s)          : ")); Serial.println(SPEED_IN_S, 3);
  Serial.print(F("accel (in/s^2)        : ")); Serial.println(ACCEL_IN_S2, 3);
  Serial.print(F("moveSteps             : ")); Serial.println(moveSteps);
  Serial.print(F("maxSpeed (steps/s)    : ")); Serial.println(maxSpeed_sps, 1);
  Serial.print(F("accel (steps/s^2)     : ")); Serial.println(accel_sps2, 1);
  Serial.println();
  Serial.println(F("Close the ENABLE switch, then press <ENTER> to run."));
}

void loop() {
  // Accumulate input and only trigger on newline (ENTER)
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      runRequested = true;
    }
  }
  if (!runRequested) return;
  runRequested = false;

  // Energize and execute one trapezoidal move
  digitalWrite(EN_PIN, HIGH);                 // Enable ClearPath
  delay(5);                                   // tiny settle

  const long moveSteps = lround(DIST_IN * STEPS_PER_IN);
  axis.move(moveSteps);                       // sign sets direction
  while (axis.run()) { /* ramp → cruise → ramp-down */ }

  digitalWrite(EN_PIN, LOW);                  // relax shaft
  Serial.println(F("\nMove complete."));
  Serial.println(F("Close the ENABLE switch, then press <ENTER> to run again."));
}