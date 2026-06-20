#include <Arduino.h>
#include <math.h>

// =====================================
// EVA 33
// Plate scan comparison:
//   1) WITHOUT input shaping
//   2) WITH input shaping in X and Y
//
// Note:
// - Middle row is Y = 9.5 because the available
//   Y shapers are:
//     14.5 -> 9.5
//     9.5  -> 4.5
//
// Scan route:
//   (0,0) -> slow to (24.7,17.0)
//   (24.7,17.0) -> slow to (22.2,14.5)
//   then scan:
//
//   (22.2,14.5) -> (17.2,14.5) -> (12.2,14.5) -> (7.2,14.5)
//   (7.2,14.5)  -> (7.2,9.5)
//   (7.2,9.5)   -> (12.2,9.5) -> (17.2,9.5) -> (22.2,9.5)
//   (22.2,9.5)  -> (22.2,4.5)
//   (22.2,4.5)  -> (17.2,4.5) -> (12.2,4.5) -> (7.2,4.5)
//
//   return slow to (0,0)
// =====================================

// ---------- X axis pins ----------
const int X_STEP_PIN = 26;
const int X_DIR_PIN  = 27;

// ---------- Y axis pins ----------
const int Y_STEP_PIN = 25;
const int Y_DIR_PIN  = 33;

// ---------- Positive direction ----------
const int X_DIR_POSITIVE_LEVEL = HIGH;
const int Y_DIR_POSITIVE_LEVEL = HIGH;

// ---------- Mechanics ----------
const float X_STEPS_PER_CM = 400.0f;
const float Y_STEPS_PER_CM = 400.0f;

// ---------- Fast profile (scan motion) ----------
const float FAST_V_CM_S  = 30.0f;
const float FAST_A_CM_S2 = 220.0f;

// ---------- Slow profile (entry / exit) ----------
const float SLOW_V_CM_S  = 14.0f;
const float SLOW_A_CM_S2 = 90.0f;

// ---------- STEP ----------
const unsigned int STEP_PULSE_US = 6;
const unsigned int MIN_STEP_INTERVAL_X_US = 85;
const unsigned int MIN_STEP_INTERVAL_Y_US = 85;

// ---------- Limits ----------
const float MAX_X_CM = 25.0f;
const float MAX_Y_CM = 19.2f;

// ---------- Pauses ----------
const unsigned long PAUSE_AT_REF_MS      = 500;
const unsigned long PAUSE_AT_FIRST_MS    = 500;
const unsigned long PAUSE_SCAN_MS        = 400;
const unsigned long PAUSE_BETWEEN_RUNS_MS = 1500;
const unsigned long PAUSE_END_MS         = 1000;

// ---------- Current state ----------
float currentXcm = 0.0f;
float currentYcm = 0.0f;

// =====================================
// Shaper struct
// =====================================
struct Shaper3 {
  const char* name;
  float freqHz;
  float zeta;
  float A[3];
  float T[3];
};

// =====================================
// X shapers
// =====================================

// x full 14.75
const Shaper3 SHAPER_X_14_5 = {
  "(x full 14.75)",
  11.1f,
  0.0350f,
  {0.278233f, 0.498490f, 0.223277f},
  {0.000000f, 0.045073f, 0.090145f}
};

// x full 9.5
const Shaper3 SHAPER_X_9_5 = {
  "(x full 9.5)",
  11.3f,
  0.0350f,
  {0.278233f, 0.498490f, 0.223277f},
  {0.000000f, 0.044275f, 0.088550f}
};

// x full 4.5
const Shaper3 SHAPER_X_4_5 = {
  "(x full 4.5)",
  11.6f,
  0.0350f,
  {0.278233f, 0.498490f, 0.223277f},
  {0.000000f, 0.043103f, 0.086207f}
};

// =====================================
// Y shapers
// =====================================

// y 14.5 -> 9.5
const Shaper3 SHAPER_Y_14_5_TO_9_5 = {
  "(y 14.5 -> 9.5)",
  14.9f,
  0.0400f,
  {0.282386f, 0.498028f, 0.219586f},
  {0.000000f, 0.033584f, 0.067168f}
};

// y 9.5 -> 4.5
const Shaper3 SHAPER_Y_9_5_TO_4_5 = {
  "(y 9.5 -> 4.5)",
  15.6f,
  0.0400f,
  {0.282386f, 0.498028f, 0.219586f},
  {0.000000f, 0.032077f, 0.064154f}
};

// =====================================
// Route
// =====================================
struct PointXY {
  float x;
  float y;
};

const int NUM_SCAN_POINTS = 12;
PointXY scanPoints[NUM_SCAN_POINTS] = {
  {22.2f, 14.5f},
  {17.2f, 14.5f},
  {12.2f, 14.5f},
  { 7.2f, 14.5f},

  { 7.2f,  9.5f},
  {12.2f,  9.5f},
  {17.2f,  9.5f},
  {22.2f,  9.5f},

  {22.2f,  4.5f},
  {17.2f,  4.5f},
  {12.2f,  4.5f},
  { 7.2f,  4.5f}
};

// =====================================
// Trapezoidal profile
// =====================================
struct MotionProfile {
  float D;
  float V;
  float A;
  float tAcc;
  float tCruise;
  float totalTime;
  float dAcc;
  bool triangular;
};

MotionProfile makeProfile(float distanceCm, float vmax, float accel) {
  MotionProfile p;

  p.D = distanceCm;
  p.V = vmax;
  p.A = accel;

  float tAccLocal = vmax / accel;
  float dAccLocal = 0.5f * accel * tAccLocal * tAccLocal;

  if (2.0f * dAccLocal >= distanceCm) {
    p.triangular = true;
    p.tAcc = sqrtf(distanceCm / accel);
    p.V = accel * p.tAcc;
    p.dAcc = 0.5f * accel * p.tAcc * p.tAcc;
    p.tCruise = 0.0f;
    p.totalTime = 2.0f * p.tAcc;
  } else {
    p.triangular = false;
    p.tAcc = tAccLocal;
    p.dAcc = dAccLocal;
    p.tCruise = (distanceCm - 2.0f * dAccLocal) / vmax;
    p.totalTime = 2.0f * p.tAcc + p.tCruise;
  }

  return p;
}

float basePosCm(const MotionProfile &p, float t) {
  if (t <= 0.0f) return 0.0f;

  if (p.triangular) {
    if (t < p.tAcc) {
      return 0.5f * p.A * t * t;
    } else if (t < p.totalTime) {
      float td = p.totalTime - t;
      return p.D - 0.5f * p.A * td * td;
    } else {
      return p.D;
    }
  } else {
    if (t < p.tAcc) {
      return 0.5f * p.A * t * t;
    } else if (t < (p.tAcc + p.tCruise)) {
      return p.dAcc + p.V * (t - p.tAcc);
    } else if (t < p.totalTime) {
      float td = p.totalTime - t;
      return p.D - 0.5f * p.A * td * td;
    } else {
      return p.D;
    }
  }
}

float shapedPosCm(const MotionProfile &p, float t, const Shaper3 &s) {
  float x = 0.0f;
  for (int i = 0; i < 3; i++) {
    x += s.A[i] * basePosCm(p, t - s.T[i]);
  }
  if (x < 0.0f) x = 0.0f;
  if (x > p.D) x = p.D;
  return x;
}

float shapedTotalTime(const MotionProfile &p, const Shaper3 &s) {
  return p.totalTime + s.T[2];
}

// =====================================
// STEP helpers
// =====================================
void pulseStepHighLow(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(pin, LOW);
}

void emitRemainingXSteps(long &emittedStepsX, long totalStepsX, uint32_t &lastStepXUs) {
  while (emittedStepsX < totalStepsX) {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastStepXUs) >= MIN_STEP_INTERVAL_X_US) {
      pulseStepHighLow(X_STEP_PIN);
      emittedStepsX++;
      lastStepXUs = nowUs;
    } else {
      delayMicroseconds(1);
    }
  }
}

void emitRemainingYSteps(long &emittedStepsY, long totalStepsY, uint32_t &lastStepYUs) {
  while (emittedStepsY < totalStepsY) {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastStepYUs) >= MIN_STEP_INTERVAL_Y_US) {
      pulseStepHighLow(Y_STEP_PIN);
      emittedStepsY++;
      lastStepYUs = nowUs;
    } else {
      delayMicroseconds(1);
    }
  }
}

// =====================================
// Shaper selection
// =====================================
const Shaper3* selectXRowShaper(float yValue) {
  if (fabsf(yValue - 14.5f) < 0.25f) return &SHAPER_X_14_5;
  if (fabsf(yValue -  9.5f) < 0.25f) return &SHAPER_X_9_5;
  if (fabsf(yValue -  4.5f) < 0.25f) return &SHAPER_X_4_5;
  return nullptr;
}

const Shaper3* selectYVerticalShaper(float yStart, float yTarget) {
  if (fabsf(yStart - 14.5f) < 0.25f && fabsf(yTarget - 9.5f) < 0.25f) {
    return &SHAPER_Y_14_5_TO_9_5;
  }
  if (fabsf(yStart - 9.5f) < 0.25f && fabsf(yTarget - 4.5f) < 0.25f) {
    return &SHAPER_Y_9_5_TO_4_5;
  }
  return nullptr;
}

// =====================================
// 1D X move
// =====================================
void moveXTo(float targetXcm, float vmax, float accel, const Shaper3* shaper) {
  if (targetXcm < 0.0f) targetXcm = 0.0f;
  if (targetXcm > MAX_X_CM) targetXcm = MAX_X_CM;

  float dx = targetXcm - currentXcm;
  float distanceCm = fabsf(dx);
  if (distanceCm < 0.0001f) return;

  bool xPositive = (dx >= 0.0f);
  digitalWrite(X_DIR_PIN, xPositive ? X_DIR_POSITIVE_LEVEL : !X_DIR_POSITIVE_LEVEL);
  delay(5);

  long totalStepsX = lroundf(distanceCm * X_STEPS_PER_CM);
  MotionProfile profile = makeProfile(distanceCm, vmax, accel);
  float totalMoveTime = shaper ? shapedTotalTime(profile, *shaper) : profile.totalTime;

  Serial.println("----------------------------------------");
  Serial.print("Move X: ");
  Serial.print(currentXcm, 3);
  Serial.print(" -> ");
  Serial.print(targetXcm, 3);
  Serial.print(" cm | ");
  if (shaper) {
    Serial.print("WITH ");
    Serial.println(shaper->name);
  } else {
    Serial.println("NO SHAPER");
  }

  uint32_t moveStartUs = micros();
  uint32_t lastStepXUs = micros();
  long emittedStepsX = 0;

  while (true) {
    uint32_t nowUs = micros();
    float t = (nowUs - moveStartUs) * 1e-6f;

    float cmdDist = shaper ? shapedPosCm(profile, t, *shaper) : basePosCm(profile, t);
    long desiredStepsX = lroundf(cmdDist * X_STEPS_PER_CM);
    if (desiredStepsX > totalStepsX) desiredStepsX = totalStepsX;

    while (emittedStepsX < desiredStepsX) {
      uint32_t stepNow = micros();
      if ((uint32_t)(stepNow - lastStepXUs) < MIN_STEP_INTERVAL_X_US) break;
      pulseStepHighLow(X_STEP_PIN);
      emittedStepsX++;
      lastStepXUs = stepNow;
    }

    if (t >= (totalMoveTime + 0.15f)) break;
    taskYIELD();
  }

  emitRemainingXSteps(emittedStepsX, totalStepsX, lastStepXUs);

  currentXcm = targetXcm;

  Serial.print("Current XY: (");
  Serial.print(currentXcm, 3);
  Serial.print(", ");
  Serial.print(currentYcm, 3);
  Serial.println(")");
}

// =====================================
// 1D Y move
// =====================================
void moveYTo(float targetYcm, float vmax, float accel, const Shaper3* shaper) {
  if (targetYcm < 0.0f) targetYcm = 0.0f;
  if (targetYcm > MAX_Y_CM) targetYcm = MAX_Y_CM;

  float dy = targetYcm - currentYcm;
  float distanceCm = fabsf(dy);
  if (distanceCm < 0.0001f) return;

  bool yPositive = (dy >= 0.0f);
  digitalWrite(Y_DIR_PIN, yPositive ? Y_DIR_POSITIVE_LEVEL : !Y_DIR_POSITIVE_LEVEL);
  delay(5);

  long totalStepsY = lroundf(distanceCm * Y_STEPS_PER_CM);
  MotionProfile profile = makeProfile(distanceCm, vmax, accel);
  float totalMoveTime = shaper ? shapedTotalTime(profile, *shaper) : profile.totalTime;

  Serial.println("----------------------------------------");
  Serial.print("Move Y: ");
  Serial.print(currentYcm, 3);
  Serial.print(" -> ");
  Serial.print(targetYcm, 3);
  Serial.print(" cm | ");
  if (shaper) {
    Serial.print("WITH ");
    Serial.println(shaper->name);
  } else {
    Serial.println("NO SHAPER");
  }

  uint32_t moveStartUs = micros();
  uint32_t lastStepYUs = micros();
  long emittedStepsY = 0;

  while (true) {
    uint32_t nowUs = micros();
    float t = (nowUs - moveStartUs) * 1e-6f;

    float cmdDist = shaper ? shapedPosCm(profile, t, *shaper) : basePosCm(profile, t);
    long desiredStepsY = lroundf(cmdDist * Y_STEPS_PER_CM);
    if (desiredStepsY > totalStepsY) desiredStepsY = totalStepsY;

    while (emittedStepsY < desiredStepsY) {
      uint32_t stepNow = micros();
      if ((uint32_t)(stepNow - lastStepYUs) < MIN_STEP_INTERVAL_Y_US) break;
      pulseStepHighLow(Y_STEP_PIN);
      emittedStepsY++;
      lastStepYUs = stepNow;
    }

    if (t >= (totalMoveTime + 0.15f)) break;
    taskYIELD();
  }

  emitRemainingYSteps(emittedStepsY, totalStepsY, lastStepYUs);

  currentYcm = targetYcm;

  Serial.print("Current XY: (");
  Serial.print(currentXcm, 3);
  Serial.print(", ");
  Serial.print(currentYcm, 3);
  Serial.println(")");
}

// =====================================
// Coordinated XY move (no shaping)
// =====================================
void moveToXYCoordinated(float targetXcm, float targetYcm, float vmax, float accel) {
  if (targetXcm < 0.0f) targetXcm = 0.0f;
  if (targetXcm > MAX_X_CM) targetXcm = MAX_X_CM;
  if (targetYcm < 0.0f) targetYcm = 0.0f;
  if (targetYcm > MAX_Y_CM) targetYcm = MAX_Y_CM;

  float dx = targetXcm - currentXcm;
  float dy = targetYcm - currentYcm;

  float D = sqrtf(dx * dx + dy * dy);
  if (D < 0.0001f) return;

  long totalStepsX = lroundf(fabsf(dx) * X_STEPS_PER_CM);
  long totalStepsY = lroundf(fabsf(dy) * Y_STEPS_PER_CM);

  bool xPositive = (dx >= 0.0f);
  bool yPositive = (dy >= 0.0f);

  digitalWrite(X_DIR_PIN, xPositive ? X_DIR_POSITIVE_LEVEL : !X_DIR_POSITIVE_LEVEL);
  digitalWrite(Y_DIR_PIN, yPositive ? Y_DIR_POSITIVE_LEVEL : !Y_DIR_POSITIVE_LEVEL);
  delay(5);

  MotionProfile profile = makeProfile(D, vmax, accel);

  Serial.println("----------------------------------------");
  Serial.print("Move XY coordinated: (");
  Serial.print(currentXcm, 3);
  Serial.print(", ");
  Serial.print(currentYcm, 3);
  Serial.print(") -> (");
  Serial.print(targetXcm, 3);
  Serial.print(", ");
  Serial.print(targetYcm, 3);
  Serial.println(")");

  uint32_t moveStartUs = micros();
  uint32_t lastStepXUs = micros();
  uint32_t lastStepYUs = micros();

  long emittedStepsX = 0;
  long emittedStepsY = 0;

  while (true) {
    uint32_t nowUs = micros();
    float t = (nowUs - moveStartUs) * 1e-6f;

    float traveled = basePosCm(profile, t);
    if (traveled > D) traveled = D;

    float progress = traveled / D;
    float xCmd = fabsf(dx) * progress;
    float yCmd = fabsf(dy) * progress;

    long desiredStepsX = lroundf(xCmd * X_STEPS_PER_CM);
    long desiredStepsY = lroundf(yCmd * Y_STEPS_PER_CM);

    if (desiredStepsX > totalStepsX) desiredStepsX = totalStepsX;
    if (desiredStepsY > totalStepsY) desiredStepsY = totalStepsY;

    while (emittedStepsX < desiredStepsX) {
      uint32_t stepNow = micros();
      if ((uint32_t)(stepNow - lastStepXUs) < MIN_STEP_INTERVAL_X_US) break;
      pulseStepHighLow(X_STEP_PIN);
      emittedStepsX++;
      lastStepXUs = stepNow;
    }

    while (emittedStepsY < desiredStepsY) {
      uint32_t stepNow = micros();
      if ((uint32_t)(stepNow - lastStepYUs) < MIN_STEP_INTERVAL_Y_US) break;
      pulseStepHighLow(Y_STEP_PIN);
      emittedStepsY++;
      lastStepYUs = stepNow;
    }

    if (t >= (profile.totalTime + 0.15f)) break;
    taskYIELD();
  }

  emitRemainingXSteps(emittedStepsX, totalStepsX, lastStepXUs);
  emitRemainingYSteps(emittedStepsY, totalStepsY, lastStepYUs);

  currentXcm = targetXcm;
  currentYcm = targetYcm;

  Serial.print("Current XY: (");
  Serial.print(currentXcm, 3);
  Serial.print(", ");
  Serial.print(currentYcm, 3);
  Serial.println(")");
}

// =====================================
// Smart scan segment WITHOUT shaping
// =====================================
void movePlateSegmentNoShaper(float targetXcm, float targetYcm) {
  float dx = targetXcm - currentXcm;
  float dy = targetYcm - currentYcm;

  bool pureX = (fabsf(dy) < 0.0001f) && (fabsf(dx) >= 0.0001f);
  bool pureY = (fabsf(dx) < 0.0001f) && (fabsf(dy) >= 0.0001f);

  if (pureX) {
    moveXTo(targetXcm, FAST_V_CM_S, FAST_A_CM_S2, nullptr);
  } else if (pureY) {
    moveYTo(targetYcm, FAST_V_CM_S, FAST_A_CM_S2, nullptr);
  } else {
    moveToXYCoordinated(targetXcm, targetYcm, FAST_V_CM_S, FAST_A_CM_S2);
  }

  delay(PAUSE_SCAN_MS);
}

// =====================================
// Smart scan segment WITH shaping
// =====================================
void movePlateSegmentShaped(float targetXcm, float targetYcm) {
  float dx = targetXcm - currentXcm;
  float dy = targetYcm - currentYcm;

  bool pureX = (fabsf(dy) < 0.0001f) && (fabsf(dx) >= 0.0001f);
  bool pureY = (fabsf(dx) < 0.0001f) && (fabsf(dy) >= 0.0001f);

  if (pureX) {
    const Shaper3* xShaper = selectXRowShaper(currentYcm);
    moveXTo(targetXcm, FAST_V_CM_S, FAST_A_CM_S2, xShaper);
  } else if (pureY) {
    const Shaper3* yShaper = selectYVerticalShaper(currentYcm, targetYcm);
    moveYTo(targetYcm, FAST_V_CM_S, FAST_A_CM_S2, yShaper);
  } else {
    moveToXYCoordinated(targetXcm, targetYcm, FAST_V_CM_S, FAST_A_CM_S2);
  }

  delay(PAUSE_SCAN_MS);
}

// =====================================
// Common entry path
// =====================================
void goToReferenceAndFirstPointSlow() {
  moveToXYCoordinated(24.7f, 17.0f, SLOW_V_CM_S, SLOW_A_CM_S2);
  delay(PAUSE_AT_REF_MS);

  moveToXYCoordinated(22.2f, 14.5f, SLOW_V_CM_S, SLOW_A_CM_S2);
  delay(PAUSE_AT_FIRST_MS);
}

// =====================================
// Run WITHOUT shaping
// =====================================
void runWithoutShaping() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("RUN 1: WITHOUT INPUT SHAPING");
  Serial.println("========================================");

  currentXcm = 0.0f;
  currentYcm = 0.0f;

  goToReferenceAndFirstPointSlow();

  for (int i = 1; i < NUM_SCAN_POINTS; i++) {
    movePlateSegmentNoShaper(scanPoints[i].x, scanPoints[i].y);
  }

  delay(PAUSE_END_MS);
  moveToXYCoordinated(0.0f, 0.0f, SLOW_V_CM_S, SLOW_A_CM_S2);
}

// =====================================
// Run WITH shaping
// =====================================
void runWithShaping() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("RUN 2: WITH INPUT SHAPING (X + Y)");
  Serial.println("========================================");

  currentXcm = 0.0f;
  currentYcm = 0.0f;

  goToReferenceAndFirstPointSlow();

  for (int i = 1; i < NUM_SCAN_POINTS; i++) {
    movePlateSegmentShaped(scanPoints[i].x, scanPoints[i].y);
  }

  delay(PAUSE_END_MS);
  moveToXYCoordinated(0.0f, 0.0f, SLOW_V_CM_S, SLOW_A_CM_S2);
}

// =====================================
// Setup / Loop
// =====================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);

  Serial.println("=== EVA 33 ===");
  Serial.println("Plate scan comparison:");
  Serial.println("1) WITHOUT input shaping");
  Serial.println("2) WITH input shaping in X and Y");
  Serial.println();
  Serial.println("Middle row used: Y = 9.5");
  Serial.println("Put the robot at (0,0) before reset.");
  delay(2000);

  runWithoutShaping();

  delay(PAUSE_BETWEEN_RUNS_MS);

  runWithShaping();

  Serial.println("=== END ===");
}

void loop() {
}