#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "esp_timer.h"

// =====================================
// EVA 33
// Settling / frequency test
// CON input shaping ZVD 11 Hz
//
// Caso:
//   x, 4.5, min
//
// Flujo por trial:
//   1) ir suave a (22.2, 4.5) sin medir
//   2) medir antes / durante / despues del tramo X:
//        (22.2, 4.5) -> (17.2, 4.5)
//      usando input shaping ZVD 11 Hz
//   3) volver suave a (22.2, 4.5) sin medir
//   4) repetir 3 veces
//   5) volver a (0,0) sin medir
//
// Analizar principalmente: az
// =====================================

// ---------- Identificador ----------
const char* RUN_LABEL = "SETTLING_X_Y4P5_MIN_ZVD_11HZ";

// ---------- Pines eje X ----------
const int X_STEP_PIN = 26;
const int X_DIR_PIN  = 27;

// ---------- Pines eje Y ----------
const int Y_STEP_PIN = 25;
const int Y_DIR_PIN  = 33;

// ---------- Direccion positiva ----------
const int X_DIR_POSITIVE_LEVEL = HIGH;
const int Y_DIR_POSITIVE_LEVEL = HIGH;

// ---------- I2C MPU6050 ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const uint8_t MPU_ADDR = 0x68;

// ---------- Mecanica ----------
const float X_STEPS_PER_CM = 400.0f;
const float Y_STEPS_PER_CM = 400.0f;

// ---------- Limites ----------
const float MAX_X_CM = 25.0f;
const float MAX_Y_CM = 19.2f;

// ---------- Movimiento de medicion ----------
const float TEST_START_X_CM   = 22.2f;
const float TEST_START_Y_CM   = 4.5f;
const float TEST_TARGET_X_CM  = 17.2f;
const float TEST_TARGET_Y_CM  = 4.5f;

// ---------- Trials ----------
const int NUM_TRIALS = 3;

// ---------- Perfil rapido (tramo medido) ----------
const float FAST_V_CM_S  = 30.0f;
const float FAST_A_CM_S2 = 220.0f;

// ---------- Perfil suave (ir al punto / volver) ----------
const float SLOW_V_CM_S  = 14.0f;
const float SLOW_A_CM_S2 = 90.0f;

// ---------- Muestreo ----------
const uint32_t SAMPLE_PERIOD_US   = 2500;   // 400 Hz
const uint32_t BEFORE_MS          = 700;
const uint32_t AFTER_MS           = 2200;
const uint32_t BETWEEN_TRIALS_MS  = 800;

// ---------- STEP ----------
const unsigned int STEP_PULSE_US = 6;
const unsigned int MIN_STEP_INTERVAL_X_US = 85;
const unsigned int MIN_STEP_INTERVAL_Y_US = 85;

// ---------- ZVD 11 Hz ----------
const float SHAPER_FN   = 11.0f;
const float SHAPER_ZETA = 0.030f;

const float SHAPER_A1 = 0.274110f;
const float SHAPER_A2 = 0.498890f;
const float SHAPER_A3 = 0.226999f;

const float SHAPER_T1 = 0.000000f;
const float SHAPER_T2 = 0.045455f;
const float SHAPER_T3 = 0.090909f;

// ---------- Buffers ----------
const int MAX_SAMPLES = 1800;

uint32_t t_us_buf[MAX_SAMPLES];
uint16_t x_milli_cm_buf[MAX_SAMPLES];
uint16_t y_milli_cm_buf[MAX_SAMPLES];
int16_t ax_buf[MAX_SAMPLES];
int16_t ay_buf[MAX_SAMPLES];
int16_t az_buf[MAX_SAMPLES];
uint8_t phase_buf[MAX_SAMPLES];
uint8_t trial_buf[MAX_SAMPLES];

volatile int sampleCount = 0;

// ---------- Estado compartido ----------
volatile bool captureActive = false;
volatile bool loggingDone   = false;
volatile bool dumpDone      = false;

volatile uint8_t phaseState = 0;   // 0 antes, 1 durante, 2 despues
volatile uint8_t trialState = 0;
volatile int64_t captureT0Us = 0;
volatile uint16_t xCmdMilliCm = 0;
volatile uint16_t yCmdMilliCm = 0;

// ---------- Estado actual ----------
float currentXcm = 0.0f;
float currentYcm = 0.0f;

// =====================================
// Perfil trapezoidal
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

float baseDistanceCm(const MotionProfile &p, float t) {
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

float shapedDistanceZVD(const MotionProfile &p, float t) {
  float d =
      SHAPER_A1 * baseDistanceCm(p, t - SHAPER_T1) +
      SHAPER_A2 * baseDistanceCm(p, t - SHAPER_T2) +
      SHAPER_A3 * baseDistanceCm(p, t - SHAPER_T3);

  if (d < 0.0f) d = 0.0f;
  if (d > p.D)  d = p.D;
  return d;
}

// =====================================
// MPU6050
// =====================================
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool mpuReadAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  int n = Wire.requestFrom((int)MPU_ADDR, 6, true);
  if (n != 6) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  return true;
}

bool initMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  mpuWrite(0x6B, 0x00);
  delay(20);

  mpuWrite(0x1C, 0x00); // accel ±2g
  delay(10);

  mpuWrite(0x1B, 0x00); // gyro ±250
  delay(10);

  int16_t ax, ay, az;
  return mpuReadAccel(ax, ay, az);
}

// =====================================
// Logging
// =====================================
void clearBuffer() {
  sampleCount = 0;
}

void logSample() {
  if (!captureActive) return;

  int idx = sampleCount;
  if (idx >= MAX_SAMPLES) return;

  int16_t ax, ay, az;
  if (!mpuReadAccel(ax, ay, az)) return;

  t_us_buf[idx]       = (uint32_t)(esp_timer_get_time() - captureT0Us);
  x_milli_cm_buf[idx] = xCmdMilliCm;
  y_milli_cm_buf[idx] = yCmdMilliCm;
  ax_buf[idx]         = ax;
  ay_buf[idx]         = ay;
  az_buf[idx]         = az;
  phase_buf[idx]      = phaseState;
  trial_buf[idx]      = trialState;

  sampleCount = idx + 1;
}

void dumpCSVBlock(int trialNumber) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("TRIAL ");
  Serial.println(trialNumber);
  Serial.println("Movimiento medido: (22.2,4.5) -> (17.2,4.5) con ZVD 11 Hz");
  Serial.println("trial,t_us,phase,x_cm,y_cm,ax,ay,az");

  for (int i = 0; i < sampleCount; i++) {
    Serial.print(trial_buf[i]);
    Serial.print(",");
    Serial.print(t_us_buf[i]);
    Serial.print(",");
    Serial.print(phase_buf[i]);
    Serial.print(",");
    Serial.print(x_milli_cm_buf[i] / 1000.0f, 3);
    Serial.print(",");
    Serial.print(y_milli_cm_buf[i] / 1000.0f, 3);
    Serial.print(",");
    Serial.print(ax_buf[i]);
    Serial.print(",");
    Serial.print(ay_buf[i]);
    Serial.print(",");
    Serial.println(az_buf[i]);
  }

  Serial.print("FIN TRIAL ");
  Serial.println(trialNumber);
  Serial.println("========================================");
  Serial.println();
}

// =====================================
// STEP
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
// Movimiento XY coordinado sin log
// =====================================
void moveToXYCoordinatedNoLog(float targetXcm, float targetYcm, float vmax, float accel) {
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

  uint32_t moveStartUs = micros();
  uint32_t lastStepXUs = micros();
  uint32_t lastStepYUs = micros();

  long emittedStepsX = 0;
  long emittedStepsY = 0;

  while (true) {
    uint32_t nowUs = micros();
    float t = (nowUs - moveStartUs) * 1e-6f;

    float traveled = baseDistanceCm(profile, t);
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
}

// =====================================
// Movimiento X medido CON ZVD 11 Hz
// =====================================
void moveXMeasuredShapedZVD(float targetXcm, float vmax, float accel) {
  float startXcm = currentXcm;
  float dx = targetXcm - startXcm;
  float distanceCm = fabsf(dx);

  if (distanceCm < 0.0001f) return;

  bool xPositive = (dx >= 0.0f);
  digitalWrite(X_DIR_PIN, xPositive ? X_DIR_POSITIVE_LEVEL : !X_DIR_POSITIVE_LEVEL);
  delay(5);

  long totalStepsX = lroundf(distanceCm * X_STEPS_PER_CM);
  MotionProfile profile = makeProfile(distanceCm, vmax, accel);

  uint32_t moveStartUs = micros();
  uint32_t lastStepXUs = micros();
  long emittedStepsX = 0;

  const float shapedTotalTime = profile.totalTime + SHAPER_T3;

  while (true) {
    uint32_t nowUs = micros();
    float t = (nowUs - moveStartUs) * 1e-6f;

    float cmdDist = shapedDistanceZVD(profile, t);
    if (cmdDist > distanceCm) cmdDist = distanceCm;

    long desiredStepsX = lroundf(cmdDist * X_STEPS_PER_CM);
    if (desiredStepsX > totalStepsX) desiredStepsX = totalStepsX;

    float xAbs = startXcm + (dx >= 0.0f ? cmdDist : -cmdDist);
    xCmdMilliCm = (uint16_t)lroundf(xAbs * 1000.0f);
    yCmdMilliCm = (uint16_t)lroundf(currentYcm * 1000.0f);

    while (emittedStepsX < desiredStepsX) {
      uint32_t stepNow = micros();
      if ((uint32_t)(stepNow - lastStepXUs) < MIN_STEP_INTERVAL_X_US) break;
      pulseStepHighLow(X_STEP_PIN);
      emittedStepsX++;
      lastStepXUs = stepNow;
    }

    if (t >= (shapedTotalTime + 0.15f)) break;
    taskYIELD();
  }

  emitRemainingXSteps(emittedStepsX, totalStepsX, lastStepXUs);

  currentXcm = targetXcm;
  xCmdMilliCm = (uint16_t)lroundf(currentXcm * 1000.0f);
  yCmdMilliCm = (uint16_t)lroundf(currentYcm * 1000.0f);
}

// =====================================
// Sensor task
// =====================================
void sensorTask(void *parameter) {
  int64_t nextSample = esp_timer_get_time();

  while (true) {
    int64_t now = esp_timer_get_time();

    if (now >= nextSample) {
      logSample();
      nextSample += SAMPLE_PERIOD_US;
    } else {
      vTaskDelay(1);
    }
  }
}

// =====================================
// Motor task
// =====================================
void motorTask(void *parameter) {
  delay(1500);

  currentXcm = 0.0f;
  currentYcm = 0.0f;
  xCmdMilliCm = 0;
  yCmdMilliCm = 0;

  moveToXYCoordinatedNoLog(TEST_START_X_CM, TEST_START_Y_CM, SLOW_V_CM_S, SLOW_A_CM_S2);
  delay(700);

  for (int trial = 1; trial <= NUM_TRIALS; trial++) {
    Serial.println("----------------------------------------");
    Serial.print("Iniciando TRIAL ");
    Serial.println(trial);

    moveToXYCoordinatedNoLog(TEST_START_X_CM, TEST_START_Y_CM, SLOW_V_CM_S, SLOW_A_CM_S2);
    delay(500);

    clearBuffer();
    trialState = trial;
    captureT0Us = esp_timer_get_time();
    captureActive = true;

    phaseState = 0;
    xCmdMilliCm = (uint16_t)lroundf(currentXcm * 1000.0f);
    yCmdMilliCm = (uint16_t)lroundf(currentYcm * 1000.0f);

    int64_t beforeEnd = esp_timer_get_time() + (int64_t)BEFORE_MS * 1000LL;
    while (esp_timer_get_time() < beforeEnd) {
      vTaskDelay(1);
    }

    phaseState = 1;
    moveXMeasuredShapedZVD(TEST_TARGET_X_CM, FAST_V_CM_S, FAST_A_CM_S2);

    phaseState = 2;
    xCmdMilliCm = (uint16_t)lroundf(currentXcm * 1000.0f);
    yCmdMilliCm = (uint16_t)lroundf(currentYcm * 1000.0f);

    int64_t afterEnd = esp_timer_get_time() + (int64_t)AFTER_MS * 1000LL;
    while (esp_timer_get_time() < afterEnd) {
      vTaskDelay(1);
    }

    captureActive = false;
    delay(80);

    dumpCSVBlock(trial);

    if (trial < NUM_TRIALS) {
      moveToXYCoordinatedNoLog(TEST_START_X_CM, TEST_START_Y_CM, SLOW_V_CM_S, SLOW_A_CM_S2);
      delay(BETWEEN_TRIALS_MS);
    }
  }

  moveToXYCoordinatedNoLog(0.0f, 0.0f, SLOW_V_CM_S, SLOW_A_CM_S2);

  loggingDone = true;
  vTaskDelete(NULL);
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

  Serial.println("Inicializando MPU6050...");
  if (!initMPU6050()) {
    Serial.println("ERROR: no se pudo inicializar el MPU6050.");
    while (true) delay(1000);
  }

  Serial.println("MPU6050 OK.");
  Serial.println("=== EVA 33 ===");
  Serial.print("RUN_LABEL = ");
  Serial.println(RUN_LABEL);
  Serial.println("Settling / frequency test CON input shaping ZVD");
  Serial.println("Caso: x, 4.5, min");
  Serial.println("Movimiento medido: (22.2,4.5) -> (17.2,4.5)");
  Serial.println("Trials = 3");
  Serial.println("Cada trial vuelve a (22.2,4.5) sin medir");
  Serial.println("Al final vuelve a (0,0)");
  Serial.println("phase = 0 antes, 1 durante, 2 despues");
  Serial.println("Analiza principalmente az.");
  Serial.println("Pon el robot en (0,0) antes de resetear.");
  Serial.println();

  Serial.println("Parametros ZVD 11 Hz:");
  Serial.print("fn = ");   Serial.println(SHAPER_FN, 4);
  Serial.print("zeta = "); Serial.println(SHAPER_ZETA, 4);
  Serial.print("A1 = ");   Serial.println(SHAPER_A1, 6);
  Serial.print("A2 = ");   Serial.println(SHAPER_A2, 6);
  Serial.print("A3 = ");   Serial.println(SHAPER_A3, 6);
  Serial.print("T1 = ");   Serial.println(SHAPER_T1, 6);
  Serial.print("T2 = ");   Serial.println(SHAPER_T2, 6);
  Serial.print("T3 = ");   Serial.println(SHAPER_T3, 6);
  Serial.println();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorTask, "motorTask", 9000, NULL, 2, NULL, 1);
}

void loop() {
  if (loggingDone && !dumpDone) {
    Serial.println("=== FIN TOTAL DEL ENSAYO ===");
    dumpDone = true;
  }
}