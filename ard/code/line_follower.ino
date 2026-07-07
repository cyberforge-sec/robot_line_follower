/**
 * Robot Line Follower — 6 TCRT5000 Sensors + PID Control
 *
 * Hardware: Arduino Uno R3, 6× TCRT5000, L298N Motor Driver, 2× DC Motor
 * Language: English (code) / Bahasa Indonesia (comments)
 *
 * Features:
 *   - Automatic sensor calibration on startup
 *   - PID-based path tracking (configurable constants)
 *   - Progressive line search when line is lost
 *   - Intersection detection (all sensors black)
 *   - Non-blocking timer-based loop
 *   - Serial debug output
 */

// ============================================================================
// CONSTANTS & CONFIGURATION
// ============================================================================

#define NUM_SENSORS 6

// --- Pin Assignments ---
const int SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
const int PIN_ENA = 5;   // L298N Enable A (left motor)
const int PIN_IN1 = 8;   // L298N Input 1
const int PIN_IN2 = 9;   // L298N Input 2
const int PIN_IN3 = 10;  // L298N Input 3
const int PIN_IN4 = 11;  // L298N Input 4
const int PIN_ENB = 6;   // L298N Enable B (right motor)

// --- Speed Parameters ---
const int SPEED_BASE = 150;   // Base speed for normal tracking
const int SPEED_MAX = 200;    // Maximum speed limit
const int SPEED_TURN = 120;   // Speed during line search
const int SPEED_SLOW = 80;    // Slow speed for precise turns

// --- PID Constants (tune these for your track) ---
const float KP = 25.0;   // Proportional gain
const float KI = 0.0;    // Integral gain
const float KD = 15.0;   // Derivative gain

// --- PID Limits ---
const float INTEGRAL_MAX = 10000.0;   // Anti-windup clamp
const int POSITION_CENTER = 2500;     // Center position (0-5000 range)

// --- Line Detection Thresholds ---
const int LINE_DETECT_THRESHOLD = 200;  // Normalized value above this = line detected
const int CALIBRATION_MIN_RANGE = 100;  // Min sensor range for valid calibration

// --- Search Strategy ---
const int SEARCH_PHASE_1_CYCLES = 10;
const int SEARCH_PHASE_2_CYCLES = 10;
const int LOOP_DELAY_MS = 10;

// ============================================================================
// GLOBAL STATE
// ============================================================================

// --- Calibration Data ---
int sensorMin[NUM_SENSORS] = {1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[NUM_SENSORS] = {0, 0, 0, 0, 0, 0};
int sensorThreshold[NUM_SENSORS] = {500, 500, 500, 500, 500, 500};

// --- Sensor Readings ---
int sensorAnalog[NUM_SENSORS];   // Normalized analog values (0-1000)
bool sensorBinary[NUM_SENSORS];  // true = white (line absent), false = black (line detected)

// --- PID State ---
float pidError = 0.0;
float pidIntegral = 0.0;
float pidLastError = 0.0;

// --- Search State ---
int lineLostCounter = 0;
bool wasSearching = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Initialize motor pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("=== Robot Line Follower ==="));
  Serial.println(F("6x TCRT5000 + PID Control"));
  Serial.println(F(""));

  calibrateSensors();
}

// ============================================================================
// MAIN LOOP — State Machine
// ============================================================================

void loop() {
  readSensors();

  bool allWhite = true;
  bool allBlack = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorBinary[i] == false) allWhite = false;  // at least one sees black
    if (sensorBinary[i] == true)  allBlack = false;   // at least one sees white
  }

  if (allWhite) {
    // EMERGENCY — line completely lost
    lineLostCounter++;
    searchLine();
    wasSearching = true;
  }
  else if (allBlack) {
    // INTERSECTION — all sensors over black line → go straight
    lineLostCounter = 0;
    resetPidOnReacquire(0);
    motorForward(SPEED_BASE);
  }
  else {
    // NORMAL — follow line with PID
    lineLostCounter = 0;

    int position = calculatePosition();
    int error = position - POSITION_CENTER;

    if (wasSearching) {
      resetPidOnReacquire(error);
    }

    float correction = computePid(error);
    applyMotorCorrection(correction);

    printDebug(position, error);
  }

  delay(LOOP_DELAY_MS);
}

// ============================================================================
// SENSORS
// ============================================================================

/** Read all sensors, update binary + analog arrays */
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(SENSOR_PINS[i]);

    // Map raw 0-1023 to normalized 0-1000 using calibration bounds
    int normalized = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
    normalized = constrain(normalized, 0, 1000);

    sensorAnalog[i] = normalized;
    sensorBinary[i] = (normalized > sensorThreshold[i]);
  }
}

/** Calculate line position using weighted average (0 = far left, 5000 = far right, 2500 = center) */
int calculatePosition() {
  bool lineDetected = false;
  long sum = 0;
  long weightedSum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Invert: low analog value = black line → becomes high
    int value = 1000 - sensorAnalog[i];

    if (value > LINE_DETECT_THRESHOLD) {
      lineDetected = true;
    }

    sum += value;
    weightedSum += (long)value * (i * 1000);
  }

  if (lineDetected && sum > 0) {
    return (int)(weightedSum / sum);
  }

  // No line detected via analog → fallback to binary
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorBinary[i] == false) {
      return (i < NUM_SENSORS / 2) ? 0 : 5000;
    }
  }

  return POSITION_CENTER;  // true center fallback
}

// ============================================================================
// CALIBRATION
// ============================================================================

/** Auto-calibrate sensors by sampling min/max over ~5 seconds */
void calibrateSensors() {
  Serial.println(F("Kalibrasi: gerakkan robot di atas garis hitam dan latar putih..."));

  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      int value = analogRead(SENSOR_PINS[j]);

      if (value < sensorMin[j]) sensorMin[j] = value;
      if (value > sensorMax[j]) sensorMax[j] = value;
    }
    delay(10);
  }

  Serial.println(F("Hasil Kalibrasi:"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] > sensorMin[i] + CALIBRATION_MIN_RANGE) {
      sensorThreshold[i] = (sensorMax[i] + sensorMin[i]) / 2;
    } else {
      sensorThreshold[i] = 500;  // fallback
    }

    Serial.print(F("  Sensor "));
    Serial.print(i);
    Serial.print(F(": Min="));
    Serial.print(sensorMin[i]);
    Serial.print(F(", Max="));
    Serial.print(sensorMax[i]);
    Serial.print(F(", Threshold="));
    Serial.println(sensorThreshold[i]);
  }

  Serial.println(F("Kalibrasi selesai!"));
  delay(1000);
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

/** Compute PID output given position error (target - actual) */
float computePid(int error) {
  float derivative = error - pidLastError;
  pidIntegral += error;

  // Anti-windup: clamp integral term
  if (pidIntegral > INTEGRAL_MAX) pidIntegral = INTEGRAL_MAX;
  if (pidIntegral < -INTEGRAL_MAX) pidIntegral = -INTEGRAL_MAX;

  float output = KP * error + KI * pidIntegral + KD * derivative;

  pidLastError = error;

  return output;
}

/** Reset PID state when reacquiring the line after search */
void resetPidOnReacquire(int error) {
  pidIntegral = 0;
  pidLastError = error;
  wasSearching = false;
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

/** Apply signed correction to left/right motor speeds */
void applyMotorCorrection(float correction) {
  int leftSpeed = SPEED_BASE - (int)correction;
  int rightSpeed = SPEED_BASE + (int)correction;

  // Clamp to [0, SPEED_MAX]
  if (leftSpeed > SPEED_MAX)  leftSpeed = SPEED_MAX;
  if (rightSpeed > SPEED_MAX) rightSpeed = SPEED_MAX;
  if (leftSpeed < 0)  leftSpeed = 0;
  if (rightSpeed < 0) rightSpeed = 0;

  motorSet(leftSpeed, rightSpeed);
}

/** Set both motors to given speeds (both forward) */
void motorSet(int leftSpeed, int rightSpeed) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);

  analogWrite(PIN_ENA, leftSpeed);
  analogWrite(PIN_ENB, rightSpeed);
}

/** Drive straight forward */
void motorForward(int speed) {
  motorSet(speed, speed);
}

/** Pivot left (left backward, right forward) */
void motorSpinLeft(int speed) {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);

  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

/** Pivot right (left forward, right backward) */
void motorSpinRight(int speed) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);

  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

/** Skid-steer turn left (left slower, right faster, both forward) */
void motorTurnLeft(int speed) {
  motorSet(speed / 2, speed);
}

/** Skid-steer turn right (left faster, right slower, both forward) */
void motorTurnRight(int speed) {
  motorSet(speed, speed / 2);
}

/** Stop both motors */
void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);

  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// ============================================================================
// LINE SEARCH STRATEGY
// ============================================================================

/**
 * Progressive line search strategy:
 *   Phase 1: Turn toward last known line direction
 *   Phase 2: Reverse direction
 *   Phase 3: Pivot in place, alternating direction each cycle
 */
void searchLine() {
  if (lineLostCounter < SEARCH_PHASE_1_CYCLES) {
    // Phase 1 — head toward last known side
    if (pidLastError < 0) {
      motorTurnLeft(SPEED_TURN);
    } else {
      motorTurnRight(SPEED_TURN);
    }
  }
  else if (lineLostCounter < SEARCH_PHASE_1_CYCLES + SEARCH_PHASE_2_CYCLES) {
    // Phase 2 — try the opposite direction
    if (pidLastError < 0) {
      motorTurnRight(SPEED_TURN);
    } else {
      motorTurnLeft(SPEED_TURN);
    }
  }
  else {
    // Phase 3 — pivot in place, alternating every cycle
    if (lineLostCounter % 2 == 0) {
      motorSpinLeft(SPEED_TURN);
    } else {
      motorSpinRight(SPEED_TURN);
    }
  }
}

// ============================================================================
// DEBUG
// ============================================================================

void printDebug(int position, int error) {
  Serial.print(F("POS:"));
  Serial.print(position);
  Serial.print(F(" S:"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorBinary[i] ? 'W' : 'B');
  }
  Serial.print(F(" ERR:"));
  Serial.print(error);
  Serial.print(F(" LOST:"));
  Serial.print(lineLostCounter);
  Serial.println();
}
