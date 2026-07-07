// Line follower 6 sensor TCRT5000 dengan Arduino Uno dan L298N
#define NUM_SENSORS 6

// Konfigurasi pin
int sensorPin[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
int ENA = 5;  // Enable motor A (kiri)
int IN1 = 8;  // Motor A input 1
int IN2 = 9;  // Motor A input 2
int IN3 = 10; // Motor B input 1
int IN4 = 11; // Motor B input 2
int ENB = 6;  // Enable motor B (kanan)

// Parameter kecepatan
int baseSpeed = 150;     // Kecepatan dasar
int maxSpeed = 200;      // Kecepatan maksimum
int turnSpeed = 120;     // Kecepatan belok

// Kalibrasi sensor
int sensorMin[NUM_SENSORS] = {1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[NUM_SENSORS] = {0, 0, 0, 0, 0, 0};
int sensorThreshold[NUM_SENSORS] = {500, 500, 500, 500, 500, 500};
int sensor[NUM_SENSORS];          // Nilai biner (0 = garis hitam, 1 = latar putih)
int sensorAnalog[NUM_SENSORS];    // Nilai analog ternormalisasi 0-1000 (dari kalibrasi)

// Parameter PID
float Kp = 25;           // Konstanta proporsional
float Ki = 0;            // Konstanta integral
float Kd = 15;           // Konstanta derivatif
float lastError = 0;     // Error sebelumnya
float integral = 0;      // Nilai integral

// Status line
int lineLostCounter = 0;
bool wasSearching = false;

void setup() {
  // Inisialisasi pin sensor
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPin[i], INPUT);
  }

  // Inisialisasi pin motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);
  Serial.println("Line Follower Robot Starting...");

  // Kalibrasi sensor
  calibrateSensors();
}

void loop() {
  bacaSensor();

  bool allWhite = (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 &&
                   sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1);
  bool allBlack = (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 &&
                   sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0);

  if (allWhite) {
    // Mode darurat — robot kehilangan garis
    lineLostCounter++;
    searchLine();
    wasSearching = true;
  }
  else if (allBlack) {
    // Persimpangan terdeteksi — lanjutkan lurus
    lineLostCounter = 0;

    // Reset PID state saat kembali ke garis
    if (wasSearching) {
      integral = 0;
      lastError = 0;
      wasSearching = false;
    }

    maju(baseSpeed);
  }
  else {
    // Mode normal — ikuti garis dengan PID
    lineLostCounter = 0;

    // Kalkulasi posisi garis (0-5000), 2500 = tengah
    int position = calculatePosition();
    int error = position - 2500;

    // Reset PID state saat kembali dari mode search
    if (wasSearching) {
      integral = 0;
      lastError = error;
      wasSearching = false;
    }

    // Kalkulasi PID
    float derivative = error - lastError;
    integral += error;
    lastError = error;

    // Anti-windup integral
    if (integral > 10000) integral = 10000;
    if (integral < -10000) integral = -10000;

    // Koreksi kecepatan
    float motorSpeed = Kp * error + Ki * integral + Kd * derivative;

    int leftMotorSpeed = baseSpeed - motorSpeed;
    int rightMotorSpeed = baseSpeed + motorSpeed;

    // Batas kecepatan
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;

    controlMotors(leftMotorSpeed, rightMotorSpeed);
    printDebugInfo(leftMotorSpeed, rightMotorSpeed, error, position);
  }

  delay(10);
}

void bacaSensor() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int nilai = analogRead(sensorPin[i]);

    // Normalisasi 0-1000 berdasarkan kalibrasi
    int normalized = map(nilai, sensorMin[i], sensorMax[i], 0, 1000);
    normalized = constrain(normalized, 0, 1000);

    sensorAnalog[i] = normalized;                       // simpan nilai analog ternormalisasi (untuk calculatePosition)
    sensor[i] = (normalized > sensorThreshold[i]) ? 1 : 0;  // simpan nilai biner (untuk logika)
  }
}

// Posisi garis: 0 (paling kiri) — 5000 (paling kanan), 2500 = tengah
int calculatePosition() {
  boolean onLine = false;
  long sum = 0;
  long weightedSum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Balik: nilai analog rendah = garis hitam → jadi tinggi
    int value = 1000 - sensorAnalog[i];

    if (value > 200) {
      onLine = true;
    }

    sum += value;
    weightedSum += (long)value * (i * 1000);
  }

  if (onLine && sum > 0) {
    return weightedSum / sum;
  }

  // Fallback: sensor terluar mana yang masih lihat hitam?
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor[i] == 0) {
      return (i < NUM_SENSORS / 2) ? 0 : 5000;
    }
  }

  return 2500; // tengah
}

void controlMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void maju(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void belokKiri(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void belokKanan(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void searchLine() {
  // Fase 1 (0-9): putar ke arah terakhir garis diketahui
  if (lineLostCounter < 10) {
    if (lastError < 0) {
      belokKiri(turnSpeed);
    } else {
      belokKanan(turnSpeed);
    }
  }
  // Fase 2 (10-19): putar kebalikan
  else if (lineLostCounter < 20) {
    if (lastError < 0) {
      belokKanan(turnSpeed);
    } else {
      belokKiri(turnSpeed);
    }
  }
  // Fase 3 (20+): spin di tempat, bergantian arah
  else {
    if (lineLostCounter % 2 == 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, turnSpeed);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, turnSpeed);
    } else {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, turnSpeed);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, turnSpeed);
    }
  }
}

void calibrateSensors() {
  Serial.println("Kalibrasi sensor dimulai — gerakkan robot di atas garis hitam dan latar putih...");

  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      int value = analogRead(sensorPin[j]);

      if (value < sensorMin[j]) sensorMin[j] = value;
      if (value > sensorMax[j]) sensorMax[j] = value;
    }
    delay(10);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] > sensorMin[i] + 100) {
      sensorThreshold[i] = (sensorMax[i] + sensorMin[i]) / 2;
    } else {
      sensorThreshold[i] = 500;
    }

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(sensorMin[i]);
    Serial.print(", Max=");
    Serial.print(sensorMax[i]);
    Serial.print(", Threshold=");
    Serial.println(sensorThreshold[i]);
  }

  Serial.println("Kalibrasi selesai!");
  delay(1000);
}

void printDebugInfo(int leftSpeed, int rightSpeed, int error, int position) {
  Serial.print("Pos:");
  Serial.print(position);
  Serial.print(" S:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensor[i]);
  }
  Serial.print(" L:");
  Serial.print(leftSpeed);
  Serial.print(" R:");
  Serial.print(rightSpeed);
  Serial.print(" Err:");
  Serial.print(error);
  Serial.print(" Lost:");
  Serial.println(lineLostCounter);
}
