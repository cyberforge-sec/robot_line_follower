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
int sensor[NUM_SENSORS];

// Parameter PID
float Kp = 25;           // Konstanta proporsional
float Ki = 0;            // Konstanta integral
float Kd = 15;           // Konstanta derivatif
float lastError = 0;     // Error sebelumnya
float integral = 0;      // Nilai integral

void setup() {
  // Inisialisasi pin
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPin[i], INPUT);
  }
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Line Follower Robot Starting...");
  
  // Kalibrasi sensor (opsional)
  calibrateSensors();
}

void loop() {
  bacaSensor();
  
  // Kalkulasi posisi garis
  int position = calculatePosition();
  
  // Hitung error
  // Posisi tengah ideal = 2500 (range 0-5000)
  int error = position - 2500;
  
  // Kalkulasi PID
  float derivative = error - lastError;
  integral += error;
  lastError = error;
  
  // Batasan nilai integral
  if (integral > 10000) integral = 10000;
  if (integral < -10000) integral = -10000;
  
  // Hitung koreksi kecepatan
  float motorSpeed = Kp * error + Ki * integral + Kd * derivative;
  
  // Set kecepatan motor berdasarkan nilai PID
  int leftMotorSpeed = baseSpeed - motorSpeed;
  int rightMotorSpeed = baseSpeed + motorSpeed;
  
  // Batasan kecepatan motor
  if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
  if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  
  // Mode darurat - jika semua sensor tidak mendeteksi garis
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && 
      sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1) {
    // Robot kehilangan garis, gunakan strategi pencarian
    searchLine();
  } 
  // Mode darurat - jika semua sensor mendeteksi garis (persimpangan)
  else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && 
           sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0) {
    // Persimpangan terdeteksi - lanjutkan lurus
    maju(baseSpeed);
  } 
  else {
    // Mode normal - ikuti garis dengan PID
    controlMotors(leftMotorSpeed, rightMotorSpeed);
  }
  
  // Debug - tampilkan nilai sensor dan kecepatan motor
  printDebugInfo(leftMotorSpeed, rightMotorSpeed, error);
  
  delay(10); // Delay kecil untuk stabilitas
}

void bacaSensor() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int nilai = analogRead(sensorPin[i]);
    // Normalisasi nilai berdasarkan kalibrasi
    nilai = map(nilai, sensorMin[i], sensorMax[i], 0, 1000);
    nilai = constrain(nilai, 0, 1000);
    
    // Tentukan status (0 = garis hitam, 1 = latar putih)
    sensor[i] = (nilai > sensorThreshold[i]) ? 1 : 0;
  }
}

// Kalkulasi posisi garis (0-5000), 2500 = tengah
int calculatePosition() {
  boolean onLine = false;
  int sum = 0;
  int weightedSum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = 1000 - (analogRead(sensorPin[i])); // Balik nilai (tinggi untuk garis)
    
    // Nilai tinggi = garis hitam
    if (value > 200) { // Pastikan ini adalah garis
      onLine = true;
    }
    
    sum += value;
    weightedSum += value * (i * 1000);
  }
  
  if (onLine && sum > 0) {
    return weightedSum / sum;
  } else {
    // Jika tidak ada garis terdeteksi, gunakan nilai error terakhir
    if (lastError < 0) {
      return 0; // Di ujung kiri
    } else {
      return 5000; // Di ujung kanan
    }
  }
}

void controlMotors(int leftSpeed, int rightSpeed) {
  // Set arah motor
  digitalWrite(IN1, HIGH);  // Motor kiri maju
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor kanan maju
  digitalWrite(IN4, LOW);
  
  // Set kecepatan motor
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
  // Strategi pencarian jika garis hilang
  // Berputar perlahan untuk mencari garis
  if (lastError < 0) {
    // Terakhir kali garis ada di kiri
    belokKiri(turnSpeed);
  } else {
    // Terakhir kali garis ada di kanan
    belokKanan(turnSpeed);
  }
}

void calibrateSensors() {
  Serial.println("Kalibrasi sensor mulai - Gerakan robot di atas garis hitam dan latar putih");
  
  // Fase kalibrasi - 5 detik untuk menggerakkan robot
  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      int value = analogRead(sensorPin[j]);
      
      // Update nilai minimum dan maksimum
      if (value < sensorMin[j]) {
        sensorMin[j] = value;
      }
      if (value > sensorMax[j]) {
        sensorMax[j] = value;
      }
    }
    delay(10);
  }
  
  // Hitung nilai threshold berdasarkan min dan max
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = 500; // Default
    
    // Jika kalibrasi berhasil, gunakan threshold di tengah
    if (sensorMax[i] > sensorMin[i] + 100) {
      sensorThreshold[i] = (sensorMax[i] + sensorMin[i]) / 2;
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

void printDebugInfo(int leftSpeed, int rightSpeed, int error) {
  // Print nilai sensor
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensor[i]);
    Serial.print(" ");
  }
  
  // Print kecepatan motor dan error
  Serial.print(" | L:");
  Serial.print(leftSpeed);
  Serial.print(" R:");
  Serial.print(rightSpeed);
  Serial.print(" | Err:");
  Serial.println(error);
}