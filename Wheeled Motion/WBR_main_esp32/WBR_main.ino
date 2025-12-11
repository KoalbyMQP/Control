#include <Wire.h>
#include "MPU6050.h"

// ---------------- Motor driver pins ----------------
const int IN1_L = 18;
const int IN2_L = 19;
const int IN1_R = 27;
const int IN2_R = 14;

// ---------------- Encoder pins ---------------------
const int encA_L = 32;
const int encB_L = 33;
const int encA_R = 34;
const int encB_R = 35;

volatile long long encoderCountL = 0;
volatile long long encoderCountR = 0;

// ---------------- IMU ------------------------------
MPU6050 mpu;
const int MPU_RESET_PIN = 23;


// complementary filter state
float pitch = 0.0;
unsigned long lastIMU = 0;
float gyroXOffset = 0;


// ---------------- Balancing Controller -------------
float Kp = -40.0;
float Kd = 1;

// stiction compensation threshold
float u_min = 0.2;
float u_max = 1;

// ---------------- Utility --------------------------
unsigned long startTime;

// ===================================================
// Encoder interrupts
// ===================================================
void IRAM_ATTR handleEncoderAL() {
  if (digitalRead(encA_L) == digitalRead(encB_L))
    encoderCountL++;
  else
    encoderCountL--;
}

void IRAM_ATTR handleEncoderAR() {
  if (digitalRead(encA_R) == digitalRead(encB_R))
    encoderCountR++;
  else
    encoderCountR--;
}

// ===================================================
// Motor helper: set PWM in range -1 to 1
// ===================================================
void setMotorPWM(int pin1, int pin2, float u) {
  u = constrain(u, -1.0f, 1.0f);
  int pwm = (int)(fabs(u) * 255);

  if (u > 0) {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  } else if (u < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, pwm);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

// ===================================================
// Stiction compensation: enforce minimum torque
// ===================================================
float applyStiction(float u) {
  if (u == 0) return 0;

  float mag = fabs(u);
  if (mag < u_min) mag = u_min;
  if (mag > u_max) mag = u_max;

  return (u > 0 ? mag : -mag);
}

// ===================================================
// Complementary filter for pitch with I2C timeout
// ===================================================
float readPitch() {
    // Raw sensor readings
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    // Try reading the MPU6050 with timeout
    unsigned long start = micros();
    const unsigned long TIMEOUT_US = 5000; // 5 ms
    bool ok = false;
    
    while (true) {
        ax = mpu.getAccelerationX();
        ay = mpu.getAccelerationY();
        az = mpu.getAccelerationZ();
        gx = mpu.getRotationX();
        gy = mpu.getRotationY();
        gz = mpu.getRotationZ();

        // valid reading if accel is non-zero
        if (ax != 0 || ay != 0 || az != 0) {
            ok = true;
            break;
        }

        if (micros() - start > TIMEOUT_US) break; // timeout
    }

    if (!ok) {
        Serial.println("MPU6050 read failed, re-initializing sensor!");
        mpu.initialize();      // software re-init
        lastIMU = micros();    // reset filter timer
        pitch = 0;             // reset complementary filter
        return pitch;          // hold last known pitch
    }

    // Convert raw readings to physical units
    float accel_x = az / 16384.0;         // g
    float accel_y = ay / 16384.0;         // g
    float gyro_x  = (gx - gyroXOffset) / 131.0;  // deg/s, remove offset

    // Compute time step
    unsigned long now = micros();
    float dt = (now - lastIMU) * 1e-6;
    if (dt <= 0 || dt > 0.02) dt = 0.002;  // safety
    lastIMU = now;

    // Compute pitch from accelerometer
    float accel_pitch = atan2(accel_x, -accel_y) * 180.0 / PI;


    // Complementary filter
    pitch = 0.98 * (pitch + gyro_x * dt) + 0.02 * accel_pitch;

    // Serial.print("Accel pitch: ");
    // Serial.print(accel_pitch, 2);
    // Serial.print(" | Gyro reading: ");
    // Serial.println(gyro_x, 2);

    return pitch;
}


void calibrateGyro() {
  const int N = 500;  // number of samples to average
  long sum = 0;

  Serial.println("Calibrating gyro X...");

  for (int i = 0; i < N; i++) {
    sum += mpu.getRotationX(); // raw gyro X
    delay(2);                  // small delay between samples
  }

  gyroXOffset = sum / (float)N;
  Serial.print("Gyro X offset: ");
  Serial.println(gyroXOffset);
}

// ===================================================
// Setup
// ===================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Setting up motors and encoders...");

  // Motor pins
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Encoders
  pinMode(encA_L, INPUT);
  pinMode(encB_L, INPUT);
  pinMode(encA_R, INPUT);
  pinMode(encB_R, INPUT);

  attachInterrupt(digitalPinToInterrupt(encA_L), handleEncoderAL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA_R), handleEncoderAR, CHANGE);

  // IMU init
  Wire.begin(21, 22);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not found! Check wiring and power.");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  pinMode(MPU_RESET_PIN, OUTPUT);
  digitalWrite(MPU_RESET_PIN, HIGH);  // keep MPU6050 running

  Serial.println("Calibrating Gyro... If you are moving the robot you fucked it up");
  calibrateGyro();
  Serial.println("Done calibrating Gyro");

  delay(1000);
  lastIMU = micros();
  startTime = millis();
}

// ===================================================
// Main Loop
// ===================================================
void loop() {
  static unsigned long lastControl = 0;
  unsigned long now = micros();

  if (now - lastControl >= 2000) { // ~500 Hz
    lastControl = now;

    // Serial.println("Control loop start"); // DEBUG

    // 1. Read IMU
    float angle = readPitch();
    // Serial.println("After readPitch");

    float rate = mpu.getRotationY() / 131.0;
    // Serial.println("After gyro read");

    // 2. BALANCE CONTROLLER (PD)
    float error = angle;
    float u = Kp * error + Kd * rate;

    float uL = -u / 300.0;
    float uR = -u / 300.0;

    // 3. Apply stiction
    if (fabs(uL) > 0) uL = applyStiction(uL);
    if (fabs(uR) > 0) uR = applyStiction(uR);

    // 4. Send to motors
    setMotorPWM(IN1_L, IN2_L, uL);
    setMotorPWM(IN1_R, IN2_R, uR);
    // Serial.println("Motors updated");
  }

  // Telemetry at 50 Hz
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 20) {
    lastPrint = millis();

    float timeSec = (millis() - startTime) / 1000.0;
    char telemetry[64];
    sprintf(telemetry, "%lld,%lld,%.1f,%.3f", encoderCountL, encoderCountR, pitch, timeSec);
    Serial.println(telemetry);
  }
}
