#include <Wire.h>

// ===================== MPU6050 =====================
#define MPU6050_ADDR 0x68

int16_t AccX, AccY, AccZ;
int16_t GyroX;

// ===================== L298N =====================
#define AIN1 7
#define AIN2 8
#define PWMA 3

#define BIN1 12
#define BIN2 13
#define PWMB 5

// ===================== TUNING =====================
float Kp = 27;
float Ki = 0.0;     // add only after Kp/Kd are good (try 0.5 - 3)
float Kd = .29;     // acts on gyro rate (deg/s)

float targetAngle = -2.6;          // trim this to the true balance point

const float GAIN_A = 0.78;        // left motor  (PWMA, pin 3)
const float GAIN_B = 1.0; 
const int DEADZONE_A= 35;    // min PWM where motors actually move
const int DEADZONE_B= 40;    // min PWM where motors actually move


// Active braking: brake when nearly upright AND nearly not rotating
const float BRAKE_ANGLE_BAND = 1;   // degrees from target
const float BRAKE_RATE_BAND  = 5.0;   // deg/s

const float ALPHA = 0.98;                 // complementary filter
const unsigned long LOOP_US = 5000;       // 200 Hz
const float FALL_ANGLE = 60.0;

// Flip to -1 if the angle sign is backwards for your mounting
const float DIR = 1.0;

// ===================== STATE =====================
float angle = 0;
float gyroRate = 0;
float integral = 0;
float gyroOffset = 0;

unsigned long previousTime;
unsigned long lastPrint = 0;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);          // fast I2C

  initMPU();
  delay(100);

  calibrateGyro();

  readMPU();
  angle = accelAngle();

  previousTime = micros();
  Serial.println("Ready - hold upright");
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = micros();
  if (now - previousTime < LOOP_US) return;

  float dt = (now - previousTime) * 1e-6f;
  previousTime = now;
  if (dt > 0.05f) dt = LOOP_US * 1e-6f;

  readMPU();

  // --- Angle estimation ---
  gyroRate = -DIR * (GyroX - gyroOffset) / 131.0f;     // deg/s
  float accAngle = accelAngle();
  angle = ALPHA * (angle + gyroRate * dt) + (1.0f - ALPHA) * accAngle;

  // --- Fall protection (coast so wheels aren't locked) ---
  if (fabs(angle) > FALL_ANGLE) {
    stopMotors();
    integral = 0;
    return;
  }

  // --- PID ---
  float error = targetAngle - angle;

  integral += error * dt;
  integral = constrain(integral, -50, 50);

  float output = Kp * error + Ki * integral - Kd * gyroRate;
  output = constrain(output, -255, 255);

  // --- Active braking when balanced ---
  if (fabs(error) < BRAKE_ANGLE_BAND && fabs(gyroRate) < BRAKE_RATE_BAND) {
    brakeMotors();
  } else {
    moveMotors(output);
  }

  // --- Debug ---
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(angle);
    Serial.print('\t');
    Serial.println(output);
  }
}

// ===================== MPU6050 =====================
void initMPU() {
  writeReg(0x6B, 0x00);   // wake up
  writeReg(0x1A, 0x03);   // DLPF ~44 Hz
  writeReg(0x19, 0x00);   // sample rate divider
  writeReg(0x1B, 0x00);   // gyro  +-250 deg/s
  writeReg(0x1C, 0x00);   // accel +-2 g
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 14) != 14) return;

  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();

  AccX  = (int16_t)(b[0] << 8 | b[1]);
  AccY  = (int16_t)(b[2] << 8 | b[3]);
  AccZ  = (int16_t)(b[4] << 8 | b[5]);
  GyroX = (int16_t)(b[8] << 8 | b[9]);
}

float accelAngle() {
  return DIR * atan2((float)AccY,
                     sqrt((float)AccX * AccX + (float)AccZ * AccZ)) * 180.0f / PI;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - keep still...");
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    readMPU();
    sum += GyroX;
    delay(2);
  }
  gyroOffset = sum / 1000.0f;
  Serial.print("Gyro offset: ");
  Serial.println(gyroOffset);
}

// ===================== MOTORS =====================
void moveMotors(float output) {
  int mag = abs((int)output);

  // Tiny command -> brake instead of coast
  if (mag < 3) { brakeMotors(); return; }

  mag = constrain(mag, 0, 200);   // clamp BEFORE map

  int speedA = map(mag, 0, 200, DEADZONE_A, 255);
  int speedB = map(mag, 0, 200, DEADZONE_B, 255);

  speedA = constrain((int)(speedA * GAIN_A), 0, 255);
  speedB = constrain((int)(speedB * GAIN_B), 0, 255);

  bool fwd = output > 0;
  digitalWrite(AIN1, fwd);  digitalWrite(AIN2, !fwd);
  digitalWrite(BIN1, fwd);  digitalWrite(BIN2, !fwd);

  analogWrite(PWMA, speedA);
  analogWrite(PWMB, speedB);
}

// Active brake: enable HIGH + both inputs same level = motor terminals shorted
void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 250);
  analogWrite(PWMB, 250);
}

// Coast (free-spin)
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
