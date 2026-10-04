#include <Wire.h>
#include <math.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>

#define MPU_ADDR 0x68
#define ALPHA 0.95

// CE on pin 9, CSN on pin 10
RF24 radio(9, 10);
const byte address[6] = "00001";

unsigned long prevTime;
float dt;
float roll = 0, pitch = 0;
float rollOffset = 0, pitchOffset = 0;

struct Angles {
  float yawAngle;
  float tiltAngle;
};
Angles angles;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(100); // stabilize sensor

  // --- Calibrate gyro zero position ---
  float rollSum = 0, pitchSum = 0;
  const int calibrationSamples = 200;  // take 200 samples for better averaging

  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true); // just accel data

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float roll_acc = atan2(ay_g, az_g) * 180 / PI;
    float pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;

    rollSum += roll_acc;
    pitchSum += pitch_acc;
    delay(5);
  }

  rollOffset = rollSum / calibrationSamples;
  pitchOffset = pitchSum / calibrationSamples;

  Serial.print("Offsets set. Roll offset: ");
  Serial.print(rollOffset);
  Serial.print(" Pitch offset: ");
  Serial.println(pitchOffset);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;

  unsigned long now = millis();
  dt = (now - prevTime) / 1000.0;
  prevTime = now;

  float roll_acc = atan2(ay_g, az_g) * 180 / PI;
  float pitch_acc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180 / PI;

  roll += gx_dps * dt;
  pitch += gy_dps * dt;

  roll  = ALPHA * roll + (1 - ALPHA) * roll_acc;
  pitch = ALPHA * pitch + (1 - ALPHA) * pitch_acc;

  float rollZeroed = roll - rollOffset;
  float pitchZeroed = pitch - pitchOffset;

  Serial.print("Roll: "); Serial.print(rollZeroed);
  Serial.print("  Pitch: "); Serial.println(pitchZeroed);
  
  delay(15);
}