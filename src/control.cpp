#include "control.h"
#include "altimeter.h"
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <math.h>

// Handles altitude estimation and control surface logic

// --- IMU Config ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

struct Quaternion {
  float w, x, y, z;
};

Quaternion q_initial = {1, 0, 0, 0};
bool isZeroed = false;
bool isCalibrated = false;

// --- Control Gain ---
float Kp = 0.5;
float Kv = 0.2;

Servo servo1, servo2, servo3, servo4;

int s1, s2, s3, s4;

float adjustedPitch, adjustedRoll, adjustedYaw;
float filteredPitch = 0, filteredRoll = 0, filteredYaw = 0;

float deadband = 0.5;
float norm;

// Update rate control
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 20; // milliseconds

// Low-pass filter coefficient
float alpha = 0.7;

// Initialization
void initServos() {
  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(8);
  servo4.attach(9);
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found");
    while (1);
  }

  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
}

void updateIMUandServos() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) return;

  float q0 = sensorValue.un.gameRotationVector.real;
  float q1 = sensorValue.un.gameRotationVector.i;
  float q2 = sensorValue.un.gameRotationVector.j;
  float q3 = sensorValue.un.gameRotationVector.k;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (abs(norm - 1.0f) > 0.01f) {
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
  }

  Quaternion q_current = {q0, q1, q2, q3};

  if (!isZeroed) {
    q_initial = q_current;
    isZeroed = true;
    Serial.println("IMU orientation zeroed.");
    Serial.println("Calibrating IMU...");
    return;
  }

  Quaternion q_inv = {q_initial.w, -q_initial.x, -q_initial.y, -q_initial.z};

  Quaternion q_rel;
  q_rel.w = q_inv.w*q_current.w - q_inv.x*q_current.x - q_inv.y*q_current.y - q_inv.z*q_current.z;
  q_rel.x = q_inv.w*q_current.x + q_inv.x*q_current.w + q_inv.y*q_current.z - q_inv.z*q_current.y;
  q_rel.y = q_inv.w*q_current.y - q_inv.x*q_current.z + q_inv.y*q_current.w + q_inv.z*q_current.x;
  q_rel.z = q_inv.w*q_current.z + q_inv.x*q_current.y - q_inv.y*q_current.x + q_inv.z*q_current.w;

  float sinr_cosp = 2.0 * (q_rel.w * q_rel.x + q_rel.y * q_rel.z);
  float cosr_cosp = 1.0 - 2.0 * (q_rel.x * q_rel.x + q_rel.y * q_rel.y);
  float roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0 * (q_rel.w * q_rel.y - q_rel.z * q_rel.x);
  float pitch = (abs(sinp) >= 1) ? copysign(PI / 2, sinp) : asin(sinp);

  float siny_cosp = 2.0 * (q_rel.w * q_rel.z + q_rel.x * q_rel.y);
  float cosy_cosp = 1.0 - 2.0 * (q_rel.y * q_rel.y + q_rel.z * q_rel.z);
  float yaw = atan2(siny_cosp, cosy_cosp);

  // Convert to degrees
  roll  *= 180.0 / PI;
  pitch *= 180.0 / PI;
  yaw   *= 180.0 / PI;

  // Axis orientation
  adjustedPitch = pitch;
  adjustedRoll  = roll;
  adjustedYaw   = yaw;

  // Low-pass filtering
  filteredPitch = alpha * filteredPitch + (1 - alpha) * adjustedPitch;
  filteredRoll  = alpha * filteredRoll  + (1 - alpha) * adjustedRoll;
  filteredYaw   = alpha * filteredYaw   + (1 - alpha) * adjustedYaw;

  if (abs(filteredPitch) < deadband) filteredPitch = 0;
  if (abs(filteredRoll)  < deadband) filteredRoll = 0;
  if (abs(filteredYaw)   < deadband) filteredYaw = 0;

  // === Combine Orientation + Velocity into Control ===
  int correctionPitch = constrain((Kp * filteredPitch + Kv * velocity), -15, 15);
  int correctionRoll  = constrain((Kp * filteredRoll), -15, 15);
  int correctionYaw   = constrain((Kp * filteredYaw), -15, 15);

  s1 = constrain(79 - correctionPitch + correctionRoll , 45, 135);
  s2 = constrain(98 + correctionPitch + correctionRoll , 45, 135);
  s3 = constrain(88  + correctionRoll + correctionYaw, 45, 135);
  s4 = constrain(86  + correctionRoll - correctionYaw, 45, 135);

  // s1 = (79);
  // s2 = (98);
  // s3 = (88);
  // s4 = (86);

  if (millis() - lastUpdate >= updateInterval)
  {
  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);
  lastUpdate = millis();
  }

  // Serial debugging -- comment out before launch
  if (isCalibrated == true)
  {
  Serial.print("Pitch: "); Serial.print(filteredPitch);
  Serial.print(" | Roll: "); Serial.print(filteredRoll);
  Serial.print(" | Yaw: "); Serial.print(filteredYaw);
  Serial.print(" | Vel: "); Serial.print(velocity);
  Serial.print(" | Alt: "); Serial.println(relativeAltitude);
  }
}

void calibrateIMU(int sampleAmount)
{
  for (int i = 0; i < sampleAmount; i++)
  {
    updateIMUandServos();
    delay(5);
  }
  isCalibrated = true;
  Serial.println("IMU calibrated...");
}
