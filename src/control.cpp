#include "control.h"
#include "altimeter.h"
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "debug.h"
#include <algorithm>

// Handles altitude estimation and control surface logic

// --- IMU Config ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

IntervalTimer startTimer;
IntervalTimer stopTimer;

struct Quaternion {
  float w, x, y, z;
};

struct Servo {
  int pin;
  int angle;
};

Quaternion q_initial = {1, 0, 0, 0};
bool isZeroed = false;
bool isCalibrated = false;

// --- Control Gain ---
float Kp = 0.5;
float Kv = 0.2;

int servoP1 = 6, servoP2 = 7, servoP3 = 8, servoP4 = 9;

int s1, s2, s3, s4;
Servo servoArray[4] = 
{ 
  {servoP1 , s1},
  {servoP2 , s2},
  {servoP3 , s3},
  {servoP4 , s4}
};

float adjustedPitch, adjustedRoll, adjustedYaw;
float filteredPitch = 0, filteredRoll = 0, filteredYaw = 0, filteredAccel;

float maxDeflectionAngle = 20;
float deadband = 0.5;
float norm;
float accel = 0;

// Update rate control
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 5; // milliseconds - 5 ms is 200 hz

// Low-pass filter coefficient
float alpha = 0.7;

int pulseWidthArray[4];
volatile int PWMIndex = 0;

int pulseWidth(int servoAngle)
{
  return map(servoAngle, 0, 90, 1100, 1900);
}

// Initialization
void initIMU() {  
  if (!bno08x.begin_I2C(0x4A, &Wire1)) {
    Serial.println("BNO08x not found");
    errorStatusLED();
    while (1);
  }
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
}

void initServos()
{
  pinMode(servoP1, OUTPUT);
  pinMode(servoP2, OUTPUT);
  pinMode(servoP3, OUTPUT);
  pinMode(servoP4, OUTPUT);
}

void updateIMUandServos() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;

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
  int correctionPitch = constrain((Kp * filteredPitch + Kv * velocity), -20, 20);
  int correctionRoll  = constrain((Kp * filteredRoll), -20, 20);
  int correctionYaw   = constrain((Kp * filteredYaw), -20, 20);

  s1 = constrain(45 - correctionPitch + correctionRoll , 25, 65);
  s2 = constrain(45 + correctionPitch + correctionRoll , 25, 65);
  s3 = constrain(45  + correctionRoll + correctionYaw, 25, 65);
  s4 = constrain(45  + correctionRoll - correctionYaw, 25, 65);

  //s1 = 0; // Min - 45 degrees counterclockwise
  //s1 = 90; // Neutral - 0 degrees
  //s1 = 180; // Max - 45 degrees clockwise

  //Serial.println(s1);

  // Serial debugging -- comment out before launch
  // if (isCalibrated == true)
  // {
  // Serial.print("Pitch: "); Serial.print(filteredPitch);
  // Serial.print(" | Roll: "); Serial.print(filteredRoll);
  // Serial.print(" | Yaw: "); Serial.print(filteredYaw);
  // Serial.print(" | Vel: "); Serial.print(velocity);
  // Serial.print(" | Alt: "); Serial.println(relativeAltitude);
  // }
}

bool sortByAngle(Servo &a, Servo &b)
{
  return a.angle < b.angle;
}

bool sortByPin(Servo &a, Servo &b)
{
  return a.pin < b.pin;
}

void stopPulse()
{
  if(PWMIndex == 3)
  {
    digitalWrite(servoArray[PWMIndex].pin, LOW);
    PWMIndex = 0;
    stopTimer.end();
  }
  else
  {
    digitalWrite(servoArray[PWMIndex].pin, LOW);
    PWMIndex++;
    int nextDelay = pulseWidthArray[PWMIndex] - pulseWidthArray[PWMIndex - 1];
    if (nextDelay <= 0) nextDelay = 1;
    stopTimer.begin(stopPulse, nextDelay);
  }
}

void startPulse()
{
  std::sort(servoArray, servoArray + 4, sortByPin);
  
  servoArray[0].angle = s1;
  servoArray[1].angle = s2;
  servoArray[2].angle = s3;
  servoArray[3].angle = s4;
  
  std::sort(servoArray, servoArray + 4, sortByAngle);
  for (int i = 0; i < 4; i++)
  {
    pulseWidthArray[i] = pulseWidth(servoArray[i].angle);
  }

  digitalWrite(servoP1, HIGH);
  digitalWrite(servoP2, HIGH);
  digitalWrite(servoP3, HIGH);
  digitalWrite(servoP4, HIGH);

  stopTimer.begin(stopPulse, pulseWidthArray[PWMIndex]);
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
  startTimer.begin(startPulse, 4000);
}
