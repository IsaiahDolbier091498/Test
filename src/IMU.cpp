#include "IMU.hpp"
#include "altimeter.hpp"
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "teensy41.hpp"
#include "SDWriter.hpp"

// --- IMU Config ---
#define BNO08X_I2C_ADDR 0x4A
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

extern Altimeter BMP390;
extern Teensy41 teensy41;
extern bool debugMode;

static bool isZeroed = false;
static bool isCalibrated = false;

static struct Quaternion q_initial = {1, 0, 0, 0};

// Initialize PID for pitch, roll, yaw
static struct PID pidPitch = {0.5, 0.0, 0.2, 0, 0};
static struct PID pidRoll  = {0.5, 0.0, 0.2, 0, 0};
static struct PID pidYaw   = {0.5, 0.0, 0.2, 0, 0};

// Update timing
static unsigned long lastUpdate = 0;

// Orientation filters / variables
static float pitch, roll, yaw = 0;
static float filteredPitch, filteredRoll, filteredYaw = 0;
static float pitchCorrection, rollCorrection, yawCorrection = 0;

static const float maxDeflectionAngle = 20;
static const float deadband = 0.5;
static float norm;

// Update rate control
static const unsigned long updateInterval = 5; // milliseconds - 5 ms is 200 hz

// Low-pass filter coefficient
static float alpha = 0.7;

// Resets IMU as it is one of the main culprits that jams the I2C line low
void IMU::resetBNO085()
{
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delay(1000);
  digitalWrite(3, HIGH);
  delay(2000);
}

// Initialization
void IMU::initIMU()
{
  if (!bno08x.begin_I2C(BNO08X_I2C_ADDR, &Wire1)) {
    log("BNO08x not found");
    teensy41.setLEDStatus(false);
    while (1);
  }

  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
}

// --- PID update helper ---
float IMU::updatePID(PID &pid, float setpoint, float measurement, float velocity, float dt) {
  float error = setpoint - measurement;

  // Integrate error with anti-windup
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -50, 50);

  // Derivative
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;

  // PID output
  return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

void IMU::updateOrientation()
{
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
    log("IMU orientation zeroed.");
    log("Calibrating IMU...");
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
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0 * (q_rel.w * q_rel.y - q_rel.z * q_rel.x);
  pitch = (abs(sinp) >= 1) ? copysign(PI / 2, sinp) : asin(sinp);

  float siny_cosp = 2.0 * (q_rel.w * q_rel.z + q_rel.x * q_rel.y);
  float cosy_cosp = 1.0 - 2.0 * (q_rel.y * q_rel.y + q_rel.z * q_rel.z);
  yaw = atan2(siny_cosp, cosy_cosp);

  // Convert to degrees
  roll  *= 180.0 / PI;
  pitch *= 180.0 / PI;
  yaw   *= 180.0 / PI;

  // Low-pass filtering
  filteredPitch = alpha * filteredPitch + (1 - alpha) * pitch;
  filteredRoll  = alpha * filteredRoll  + (1 - alpha) * roll;
  filteredYaw   = alpha * filteredYaw   + (1 - alpha) * yaw;

  if (abs(filteredPitch) < deadband) filteredPitch = 0;
  if (abs(filteredRoll)  < deadband) filteredRoll = 0;
  if (abs(filteredYaw)   < deadband) filteredYaw = 0;

  // --- Calculate delta time for PID ---
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  if (dt <= 0) dt = 0.005;  // avoid zero if called faster than expected
  lastUpdate = now;

  // --- Update PID controllers ---
  pitchCorrection = updatePID(pidPitch, 0.0, filteredPitch, BMP390.getVelocity(), dt);
  rollCorrection  = updatePID(pidRoll,  0.0, filteredRoll, BMP390.getVelocity(),  dt);
  yawCorrection   = updatePID(pidYaw,   0.0, filteredYaw, BMP390.getVelocity(),   dt);

  // === Combine Orientation + Velocity into Control ===
  pitchCorrection = constrain(pitchCorrection, -maxDeflectionAngle, maxDeflectionAngle);
  rollCorrection  = constrain(rollCorrection,  -maxDeflectionAngle, maxDeflectionAngle);
  yawCorrection   = constrain(yawCorrection,   -maxDeflectionAngle, maxDeflectionAngle);

  // Serial debugging -- comment out before launch
  // if (isCalibrated == true && debugMode)
  // {
  //   Serial.print("Pitch: "); Serial.print(filteredPitch);
  //   Serial.print(" | Roll: "); Serial.print(filteredRoll);
  //   Serial.print(" | Yaw: "); Serial.print(filteredYaw);
  //   Serial.print(" | Vel: "); Serial.print(BMP390.getVelocity());
  //   Serial.print(" | Alt: "); Serial.println(BMP390.getRelativeAltitude());
  // }
}

void IMU::calibrateIMU(int sampleAmount)
{
  for (int i = 0; i < sampleAmount; i++)
  {
    updateOrientation();
    delay(5);
  }
  isCalibrated = true;
  log("IMU calibrated...");
}

float IMU::getPitch()
{
  return filteredPitch;
}

float IMU::getRoll()
{
  return filteredRoll;
}

float IMU::getYaw()
{
  return filteredYaw;
}

float IMU::getPitchCorrection()
{
  return pitchCorrection;
}

float IMU::getRollCorrection()
{
  return rollCorrection;
}

float IMU::getYawCorrection()
{
  return yawCorrection;
}
