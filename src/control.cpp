#include "control.h"
#include "sensors.h"
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <math.h>
//Handles altitude estimation and control surface logic (Initializing and reading the IMU, Calculating orientation using quaternions and converting to pitch/roll/yaw, computing servo corrections from orientation and velocity, driving servo motors to correct rocket orientation)

unsigned long logTime = millis();

// --- IMU Config ---
Adafruit_BNO08x bno08x(5); // reset pin
sh2_SensorValue_t sensorValue;

//Defines quaternion structure - orientation in 3D space
struct Quaternion {
  float w, x, y, z;
};

Quaternion q_initial = {1,0,0,0}; //Used to zero out the IMU
bool isCalibrated = false; //Checking if orientation has been zeroed out

// --- Control Gain ---
float Kp = 1.0; //Proportional gain for correcting pitch/roll angles
float Kv = 0.5; //Gain for adding vertical velocity (climb rate) influence into the control

Servo servo1, servo2, servo3, servo4; //Create servo objects

int s1, s2, s3, s4;

float adjustedPitch, adjustedRoll, adjustedYaw;

float deadband = 0.5;

float norm;


//Attaching servos to pins
void initServos() {
  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(8);
  servo4.attach(9);


  //Initializing the IMU over I2C, if it fails the program stops
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found");
    while (1);
  }

  //Tells IMU to send quaternion-based orientation data
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
}

void updateIMUandServos() {
  if (!bno08x.getSensorEvent(&sensorValue)) return; //reads latest IMU data
  if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) return; //Only continues if the data is rotation vector type

  //Extracts the orientation quaternion from the sensor reading
  float q0 = sensorValue.un.gameRotationVector.real;
  float q1 = sensorValue.un.gameRotationVector.i;
  float q2 = sensorValue.un.gameRotationVector.j;
  float q3 = sensorValue.un.gameRotationVector.k;

  // Prevents float precision drift over time (maybe)
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (abs(norm - 1.0f) > 0.01f)
  {
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
  }

  Quaternion q_current = {q0, q1, q2, q3};

  //If not calibrated, stores the current orientation as the zero orientation and exits
  if (!isCalibrated) {
    q_initial = q_current;
    isCalibrated = true;
    Serial.println("IMU orientation zeroed.");
    return;
  }

  Quaternion q_inv = {q_initial.w, -q_initial.x, -q_initial.y, -q_initial.z};  //Inverts the initial quaternion to cancel out the origianl launch orientation

  //Multiplies the inverse of the initial quaternion with the current one to get reltive orientation
  Quaternion q_rel;
  q_rel.w = q_inv.w*q_current.w - q_inv.x*q_current.x - q_inv.y*q_current.y - q_inv.z*q_current.z;
  q_rel.x = q_inv.w*q_current.x + q_inv.x*q_current.w + q_inv.y*q_current.z - q_inv.z*q_current.y;
  q_rel.y = q_inv.w*q_current.y - q_inv.x*q_current.z + q_inv.y*q_current.w + q_inv.z*q_current.x;
  q_rel.z = q_inv.w*q_current.z + q_inv.x*q_current.y - q_inv.y*q_current.x + q_inv.z*q_current.w;

  // Convert to Euler Angles (roll, pitch, yaw)
  float sinr_cosp = 2.0 * (q_rel.w * q_rel.x + q_rel.y * q_rel.z);
  float cosr_cosp = 1.0 - 2.0 * (q_rel.x * q_rel.x + q_rel.y * q_rel.y);
  float roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0 * (q_rel.w * q_rel.y - q_rel.z * q_rel.x);
  float pitch = (abs(sinp) >= 1) ? copysign(PI / 2, sinp) : asin(sinp);

  float siny_cosp = 2.0 * (q_rel.w * q_rel.z + q_rel.x * q_rel.y);
  float cosy_cosp = 1.0 - 2.0 * (q_rel.y * q_rel.y + q_rel.z * q_rel.z);
  float yaw = atan2(siny_cosp, cosy_cosp);

  // Radians → Degrees
  roll  *= 180.0 / PI;
  pitch *= 180.0 / PI;
  yaw   *= 180.0 / PI;

  //Axis orientation
  adjustedPitch = pitch;
  adjustedRoll  = roll;
  adjustedYaw   = yaw;

  //Sends data over serial
  Serial.print("Pitch: "); Serial.print(adjustedPitch);
  Serial.print(" | Roll: "); Serial.print(adjustedRoll);
  Serial.print(" | Yaw: "); Serial.print(adjustedYaw);
  Serial.print(" | Vel: "); Serial.print(velocity);
  Serial.print(" | Alt: "); Serial.println(relativeAltitude);

  if (abs(adjustedPitch) < deadband) adjustedPitch = 0;
  if (abs(adjustedRoll) < deadband) adjustedRoll = 0;
  if (abs(adjustedYaw) < deadband) adjustedYaw = 0;

  // === Combine Orientation + Velocity into Control ===
  //Change the "-" signs before "(Kp" if fins are doing the opposite
  int correctionPitch = constrain((Kp * adjustedPitch + Kv * velocity), -45, 45);
  int correctionRoll = constrain((Kp * adjustedRoll),  -45, 45);
  int correctionYaw = constrain((Kp * adjustedYaw), -45, 45);

  //Sets each servo based on correction value. Neutral = 90°, deflections add or subtract
  //(Place fin as straight:pushes fin one way or the other:sets range for servo for safety)
  s1 = constrain(80 - correctionPitch + correctionRoll , 45, 135);
  s2 = constrain(98 + correctionPitch + correctionRoll , 45, 135);
  s3 = constrain(87  + correctionRoll + correctionYaw, 45, 135);
  s4 = constrain(85  + correctionRoll - correctionYaw, 45, 135);





  //pairing servos, moving in opposite directions to create pitch/roll forces
  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);

}
