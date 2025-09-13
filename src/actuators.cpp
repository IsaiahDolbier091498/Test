#include <Arduino.h>
#include "actuators.hpp"
#include "IMU.hpp"
#include <algorithm>

IntervalTimer startTimer;
IntervalTimer stopTimer;

extern IMU BNO08X;
extern bool debugMode;

static const int servoP1 = 6, servoP2 = 7, servoP3 = 8, servoP4 = 9;
static float s1 = 0, s2 = 0, s3 = 0, s4 = 0;
static int minTravelDegrees = 0, maxTravelDegrees = 90;
static int minTravelMicros = 1100, maxTravelMicros = 1900;
static float minStep = 0.1125;

static int pulseWidthArray[4];
static volatile int PWMIndex = 0;

static struct Servo servoArray[4] =
{
    {servoP1, s1},
    {servoP2, s2},
    {servoP3, s3},
    {servoP4, s4},
};

int Actuators::pulseWidth(float servoAngle)
{
  return (servoAngle - minTravelDegrees) * (maxTravelMicros - minTravelMicros)
  / (maxTravelDegrees - minTravelDegrees) + minTravelMicros;
}

void Actuators::initServos()
{
  pinMode(servoP1, OUTPUT);
  pinMode(servoP2, OUTPUT);
  pinMode(servoP3, OUTPUT);
  pinMode(servoP4, OUTPUT);

  startTimer.begin(startPulse, 4000);
}

void Actuators::updateAngles()
{
  //s1 = 0; // Min - 45 degrees counterclockwise
  //s1 = 45; // Neutral - 0 degrees
  //s1 = 90; // Max - 45 degrees clockwise
  s1 = round(constrain(45 - BNO08X.getPitchCorrection() + BNO08X.getRollCorrection() , 25, 65) / minStep) * minStep;
  s2 = round(constrain(45 + BNO08X.getPitchCorrection() + BNO08X.getRollCorrection() , 25, 65) / minStep) * minStep;
  s3 = round(constrain(45  + BNO08X.getRollCorrection() + BNO08X.getYawCorrection(), 25, 65) / minStep) * minStep;
  s4 = round(constrain(45  + BNO08X.getRollCorrection() - BNO08X.getYawCorrection(), 25, 65) / minStep) * minStep;

  if (debugMode)
  {
    Serial.printf("| Servo 1: %f | Servo 2: %f | Servo 3: %f | Servo 4: %f |\n", s1,s2,s3,s4);
  }

}

bool Actuators::sortByAngle(Servo &a, Servo &b)
{
  return a.angle < b.angle;
}

bool Actuators::sortByPin(Servo &a, Servo &b)
{
  return a.pin < b.pin;
}

void Actuators::stopPulse()
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

void Actuators::startPulse()
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

int Actuators::getS1()
{
  return s1;
}

int Actuators::getS2()
{
  return s2;
}

int Actuators::getS3()
{
  return s3;
}

int Actuators::getS4()
{
  return s4;
}