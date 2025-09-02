#include <Arduino.h>
#include "altimeter.h"
#include "IMU.h"
#include "actuators.h"
#include "SDWriter.h"
#include "teensy41.h"
#include "GNSS.h"
#include <Wire.h>
#include "events.h"

static const unsigned long logInterval = 40;

static unsigned long initTimeTaken;

static const unsigned long setGNSSFrequency = 100; // ms - 100ms = 10hz

volatile bool BMP390DataReady = false;

Teensy41 teensy41;
IMU BNO08X;
Altimeter BMP390;
GNSS ZOEM8Q;
Actuators fins;
SDWriter sdWriter;

void BMP390Interrupt()
{
  BMP390DataReady = true;
}

// Initializes and calibrates the components during setup()
void setup() {
  Serial.begin(2000000);
  Serial1.begin(115200);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), BMP390Interrupt, FALLING);

  delay(3000);
  Serial.printf("CPU speed: %lu MHz\n", F_CPU / 1000000);
  teensy41.measureBattery();
  teensy41.checkI2CLines();

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);

  delay(3000); // 60,000 ms set to allow nosecone installation before program runs

  Serial.println("Initializing components...");
  BMP390.initAltimeter();
  fins.initServos();
  BNO08X.initIMU();
  sdWriter.initSDCard();
  ZOEM8Q.initGnss();
  //setOrigin(50);
  Serial.println("Components initialized");

  BMP390.calibrateAltimeter(100); // Sample amount
  BNO08X.calibrateIMU(1000); // Sample amount

  teensy41.setLEDStatus(true);
  Serial.println("System ready");
  initTimeTaken = millis();
}

//Repeatedly updates sensors and control surfaces in loop()

// unsigned long avgRunTime = 0;
// unsigned long totalTime = 0;
// int loopCount = 0;

void loop()
{
  //unsigned long startTime = micros();

  fins.updateAngles();

  if (BMP390DataReady)
  {
    newDataFlag = true;
    BMP390DataReady = false;
    BMP390.updateAltitude();
    Serial.println("True");
  }

  //getAvgAlt(newDataFlag);

  //updateIMUandServos();
  //ZOEM8Q.getGnssCoords();

  //sdWriter.logTelemetry(millis(), initTimeTaken, logInterval);


  // unsigned long endTime = micros();

  // Serial.println((endTime-startTime) / 1000.0);

  // totalTime += (endTime - startTime);
  // loopCount++;

  // if(loopCount >= 15000)
  // {
  //   avgRunTime = totalTime / loopCount;
  //   Serial.println(avgRunTime  / 1000.0);
  //   while(1);
  // }

}