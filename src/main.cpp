#include <Arduino.h>
#include "altimeter.hpp"
#include "IMU.hpp"
#include "actuators.hpp"
#include "SDWriter.hpp"
#include "teensy41.hpp"
#include "GNSS.hpp"
#include <Wire.h>
#include "events.hpp"

static const unsigned long logInterval = 40;

static unsigned long initTimeTaken;

static const unsigned long setGNSSFrequency = 100; // ms - 100ms = 10hz

volatile bool BMP390DataReady = false;

bool debugMode = true; // Enables Serial and prints line to console if true
volatile bool loggingEnabled = false;

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
  if (debugMode) Serial.begin(2000000);
  
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), BMP390Interrupt, FALLING);
  
  delay(3000);
  sdWriter.initSDCard();
  log("CPU speed: %lu MHz", F_CPU / 1000000);
  teensy41.measureBattery();
  teensy41.checkI2CLines();

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);

  delay(3000); // 60,000 ms set to allow nosecone installation before program runs

  log("Initializing components...");
  BMP390.initAltimeter();
  fins.initServos();
  BNO08X.initIMU();
  ZOEM8Q.initGnss();
  //setOrigin(50);
  log("Components initialized");

  BMP390.calibrateAltimeter(1000); // Sample amount
  BNO08X.calibrateIMU(100); // Sample amount

  teensy41.setLEDStatus(true);
  log("System ready");
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
  }

  //getAvgAlt(newDataFlag);
  BNO08X.updateOrientation();
  //ZOEM8Q.getGnssCoords();

  sdWriter.logTelemetry(millis(), initTimeTaken, logInterval);


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