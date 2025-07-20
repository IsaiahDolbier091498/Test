#include <Arduino.h>
#include "altimeter.h"
#include "control.h"
#include "SDWriter.h"
#include "debug.h"
#include "GNSS.h"
#include <Wire.h>
#include "RunCam.h"

const unsigned long logInterval = 40;

int GreenLedPin = 13;

unsigned long initTimeTaken;

unsigned long setGNSSFrequency = 100; // ms - 100ms = 10hz

volatile bool BMP390DataReady = false;

void BMP390Interrupt()
{
  BMP390DataReady = true;
}


// Initializes and calibrates the components during setup()
void setup() {
  Serial.begin(2000000);
  //Serial1.begin(115200);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), BMP390Interrupt, FALLING);

  delay(3000);
  Serial.printf("CPU speed: %lu MHz\n", F_CPU / 1000000);
  checkI2CLines();

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);

  delay(3000); // 60,000 ms set to allow nosecone installation before program runs


  Serial.println("Initializing components...");
  initSensors();
  initServos();
  initSDCard();
  initGnss();
  //setOrigin(50);
  Serial.println("Components initialized");

  pinMode(GreenLedPin, OUTPUT);

  //startRecording();
  
  calibrateAltimeter(1000); // Sample amount
  calibrateIMU(1000); // Sample amount

  Serial.println("System ready");
  digitalWrite(GreenLedPin, HIGH);
  initTimeTaken = millis();
}

//Repeatedly updates sensors and control surfaces in loop()

// unsigned long avgRunTime = 0;
// unsigned long totalTime = 0;
// int loopCount = 0;

void loop() 
{
  // unsigned long startTime = micros();

  if (BMP390DataReady)
  {
    BMP390DataReady = false;
    updateAltitude();
  }

  updateIMUandServos();
  getGnssCoords();
  //logTelemetry(millis(), initTimeTaken, 50);


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