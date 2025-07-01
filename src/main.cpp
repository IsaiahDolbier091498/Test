#include <Arduino.h>
#include "altimeter.h"
#include "control.h"
#include "SDWriter.h"
#include "debug.h"
#include "GNSS.h"
#include <TeensyThreads.h>
#include <Wire.h>

const unsigned long logInterval = 40;

int GreenLedPin = 13;

unsigned long initTimeTaken;

unsigned long setGNSSFrequency = 100; // ms - 100ms = 10hz

void AFS()
{
  while (1)
  {
    unsigned long now = millis();
    updateAltitude();
    updateIMUandServos();
    unsigned long finished = millis();
    Serial.print(finished - now);
    Serial.println(" ms elapsed");
    threads.yield();    
  }
}

void GNSS()
{
  while (1)
  {
    getGnssCoords();
    threads.delay(setGNSSFrequency);
  }
}

// Initializes and calibrates the components during setup()
void setup() {
  delay(3000);
  Serial.begin(115200);
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

  Serial.println("Waiting on sensor calibration...");
  calibrateAltimeter(1000); // Sample amount
  calibrateIMU(1000); // Sample amount

  Serial.println("System ready");
  digitalWrite(GreenLedPin, HIGH);
  initTimeTaken = millis();

  threads.addThread(AFS);
  threads.addThread(GNSS);
}

//Repeatedly updates sensors and control surfaces in loop()
void loop() 
{
  //logTelemetry(millis(), initTimeTaken, logInterval);
}
