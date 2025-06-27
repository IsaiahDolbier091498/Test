#include <Arduino.h>
#include "altimeter.h"
#include "control.h"
#include "SDWriter.h"
#include "avionicsCommands.h"
#include "GNSS.h"
#include <TeensyThreads.h>

const unsigned long logInterval = 40;

int GreenLedPin = 13;

unsigned long initTimeTaken;

unsigned long setGNSSFrequency = 100; // ms (5 hz)

void AFS()
{
  unsigned long last = 0;
  while (1)
  {
    updateAltitude();
    updateIMUandServos();
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
  Serial.begin(115200);
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
