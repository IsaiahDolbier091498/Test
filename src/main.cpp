#include <Arduino.h>
#include "altimeter.h"
#include "control.h"
#include "SDWriter.h"
#include "avionicsCommands.h"
#include "GNSS.h"

const unsigned long logInterval = 40;

int GreenLedPin = 13;

unsigned long initTimeTaken;

// Initializes and calibrates the components during setup()
void setup() {
  Serial.begin(115200);
  delay(3000); // 60,000 ms set to allow nosecone installation before program runs
  Serial.println("Initializing components...");
  //initSensors();
  //initServos();
  //initSDCard();
  initGnss();
  setOrigin(200);
  Serial.println("Components initialized");

  pinMode(GreenLedPin, OUTPUT);

  Serial.println("Waiting on sensor calibration...");
  //calibrateAltimeter(1000); // Sample amount
  //calibrateIMU(1000); // Sample amount

  Serial.println("System ready");
  digitalWrite(GreenLedPin, HIGH);
  initTimeTaken = millis();
}

//Repeatedly updates sensors and control surfaces in loop()
void loop()
{
  getGnssCoords();
  //updateAltitude();
  //updateIMUandServos();
  //logTelemetry(millis(), initTimeTaken, logInterval);
}
