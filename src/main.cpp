#include <Arduino.h>
#include "sensors.h"
#include "control.h"
#include "SDWriter.h"

unsigned long lastLogTime = 0;
const unsigned long logInterval = 20;

int GreenLedPin = 13;

unsigned long initTimeTaken;

//Calls initialization routines(initSensors, initServos, ) during setup()
//Repeatedly updates sensors and control surfaces in loop()
void setup() {
  Serial.begin(115200);
  delay(60000);
  initSensors();
  initServos();
  initSDCard();
  pinMode(GreenLedPin, OUTPUT);
  Serial.println("System ready.");
  for (int i = 0; i < 200; i++)
  {
    updateAltitude();
    updateIMUandServos();
  }

  digitalWrite(GreenLedPin, HIGH);
  initTimeTaken = millis();
}

void loop() {
  updateAltitude();
  updateIMUandServos();

  unsigned long now = millis() - initTimeTaken;
  if (now - lastLogTime >= logInterval)
  {
    SDCardWrite(now);
    lastLogTime = now;
  }

}
