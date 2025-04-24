#include <Arduino.h>
#include "sensors.h"
#include "control.h"

//Calls initialization routines(initSensors, initServos) during setup()
//Repeatedly updates sensors and control surfaces in loop()
void setup() {
  Serial.begin(115200);
  initSensors();
  initServos();
  Serial.println("System ready.");
}

void loop() {
  updateAltitude();
  updateIMUandServos();
}
