#include <Arduino.h>
#include "sensors.h"
#include "control.h"
#include <SD.h>
File dataFile;

//Calls initialization routines(initSensors, initServos) during setup()
//Repeatedly updates sensors and control surfaces in loop()
void setup() {
  Serial.begin(115200);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card ready.");
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Time, Pitch, Roll, Yaw, Velocity, Altitude");
      dataFile.close();
    }
  }
  initSensors();
  initServos();
  Serial.println("System ready.");
}

void loop() {
  updateAltitude();
  updateIMUandServos();
}
