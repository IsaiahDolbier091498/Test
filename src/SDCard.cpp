#include <Arduino.h>
#include "sensors.h"
#include "control.h"
#include <SPI.h>
#include <SD.h>

File myFile = SD.open("Telemetry.txt", FILE_WRITE);

void SDCardInit()
{
    Serial.print("Initializing SD card...");

    if (!SD.begin(4))
    {
      Serial.println("initialization failed!");
      while (1);
    }

    Serial.println("initialization complete");
}

void SDCardWrite()
{
    if (myFile)
    {
        myFile.println("Altitude: ");
        myFile.close();
      }
      else
      {
        Serial.println("error opening test.txt");
      }

}
