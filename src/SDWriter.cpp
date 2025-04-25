#include <Arduino.h>
#include "sensors.h"
#include "control.h"
#include <SPI.h>
#include <SD.h>

// Initializes the SD card. Holds in an indefinite loop if the Initialization fails
void initSDCard()
{
    Serial.print("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) // If BUILTIN_SDCARD doesn't work try replacing it with 4 (  if (!SD.begin(4))  )
    {
        Serial.println("initialization failed!");
        while (1);
    }

    Serial.println("initialization complete");
}

// Main function that writes to the SD card. Uses hyphens as delimiter
void SDCardWrite()
{
    File myFile = SD.open("Telemetry.txt", FILE_WRITE);

    if (myFile)
    {
        // If the file is empty, write in the headers
        if (myFile.size() == 0)
        {
            myFile.println("Altitude (m)-Velocity (m/s)-Roll (°)-Pitch (°)-Yaw (°)-Timestamp (milliseconds)");
        }

        // If the file exists write the Telemetry rounded to the fifth decimal point
        myFile.printf("%.5f-%.5f-%.5f-%.5f-%.5f-%lu\n", relativeAltitude, velocity, adjustedRoll, adjustedPitch, adjustedYaw, millis());
        myFile.close();
    }
    else
    {
        Serial.println("error opening Telemetry.txt");
    }
}
