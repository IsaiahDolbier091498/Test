#include <Arduino.h>
#include "SDWriter.h"
#include "altimeter.h"
#include "control.h"
#include <SPI.h>
#include <SD.h>

// WARNING: The Arduino SD library may not support file systems other than FAT16 and FAT32.
// If your SD card is larger than 32 GB or formatted as exFAT, it may not work correctly.
// Use an SD card formatter tool to reformat the card to FAT32 if you encounter issues.

unsigned long lastLogTime = 0;

// Initializes the SD card. Holds in an indefinite loop if the Initialization fails
void initSDCard()
{
    Serial.println("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) // If BUILTIN_SDCARD doesn't work try replacing it with 4 (  if (!SD.begin(4))  )
    {
        Serial.println("Insert/check SD card");
        while (1);
    }

    // Clears old telemetry file
    // if (SD.exists("Telemetry.csv"))
    // {
    //     SD.remove("Telemetry.csv");
    // }

    Serial.println("SD card initialization complete");
}

// Main function that writes to the SD card. Uses commas as delimiter
void SDCardWrite(unsigned long timeStamp)
{
    File myFile = SD.open("Telemetry.csv", FILE_WRITE);

    if (myFile)
    {
        // If the file is empty, write in the headers
        if (myFile.size() == 0)
        {
            myFile.println("Altitude (m),Velocity (m/s),Roll (deg),Pitch (deg),Yaw (deg),Servo1 (deg),Servo2 (deg),Servo3 (deg),Servo4 (deg),Timestamp (milliseconds)");
        }

        // If the file exists write the Telemetry rounded to the fifth decimal point
        myFile.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%lu\n", relativeAltitude, velocity, adjustedRoll, adjustedPitch, adjustedYaw, s1, s2, s3, s4, timeStamp);
        myFile.close();
    }
    else
    {
        Serial.println("error opening Telemetry.csv");
    }
}

void logTelemetry(unsigned long ms, unsigned long initTimeTaken, unsigned long interval)
{
    unsigned long now = ms - initTimeTaken;
    if (now - lastLogTime >= interval)
    {
    SDCardWrite(now);
    lastLogTime = now;
    }
}
