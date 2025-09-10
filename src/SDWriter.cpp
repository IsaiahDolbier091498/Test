#include <Arduino.h>
#include "SDWriter.hpp"
#include "teensy41.hpp"
#include "altimeter.hpp"
#include "IMU.hpp"
#include "actuators.hpp"
#include <SPI.h>
#include <SdFat.h>
#include "RingBuf.h"

#define SD_CONFIG SdioConfig(FIFO_SDIO)
#define RING_BUF_CAPACITY 8192
#define LOG_FILENAME "Telemetry.csv"
#define LOG_FILE_SIZE 100 * 1024 * 1024 // 100 MB

static unsigned long lastLogTime = 0;
static unsigned long lastSyncTime = 0;
extern volatile bool loggingEnabled;

extern Teensy41 teensy41;
extern IMU BNO08X;
extern Actuators fins;
extern Altimeter BMP390;

static SdFs sd;
static FsFile file;

static RingBuf<FsFile, RING_BUF_CAPACITY> rb;

static size_t maxUsed = 0;

static unsigned long syncInterval = 500; // ms

// Initializes the SD card. Holds in an indefinite loop if the Initialization fails
void SDWriter::initSDCard()
{
    Serial.println("Initializing SD card...");
    if (!sd.begin(SD_CONFIG))
    {
        Serial.println("Insert/check SD card");
        teensy41.setLEDStatus(false);
        while (1);
    }

    if (!file.open(LOG_FILENAME, O_WRONLY | O_CREAT | O_AT_END))
    {
        Serial.println("File open failed...");
        while (1);
    }

    if (file.size() == 0)
    {
        if (!file.preAllocate(LOG_FILE_SIZE))
        {
            Serial.println("preallocation failed...");
            file.close();
            while (1);
        }

        file.println("Altitude (m),Velocity (m/s),Roll (deg),Pitch (deg),Yaw (deg),Servo1 (deg),Servo2 (deg),Servo3 (deg),Servo4 (deg),Timestamp (milliseconds)");
    }

    rb.begin(&file);
    Serial.println("SD card initialization complete");
}

// Main function that writes to the SD card. Uses commas as delimiter
void SDWriter::SDCardWrite(unsigned long timeStamp)
{
    size_t n = rb.bytesUsed();

    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
    {
      Serial.println("File full - quitting.");
      loggingEnabled = false;
      return;
    }

    if (n > maxUsed)
    {
      maxUsed = n;
    }

    if (n >= 512 && !file.isBusy())
    {
        if (512 != rb.writeOut(512))
        {
            Serial.println("writeOut failed");
            return;
        }
    }

    rb.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%lu\n", BMP390.getRelativeAltitude(), BMP390.getVelocity(),
    BNO08X.getRoll(), BNO08X.getPitch(), BNO08X.getYaw(),
    fins.getS1(), fins.getS2(), fins.getS3(), fins.getS4(), timeStamp);
}

void SDWriter::logTelemetry(unsigned long ms, unsigned long initTimeTaken, unsigned long interval)
{

    if (!loggingEnabled) return;

    if (rb.getWriteError())
    {
    Serial.println("RingBuf writing error");
    rb.clearWriteError();
    return;
    }

    unsigned long now = ms - initTimeTaken;
    bool alreadySynced = false;
    if (now - lastLogTime >= interval)
    {
        SDCardWrite(now);
        lastLogTime = now;
    }

    if (rb.bytesUsed() > RING_BUF_CAPACITY * 0.75)
    {
        rb.sync();
        file.sync();
        lastSyncTime = now;
        alreadySynced = true;
    }

    if (!alreadySynced && (now - lastSyncTime >= syncInterval))
    {
        rb.sync();
        file.sync();
        lastSyncTime = now;
    }
}
