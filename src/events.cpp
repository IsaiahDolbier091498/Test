#include <Arduino.h>
#include "altimeter.h"
#include <cstring>

extern Altimeter BMP390;

float avgAltitude = 0;

const int bufferSize = 50;

float altitudeBuffer[bufferSize];
int altBufferIndex = 0;

volatile bool newDataFlag = false;

float getAvgAlt(bool newData)
{
    newDataFlag = false;
    if (altBufferIndex >= bufferSize)
    {
        float sum = 0;
        for (int i = 0; i < bufferSize; i++)
        {
            sum += altitudeBuffer[i];
        }
        avgAltitude = sum / bufferSize;
        altBufferIndex = 0;
        Serial.println(avgAltitude);
        return avgAltitude;
    }

    if (newData)
    {
    altitudeBuffer[altBufferIndex] = BMP390.getRelativeAltitude();
    altBufferIndex++;
    }

    return avgAltitude;
}

void ejectionChargeMain()
{
    pinMode(31, INPUT);
    digitalWrite(31, HIGH);
    delay(5000);
    digitalWrite(31, LOW);
}

void ejectionChargeBackup()
{
    pinMode(34, INPUT);
    digitalWrite(34, HIGH);
    delay(5000);
    digitalWrite(34, LOW);
}

void preLaunch()
{

}

void ascent()
{

}

void apogee()
{

}

void descent()
{

}

void landed()
{

}