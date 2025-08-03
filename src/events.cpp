#include <Arduino.h>
#include "altimeter.h"
#include <cstring>

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
    altitudeBuffer[altBufferIndex] = relativeAltitude;
    altBufferIndex++;
    }

    return avgAltitude;
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