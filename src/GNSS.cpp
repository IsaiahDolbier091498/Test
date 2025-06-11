#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS gnss;

void initGnss()
{
    Wire1.begin();
    Wire1.setClock(400000);

    if (!gnss.begin(Wire1))
    {
        Serial.println("ZOE-M8Q not found");
        while (1);
    }

    Serial.println("GNSS initialized");
}

void getGnssCoords()
{
    if (gnss.getPVT())
    {
        Serial.print("Fix type: ");
        Serial.println(gnss.getFixType());

        Serial.print("Lat: ");
        Serial.println(gnss.getLatitude() / 1e7, 7);

        Serial.print("Long: ");
        Serial.println(gnss.getLongitude() / 1e7, 7);

        Serial.print("Alt: ");
        Serial.println(gnss.getAltitude() / 1000.0, 2);

        delay(3000);
    }
}
