#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS gnss;
const float alpha = 0.9;
float filteredLat;
float filteredLong;
float latSum;
float longSum;
float originLat;
float originLong;

void initGnss()
{
    Wire1.begin();
    Wire1.setClock(400000);

    if (!gnss.begin(Wire1))
    {
        Serial.println("ZOE-M8Q not found");
        while (1);
    }

    gnss.setNavigationFrequency(5);

    Serial.println("Waiting for accurate readings...");
    while(gnss.getFixType() != 3)

    delay(5000);

    Serial.println("GNSS initialized");
}

void setOrigin(int samples)
{
    for (int i =0; i < samples;)
    {
        if(gnss.getPVT())
        {
        filteredLat = alpha * filteredLat + (1 - alpha) * (gnss.getLatitude() / 1e7);
        filteredLong = alpha * filteredLong + (1 - alpha) * (gnss.getLongitude() / 1e7);
        i++;
        }
    }

    for (int i =0; i < samples;)
    {
        if(gnss.getPVT())
        {
        filteredLat = alpha * filteredLat + (1 - alpha) * (gnss.getLatitude() / 1e7);
        filteredLong = alpha * filteredLong + (1 - alpha) * (gnss.getLongitude() / 1e7);
        latSum += filteredLat;
        longSum += filteredLong;
        i++;
        }
    }

    originLat = latSum/samples;
    originLong = longSum/samples;

    Serial.print("Lat origin: ");
    Serial.println(originLat, 7);

    Serial.print("Long origin: ");
    Serial.println(originLong, 7);

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
    }
}
