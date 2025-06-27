#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS gnss;
const float alpha = 0.05;
float filteredLat;
float filteredLong;
float latOrigin;
float longOrigin;

float currentLat;
float currentLong;
int fixType;

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

    gnss.setNavigationFrequency(10);
    gnss.setUART1Output(0);
    gnss.setI2COutput(COM_TYPE_UBX);
    gnss.setAutoPVT(true);

    // Serial.println("Waiting for accurate readings...");

    // while (gnss.getFixType() != 3)
    // {
    //     switch (gnss.getFixType())
    //     {
    //     case 0:
    //         Serial.println("Accuracy: Extremely low/nonexistant");
    //         break;

    //     case 1:
    //         Serial.println("Accuracy: Low");
    //         break;

    //     case 2:
    //         Serial.println("Accuracy: Decent (2D fix)");
    //         break;
    //     }
    //     delay(1000);
    // }
    // Serial.println("Accuracy: Maxed (3D fix acquired)");

    // delay(5000);
}

void setOrigin(int samples)
{
    for (int i = 0; i < samples;)
    {
        if (gnss.getPVT())
        {
            filteredLat = alpha * filteredLat + (1 - alpha) * (gnss.getLatitude());
            filteredLong = alpha * filteredLong + (1 - alpha) * (gnss.getLongitude());
            i++;
        }
    }

    latOrigin = filteredLat;
    longOrigin = filteredLong;

    Serial.print("Lat origin: ");
    Serial.println(latOrigin, 7);

    Serial.print("Long origin: ");
    Serial.println(longOrigin, 7);
}

void getGnssCoords()
{


        // Serial.print("Fix type: ");
        // Serial.println(gnss.getFixType());

        // Serial.print("Lat: ");
        // Serial.println(gnss.getLatitude() / 1e7, 7);

        // Serial.print("Long: ");
        // Serial.println(gnss.getLongitude() / 1e7, 7);

        //Serial.println(gnss.getSIV());

        fixType = gnss.getFixType();
        currentLat = gnss.getLatitude();
        currentLong = gnss.getLongitude();
}
