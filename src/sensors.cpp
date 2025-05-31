#include "sensors.h"
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
//Hanldes all sensor-related tasks (Initializing the altimeter, Reading and filtering altitude data, managing altitude state)

// --- Altimeter ---
Adafruit_BMP3XX bmp;
const float seaLevelPressure = 1013.25; //in hPa

float rawAltitude = 0.00, filteredAltitude = 0.00, relativeAltitude = 0.0;
float lastFilteredAltitude = 0.00, velocity = 0.0;
bool initialAltitudeSet = false;
float initialAltitude = 0.0;
unsigned long lastTime = 0;
int filterCount = 0;

// Kalman filter
float kalmanAltitude = 0.0, kalmanVelocity = 0.0, kalmanErrorEstimate = 1.0;
float kalmanErrorMeasure = 1.0, kalmanGain = 0.0;
float kalmanProcessNoise = 0.065; //Increasing (The filter altitude could change more between readings - It becomes more responsive to new measurements - Smoother in low settings, snappier in high settings
float kalmanMeasurementNoise = 2.5; //Increasing (Filter thinks the sensor is noisy/unreliable - It trusts previous estimates more, and new data less - Filter becomes slower to respond, but smoother)

//Initializes I2C communication
void initSensors() {
  Wire.begin();
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not found!");
    while (1);
  }

  //Configure Sensor Settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  delay(100);
  lastTime = millis();
}

void updateAltitude() {

  if (!bmp.performReading()) return;

  //get raw sensor values (hPa and C)
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;

  //Converting pressure to altitude using the barometric formula
  rawAltitude = (1 - pow(pressure / seaLevelPressure, 0.190284)) * 44330.0;


  //Calculating the time elapsed since last update
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0 || dt > 1.0) return;

  //Updates the Kalman filter
  kalmanErrorEstimate += kalmanProcessNoise; //Estimate uncertainty increases with process noise
  kalmanGain = kalmanErrorEstimate / (kalmanErrorEstimate + kalmanMeasurementNoise); //Calculates Kalman gain
  kalmanAltitude += kalmanGain * (rawAltitude - kalmanAltitude); //Blends new measurement with previous estimate
  kalmanErrorEstimate *= (1 - kalmanGain); //Updates uncertainty for the next cycle

  filteredAltitude = kalmanAltitude; //Store the filtered altitude 
  filterCount++; 


  //After 100 good samples, we lock in a baseline for referenced altitude
  if (!initialAltitudeSet && filterCount >= 1000) {
    initialAltitude = filteredAltitude;
    initialAltitudeSet = true;
    Serial.println("Initial altitude set.");
  }

  relativeAltitude = filteredAltitude - initialAltitude; //height above ground
  velocity = (filteredAltitude - lastFilteredAltitude) / dt; 
  lastFilteredAltitude = filteredAltitude;
}
