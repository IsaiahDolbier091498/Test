#include "altimeter.h"
#include <Adafruit_BMP3XX.h>
#include <Wire.h>

// --- Altimeter ---
Adafruit_BMP3XX bmp;
const float seaLevelPressure = 1013.25; // in hPa
int samples;
float drift;
bool launchDetected = false;

// Kalman state variables
float state[2] = {0.0, 0.0}; // state[0] = altitude, state[1] = velocity
float covariance[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};

// Kalman filter tuning parameters
float measurementNoise = 8;      // Measurement noise
float processAltitudeNoise = 0.01; // Process noise for altitude
float processVelocityNoise = 0.1;  // Process noise for velocity

// Outputs
float rawAltitude = 0.0, filteredAltitude = 0.0, relativeAltitude = 0.0;
float velocity = 0.0;
bool initialAltitudeSet = false;
float initialAltitude = 0.0;
unsigned long lastTime = 0;
int filterCount = 0;

// Initializes I2C and sensor
void initSensors() {
  Wire.begin();
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not found!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  delay(100);

  lastTime = millis();
}

// Runs the 2 state Kalman filter to get altitude and velocity
void updateAltitude() {
  if (!bmp.performReading()) return;

  // Read and convert to altitude
  float pressure = bmp.pressure / 100.0;
  rawAltitude = (1 - pow(pressure / seaLevelPressure, 0.190284)) * 44330.0;

  // Time delta in seconds
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0 || dt > 1.0) return;

  // Predict next state
  float nextState[2];
  nextState[0] = state[0] + state[1] * dt; // predicted altitude
  nextState[1] = state[1];             // velocity constant during prediction

  // Predict covariance matrix
  float predictedCovariance[2][2];
  predictedCovariance[0][0] = covariance[0][0] + dt * (covariance[1][0] + covariance[0][1]) + dt * dt * covariance[1][1] + processAltitudeNoise;
  predictedCovariance[0][1] = covariance[0][1] + dt * covariance[1][1];
  predictedCovariance[1][0] = covariance[1][0] + dt * covariance[1][1];
  predictedCovariance[1][1] = covariance[1][1] + processVelocityNoise;

  // Kalman gain
  float measurementUncertainty = predictedCovariance[0][0] + measurementNoise;
  float gain[2];
  gain[0] = predictedCovariance[0][0] / measurementUncertainty;
  gain[1] = predictedCovariance[1][0] / measurementUncertainty;

  // Update state with measurement
  float measurementDifference = rawAltitude - nextState[0];
  state[0] = nextState[0] + gain[0] * measurementDifference;
  state[1] = nextState[1] + gain[1] * measurementDifference;

  // Update covariance
  covariance[0][0] = (1 - gain[0]) * predictedCovariance[0][0];
  covariance[0][1] = (1 - gain[0]) * predictedCovariance[0][1];
  covariance[1][0] = predictedCovariance[1][0] - gain[1] * predictedCovariance[0][0];
  covariance[1][1] = predictedCovariance[1][1] - gain[1] * predictedCovariance[0][1];

  // Store results
  filteredAltitude = state[0];
  velocity = state[1];
  filterCount++;

  // Set baseline after stable reading
  if (!initialAltitudeSet && filterCount >= samples) {
    initialAltitude = filteredAltitude;
    initialAltitudeSet = true;
    Serial.println("Initial altitude set.");
  }

  relativeAltitude = filteredAltitude - initialAltitude;

  // Adjusts for drift when stationary
  if (state[1] < 0.5 && filterCount > samples && launchDetected == false) {
  drift = filteredAltitude - initialAltitude;
  initialAltitude += drift * 0.05;
  }
  else if (state[1] >= 0.5)
  {
    launchDetected = true;
  }
}

void calibrateAltimeter(int sampleAmount)
{
  samples = sampleAmount;
  for (int i = 0; i < sampleAmount; i++)
  {
    while (!bmp.performReading()) delay (5);
    updateAltitude();
  }
}

/* --- Kalman Filter ---

State variables:
- state: current state of the system. takes in two parameters altitude and velocity
- covariance: keeps track of how certain the filter is in altitude and velocity's estimates and their correlation


Kalman filter tuning parameters:
- measurementNoise: how much the filter trusts incoming sensor readings. smaller means more trust, larger means less trust
- processAltitudeNoise: how much the filter expects altitude to naturally change in between measurements. larger values expect larger fluctuations, smaller values expect smoother fluctuations
- processVelocityNoise: how much the filter expects velocity to change unpredictably over time. larger values expect larger fluctuations, smaller values expect smoother fluctuations


Output variables:
- rawAltitude: raw altitude data being read from the sensor
- filteredAltitude: filtered altitude data
- relativeAltitude: the distance the sensor is from the ground
- velocity: derived from the change of filtered altitude readings over time
- initialAltitudeSet: a true/false flag to know if the starting altitude baseline has been established
- initialAltitude: the baseline altitude set during calibration. All altitude changes are measured relative to this
- lastTime: the time when the sensor was last read, used to calculate how much time passed between readings
- filterCount: how many times the filter has run since startup


updateAltitude():
- if (!bmp.performReading()) return; 
  function only runs when the sensor successfully provides a new reading

- float pressure = bmp.pressure / 100.0;
  converts pressure from Pa to hPa (hectopascals)

- rawAltitude = (1 - pow(pressure / seaLevelPressure, 0.190284)) * 44330.0;
  calculates altitude from pressure using the barometric formula

- unsigned long now = millis();
  current time in milliseconds

- float dt = (now - lastTime) / 1000.0;
  calculates how much time has passed since the last reading, in seconds

- lastTime = now;
  updates lastTime for the next iteration

- if (dt <= 0 || dt > 1.0) return;
  skips update if time step is invalid (either zero or too large)


Prediction step:
- nextState[0] = state[0] + state[1] * dt;
  predicts the next altitude by applying the velocity over the elapsed time

- nextState[1] = state[1];
  assumes velocity remains constant (no acceleration model used)


Covariance prediction:
- predictedCovariance[0][0] = covariance[0][0] + dt * (covariance[1][0] + covariance[0][1]) + dt * dt * covariance[1][1] + processAltitudeNoise;
  calculates new uncertainty in altitude by propagating old uncertainties and adding expected altitude process noise

- predictedCovariance[0][1] = covariance[0][1] + dt * covariance[1][1];
  updates correlation between altitude and velocity

- predictedCovariance[1][0] = covariance[1][0] + dt * covariance[1][1];
  same as above, transposed (velocity to altitude)

- predictedCovariance[1][1] = covariance[1][1] + processVelocityNoise;
  increases uncertainty in velocity to account for possible drift


Kalman gain:
- measurementUncertainty = predictedCovariance[0][0] + measurementNoise;
  total expected uncertainty in the altitude measurement

- gain[0] = predictedCovariance[0][0] / measurementUncertainty;
  how much the filter should adjust the altitude estimate based on the new measurement

- gain[1] = predictedCovariance[1][0] / measurementUncertainty;
  how much the filter should adjust the velocity estimate based on the new altitude measurement


Measurement update:
- measurementDifference = rawAltitude - nextState[0];
  difference between actual sensor reading and predicted altitude

- state[0] = nextState[0] + gain[0] * measurementDifference;
  updates the altitude estimate using the Kalman gain and measurement error

- state[1] = nextState[1] + gain[1] * measurementDifference;
  updates the velocity estimate using the Kalman gain and measurement error


Covariance update:
- covariance[0][0] = (1 - gain[0]) * predictedCovariance[0][0];
  adjusts the uncertainty in altitude based on how much we trusted the new measurement

- covariance[0][1] = (1 - gain[0]) * predictedCovariance[0][1];
  adjusts the covariance between altitude and velocity accordingly

- covariance[1][0] = predictedCovariance[1][0] - gain[1] * predictedCovariance[0][0];
  adjusts the reverse covariance (velocity to altitude)

- covariance[1][1] = predictedCovariance[1][1] - gain[1] * predictedCovariance[0][1];
  adjusts the uncertainty in velocity


Final outputs:
- filteredAltitude = state[0];
  stores the updated altitude estimate

- velocity = state[1];
  stores the updated vertical velocity

- filterCount++;
  tracks how many times the filter has successfully run


Initial altitude setup:
- if (!initialAltitudeSet && filterCount >= samples) {}
  waits until enough filter cycles have occurred before locking in a baseline ground altitude

- relativeAltitude = filteredAltitude - initialAltitude;
  calculates how far the current altitude is from the initial reference point (ground level)
*/

