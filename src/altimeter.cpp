#include "altimeter.hpp"
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include "teensy41.hpp"
#include "SDWriter.hpp"

// --- Altimeter ---
#define BMP390_I2C_ADDR 0x77
Adafruit_BMP3XX bmp;

extern Teensy41 teensy41;

static const float seaLevelPressure = 1013.25; // in hPa
static const float pressureExponent = 0.190284;
static const float altitudeScale = 44330.0;

static int samples;
static float drift;
static bool launchDetected = false;
static unsigned long lastTime = 0;

// Kalman state variables
float state[2] = {0.0, 0.0}; // state[0] = altitude, state[1] = velocity
float covariance[2][2] =
{
  {1.0, 0.0},
  {0.0, 1.0}
};

// Kalman filter tuning parameters
static const float measurementNoise = 6.0;      // Measurement noise
static const float processAltitudeNoise = 0.015; // Process noise for altitude
static const float processVelocityNoise = 0.05;  // Process noise for velocity

// Outputs
static float rawAltitude, filteredAltitude, relativeAltitude = 0.0;
static float velocity = 0.0;
static bool initialAltitudeSet = false;
static float initialAltitude = 0.0;
static int filterCount = 0;

// Runs the 2 state Kalman filter to get altitude and velocity
void Altimeter::updateAltitude()
{
  float pressure = bmp.readPressure();
  if(isnan(pressure)) return;

  // Read and convert to altitude
  pressure /= 100.0;
  rawAltitude = (1 - pow(pressure / seaLevelPressure, pressureExponent)) * altitudeScale;

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
  if (!initialAltitudeSet && filterCount >= samples)
  {
    initialAltitude = filteredAltitude;
    initialAltitudeSet = true;
    log("Initial altitude set...");
  }

  relativeAltitude = filteredAltitude - initialAltitude;

  // Adjusts for drift when stationary
  if (state[1] < 0.6 && filterCount > samples && launchDetected == false)
  {
  drift = filteredAltitude - initialAltitude;
  initialAltitude += drift * 0.2;
  }
  else if (state[1] >= 0.6)
  {
    launchDetected = true;
  }

  // Serial.print(" | Vel: "); Serial.print(velocity);
  // Serial.print(" | Alt: "); Serial.println(relativeAltitude);
}

void Altimeter::calibrateAltimeter(int sampleAmount)
{
  log("Calibrating altimeter...");
  samples = sampleAmount;
  for (int i = 0; i < sampleAmount; i++)
  {
    while (!bmp.performReading()) delay (5);
    updateAltitude();
  }
}

// Writes command to the register. In this case it's to enable the interrupt pin
// Using the interrupt pin allows the sensor to be read only when ready. performReading() is blocking.
bool Altimeter::i2c_write_register(uint8_t deviceAddr, uint8_t regAddr, uint8_t value)
{
  Wire1.beginTransmission(deviceAddr);
  Wire1.write(regAddr);
  Wire1.write(value);
  return Wire1.endTransmission() == 0; // returns true if successful
}

bool Altimeter::enableBmp390Interrupt()
{
  const uint8_t int_ctrl_reg = 0x19;
  const uint8_t config = 0x40;

  if (!i2c_write_register(BMP390_I2C_ADDR, int_ctrl_reg, config)) {
    log("Failed to write INT_CTRL");
    return false;
  }

  log("BMP390 interrupt enabled (data-ready only)");
  return true;
}

// Initializes I2C and sensor
void Altimeter::initAltimeter()
{
  if (!bmp.begin_I2C(0x77, &Wire1))
  {
    log("BMP390 not found!");
    teensy41.setLEDStatus(false);
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  delay(100);

  if (!enableBmp390Interrupt())
  {
    log("Failed to enable BMP390 interrupt!");
    while (1);
  }

  lastTime = millis();
}

float Altimeter::getVelocity()
{
  return velocity;
}

float Altimeter::getRelativeAltitude()
{
  return relativeAltitude;
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

