#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "teensy41.h"

volatile bool loggingEnabled = true;

// Resets teensy 4.1
void Teensy41::reset()
{
  // Shuts down SDWriter
  loggingEnabled = false;
  delay(50);

  // I2C shutdown
  Wire.end();
  Wire1.end();

  delay(20);

  noInterrupts();
  SCB_AIRCR = 0x05FA0004;
}

// Resets IMU as it is one of the main culprits that jams the I2C line low
void Teensy41::resetBNO085()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1000);
  digitalWrite(2, HIGH);
  delay(2000);
}

// Checks / resets I2C line. High status is normal and low means the I2C line is stuck and needs to be rebooted
void Teensy41::checkI2CLines()
{
  Wire.end();
  Wire1.end();

  Serial.println("Resetting IMU...");
  resetBNO085();

  Serial.println("Checking I2C lines...");
  pinMode(19, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);

  int scl = digitalRead(19);
  int sda = digitalRead(18);
  int sda1 = digitalRead(17);
  int scl1 = digitalRead(16);

  Serial.printf("SDA: %s\n", sda ? "HIGH" : "LOW");
  Serial.printf("SDA1: %s\n", sda1 ? "HIGH" : "LOW");
  Serial.printf("SCL: %s\n", scl ? "HIGH" : "LOW");
  Serial.printf("SCL1: %s\n", scl1 ? "HIGH" : "LOW");

  if (sda == 0)
  {
    Serial.println("SDA stuck LOW, rebooting SDA line...");
    pinMode(19, OUTPUT);

    for (int i = 0; i < 9; i++)
    {
      digitalWrite(19, HIGH);
      delayMicroseconds(5);
      digitalWrite(19, LOW);
      delayMicroseconds(5);
    }

    pinMode(19, INPUT_PULLUP);
    sda = digitalRead(18);

    Serial.printf("SDA: %s\n", sda ? "HIGH" : "LOW");
  }

  if (sda1 == 0)
  {
    Serial.println("SDA1 stuck LOW, rebooting SDA1 line...");
    pinMode(16, OUTPUT);

    for (int i = 0; i < 9; i++)
    {
      digitalWrite(16, HIGH);
      delayMicroseconds(5);
      digitalWrite(16, LOW);
      delayMicroseconds(5);
    }

    pinMode(16, INPUT_PULLUP);
    sda1 = digitalRead(17);

    Serial.printf("SDA1: %s\n", sda1 ? "HIGH" : "LOW");
  }

  delay(50);
}

// Helper function for measureBattery. returns text status
const char* Teensy41::batteryStatus(float percentage)
{
  if (percentage >= 95.0) return "FULL";
  if (percentage >= 70.0) return "HIGH";
  if (percentage >= 50.0) return "MEDIUM";
  if (percentage >= 20.0) return "LOW";
  else return "LOW -> Charge Immediately";
}

// Reads voltage from the main battery using voltage divider circuit
void Teensy41::measureBattery()
{
  float maxLipoCharge = 8.4;
  float minLipoCharge = 6.0;

  float resistor1 = 9100.0;
  float resistor2 = 5100.0;

  float voltageOut = 0.0;
  for (int i = 0; i < 10; i++)
  {
    voltageOut += analogRead(A10) * (3.3 / 1023.0); // A10 is pin 24
    delay(2);
  }
  voltageOut /= 10.0;
  float voltageIn = (voltageOut * (resistor1 + resistor2)) / resistor2;

  float percentage = ((voltageIn - minLipoCharge) / (maxLipoCharge - minLipoCharge)) * 100.0;
  if (percentage < 0) percentage = 0;

  Serial.printf("Main Battery Voltage: %.2fV ~ %.1f%% -> %s\n", voltageIn, percentage, batteryStatus(percentage));
}

void Teensy41::ejectionChargeMain()
{
    pinMode(31, INPUT);
    digitalWrite(31, HIGH);
    delay(5000);
    digitalWrite(31, LOW);
}

void Teensy41::ejectionChargeBackup()
{
    pinMode(34, INPUT);
    digitalWrite(34, HIGH);
    delay(5000);
    digitalWrite(34, LOW);
}

// Checks the fault flag of the main ejection charge driver (MIC2544) - pulls low to indicate over current or thermal shutdown
void Teensy41::checkMIC2544MainFlag()
{
  pinMode(32, INPUT);
  float faultFlag = digitalRead(32);
  Serial.printf("MIC2544 Main Fault Flag: %s\n", faultFlag ? "INACTIVE" : "ACTIVE"); // Inactive good - active bad
}

// Checks the fault flag of the backup ejection charge driver (MIC2544) - pulls low to indicate over current or thermal shutdown
void Teensy41::checkMIC2544BackupFlag()
{
  pinMode(33, INPUT);
  float faultFlag = digitalRead(33);
  Serial.printf("MIC2544 Backup Fault Flag: %s\n", faultFlag ? "INACTIVE" : "ACTIVE"); // Inactive good - active bad
}

void Teensy41::setLEDStatus(bool nominal)
{
  if (nominal)
  {
    // Turns on green LED to indicate everything is performing as expected
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW); // Pull low to turn on common anode LED
  }
  else
  {
    // Turns on red LED to indicate sensor or other module isn't working
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW); // Pull low to turn on common anode LED
  }
}