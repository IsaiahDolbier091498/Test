#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

volatile bool loggingEnabled = true;

void reset()
{
  // Shuts down SDWriter
  loggingEnabled = false;
  delay(50);

  // manually toggles SCL to unstick
  pinMode(19, OUTPUT);
  for (int i = 0; i < 9; i++)
  {
    digitalWrite(19, HIGH);
    delayMicroseconds(5);
    digitalWrite(19, LOW);
    delayMicroseconds(5);
  }

  // I2C shutdown
  Wire.end();
  pinMode(18, INPUT);
  pinMode(19, INPUT);

  delay(20);

  noInterrupts();
  SCB_AIRCR = 0x05FA0004;
}
