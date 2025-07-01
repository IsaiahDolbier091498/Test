#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "debug.h"

volatile bool loggingEnabled = true;

void reset()
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

void resetBNO085()
{
  Serial.println("Resetting BNO085...");
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(2000);
  digitalWrite(2, HIGH);
  delay(2000);
}


void checkI2CLines()
{
  Wire.end();
  Wire1.end();

  pinMode(19, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);

  int scl = digitalRead(19);
  int sda = digitalRead(18);
  int sda1 = digitalRead(17);
  int scl1 = digitalRead(16);

  Serial.printf("SDA: %s\n ", sda ? "HIGH" : "LOW");
  Serial.printf("SDA1: %s\n ", sda1 ? "HIGH" : "LOW");
  Serial.printf("SCL: %s\n ", scl ? "HIGH" : "LOW");
  Serial.printf("SCL1: %s\n ", scl1 ? "HIGH" : "LOW");

  if (sda == 0)
  {
    Serial.println("SDA stuck LOW, rebooting SDA line...");
    resetBNO085();
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

    Serial.printf("SDA: %s\n ", sda ? "HIGH" : "LOW");
  }

  delay(50); 
}
