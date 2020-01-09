#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "BluetoothSerial.h"
#include <analogWrite.h>

const int DV1_1 = 12;
const int DV1_2 = 13;
const int DV2_1 = 14;
const int DV2_2 = 27;

void motorA(int speed_A)
{
  digitalWrite(DV1_1, LOW);
  analogWrite(DV1_2, speed_A);
}

void motorB(int speed_B)
{
  digitalWrite(DV2_1, LOW);
  analogWrite(DV2_2, speed_B);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin init...");

  pinMode(DV1_1, OUTPUT);
  pinMode(DV1_2, OUTPUT);
  pinMode(DV2_1, OUTPUT);
  pinMode(DV2_2, OUTPUT);

  Serial.println("Finished");
}

void loop()
{
  motorA(80);
  motorB(0);
  delay(2000);

  motorA(0);
  motorB(80);
  delay(2000);
  
  motorA(100);
  motorB(100);
  delay(2000);

  motorA(0);
  motorB(0);
  delay(2000);
}
