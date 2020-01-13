#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "BluetoothSerial.h"
#include <MotorControl.h>

#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 27

MotorControl motor1(IN1, IN2);
MotorControl motor2(IN3, IN4);

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  Serial.println("Initialization completed");
}

int incomingByte = 0;
int inpSpeed = 80;

// Управление через Serial Monitor 
// 1 - движение вперед
// 2 - движение назад
// 3 - стоп

void loop()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();

    switch (incomingByte) {
      case '1':
        Serial.print("Forward");
        Serial.println(inpSpeed);
        motor1.setMode(FORWARD);
        motor1.setSpeed(inpSpeed);
        motor2.setMode(FORWARD);
        motor2.setSpeed(inpSpeed);

        break;

      case '2':
        Serial.print("Backward");
        Serial.println(inpSpeed);
        motor1.setMode(BACKWARD);
        motor1.setSpeed(inpSpeed);
        motor2.setMode(BACKWARD);
        motor2.setSpeed(inpSpeed);
        break;

      case '3':
        Serial.println("Stop");
//        motor1.setSpeed(0);
//        motor2.setSpeed(0);
        motor1.setMode(STOP);
        motor2.setMode(STOP);
        break;

      default:        break;
    }
  }
}
