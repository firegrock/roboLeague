#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "BluetoothSerial.h"
#include <MotorControl.h>

// зел кор жел оранж

// TX2 = 17 - Tilt D0
// D27 - IN1
// D14 - IN2
// D12 - IN3
// D13 - IN4

#define switchTilt 17
int tiltVal = 0;

bool inversed()
{
  tiltVal = digitalRead(switchTilt);

  if (tiltVal == HIGH)
    return 0;
  else
    return 1;
}

#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 27

MotorControl motorL(IN1, IN2);
MotorControl motorR(IN4, IN3);

BluetoothSerial SerialBT;                 // Object for bluetooth

#define MOTOR_MAX 200                     // Max PWM possible
#define JOY_MAX 40                        // Joystick max-amplitude

// Parsing string
int intData[2];                           // array of received data
boolean recievedFlag;
int dutyR, dutyL;
int signalX, signalY;
int dataX, dataY;

boolean getStarted;
byte index_x;
String string_convert = "";

void parsing()
{
  if (SerialBT.available())
  {
    char incomingByte = SerialBT.read();                // обязательно ЧИТАЕМ входящий символ

    if (getStarted)
    { // если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';')     // если это не пробел И не конец
        string_convert += incomingByte;                   // складываем в строку
      else
      { // если это пробел или ; конец пакета
        intData[index_x] = string_convert.toInt();        // преобразуем строку в int и кладём в массив
        string_convert = "";                              // очищаем строку
        index_x++;                                        // переходим к парсингу следующего элемента массива
      }
    }

    if (incomingByte == '$')
    { // если это $
      getStarted = true;                        // поднимаем флаг, что можно парсить
      index_x = 0;                               // сбрасываем индекс
      string_convert = "";                      // очищаем строку
    }

    else if (incomingByte == ';')
    { // если таки приняли ; - конец парсинга
      getStarted = false;                       // сброс
      recievedFlag = true;                      // флаг на принятие
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  delay(5);
  if (!SerialBT.begin("ROBOLEAGUE"))
    Serial.println("An error occurred initializing Bluetooth");
  else
    Serial.println("Bluetooth initialized");
  delay(5);

  pinMode (switchTilt, INPUT);
  
  Serial.println("Initialization completed");
}

void loop()
{
  tiltVal = inversed();

  while (SerialBT.hasClient() == 0)
  {
    Serial.println("No client available");
    delay(1500);

    if (SerialBT.hasClient() == 1) Serial.println("Client available");
  }

  parsing();

  if (recievedFlag)
  {
    recievedFlag = false;
    dataX = intData[0];
    dataY = intData[1];
  }

  if (dataX == 0 && dataY == 0)
  {
    dutyR = 0;
    dutyL = dutyR;

    motorL.setMode(STOP);
    motorR.setMode(STOP);
  }

  else
  {
    signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // сигнал по У
    signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // сигнал по Х

    dutyR = signalY + signalX;
    dutyL = signalY - signalX;

    if (dutyR > 0) motorL.setMode(FORWARD);
    else motorL.setMode(BACKWARD);

    if (dutyL > 0) motorR.setMode(FORWARD);
    else motorR.setMode(BACKWARD);

    dutyR = constrain(abs(dutyR), 0, MOTOR_MAX);
    dutyL = constrain(abs(dutyL), 0, MOTOR_MAX);
  }

  Serial.print("Speed : ");
  Serial.print(dutyL);
  Serial.print(' ');
  Serial.print(dutyR);
  Serial.print(' ');
  Serial.println(tiltVal);  
  
  motorL.setSpeed(dutyR);
  motorR.setSpeed(dutyL);
}
