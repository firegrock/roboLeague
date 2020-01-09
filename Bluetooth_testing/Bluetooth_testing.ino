#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;               // Object for bluetooth

#define MOTOR_MAX 200                    // Max PWM possible
#define JOY_MAX 40                      // Joystick max-amplitude

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
  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации
}

void loop()
{
  parsing();

  if (recievedFlag)
  {
    recievedFlag = false;
    dataX = intData[0];
    dataY = intData[1];

    signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // Y-axis signal
    signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // Х-axis signal

    dutyR = signalY + signalX;
    dutyL = signalY - signalX;

    Serial.println("SPEED: ");
    Serial.print(dutyR);
    Serial.print(" ");
    Serial.println(dutyL);
  }
}
