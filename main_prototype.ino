#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
//#include <TroykaIMU.h>
#include <WiFiClient.h>
#include "WEMOS_Motor.h"

// ------ UDP and AP ------ //
const char* ssid = "ROBOLEAGUE";
const char* password = "1234567890";

WiFiUDP Udp;
unsigned int UdpPort = 4210;                    // local port to listen on
char packetBuffer[255];      // buffer to hold incoming packet
char  ReplyBuffer[] = "pososi, potom prosi";    // a string to send backchar

// ------ ---------- ------ //

// STATE MACHINE
enum state_machine { ST_INIT, ST_WAIT_CONNECT, ST_IDLE, ST_MOVE, ST_STOP };

// множитель фильтра
#define BETA 0.22f

//Madgwick filter;     // создаём объект для фильтра Madgwick
//Accelerometer accel; // создаём объект для работы с акселерометром
//Gyroscope gyro;      // создаём объект для работы с гироскопом
//
float gx, gy, gz, ax, ay, az;   // переменные для данных с гироскопов, акселерометров
float yaw, pitch, roll;         // получаемые углы ориентации
float fps = 100;                // переменная для хранения частоты выборок фильтра
bool reversed = false;          // флаг состояния машинки
int leftSpeed = 0;
int rightSpeed = 0;

void moveVehicle(int speedy, bool inverted, Motor M1, Motor M2) 
{
  // машина состояний, отрабатывающая разные ситуации (перевернутость машинки и направление вектора скорости)
  if (inverted == false)
  {
    M1.setmotor(_CW, speedy);
    M2.setmotor(_CW, speedy);
  }

  else if (inverted == true)
  {
    M1.setmotor(_CCW, speedy);
    M2.setmotor(_CCW, speedy);
  }
}

void waitConnection()
{
  // wait until smb connects to AP
}

void waitStart()
{
  // ST IDLE block code
}

Motor motorA(0x30, _MOTOR_A, 1000);
Motor motorB(0x30, _MOTOR_B, 1000);
uint8_t state;

void setup()
{
  // открываем последовательный порт
  Serial.begin(115200);
  Serial.println("Begin init...");
  delay(5);
  Serial.println(WiFi.softAP(ssid, password) ? "Ready" : "Failed!");
  delay(5);
  Udp.begin(UdpPort);
  delay(5);

  // Добавить функции с возвращающими значениями ( if return true -> init sucess )
//  accel.begin();  // инициализация акселерометра
//  gyro.begin();   // инициализация гироскопа
  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации
}

void loop()
{
  while (true)
  {
    switch(state)
    {
      case ST_WAIT_CONNECT:
        waitConnection(); // Ожидания соединения
        break;
      case ST_IDLE:
        waitStart();
        break;
      case ST_MOVE:
        unsigned long startMillis = millis(); // запоминаем текущее время// запоминаем текущее время
//        accel.readGXYZ(&ax, &ay, &az); // считываем данные с акселерометра в единицах G
//        gyro.readRadPerSecXYZ(&gx, &gy, &gz); // считываем данные с акселерометра в радианах в секунду
//
//        filter.setKoeff(fps, BETA); // устанавливаем коэффициенты фильтра
//        filter.update(gx, gy, gz, ax, ay, az); // обновляем входные данные в фильтр

//        roll = filter.getRollDeg(); // получение углов yaw, pitch и roll из фильтра

        //Serial.print("roll: "); // выводим полученные углы в serial-порт
        //Serial.println(roll);

        unsigned long deltaMillis = millis() - startMillis; // вычисляем затраченное время на обработку данных

        fps = 1000 / deltaMillis; // вычисляем частоту обработки фильтра

        if (roll < 150.0 && roll > -150.0) // проверяем перевернута ли машинка
          reversed = false;
        else
          reversed = true;

        int packetSize = Udp.parsePacket();

        if (packetSize)
        {
          // read the packet into packetBufffer
          int len = Udp.read(packetBuffer, 255);
          if (len > 0)
            packetBuffer[len] = 0;
          //Serial.println("Contents:");
          //Serial.println(packetBuffer);

          String str(packetBuffer);
          String lstr, rstr;
          str = str.substring(0, str.indexOf(';'));
          lstr = str.substring(0, str.indexOf(','));
          rstr = str.substring(str.indexOf(',') + 1, str.indexOf(';'));

          leftSpeed = lstr.toInt();
          rightSpeed = rstr.toInt();

          Serial.print(leftSpeed);
          Serial.print(" ");
          Serial.println(rightSpeed);

          //Udp.flush();
          // send a reply, to the IP address and port that sent us the packet we received
          //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          //Udp.write(ReplyBuffer);
          //Udp.endPacket();
        }

        moveVehicle(leftSpeed, reversed, motorA, motorB);
        
        break;
//      case ST_STOP:
//        Serial.print("Stop");
//        break;
    }
  }

}
