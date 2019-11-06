#include <Wire.h>
#include <TroykaIMU.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>                      // changed from UDP to Udp for linux-distros

// множитель фильтра
#define BETA 0.22f

// STATE MACHINE
//#define ST_INIT 1
//#define ST_WAIT_CONNECT 2
//#define ST_IDLE 3
//#define ST_MOVE 4
//#define ST_STOP 5
enum state_machine { ST_INIT, ST_WAIT_CONNECT, ST_IDLE, ST_MOVE, ST_STOP };

int DV1_1 = 12;
int DV1_2 = 14;
int DV2_1 = 15;
int DV2_2 = 13;

const char* ssid = "ROBOLEAGUE";
const char* password = "1234567890";

WiFiUDP Udp;
unsigned int UdpPort = 4210;                    // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];      // buffer to hold incoming packet
char  ReplyBuffer[] = "pososi, potom prosi";    // a string to send backchar

Madgwick filter;     // создаём объект для фильтра Madgwick
Accelerometer accel; // создаём объект для работы с акселерометром
Gyroscope gyro;      // создаём объект для работы с гироскопом

float gx, gy, gz, ax, ay, az;   // переменные для данных с гироскопов, акселерометров
float yaw, pitch, roll;         // получаемые углы ориентации
float fps = 100;                // переменная для хранения частоты выборок фильтра
bool reversed = false;          // флаг состояния машинки
int leftSpeed = 0;
int rightSpeed = 0;

class Motor                     // класс для работы с ДПТ, подключенными через драйвер
{
  public:
    int pin1; //пин для задания направления или ШИМ
    int pin2; //пин для задания направления или ШИМ//пин для задания направления или ШИМ
  public:
    Motor(int p1, int p2): // ну конструктор, ну ясно жи
      pin1(p1),
      pin2(p2)
    {
    }
    void initilize() //инициализация пинов, к которым подключен двигатель
    {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }
    void moveConst(int speedy, bool inverted) // постоянное вращение с учетом перевернутости машинки
    {
      // машина состояний, отрабатывающая разные ситуации (перевернутость машинки и направление вектора скорости)
      if (inverted == false && speedy > 0)
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, speedy);
      }
      else if (inverted == false && speedy < 0)
      {
        digitalWrite(pin1, speedy);
        analogWrite(pin2, LOW);
      }
      else if (inverted == true && speedy > 0)
      {
        digitalWrite(pin1, speedy);
        analogWrite(pin2, LOW);
      }
      else if (inverted == true && speedy < 0)
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, speedy);
      }
      else if (speedy == 0)
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, LOW);
      }
    }
    void moveConst(int speedy) // постоянное вращение без учета перевернутости машинки
    {
      if (speedy > 0)
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, speedy);
      }
      if (speedy < 0)
      {
        Serial.println(pin1);
        digitalWrite(pin1, speedy);
        analogWrite(pin2, LOW);
      }
    }
};
Motor leftMotor(DV1_1, DV1_2);
Motor rightMotor(DV2_1, DV2_2);

void moveVehicle(int l, int r, bool inv)
{
  leftMotor.moveConst(l, inv);
  rightMotor.moveConst(r, inv);
}

void waitConnection()
{
  // wait until smb connects to AP
}

void waitStart()
{
  // ST IDLE block code
}

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
//  leftMotor.initilize();
//  rightMotor.initilize();

  accel.begin();  // инициализация акселерометра
  gyro.begin();   // инициализация гироскопа

  int state = ST_INIT;
}

void loop() 
{
  while (true)
  {
    switch(state)
    {
      case ST_INIT:
        leftMotor.initilize();
        rightMotor.initilize();
        Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации

        // Добавить проверку на инициализацию
        
        break;
      case ST_WAIT_CONNECT:
        waitConnection(); // Ожидания соединения
        break;
      case ST_IDLE:
        waitStart();
        break;
      case ST_MOVE:
        // Проверка данных с IMU, настройка флагов и скоростей, считывание данных и запуск
        break;
      case ST_STOP:
        break;
    }
  }

}
