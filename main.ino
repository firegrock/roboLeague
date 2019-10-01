#include <Wire.h>
#include <TroykaIMU.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>

// множитель фильтра
#define BETA 0.22f

const char* ssid = "ROBOLEAGUE";
const char* password = "1234567890";

WiFiUDP Udp;
unsigned int UdpPort = 4210;  // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet
char  ReplyBuffer[] = "pososi, potom prosi";       // a string to send backchar

Madgwick filter; // создаём объект для фильтра Madgwick

Accelerometer accel; // создаём объект для работы с акселерометром
Gyroscope gyro; // создаём объект для работы с гироскопом

float gx, gy, gz, ax, ay, az; // переменные для данных с гироскопов, акселерометров

float yaw, pitch, roll; // получаемые углы ориентации

float fps = 100; // переменная для хранения частоты выборок фильтра

bool reversed = false; // флаг состояния машинки

int leftSpeed = 0;
int rightSpeed = 0;

class Motor // класс для работы с ДПТ, подключенными через драйвер
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

int DV1_1 = 12;
int DV1_2 = 14;
int DV2_1 = 15;
int DV2_2 = 13;

Motor leftMotor(DV1_1, DV1_2);
Motor rightMotor(DV2_1, DV2_2);


void moveVehicle(int l, int r, bool inv)
{
  leftMotor.moveConst(l, inv);
  rightMotor.moveConst(r, inv);
}

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
  leftMotor.initilize();
  rightMotor.initilize();

  accel.begin();  // инициализация акселерометра
  gyro.begin(); // инициализация гироскопа

  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации
}

void loop()
{
  char charBuf[10];
  int speedy;
  char charSpeed[10];

  /*while (Serial.available() > 0)
    {
    Serial.readString().toCharArray(charBuf, 10);
    sscanf(charBuf, "%s", charSpeed);

    speedy = atof(charSpeed);
    Serial.print("speedy: ");
    }*/

  unsigned long startMillis = millis(); // запоминаем текущее время// запоминаем текущее время

  accel.readGXYZ(&ax, &ay, &az); // считываем данные с акселерометра в единицах G
  gyro.readRadPerSecXYZ(&gx, &gy, &gz); // считываем данные с акселерометра в радианах в секунду

  filter.setKoeff(fps, BETA); // устанавливаем коэффициенты фильтра
  filter.update(gx, gy, gz, ax, ay, az); // обновляем входные данные в фильтр

  roll = filter.getRollDeg(); // получение углов yaw, pitch и roll из фильтра

  //Serial.print("roll: "); // выводим полученные углы в serial-порт
  //Serial.println(roll);

  unsigned long deltaMillis = millis() - startMillis; // вычисляем затраченное время на обработку данных

  fps = 1000 / deltaMillis; // вычисляем частоту обработки фильтра

  if (roll < 150.0 && roll > -150.0) // проверяем перевернута ли машинка
  {
    reversed = false;
  }
  else
  {
    reversed = true;
  }

  int packetSize = Udp.parsePacket();

  if (packetSize)
  {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
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

  moveVehicle(leftSpeed, rightSpeed, reversed);
}
