#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "BluetoothSerial.h"
#include <MotorControl.h>


// TX2 (17) - Tilt D0
// D27 - IN1
// D14 - IN2
// D12 - IN3
// D13 - IN4


// ---------- Define variables
#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 27

#define switchTilt 17

#define MOTOR_MAX 200                     // Max PWM possible
#define JOY_MAX 40                        // Joystick max-amplitude
// -------------------- //


// ---------- Declare objects
MotorControl motorL(IN1, IN2);
MotorControl motorR(IN4, IN3);

WiFiUDP Udp;                             // UDP conenction
BluetoothSerial SerialBT;                // Object for bluetooth
// -------------------- //


// ---------- State machine
enum state_machine { ST_INIT, ST_CONNECT, ST_IDLE, ST_MOVE, ST_STOP };
uint8_t state;
// -------------------- //


// -------------------- //

// ---------- Class RobotCar
class RobotCar
{
  public:
    const char* m_ssid = "ROBOLEAGUE";
    const char* m_password = "1234567890";
    unsigned int m_UdpPort = 4210;                      // local port to listen on
    char m_packetBuffer[255];                           // buffer to hold incoming packet


    bool UDP_connected = 0;
    bool BT_connected = 0;
    

    int intData[2];                                     // array of received data
    boolean recievedFlag;
    int dutyR, dutyL;
    int signalX, signalY;
    int dataX, dataY;

    boolean getStarted;
    byte index_x;
    String string_convert = "";

    int tiltVal = 0;

  public:
    void initialize()
    {
      pinMode (switchTilt, INPUT);

      Serial.println("Initialization completed");
    }

    void waitConnection(bool UDPenable = 0, bool BTenable = 1)
    {
      if (UDPenable == 1)
      {
        Serial.println(WiFi.softAP(m_ssid, m_password) ? "AP connected" : "Failed to connect!");
        Udp.begin(m_UdpPort);

        UDP_connected = 1;
      }

      if (BTenable == 1)
      {
        SerialBT.begin("ROBOLEAGUE");
        Serial.println("Bluetooth module enabled and Ready to Pair");

        while (SerialBT.hasClient() == 0) {
          Serial.println("No client available");
          delay(1000);
        }

        BT_connected = 1;
      }
    }

    void idleVehicle()
    {
      if (SerialBT.hasClient() == 0) {
        stopVehicle();

        state = ST_CONNECT;
      }
      else
        state = ST_IDLE;

      tiltVal = inversed();
      parsing();

      if (recievedFlag) speedCalc();
    }

    void moveVehicle()
    {
      motorL.setSpeed(dutyR);
      motorR.setSpeed(dutyL);
    }

    void stopVehicle()
    {
      motorL.setMode(STOP);
      motorR.setMode(STOP);
    }

    void speedCalc()
    {
      recievedFlag = false;
      dataX = intData[0];
      dataY = intData[1];
      
      if (dataX == 0 && dataY == 0)
      {
        dutyR = dutyL = 0;
        stopVehicle();
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

        moveVehicle();
      }
    }

    void parsing()
    {
      if (SerialBT.available())
      {
        char incomingByte = SerialBT.read();                  // обязательно ЧИТАЕМ входящий символ

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

    bool inversed()
    {
      tiltVal = digitalRead(switchTilt);

      if (tiltVal == HIGH)
        return 0;
      else
        return 1;
    }

} car;

void setup()
{
  Serial.println("Begin initialization ...");
  Serial.begin(115200);

  car.initialize();
  state = ST_INIT;
  delay(10);

  Serial.println("Waiting for connection ...");
  car.waitConnection(0, 1);
  state = ST_CONNECT;
  delay(10);
}

void loop()
{
  switch (state)
  {
    case ST_INIT:
      Serial.println("State = initalization");
      state = ST_INIT;
      car.initialize();

      break;
    case ST_CONNECT:
      Serial.println("State = connection");
      state = ST_CONNECT;
      car.waitConnection();

      if (car.BT_connected == 1 || car.UDP_connected == 1) state = ST_IDLE;
      
      break;
    case ST_IDLE:
      Serial.println("State = idle");
      state = ST_IDLE;
      car.idleVehicle();

      break;
    case ST_MOVE:
      Serial.println("State = move");
      state = ST_MOVE;
      car.moveVehicle();

      Serial.print(car.tiltVal);
      Serial.print(' ');
      Serial.print(car.dutyL);
      Serial.print(' ');
      Serial.println(car.dutyR);

      break;
    case ST_STOP:
      Serial.println("State = stop");
      state = ST_STOP;
      car.stopVehicle();

      break;
  }
}
