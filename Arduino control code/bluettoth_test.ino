#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "WEMOS_Motor.h"
#include <PID_v1.h>
#include "BluetoothSerial.h"

// ---------- Declare all objects
WiFiUDP Udp;                            // UDP conenction
BluetoothSerial ESP_BT;                 // Object for bluetooth

Motor motorA(0x30, _MOTOR_A, 1000);     // Motors left and right side
Motor motorB(0x30, _MOTOR_B, 1000);
                                        // Joystick control
#define MOTOR_MAX 255                   // Max PWM possible
#define JOY_MAX 40                      // Joystick max-amplitude

const int statusLED = 13;
const int switchTilt = 17;
// -------------------- //


// ---------- PID initialization
double input, output, setpoint;
double Kp=4, Ki=1, Kd=0.01;
PID pidControl(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// -------------------- //


// ---------- State machine
enum state_machine { ST_INIT, ST_CONNECT, ST_IDLE, ST_MOVE, ST_STOP };
uint8_t state;
// -------------------- //


// ---------- Class robot
class Robot
{
public:
  const char* m_ssid = "ROBOLEAGUE";
  const char* m_password = "1234567890";
  unsigned int m_UdpPort = 4210;                      // local port to listen on
  char m_packetBuffer[255];                           // buffer to hold incoming packet

  int val = 0;
public:
  void initialize()
  {
    state = ST_INIT;
  }

  void waitConnection(bool UDPenable = 1, bool BTenable = 0)
  {
    if (UDPenable == 1)
    {
      Serial.println(WiFi.softAP(m_ssid, m_password) ? "AP connected" : "Failed to connect!");
      Udp.begin(m_UdpPort);
    }

    if (BTenable == 1)
      ESP_BT.begin("Robot controller");

    state = ST_CONNECT;
  }

  void idleVehicle()
  {
    motorA.setmotor(_STANDBY);
    motorB.setmotor(_STANDBY);
    
    state = ST_IDLE;
  }

  void moveVehicle(int speedLeft, int speedRight, int inverse = 0)
  {
    if (inverse)
    {
      motorA.setmotor(_CW, speedLeft);
      motorB.setmotor(_CCW, speedRight);
    }

    else
    {
      motorA.setmotor(_CCW, speedLeft);
      motorB.setmotor(_CW, speedRight);
    }

    state = ST_MOVE;
  }

  void stopVehicle()
  {
    motorA.setmotor(_STOP);
    motorB.setmotor(_STOP);

    state = ST_STOP;
  }

  bool inversed()
  {
    val = digitalRead(switchTilt);

    if (val == HIGH)
      return 1;
    else
      return 0;
  }
};

Robot car;
// -------------------- //

int inverse = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  delay(5);
  car.waitConnection(1, 0);
  delay(5);
  pidControl.SetMode(AUTOMATIC);
  pidControl.SetTunings(Kp, Ki, Kd);
  Serial.println("PID enable");
  delay(5);
  pinMode (statusLED, OUTPUT);
  pinMode (switchTilt, INPUT);
  delay(5);
  state = ST_MOVE;

  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации
}

void loop()
{
  switch (state)
  {
    case ST_INIT:
      Serial.println("State = initalization");
      car.initialize();
      break;
    case ST_CONNECT:
      Serial.println("State = connection");
      car.waitConnection();
      break;
    case ST_IDLE:
      Serial.println("State = idle");
      car.idleVehicle();
      break;
    case ST_MOVE:
    {
      Serial.println("State = move");
      inverse = car.inversed();
      Serial.println(inverse);

      for ( int i = 0; i < 10; ++i)
      {
        car.moveVehicle(15, 15, inverse);
        delay(50);
        Serial.print("Delay iteration");
        Serial.println(i);
        car.stopVehicle();
      }
//      Control using UDP
//      int input = Udp.parsePacket();
//
//      if (input)
//      {
//        // pidControl.Compute();
//        int len = Udp.read(car.m_packetBuffer, 255);
//        if (len > 0) car.m_packetBuffer[len] = 0;
//
//        String speedy_str(car.m_packetBuffer);
//        car.m_speedy = speedy_str.toInt();
//
//        Serial.println(car.m_speedy);
//        Serial.print(" ");
//
//        car.moveVehicle();
//      }
      break;
    }
    case ST_STOP:
      Serial.println("State = stop");
      car.stopVehicle();
      break;
    default: break;
  }
}


//// ---------- Parsing string
//int intData[2];                           // array of received data
//boolean recievedFlag;
//int dutyR, dutyL;
//int signalX, signalY;
//int dataX, dataY;
//
//boolean getStarted;
//byte index_x;
//String string_convert = "";
//
//void parsing()
//{
//  if (ESP_BT.available())
//  {
//    char incomingByte = ESP_BT.read();                // обязательно ЧИТАЕМ входящий символ
//
//    if (getStarted)
//    {                                                     // если приняли начальный символ (парсинг разрешён)
//      if (incomingByte != ' ' && incomingByte != ';')     // если это не пробел И не конец
//        string_convert += incomingByte;                   // складываем в строку
//      else
//      {                                               // если это пробел или ; конец пакета
//        intData[index_x] = string_convert.toInt();    // преобразуем строку в int и кладём в массив
//        string_convert = "";                          // очищаем строку
//        index_x++;                                    // переходим к парсингу следующего элемента массива
//      }
//    }
//
//    if (incomingByte == '$')
//    {                                           // если это $
//      getStarted = true;                        // поднимаем флаг, что можно парсить
//      index_x= 0;                               // сбрасываем индекс
//      string_convert = "";                      // очищаем строку
//    }
//
//    else if (incomingByte == ';')
//    {                                           // если таки приняли ; - конец парсинга
//      getStarted = false;                       // сброс
//      recievedFlag = true;                      // флаг на принятие
//    }
//  }
//}
//// -------------------- //
//
//  void moveVehicle(int leftSpeed, int rightSpeed)
//  {
//    if (carPosition == "Forward")
//    {
//      motorL.setmotor(_CW, leftSpeed);
//      motorR.setmotor(_CCW, rightSpeed);
//    }
//
//    if (carPosition == "Backward")
//    {
//      motorL.setmotor(_CCW, leftSpeed);
//      motorR.setmotor(_CW, rightSpeed);
//    }
//
//    state = ST_MOVE;
//  }
//
//int incoming;
//
//    parsing();
//    
//    if (recievedFlag)
//    {
//      recievedFlag = false;
//      dataX = intData[0];
//      dataY = intData[1];
//    
//      Serial.print(dataX);
//      Serial.print(" ");
//      Serial.println(dataY);
//    }
//    
//    if (dataX == 0 && dataY == 0)
//    {
//      car.stopVehicle();                  // "Death zone" - no movement
//    
//      dutyR = dutyL = 0;
//    } else {
//      signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // Y-axis signal
//      signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // Х-axis signal
//    
//      dutyR = signalY + signalX;
//      dutyL = signalY - signalX;
//    
//      if (dutyL > 0) car.carPosition = "Forward";
//      else car.carPosition = "Backward";
//    
//      if (dutyR > 0) car.carPosition = "Backward";
//      else car.carPosition = "Forward";
//    
//      dutyR = constrain(abs(dutyR), 0, MOTOR_MAX);
//      dutyL = constrain(abs(dutyL), 0, MOTOR_MAX);
//      
//      Serial.println("SPEED R SPEED L");
//      Serial.print(dutyR);
//      Serial.print(" ");
//      Serial.println(dutyL);
//    }
//
//    //  speed_left = input(dutyL);
//    //  speed_right = input(dutyR);
//    //  car.moveVehicle(speed_left, speed_right);
//}
