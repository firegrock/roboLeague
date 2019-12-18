#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "WEMOS_Motor.h"
#include <PID_v1.h>
#include "BluetoothSerial.h"

// ---------- Declare all objects
WiFiUDP Udp;                            // UDP conenction
BluetoothSerial SerialBT;                 // Object for bluetooth

Motor motorA(0x30, _MOTOR_A, 1000);     // Motors left and right side
Motor motorB(0x30, _MOTOR_B, 1000);
                                        // Joystick control
#define MOTOR_MAX 100                   // Max PWM possible
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


// ---------- Parsing string
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
    {                                                     // если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';')     // если это не пробел И не конец
        string_convert += incomingByte;                   // складываем в строку
      else
      {                                               // если это пробел или ; конец пакета
        intData[index_x] = string_convert.toInt();    // преобразуем строку в int и кладём в массив
        string_convert = "";                          // очищаем строку
        index_x++;                                    // переходим к парсингу следующего элемента массива
      }
    }

    if (incomingByte == '$')
    {                                           // если это $
      getStarted = true;                        // поднимаем флаг, что можно парсить
      index_x= 0;                               // сбрасываем индекс
      string_convert = "";                      // очищаем строку
    }

    else if (incomingByte == ';')
    {                                           // если таки приняли ; - конец парсинга
      getStarted = false;                       // сброс
      recievedFlag = true;                      // флаг на принятие
    }
  }
}
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
  int speedL = 0;
  int speedR = 0;
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
    {     
      if(!SerialBT.begin("Robot controller 2"))
        Serial.println("An error occurred initializing Bluetooth");
      else
        Serial.println("Bluetooth initialized");
    }

    state = ST_CONNECT;
  }

  void idleVehicle()
  {
    motorA.setmotor(_STANDBY);
    motorB.setmotor(_STANDBY);
    
    state = ST_IDLE;
  }

  void leftSide(int speedLeft, int inverse = 0)
  {
    speedL = constrain(abd(speedLeft), 0, 100);
    if (speedL > 0)
      motorA.setmotor(_CW, speedL)
    else 
      motorA.setmotor(_CCW, speedL)
  }

  void rightSide(int speedLeft, int inverse = 0)
  {
    speedR = constrain(abd(speedRight), 0, 100);
    if (speedR > 0)
      motorA.setmotor(_CCW, speedR)
    else 
      motorA.setmotor(_CW, speedR)
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

int inverse = 0, speedRight = 0, speedLeft = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  delay(5);
  car.waitConnection(0, 1);
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

int incoming;
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
      state = ST_MOVE;
      inverse = car.inversed();
      parsing();

      if (recievedFlag)
      {
        recievedFlag = false;
        dataX = intData[0];
        dataY = intData[1];
        SerialBT.flush();
        
        signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // Y-axis signal
        signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // Х-axis signal
        
        dutyR = signalY + signalX;
        dutyL = signalY - signalX;
      
        Serial.println("SPEED R SPEED L");
        Serial.print(dutyR);
        Serial.print(" ");
        Serial.println(dutyL);

        car.leftSide(dutyL, inverse);
        car.rightSide(dutyR, inverse);
      }
      break;
    case ST_STOP:
      Serial.println("State = stop");
      car.stopVehicle();
      break;
    default: break;
  }
}
