#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "WEMOS_Motor.h"
#include <PID_v1.h>
#include "BluetoothSerial.h"

// ---------- Declare all objects
WiFiUDP Udp;                            // UDP conenction
BluetoothSerial SerialBT;               // Object for bluetooth

Motor motorA(0x30, _MOTOR_A, 1000);     // Motors left and right side
Motor motorB(0x30, _MOTOR_B, 1000);
uint8_t forwardL = _CW;
uint8_t backwardL = _CCW;
uint8_t forwardR = _CCW;
uint8_t backwardR = _CW;
                                        // Joystick control
#define MOTOR_MAX 60                    // Max PWM possible
#define JOY_MAX 40                      // Joystick max-amplitude

const int statusLED = 13;
const int switchTilt = 17;
// -------------------- //


// ---------- PID initialization
double input, output, setpoint;
double Kp = 4, Ki = 1, Kd = 0.01;
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
      {                                                   // если это пробел или ; конец пакета
        intData[index_x] = string_convert.toInt();        // преобразуем строку в int и кладём в массив
        string_convert = "";                              // очищаем строку
        index_x++;                                        // переходим к парсингу следующего элемента массива
      }
    }

    if (incomingByte == '$')
    {                                           // если это $
      getStarted = true;                        // поднимаем флаг, что можно парсить
      index_x = 0;                               // сбрасываем индекс
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

  int carInverse = 0;
  
public:
  void initialize() 
  { 
    state = ST_INIT; 
    
    pidControl.SetMode(AUTOMATIC);
    pidControl.SetTunings(Kp, Ki, Kd);
    Serial.println("PID tuned up");
    
    pinMode (statusLED, OUTPUT);
    pinMode (switchTilt, INPUT);
  }

  void waitConnection(bool UDPenable = 0, bool BTenable = 1)
  {
    state = ST_CONNECT;
    
    if (UDPenable == 1)
    {
      Serial.println(WiFi.softAP(m_ssid, m_password) ? "AP connected" : "Failed to connect!");
      Udp.begin(m_UdpPort);
    }

    if (BTenable == 1)
    {
      SerialBT.begin("ROBOLEAGUE");
      Serial.println("Bluetooth module enabled");
      Serial.println("Bluetooth module is Ready to Pair");

      while (SerialBT.hasClient() == 0) { Serial.println("No client available"); delay(1000); }

      state = ST_IDLE;
    }
  }

  void idleVehicle()
  {
    state = ST_IDLE;

    if (SerialBT.hasClient() == 0) { Serial.println("No client available"); delay(1000); }
    
    motorA.setmotor(_STANDBY);
    motorB.setmotor(_STANDBY);

    parsing();

    if (recievedFlag)
    {
      state = ST_MOVE;
    }
  }

  void leftSide(int speedLeft, int inverse = 0)
  {  
    if (speedL > 0) {
      speedL = constrain(abs(speedLeft), 0, MOTOR_MAX);
      motorA.setmotor(forwardL, speedL);
    }
    else {
      speedL = constrain(abs(speedLeft), 0, MOTOR_MAX);
      motorA.setmotor(backwardL, speedL);
    }
  }

  void rightSide(int speedRight, int inverse = 0)
  {
    
    if (speedR > 0) {
      speedR = constrain(abs(speedRight), 0, MOTOR_MAX);
      motorA.setmotor(forwardR, speedR);
    }
    else {
      speedR = constrain(abs(speedRight), 0, MOTOR_MAX);
      motorA.setmotor(backwardR, speedR); 
    }
  }

  void stopVehicle()
  {
    motorA.setmotor(_STOP);
    motorB.setmotor(_STOP);

    state = ST_IDLE;
  }

  bool inversed()
  {
    val = digitalRead(switchTilt);

    if (val == HIGH)
      return 1;
    else {
      forwardL  = _CCW;
      backwardL = _CW;
      forwardR  = _CW;
      backwardR = _CCW;
      return 0;
    }
  }
} car;
// -------------------- //


void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  car.initialize();
  delay(5);
  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации

  car.waitConnection(0, 1);
  delay(5);
}

int inverse = 0, speedRight = 0, speedLeft = 0;
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
      Serial.println("State = idle - Robot stopped");
      car.idleVehicle();

      break;
      
    case ST_MOVE:
      if (SerialBT.hasClient() == 0) { state = ST_IDLE; break; }
      
      inverse = car.inversed();
      parsing();

      if (recievedFlag)
      {
        state = ST_MOVE;
        recievedFlag = false;
        dataX = intData[0];
        dataY = intData[1];
        SerialBT.flush();
    
        signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // Y-axis signal
        signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // Х-axis signal
    
        dutyR = signalY + signalX;
        dutyL = signalY - signalX;
        
        Serial.println("SPEED: ");
        Serial.print(dutyR);
        Serial.print(" ");
        Serial.println(dutyL);
    
        motorA.setmotor(_CW, dutyL);
        motorB.setmotor(_CCW, dutyR);
      } 

      break;
      
    case ST_STOP:
      Serial.println("State = stop");
      car.stopVehicle();

      break;
    default: break;
  }
}
