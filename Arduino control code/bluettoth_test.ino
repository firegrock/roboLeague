  #include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
#include "WEMOS_Motor.h"

// ---------- Declare all objects
BluetoothSerial ESP_BT;                 // Object for bluetooth
Motor motorL(0x30, _MOTOR_A, 1000);     // Motors left and right side
Motor motorR(0x30, _MOTOR_B, 1000);

#define MOTOR_MAX 255                   // Max PWM possible
#define JOY_MAX 40                      // Joystick max-amplitude

// ---------- PID controller
// double input, output, setpoint;
// double Kp=4, Ki=1, Kd=0.01;
// PID pidControl(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

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
  if (ESP_BT.available())
  {
    char incomingByte = ESP_BT.read();                // обязательно ЧИТАЕМ входящий символ

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
//  bool m_carPosition = false;          // vehicle position
  String carPosition = "";
public:
  void initialize()
  {
    state = ST_INIT;
  }

  void waitConnection()
  {
    ESP_BT.begin("Robot controller");

    state = ST_CONNECT;
  }

  void idleVehicle()
  {
    motorL.setmotor(_STANDBY);
    motorR.setmotor(_STANDBY);

    state = ST_IDLE;
  }

  void moveVehicle(int leftSpeed, int rightSpeed)
  {
    if (carPosition == "Forward")
    {
      motorL.setmotor(_CW, leftSpeed);
      motorR.setmotor(_CCW, rightSpeed);
    }

    if (carPosition == "Backward")
    {
      motorL.setmotor(_CCW, leftSpeed);
      motorR.setmotor(_CW, rightSpeed);
    }

    state = ST_MOVE;
  }

  void stopVehicle()
  {
    motorR.setmotor(_STOP);
    motorL.setmotor(_STOP);

    state = ST_STOP;
  }
};
Robot car;
// -------------------- //

int incoming;
void setup()
{
  Serial.begin(115200);

  car.initialize();
  delay(5);
  car.waitConnection();
  delay(5);
  // pidControl.SetMode(AUTOMATIC);
  // pidControl.SetTunings(Kp, Ki, Kd);
  delay(5);
  
  Serial.println("Bluetooth Device is Ready to Pair");
}

void loop()
{
   switch (state)
  // {
  //   case ST_INIT:
  //     car.initialize();
  //     break;
  //   case ST_CONNECT:
  //     car.waitConnection();
  //     break;
  //   case ST_IDLE:
  //     car.idleVehicle();
  //     break;
  //   case ST_MOVE:
          parsing();
    
    if (recievedFlag)
    {
      recievedFlag = false;
      dataX = intData[0];
      dataY = intData[1];
    
      Serial.print(dataX);
      Serial.print(" ");
      Serial.println(dataY);
    }
    
    if (dataX == 0 && dataY == 0)
    {
      car.stopVehicle();                  // "Death zone" - no movement
    
      dutyR = dutyL = 0;
    } else {
      signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // Y-axis signal
      signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); // Х-axis signal
    
      dutyR = signalY + signalX;
      dutyL = signalY - signalX;
    
      if (dutyL > 0) car.carPosition = "Forward";
      else car.carPosition = "Backward";
    
      if (dutyR > 0) car.carPosition = "Backward";
      else car.carPosition = "Forward";
    
      dutyR = constrain(abs(dutyR), 0, MOTOR_MAX);
      dutyL = constrain(abs(dutyL), 0, MOTOR_MAX);
      
      Serial.println("SPEED R SPEED L");
      Serial.print(dutyR);
      Serial.print(" ");
      Serial.println(dutyL);
    }

    //  speed_left = input(dutyL);
    //  speed_right = input(dutyR);
    //  car.moveVehicle(speed_left, speed_right);
  //     break;
  //   case ST_STOP:
  //     car.stopVehicle();
  //     break;
  //   default: break;
  // }
}
