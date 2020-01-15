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
MotorControl motorR(IN3, IN4);

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


    bool initialized = 0;
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
    float k_l = 1;
    float k_r = 1;

  public:
    void initialize()
    {
      state = ST_INIT;
      pinMode (switchTilt, INPUT);

      Serial.println("Initialization completed");
      initialized = 1;
    }

    void waitConnection(bool UDPenable = 0, bool BTenable = 1)
    {
      state = ST_CONNECT;
      if (UDPenable == 1)
      {
        Serial.println(WiFi.softAP(m_ssid, m_password) ? "AP connected" : "Failed to connect!");
        Udp.begin(m_UdpPort);

        UDP_connected = 1;
      }

      if (BTenable == 1)
      {
        SerialBT.begin("ROBOLEAGUE");

        while (SerialBT.hasClient() == 0) {
          Serial.println("No client available");
          delay(1000);
        }

        Serial.println("BT connected");
        
        BT_connected = 1;
      }
    }

    void idleVehicle()
    {
      if (SerialBT.hasClient() == 0) state = ST_CONNECT;
      else state = ST_IDLE;
      
      parsing();
      if (recievedFlag) state = ST_MOVE;
    }

    void moveVehicle()
    {
      motorL.setSpeed(dutyL * k_l);
      motorR.setSpeed(dutyR * k_r);
    }

    void stopVehicle()
    {
      motorL.setMode(STOP);
      motorR.setMode(STOP);

      state = ST_IDLE;
    }

    void speedCalc()
    {
      recievedFlag = false;
      dataX = intData[0];
      dataY = intData[1];
  
      if (dataX == 0 && dataY == 0)
      {
        dutyR = 0;
        dutyL = 0;
        
        state = ST_STOP;
      }
  
      else
      {
        tiltVal = getTilt_pos();
        
        signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         
        signalX = map((dataX), -JOY_MAX, JOY_MAX, -MOTOR_MAX / 2, MOTOR_MAX / 2); 
  
        dutyR = signalY + signalX;
        dutyL = signalY - signalX;
  
        if (dutyR > 0) { motorL.setMode(FORWARD); k_l = 1.23; }
        else { motorL.setMode(BACKWARD); k_l = 1.74; }
  
        if (dutyL > 0) { motorR.setMode(FORWARD); k_r = 1; }
        else { motorR.setMode(BACKWARD); k_r = 0.823; }
  
        dutyR = constrain(abs(dutyR), 0, MOTOR_MAX);
        dutyL = constrain(abs(dutyL), 0, MOTOR_MAX);

        state = ST_MOVE;
      }
    }

    void parsing()
    {
      if (SerialBT.available())
      {
        char incomingByte = SerialBT.read();                  

        if (getStarted)
        {
          if (incomingByte != ' ' && incomingByte != ';')     
            string_convert += incomingByte;                   
          else
          {
            intData[index_x] = string_convert.toInt();        
            string_convert = "";                              
            index_x++;                                        
          }
        }

        if (incomingByte == '$')
        { 
          getStarted = true;                       
          index_x = 0;                             
          string_convert = "";                     
        }

        else if (incomingByte == ';')
        { 
          getStarted = false;                       
          recievedFlag = true;                      
        }
      }
    }

    bool getTilt_pos()
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
  if (car.initialized) state = ST_INIT;
  delay(10);

  Serial.println("Waiting for connection ...");
  car.waitConnection();
  if (car.BT_connected || car.UDP_connected) state = ST_IDLE;
  delay(10);
}

void loop()
{
  switch (state)
  {
    case ST_CONNECT:
      while (!car.BT_connected || !car.UDP_connected) 
      { 
        Serial.println("No client available"); 
        delay(1000);
        
        if (car.BT_connected || car.UDP_connected) state = ST_IDLE;
      }

      break;
    
    case ST_IDLE:
      Serial.println("State = idle");
      car.idleVehicle();

      break;
      
    case ST_MOVE:
      car.parsing();
      if (car.recievedFlag) car.speedCalc();
      
      car.moveVehicle();

      Serial.print(car.tiltVal);
      Serial.print(' ');
      Serial.print(car.dutyL);
      Serial.print(' ');
      Serial.println(car.dutyR);

      break;
      
    case ST_STOP:
      Serial.println("State = stop");
      car.stopVehicle();

      break;
  }
}
