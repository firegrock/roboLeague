#include <Robot_football.h>

// ---------- Declare all objects
WiFiUDP Udp;                            // UDP conenction
Motor motorA(0x30, _MOTOR_A, 1000);     // Motors left and right side
Motor motorB(0x30, _MOTOR_B, 1000);
// -------------------- //

// ---------- State machine
enum state_machine { ST_INIT, ST_CONNECT, ST_IDLE, ST_MOVE, ST_STOP };
uint8_t state;
// -------------------- //

// ---------- PID initialization
double input, output, setpoint;
double Kp=4, Ki=1, Kd=0.01;
PID pidControl(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// -------------------- //

// ---------- Class robot
class Robot
{
public:
  const char* m_ssid = "ROBOLEAGUE";
  const char* m_password = "1234567890";
  unsigned int m_UdpPort = 4210;                      // local port to listen on
  char m_packetBuffer[255];                           // buffer to hold incoming packet

  bool m_carPosition = false;                         // vehicle position
  int m_speedy = 0;                                   // initial speed

public:
  void initialize()
  {
    state = ST_INIT;
  }

  void waitConnection()
  {
    Serial.println(WiFi.softAP(m_ssid, m_password) ? "AP connected" : "Failed to connect!");
    Udp.begin(m_UdpPort);

    state = ST_CONNECT;
  }

  void idleVehicle()
  {
    state = ST_IDLE;

    motorA.setmotor(_STANDBY);
    motorB.setmotor(_STANDBY);
  }

  void moveVehicle()
  {
    state = ST_MOVE;
    m_speedy = output;

    if (m_carPosition)
    {
      motorA.setmotor(_CW, m_speedy);
      motorB.setmotor(_CCW, m_speedy);
    }

    else
    {
      motorA.setmotor(_CCW, m_speedy);
      motorB.setmotor(_CW, m_speedy);
    }
  }

  void stopVehicle()
  {
    state = ST_STOP;

    motorA.setmotor(_STOP);
    motorB.setmotor(_STOP);
  }
};
// -------------------- //

Robot car;
void setup()
{
  Serial.begin(115200);
  Serial.println("Begin initialization ...");

  delay(5);
  car.waitConnection();
  pidControl.SetMode(AUTOMATIC);
  pidControl.SetTunings(Kp, Ki, Kd);
  delay(5);

  Serial.println("Initialization completed"); // выводим сообщение об удачной инициализации
}

void loop()
{
  switch (state)
  {
    case ST_INIT:
      car.initialize();
      break;
    case ST_CONNECT:
      car.waitConnection();
      break;
    case ST_IDLE:
      car.idleVehicle();
      break;
    case ST_MOVE:
      Serial.println(state);
      input = Udp.parsePacket();

      if (input)
      {
        pidControl.Compute();
        // read the packet into packetBufffer
        int len = Udp.read(car.m_packetBuffer, 255);
        if (len > 0) car.m_packetBuffer[len] = 0;

        String speedy_str(car.m_packetBuffer);
        car.m_speedy = speedy_str.toInt();

        Serial.println(car.m_speedy);
        Serial.print(" ");
      }

      car.moveVehicle();
      break;
    case ST_STOP:
      car.stopVehicle();
      break;
    default: break;
  }
}
