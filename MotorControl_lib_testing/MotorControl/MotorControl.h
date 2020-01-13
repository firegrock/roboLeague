#ifndef MotorControl_h
#define MotorControl_h
#include <Arduino.h>
#include <analogWrite.h>


#define NORMAL 	0
#define REVERSE 1
#define FORWARD		0
#define BACKWARD	1
#define STOP		2

class MotorControl
{
private:
	void runFrw();
	void runBkw();
	uint8_t _dig_pin = 0, _pwm_pin = 0, _mode = 0, _reverse = 0;
	int16_t _duty = 0;

public:
	MotorControl(uint8_t dig_pin, uint8_t pwm_pin); 		// dig_pin - пин направления
																									// pwm_pin - ШИМ пин

	void setSpeed(uint16_t duty);
	void setMode(uint8_t mode);					// режим работы мотора:
																		 	// FORWARD
																			// BACKWARD
																			// STOP
	void setDirection(boolean direction);		// направление вращения (один раз настроить в setup вместо переподключения мотора)
																				  // NORMAL
																				  // REVERSE
};

#endif
