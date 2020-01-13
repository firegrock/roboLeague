#include "MotorControl.h"

MotorControl::MotorControl(uint8_t dig_pin, uint8_t pwm_pin) {
	_dig_pin = dig_pin;
	_pwm_pin = pwm_pin;

	pinMode(_dig_pin, OUTPUT);
	pinMode(_pwm_pin, OUTPUT);
	digitalWrite(_dig_pin, 0);
	digitalWrite(_pwm_pin, 0);
}

void MotorControl::setDirection(boolean direction) {
	_reverse = direction;
}

void MotorControl::setSpeed(uint16_t duty) {
	_duty = duty;

	if (_mode == FORWARD) {
		if (_reverse) runFrw();
		else runBkw();
	}

	else if (_mode == BACKWARD) {
		if (_reverse) runBkw();
		else runFrw();
	} else if (_mode == STOP) {
		digitalWrite(_pwm_pin, 0);
	}
}

void MotorControl::runFrw() {
	digitalWrite(_dig_pin, 0);
	analogWrite(_pwm_pin, _duty);
}
void MotorControl::runBkw() {
	digitalWrite(_dig_pin, 1);
	analogWrite(_pwm_pin, 255 - _duty);
}

void MotorControl::setMode(uint8_t mode) {
	_mode = mode;

	if (mode == STOP) {
		digitalWrite(_dig_pin, 0);
		analogWrite(_pwm_pin, 0);
	}
}
