#pragma once

#include <cstdint>

//#define DUMMY 1

#ifndef DUMMY
class adafruitss;
class PWM;
#else
#include <stdio.h>
class DummyServo
{
public:
	void servo(uint8_t channel, uint8_t type, float a) { printf("channel: %d, angle: %f\n", channel, a); }
};
#endif

class Servo
{
public:
	Servo();
	~Servo();
	void move(uint8_t port, float rads);
	void updateServo(uint8_t channel, float angle);
	const static uint8_t NSERVOS= 18;

private:
#ifndef DUMMY
	adafruitss* servos;
	PWM *pwm;
#else
	DummyServo* servos;
#endif
	const uint8_t type= 2;
	float current_angle[NSERVOS];
};
