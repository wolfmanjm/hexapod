#pragma once

#include <cstdint>

//#define DUMMY 1

#ifndef DUMMY
#define USEGPIO 1
class adafruitss;
#ifdef USEGPIO
class PWM;
#endif

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
	#ifdef USEGPIO
	PWM *pwm;
	#else
	adafruitss* servos2;
	#endif
#else
	DummyServo* servos;
#endif
	const uint8_t type= 2;
	float current_angle[NSERVOS];
};
