#pragma once

#include <cstdint>


//#define DUMMY 1

#ifndef DUMMY
namespace upm {
	class adafruitss;
}
class PWM;
#else
#include <stdio.h>
class DummyServo
{
public:
	void servo(uint8_t channel, uint8_t type, uint16_t a) { printf("channel: %d, angle: %d\n", channel, a); }
};
#endif

class Servo
{
public:
	Servo();
	~Servo();
	void move(uint8_t port, float rads);
	void updateServo(uint8_t channel, float angle);

private:
#ifndef DUMMY
	upm::adafruitss* servos;
	PWM *pwm;
#else
	DummyServo* servos;
#endif
	const uint8_t type= 1;
};
