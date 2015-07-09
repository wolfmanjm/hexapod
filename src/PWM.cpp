#include "Servo.h"

#ifdef USEGPIO
#include "PWM.h"

#include <cmath>

#include "mraa.hpp"

PWM::PWM(uint8_t type)
{
	this->type= type;
	// 2 GPIO based PWMs
	pwm.push_back(new mraa::Pwm(20)); // PWM0 J18-7
	pwm.push_back(new mraa::Pwm(21)); // PWM3 J18-8
	// seems it only support 60Hz and a few other frequencies not very accurately
	setFrequency(0, 60); // 60Hz
	setFrequency(1, 60); // 60Hz
	enable(0, true);
	enable(1, true);
	setAngle(0, 90);
	setAngle(1, 90);
}

PWM::~PWM()
{
	delete pwm[0];
	delete pwm[1];
}

void PWM::enable(int channel, bool f)
{
	pwm[channel]->enable(f);
}

void PWM::setFrequency(int channel, float f)
{
	int p= round(1000000.0/f);
	period_us= p;
	pwm[channel]->period_us(p);
}


void PWM::setAngle(int channel, float a)
{
	float value= dutyCycle(a);
	pwm[channel]->write(value);
}

// derived from adafruitss.cxx in the upm library
float PWM::dutyCycle(float angle)
{
	// figure out the duty cycle 0.0 to 1.0 (0% to 100%)
	// angle is from 0 to 180
	// servo_type: 0 = standard 1ms to 2ms
	//             1 = extended 0.6ms to 2.4ms
	//             2 = extended 0.8ms to 2.2ms
	const float duty_cycle_1ms = 1000.0F/period_us; // This is the duty cycle for a 1ms pulse
	float duty_cycle= duty_cycle_1ms;

	if(angle>180) angle=180;        // Ensure within bounds
	if (angle<0) angle=0;

	switch (type) {
	  case 0:              // Standard Servo 1ms to 2ms
		 duty_cycle= duty_cycle_1ms + ((duty_cycle_1ms*angle)/180);
		 break;

	  case 1:              // Extended Servo 0.6ms to 2.4ms, i.e. 1.8ms from 0 to 180
		 //duty_cycle= (duty_cycle_1ms*0.6) + ((duty_cycle_1ms*1.8*degrees)/180); simplified to..
		 duty_cycle= (duty_cycle_1ms*0.6) + ((duty_cycle_1ms*angle)/100);
		 break;

	  case 2:              // Extended Servo 0.8ms to 2.2ms, i.e. 1.4ms from 0 to 180
		 //duty_cycle= (duty_cycle_1ms*0.8) + ((duty_cycle_1ms*1.4*degrees)/180); simplified to..
		 duty_cycle= (duty_cycle_1ms*0.8) + ((duty_cycle_1ms*angle)/128);
		 break;
	  case 3:              // Extended Servo 0.9ms to 2.1ms,  - GWS Mini STD BB servo
		 //duty_cycle= (duty_cycle_1ms*0.8) + ((duty_cycle_1ms*1.4*degrees)/180); simplified to..
		 duty_cycle= (duty_cycle_1ms*0.9) + ((duty_cycle_1ms*angle)/120);
		 break;
   }

   return duty_cycle;
}
#endif
