#include "Servo.h"

#include <cmath>
#include <stdexcept>

#ifndef DUMMY
#include "mraa.hpp"
#include "adafruitss.h"
#include "PWM.h"
#endif

#ifdef RPI
#define I2C_CH 1
#define ENABLE_PIN 11 // BCM 17
#else
#define I2C_CH 6
#define ENABLE_PIN 20 // GP12 GPIO-12 J18-7
#endif

const static float TAU = M_PI * 2;
const static float PI2 = M_PI_2;

Servo::Servo()
{
	servos= nullptr;
}

bool Servo::init()
{
#ifndef DUMMY
	const int freqhz= 60;
	servos = new adafruitss(I2C_CH, 0x40);
	servos->setPWMFreq(freqhz); // actual 60Hz is 17.39 57.5Hz
	// ss 1.787 90°
	// PWM 1.51 90°

	#ifdef USEGPIO
	// GPIO based PWM
	// 60Hz is actual 16.8ms
	pwm = new PWM(type);
	// match the frequency that the adafruitss actually generates
	// const float freq2hz= 60; // frequency to match above
	// pwm->setFrequency(0, freq2hz);
	// pwm->setFrequency(1, freq2hz);
	#else
	servos2 = new adafruitss(I2C_CH, 0x41);
	servos2->setPWMFreq(freqhz); // actual 60Hz is 17.39 57.5Hz
	#endif

	// start with PWM disabled
	enable_pin= new mraa::Gpio(ENABLE_PIN);
	enable_pin->dir(mraa::DIR_OUT);
	enableServos(false);

#else
	servos = new DummyServo();
#endif
	for (int i = 0; i < NSERVOS; ++i) {
		current_angle[i]= 9999.9; // set to an angle we would never have set
	}

	return true;
}

Servo::~Servo()
{
	if(servos != nullptr) {
		enableServos(false);
		delete servos;

#ifndef DUMMY
		delete enable_pin;

#ifdef USEGPIO
		delete pwm;
#else
		delete servos2;
#endif
#endif
	}
}

// defines which servos need to be reversed
static const int8_t SERVO_REVERSE[Servo::NSERVOS] = {
	// ankle, knee, hip
	-1,  1, 1,    // front left
	-1,  1, 1,    // middle left
	-1,  1, 1,    // back left
	-1,  1, 1,    // back right
	-1,  1, 1,    // middle right
	-1,  1, 1     // front right
};

#ifndef DUMMY
static const int8_t SERVO_TRIM[Servo::NSERVOS] = {
	// ankle, knee, hip
	 10, 20,   7,    // front left
	 20, 20,   0,    // middle left
	  0,  0,   0,    // back left
	 20, 20,  10,    // back right
	 20, 30, -10,    // middle right
	 22, 30,  15     // front right
};
#else
static const int8_t SERVO_TRIM[Servo::NSERVOS]{0,0,0,0,0,0,0,0,0,0,0,0};
#endif

void Servo::updateServo(uint8_t channel, float angle)
{
	if(channel >= NSERVOS) throw std::invalid_argument("channel");
	if(servos == nullptr) throw std::invalid_argument("not inited");

	if(!enabled) enableServos(true);

	if(debug_verbose) {
		// Note seems to go to 220° with type 1
		if(angle < 0 || angle > 180) printf("WARNING: angle is too big for channel %d, %f\n", channel ,angle);
	}

	// check if any change to avoid unecessary I2C traffic
	if(angle == current_angle[channel]) return;

	current_angle[channel]= angle;

#ifndef DUMMY

#ifdef USEGPIO
	if(channel >= 0 && channel <= 15) {
		servos->servo(channel, type, angle);
	}else{
		pwm->setAngle(channel-16, angle);
	}
#else
	if(channel <= 8) {
		servos->servo(channel, type, angle);
	}else{
		servos2->servo(channel-9, type, angle);
	}
#endif

#else
	servos->servo(channel, type, angle);
#endif
}

// Move a servo to a position in radians between -PI/2 and PI/2.
void Servo::move(uint8_t channel, float rads)
{
	if(channel >= NSERVOS) throw std::invalid_argument("channel");
	if(servos == nullptr) throw std::invalid_argument("not inited");

	rads = rads * SERVO_REVERSE[channel] + PI2;
	while (rads > TAU) {
		rads -= TAU;
	}
	while (rads < 0) {
		rads += TAU;
	}

	float angle = (rads * 180.0F / M_PI) + SERVO_TRIM[channel];

	//printf("rads %f, angle %f\n", rads, angle);
	updateServo(channel, angle);
}

void Servo::enableServos(bool on)
{
	if(servos == nullptr) throw std::invalid_argument("not inited");

#ifndef DUMMY
	enable_pin->write(on?0:1);
#endif
	enabled= on;
}
