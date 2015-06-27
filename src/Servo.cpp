#include "Servo.h"

#include <cmath>

#ifndef DUMMY
#include "adafruitss.h"
#include "PWM.h"
#endif

const static float TAU = M_PI * 2;
const static float PI2 = M_PI_2;

Servo::Servo()
{
#ifndef DUMMY
	const int freqhz= 60;
    servos = new upm::adafruitss(6, 0x40);
    servos->setPWMFreq(freqhz); // actual 60Hz is 17.39 57.5Hz
    // ss 1.787 90°
    // PWM 1.51 90°

    // GPIO based PWM
    // 60Hz is actual 16.8ms
    pwm = new PWM(type);
    // match the frequency that the adafruitss actually generates
    // const float freq2hz= 60; // frequency to match above
    // pwm->setFrequency(0, freq2hz);
    // pwm->setFrequency(1, freq2hz);

#else
    servos = new DummyServo();
#endif
}

Servo::~Servo()
{
    delete servos;
#ifndef DUMMY
    delete pwm;
#endif
}

// defines which servos need to be reversed
static const char SERVO_REVERSE[] = {
    // ankle, knee, hip
    -1,  1, 1,    // front left
    -1,  1, 1,    // middle left
    -1,  1, 1,    // back left
    -1,  1, 1,    // back right
    -1,  1, 1,    // middle right
    -1,  1, 1     // front right
};

static const char SERVO_TRIM[] = {
    // ankle, knee, hip
    10, 20,  0,    // front left
     0,  0,  0,    // middle left
     0,  0,  0,    // back left
     0,  0,  0,    // back right
     0,  0,  0,    // middle right
    10, 20,  0     // front right
};

void Servo::updateServo(uint8_t channel, float angle)
{
#ifndef DUMMY
	if(channel >= 0 && channel <= 15) {
		uint16_t a = roundf(angle);
		servos->servo(channel, type, a);
	}else{
		pwm->setAngle(channel-16, angle);
	}
#else
	servos->servo(channel, type, roundf(angle));
#endif
}

// Move a servo to a position in radians between -PI/2 and PI/2.
void Servo::move(uint8_t channel, float rads)
{
    rads = rads * SERVO_REVERSE[channel] + PI2;
    while (rads > TAU) {
        rads -= TAU;
    }
    while (rads < 0) {
        rads += TAU;
    }

    float angle = (rads * 180.0F / M_PI) +
#ifndef DUMMY
                  SERVO_TRIM[channel];
#else
                  0;
#endif

    //printf("rads %f, angle %f\n", rads, angle);
    updateServo(channel, angle);
}
