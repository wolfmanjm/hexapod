#include "Servo.h"

#include <cmath>

const static float TAU = M_PI * 2;
const static float PI2 = M_PI_2;

Servo::Servo()
{
#ifndef DUMMY
	servos = new upm::adafruitss(6, 0x40);
    servos->setPWMFreq(60);
#else
	servos= new DummyServo();
#endif
}

Servo::~Servo()
{
	delete servos;
}

// defines which servos need to be reversed
static const char SERVO_REVERSE[] = {
    // ankle, knee, hip
     -1,  1, 1,    // front left
     -1,  1, 1,    // hind left
     -1,  1, 1,    // hind right
     -1,  1, 1     // front right
};

static const char SERVO_TRIM[] = {
    // ankle, knee, hip
    20, 25, 0,    // front left
    0, 0, 0,      // hind left
    20, 15, 0,    // hind right
    20, 15, 0     // front right
};

// Move a servo to a position in radians between -PI/2 and PI/2.
void Servo::move(uint8_t channel, float rads)
{
    rads = rads*SERVO_REVERSE[channel] + PI2;
    while (rads > TAU) {
        rads -= TAU;
    }
    while (rads < 0) {
        rads += TAU;
    }

    float angle= (rads*180.0F/M_PI) +
    #ifndef DUMMY
        SERVO_TRIM[channel];
    #else
        0;
    #endif
    // if(angle > 175) angle= 175;
    // if(angle < 2) angle= 2;

    //printf("rads %f, angle %f\n", rads, angle);
    uint16_t a= roundf(angle);
    servos->servo(channel, type, a);
}

void Servo::rawMove(uint8_t channel, float angle)
{
 	uint16_t a= roundf(angle);
	servos->servo(channel, type, a);
}
