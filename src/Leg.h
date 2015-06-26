#pragma once

#include <math.h>
#include <cstdint>
#include <tuple>

class Servo;

#define	COXA 36.5F
#define	FEMUR 32.5F
#define	TIBIA 65.0F
#define	BASE 57.0F
#define	HOME ((COXA + FEMUR) / M_SQRT2)

class Leg
{
public:
	Leg(float pos, int8_t x, int8_t y, uint8_t joint0, uint8_t joint1, uint8_t joint2, Servo& servo);

	// must not allow copies of this Leg to be used accidently
	Leg( const Leg& other ) = delete; // non construction-copyable
    Leg& operator=( const Leg& ) = delete; // non copyable
    // required to use emplace_back for creation
	Leg(Leg&&) = default;

	bool home();
	bool move(float x, float y, float z);
	bool moveBy(float dx, float dy, float dz);
	bool rotateBy(float rad);
	const float *getPosition() const { return position; }

private:
	float solveTriangle(float a, float b, float c);
	float norm(float a, float b){ return sqrtf(a * a + b * b); }
	void transform(float& x, float& y);

	std::tuple<float, float, float> inverseKinematics(float x, float y, float z);

	Servo& servo;
	float mat[2][2];
	int8_t xp, yp;
	float position[3];
	uint8_t joint[3];
};
