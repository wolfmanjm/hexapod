#pragma once

#include <math.h>
#include <cstdint>
#include <tuple>

class Servo;

#define	COXA 36.5F
#define	FEMUR 32.5F
#define	TIBIA 65.0F
#define	BASE_RADIUS (134/2+26.5)
#define	HOME ((COXA + FEMUR) / M_SQRT2)

class Leg
{
public:
	Leg(float pos, float home_angle, uint8_t joint0, uint8_t joint1, uint8_t joint2, Servo& servo);

	// must not allow copies of this Leg to be used accidently
	Leg( const Leg& other ) = delete; // non construction-copyable
    Leg& operator=( const Leg& ) = delete; // non copyable
    // required to use emplace_back for creation
	Leg(Leg&&) = default;

	bool home();
	bool move(float x, float y, float z);
	bool move(float x, float y);
	bool moveBy(float dx, float dy, float dz);
	bool rotateBy(float rad);
	bool onGround() const { return on_ground; }
	bool setOnGround(bool flg) { on_ground= flg; }

	using Vec3 = std::tuple<float, float, float>;
	Vec3 getPosition() const { return position; }

private:
	float solveTriangle(float a, float b, float c);
	float norm(float a, float b){ return sqrtf(a * a + b * b); }
	void transform(float mat[2][2], float& x, float& y);

	Vec3 inverseKinematics(float x, float y, float z);

	Servo& servo;
	float mat[2][2];
	float home_mat[2][2];
	Vec3 position;
	uint8_t joint[3];
	bool on_ground;
};
