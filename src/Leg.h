#pragma once

#include <math.h>
#include <cstdint>
#include <tuple>
#include <string>

class Servo;

#define	COXA 36.5F
#define	FEMUR 32.5F
#define	TIBIA 65.0F
#define	BASE_RADIUS (134/2+26.5)

class Leg
{
public:
	using Vec3 = std::tuple<float, float, float>;

	Leg(const char *name, float pos, float home_angle, uint8_t joint0, uint8_t joint1, uint8_t joint2, Servo& servo);

	// must not allow copies of this Leg to be used accidently
	Leg( const Leg& other ) = delete; // non construction-copyable
    Leg& operator=( const Leg& ) = delete; // non copyable
    // required to use emplace_back for creation
	Leg(Leg&&) = default;

	void home();
	void idle();
	void move(float x, float y, float z, bool raw= false);
	void move(float x, float y);
	void moveBy(float dx, float dy, float dz);
	void rotateBy(float rad);
	Vec3 calcRotation(float rad, bool abs) const;
	Vec3 getHomeCoordinates() const;
	Vec3 getCoordinates(float x, float y, float z) const;

	void setAngle(float hip, float knee, float ankle);

	bool onGround() const { return on_ground; }
	void setOnGround(bool flg) { on_ground= flg; }

	Vec3 getPosition() const { return position; }

private:
	float solveTriangle(float a, float b, float c) const;
	float norm(float a, float b) const { return sqrtf(a * a + b * b); }
	void transform(const float mat[2][2], float& x, float& y) const;

	Vec3 inverseKinematics(float x, float y, float z) const;
	Vec3 forwardKinematics(float a, float k, float h) const;

	std::string name;
	Servo& servo;
	float mat[2][2];
	float inv_mat[2][2];
	Vec3 position;
	float origin[2];
	uint8_t joint[3];
	bool on_ground;
};
