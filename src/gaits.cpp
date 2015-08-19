#include "Servo.h"
#include "Leg.h"
#include "Timed.h"
#include "helpers.h"

#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <functional>
#include <vector>
#include <atomic>
#include <iostream>
#include <stdarg.h>
#include <csignal>

#define RADIANS(a) ((a) * M_PI / 180.0F)
#define DEGREES(r) ((r) * 180.0F / M_PI)

using Pos3 = std::tuple<int, float, float, float>;
using Pos2 = std::tuple<int, float, float>;

// defined in main.cpp
extern float MAX_RAISE;
extern std::vector<Leg> legs;
extern void interpolatedMoves(std::vector<Pos3> pos, float time, bool relative = true);
extern float update_frequency;
extern Timed timed;

void raiseLeg(int leg, bool lift = true, int raise = 16, float speed = 60)
{
	legs[leg].setOnGround(!lift);
	float time = raise / speed;
	interpolatedMoves({Pos3(leg, 0, 0, lift ? raise : -raise)}, time, true);
}

void raiseLegs(std::vector<int> legn, bool lift = true, int raise = 16, float speed = 60)
{
	std::vector<Pos3> v;
	for(int leg : legn) {
		legs[leg].setOnGround(!lift);
		v.push_back(Pos3(leg, 0, 0, lift ? raise : -raise));
	}

	float time = raise / speed;
	interpolatedMoves(v, time, true);
}

// initialize legs to the specified positions
void initLegs(std::vector<Pos2> pos, bool relative = true)
{
	float raise = MAX_RAISE;
	float raise_speed = 200;

	// lift leg, move to position, lower leg
	for(auto &i : pos) {
		int leg;
		float x, y;
		std::tie(leg, x, y) = i;
		//printf("move leg: %d, x: %f, y: %f relative %d\n", leg, x, y, relative);
		//printf("Raise leg %d\n", leg);
		raiseLeg(leg, true, raise, raise_speed);
		if(relative)
			legs[leg].moveBy(x, y, 0);
		else
			legs[leg].move(x, y);

		// lower leg
		//printf("Lower leg %d\n", leg);
		raiseLeg(leg, false, raise, raise_speed);
	}
}

// rotate about robot center using a wave gait
void rotateWaveGait(int reps, float angle, float speed, bool init)
{
	float raise = MAX_RAISE;
	float raise_speed = 200;
	float half_angle = angle / 2;
	float rotate_inc = angle / 5; // the amount it rotates per step
	static float last_angle = 0;

	// if the sign changes we need to re init legs
	if(sgn(last_angle) != sgn(angle)) init = true;

	// initialize legs to start positions
	if(init) {
		for (int l = 0; l < 6; ++l) {
			raiseLeg(l, true, raise, raise_speed);
			float x, y;
			std::tie(x, y, std::ignore) = legs[l].getHomeCoordinates();
			legs[l].move(x, y);
			float a = half_angle - (rotate_inc * l);
			legs[l].rotateBy(RADIANS(-a));
			raiseLeg(l, false, raise, raise_speed);
		}
		last_angle = angle;
	}

	// figure out speed of rotation
	// calculate the approximate distance moved for the given angle on leg 0
	float rad = RADIANS(rotate_inc);
	float x, y, tx, ty;
	std::tie(x, y, std::ignore) = legs[0].getHomeCoordinates();
	std::tie(tx, ty, std::ignore) = legs[0].calcRotation(rad, true); // all legs seem to move the same
	float dist = sqrtf(powf(x - tx, 2) + powf(y - ty, 2)); // the distance angle° moves the leg

	int iterations = roundf((dist * update_frequency) / speed); // number of iterations
	float da = rotate_inc / iterations; // delta angle to move each step
	float ra = angle / iterations; // delta angle to move each iteration for reset
	//printf("Distance moved: %f, iterations: %d, da: %f\n", dist, iterations, da);

	// basically ripple legs around
	for (int i = 0; i < reps; ++i) {
		for (int s = 0; s < 6; ++s) { // foreach step
			raiseLeg(s, true, raise, raise_speed);
			// execute the lambda iterations times at the specified frequency for timed
			timed.run(iterations, [s, da, ra]() {
				for (int l = 0; l < 6; ++l) {
					if(l != s)
						legs[l].rotateBy(RADIANS(-da));
					else
						legs[l].rotateBy(RADIANS(ra));
				}
			});

			raiseLeg(s, false, raise, raise_speed);
		}
	}
}
// rotate about robot center using a tripod gait
void rotateTripodGait(int reps, float angle, float speed, bool init)
{
	float raise = MAX_RAISE;
	float raise_speed = 200;
	float half_angle = angle / 2;
	uint8_t legorder[][3] { {0, 2, 4}, {1, 3, 5} };
	float tx, ty;
	static float last_angle = 0;

	// if the sign changes we need to re init legs
	if(sgn(last_angle) != sgn(angle)) init = true;

	// initialize legs to start positions
	if(init) {
		for (int i = 0; i < 2; ++i) {
			raiseLegs({legorder[i][0], legorder[i][1], legorder[i][2]}, true, raise, raise_speed);
			for (int j = 0; j < 3; ++j) {
				uint8_t l = legorder[i][j];
				float r = (i == 0) ? RADIANS(-half_angle) : RADIANS(half_angle);
				std::tie(tx, ty, std::ignore) = legs[l].calcRotation(r, true); // absolute position from home position
				legs[l].move(tx, ty);
			}
			raiseLegs({legorder[i][0], legorder[i][1], legorder[i][2]}, false, raise, raise_speed);
		}
		last_angle = angle;
	}

	// figure out speed of rotation
	// calculate the distance moved for the given angle on leg 0
	// Currently based on the distance a leg will move for the angle anfd using the speed in mm/sec
	// TODO should really be the speed at which the body turns in degrees/sec
	float rad = RADIANS(angle);
	float x, y;
	std::tie(x, y, std::ignore) = legs[0].getHomeCoordinates();
	std::tie(tx, ty, std::ignore) = legs[0].calcRotation(rad, true); // all legs seem to move the same
	float dist = sqrtf(powf(x - tx, 2) + powf(y - ty, 2)); // the distance angle° moves the leg

	int iterations = roundf((dist * update_frequency) / speed); // number of iterations
	float da = angle / iterations; // delta angle to move each iteration
	//printf("Distance moved: %f, iterations: %d, da: %f\n", dist, iterations, da);

	for (int i = 0; i < reps; ++i) {
		float dca = 0;
		// need to check if angle has changed since last full step at the start of the step phase
		if(last_angle != angle) {
			// if so adjust the legs by an appropriate offset on first phase of step
			dca = (angle - last_angle) / 2;
			last_angle = angle;
		}

		// two phases
		for (int s = 0; s < 2; ++s) {
			raiseLegs({legorder[s][0], legorder[s][1], legorder[s][2]}, true, raise, raise_speed);
			int s1 = (s == 0) ? 1 : 0;
			// execute the lambda iterations times at the specified frequency for timed
			timed.run(iterations, [legorder, s, s1, da, dca]() {
				for (int j = 0; j < 3; ++j) {
					legs[legorder[s][j]].rotateBy(RADIANS(da - dca));
					legs[legorder[s1][j]].rotateBy(RADIANS(-da + dca));
				}
			});
			raiseLegs({legorder[s][0], legorder[s][1], legorder[s][2]}, false, raise, raise_speed);
			dca = 0; // only will adjust on first phase
		}
	}
}

// tripod gait. setup so it moves the body stride distance after a full step cycle (two phases per step)
// this means that each leg moves ± stride/2 around its home position, a full step moves 2xstride
void tripodGait(int reps, float stridex, float stridey, float speed, bool init)
{
	static float last_stridex = 0, last_stridey = 0;
	float half_stridex = stridex / 2; // half stride in mm
	float half_stridey = stridey / 2; // half stride in mm
	float raise = MAX_RAISE;
	float raise_speed = 200;

	const uint8_t legorder[][3] { {0, 2, 4}, {1, 3, 5} };

	if(init) {
		// set legs to initial positions from home positions
		// should not matter where legs actually are
		for (int i = 0; i < 2; ++i) {
			raiseLegs({legorder[i][0], legorder[i][1], legorder[i][2]}, true, raise, raise_speed);
			for (int j = 0; j < 3; ++j) {
				uint8_t l = legorder[i][j];
				float x, y;
				std::tie(x, y, std::ignore) = legs[l].getHomeCoordinates();
				if(i == 0)
					legs[l].move(x - half_stridex, y - half_stridey);
				else
					legs[l].move(x + half_stridex, y + half_stridey);
			}
			raiseLegs({legorder[i][0], legorder[i][1], legorder[i][2]}, false, raise, raise_speed);
		}
		last_stridex = stridex;
		last_stridey = stridey;
	}

	// calculate time this move should take, based on the amount the body will move over the ground
	float dist = sqrtf(powf(stridex, 2) + powf(stridey, 2)); // distance over the ground
	float time = dist / speed; // the time it will take to move that distance at the given speed (mm/sec)

	float dx = 0, dy = 0;
	// need to check if stride has changed since last full step
	if(last_stridex != stridex || last_stridey != stridey) {
		// if so adjust the legs by an appropriate offset on first phase of step
		dx = (stridex - last_stridex) / 2;
		dy = (stridey - last_stridey) / 2;

		last_stridex = stridex;
		last_stridey = stridey;

	}

	for (int i = 0; i < reps; ++i) {
		// this is two strides we need to move each stride in the calculated time
		// execute step state 1
		raiseLegs({legorder[0][0], legorder[0][1], legorder[0][2]}, true, raise, raise_speed);
		interpolatedMoves({
			Pos3(legorder[0][0],  stridex - dx,  stridey - dy, 0),
			Pos3(legorder[0][1],  stridex - dx,  stridey - dy, 0),
			Pos3(legorder[0][2],  stridex - dx,  stridey - dy, 0),
			Pos3(legorder[1][0], -stridex + dx, -stridey + dy, 0),
			Pos3(legorder[1][1], -stridex + dx, -stridey + dy, 0),
			Pos3(legorder[1][2], -stridex + dx, -stridey + dy, 0)
		},
		time, true);
		raiseLegs({legorder[0][0], legorder[0][1], legorder[0][2]}, false, raise, raise_speed);

		// execute step state 2
		raiseLegs({legorder[1][0], legorder[1][1], legorder[1][2]}, true, raise, raise_speed);
		interpolatedMoves({
			Pos3(legorder[1][0],  stridex,  stridey, 0),
			Pos3(legorder[1][1],  stridex,  stridey, 0),
			Pos3(legorder[1][2],  stridex,  stridey, 0),
			Pos3(legorder[0][0], -stridex, -stridey, 0),
			Pos3(legorder[0][1], -stridex, -stridey, 0),
			Pos3(legorder[0][2], -stridex, -stridey, 0)
		},
		time, true);
		raiseLegs({legorder[1][0], legorder[1][1], legorder[1][2]}, false, raise, raise_speed);
	}
}

// wave gait. setup so it moves ±stride/2 around the home position
void waveGait(int reps, float stridex, float stridey, float speed, bool init)
{
	const uint8_t legorder[] {2, 1, 0, 3, 4, 5};
	static float last_stridex = 0, last_stridey = 0;
	float half_stridex = stridex / 2; // half stride in mm
	float half_stridey = stridey / 2; // half stride in mm
	float stridex_inc = stridex / 5; // the amount it moves per step
	float stridey_inc = stridey / 5; // the amount it moves per step
	float raise = MAX_RAISE;
	float raise_speed = 200;

	// set legs to initial positions from home positions at start of new gait
	// should not matter where legs actually are
	if(init) {
		std::vector<Pos2> v;
		for (int i = 0; i < 6; ++i) { // for each leg
			uint8_t l = legorder[i];
			float x, y, z;
			std::tie(x, y, z) = legs[l].getHomeCoordinates();
			v.push_back(Pos2(l, x - (half_stridex - stridex_inc * i), y - (half_stridey - stridey_inc * i)));
			//std::cout << "init: "; print(v.back());
		}

		initLegs(v, false); // absolute positions
		last_stridex = stridex;
		last_stridey = stridey;
	}

	// calculate time this move should take, based on the amount the body will move over the ground
	float dist = sqrtf(powf(stridex, 2) + powf(stridey, 2)); // distance over the ground
	float time = dist / speed; // the time it will take to move that distance at the given speed (mm/sec)

	// need to check if stride has changed since last full step at the start of the step phase
	float dx = 0, dy = 0, dxi = 0, dyi = 0;
	if(last_stridex != stridex || last_stridey != stridey) {
		// if so adjust the legs to appropriate positions
		dx = (stridex - last_stridex) / 2;
		dy = (stridey - last_stridey) / 2;
		dxi = dx / 2.5; // stride change/5
		dyi = dy / 2.5;
		last_stridex = stridex;
		last_stridey = stridey;
	}

	for (int i = 0; i < reps; ++i) {

		for (int s = 0; s < 6; ++s) { // foreach step
			uint8_t leg = legorder[s];
			std::vector<Pos3> v;

			// reset the current leg
			v.push_back(Pos3(leg, stridex - dx, stridey - dy, 0));
			// printf("reset l= %d, dx= %f, dy= %f\n", leg, dx/2, dy/2); std::cout << "reset: "; print(v.back());

			// move the other legs backwards
			for (int j = 0; j < 6; ++j) { // for each leg
				uint8_t l = legorder[j];
				if(l != leg) {
					v.push_back(Pos3(l, -stridex_inc - (dx - (dxi * j)), -stridey_inc - (dy - (dyi * j)), 0));
					// printf("move l= %d, dx= %f, dy= %f\n", l, (dx/2 - (dx/5*j)), (dy/2 - (dy/5*j))); std::cout << "move: "; print(v.back());
				}
			}

			// execute actual step
			raiseLeg(leg, true, raise, raise_speed);
			interpolatedMoves(v, time / 6, true); // each step should take 1/6 of the time calculated for the total move
			raiseLeg(leg, false, raise, raise_speed);
			// only adjust first phase of step
			dx = dy = dxi = dyi = 0;
		}
	}
}
