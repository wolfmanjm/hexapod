#include "Servo.h"
#include "Leg.h"
#include "Timed.h"

#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <functional>
#include <vector>
#include <atomic>

#define RADIANS(a) (a * M_PI / 180.0F)
#define DEGREES(r) (r * 180.0F / M_PI)


/*
	coordinate system is the same for every leg, so a move in X will move all legs the same, this is done by applying the transform to the x y being moved

			  Y
			  |
			  |
			  |
	X---------+

	+Z is up

	leg positions

		  0---------------5
		 /                 \
		/                   \
	   /                     \
	  /                       \
	 /                         \
	1                           4
	 \                         /
	  \                       /
	   \                     /
		\                   /
		 \                 /
		  2---------------3

*/

using Pos3 = std::tuple<int, float, float, float>;
using Pos2 = std::tuple<int, float, float>;

// array of legs
static std::vector<Leg> legs;
static Servo servo;

static bool quit = false;
float optimal_stride = 30;
float max_stride = 70;
float min_stride = 5;
float max_speed = 200;

enum GAIT { NONE, WAVE, TRIPOD, WAVE_ROTATE, TRIPOD_ROTATE };
std::atomic<GAIT>  gait {NONE};
std::atomic<float> current_x {0};
std::atomic<float> current_y {0};
std::atomic<float> current_stride {optimal_stride};
std::atomic<float> current_rotate {0};

float body_height = TIBIA; // Body height.
static float update_frequency = 61.0; // 60Hz update frequency we need to be a little faster to make up for overhead
static Timed timed(update_frequency); // timer that repeats a given function at the given frequency (also provides micros())

// Interpolate a list of moves at update rate and issue to servos at the update rate]
void interpolatedMoves(std::vector<Pos3> pos, float speed, bool relative = true, bool force = false)
{
	float maxdist = 0;
	std::vector<Pos3> moves;

	for(auto &l : pos) {
		int leg;
		float x, y, z;
		std::tie(leg, x, y, z) = l;
		if(!relative) {
			x -= std::get<0>(legs[leg].getPosition());
			y -= std::get<1>(legs[leg].getPosition());
			z -= std::get<2>(legs[leg].getPosition());
			moves.push_back(Pos3(leg, x, y, z));
		}
		// we don't include legs that are off ground in the distance so they can move faster
		if(legs[leg].onGround() || force) {
			float dist = sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
			if(dist > maxdist) maxdist = dist;
		}
	}

	if(relative) moves = pos;

	float slice = speed / (maxdist * update_frequency); // the slice period based on speed and update frequency
	int iterations = roundf(1.0F / slice); // number of iterations

	uint32_t s = timed.micros();
	timed.run(iterations, [moves, slice]() {
		for(auto &p : moves) {
			float x, y, z;
			int leg;
			std::tie(leg, x, y, z) = p;
			legs[leg].moveBy(x * slice, y * slice, z * slice);
		}
	});
	uint32_t e = timed.micros();
	printf("update rate %lu us for %d iterations= %fHz\n", e - s, iterations, iterations * 1000000.0F / (e - s));
}

void home(int8_t l = -1)
{
	if(l >= 0) {
		legs[l].home();
		return;
	}

	for(auto &i : legs) {
		i.home();
	}
}

void raiseLeg(int leg, bool lift = true, int raise = 32, float speed = 60)
{
	legs[leg].setOnGround(!lift);
	interpolatedMoves({Pos3(leg, 0, 0, lift ? raise : -raise)}, speed, true, true);
}

void raiseLegs(std::vector<int> legn, bool lift = true, int raise = 32, float speed = 60)
{
	std::vector<Pos3> v;
	for(int leg : legn) {
		legs[leg].setOnGround(!lift);
		v.push_back(Pos3(leg, 0, 0, lift ? raise : -raise));
	}

	interpolatedMoves(v, speed, true, true);
}

// initialize legs to the specified positions
void initLegs(std::vector<Pos2> pos, bool relative = true)
{
	float raise = 25;
	float raise_speed = 100;

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

// rotate about robot center using a ripple gait
void rotateRippleGait(int reps, float angle, float speed)
{
	float raise = 30;
	float raise_speed = 200;
	float half_angle = angle / 2;
	float rotate_inc = angle / 5; // the amount it rotates per step

	// initialize legs to start positions
	if(reps == -1) {
		for (int l = 0; l < 6; ++l) {
			raiseLeg(l, true, 20, 100);
			float a = half_angle - (rotate_inc * l);
			//printf("angle: %f\n", a);
			legs[l].rotateBy(RADIANS(-a));
			raiseLeg(l, false, 20, 100);
		}
	}

	// figure out speed of rotation
	// calculate the approximate distance moved for the given angle on leg 0
	float rad = RADIANS(angle);
	float x = std::get<0>(legs[0].getPosition());
	float y = std::get<1>(legs[0].getPosition());
	float tx, ty, tz;
	std::tie(tx, ty, std::ignore) = legs[0].calcRotation(rad); // all legs seem to move the same amount
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
void rotateTripodGait(int reps, float angle, float speed)
{
	float raise = 25;
	float raise_speed = 200;
	float half_angle = angle / 2;
	uint8_t legorder[][3] { {0, 2, 4}, {1, 3, 5} };
	float tx, ty, tz;

	// initialize legs to start positions
	if(reps == -1) {
		std::vector<Pos2> v;
		for (int i = 0; i < 2; ++i) {
			for (int j = 0; j < 3; ++j) {
				uint8_t l = legorder[i][j];
				float tx, ty;
				l = legorder[i][j];
				float r = (i == 0) ? RADIANS(-half_angle) : RADIANS(half_angle);
				std::tie(tx, ty, std::ignore) = legs[l].calcRotation(r);
				v.push_back(Pos2(l, tx, ty));
			}
		}
		initLegs(v, false); // absolute positions
	}

	// figure out speed of rotation
	// calculate the distance moved for the given angle on leg 0
	float rad = RADIANS(angle);
	float x = std::get<0>(legs[0].getPosition());
	float y = std::get<1>(legs[0].getPosition());
	std::tie(tx, ty, std::ignore) = legs[0].calcRotation(rad); // all legs seem to move the same
	float dist = sqrtf(powf(x - tx, 2) + powf(y - ty, 2)); // the distance angle° moves the leg

	int iterations = roundf((dist * update_frequency) / speed); // number of iterations
	float da = angle / iterations; // delta angle to move each iteration
	//printf("Distance moved: %f, iterations: %d, da: %f\n", dist, iterations, da);

	for (int i = 0; i < reps; ++i) {

		// two phases
		for (int s = 0; s < 2; ++s) {
			raiseLegs({legorder[s][0], legorder[s][1], legorder[s][2]}, true, raise, raise_speed);
			int s1 = (s == 0) ? 1 : 0;
			// execute the lambda iterations times at the specified frequency for timed
			timed.run(iterations, [legorder, s, s1, da]() {
				for (int j = 0; j < 3; ++j) {
					legs[legorder[s][j]].rotateBy(RADIANS(da));
					legs[legorder[s1][j]].rotateBy(RADIANS(-da));
				}
			});
			raiseLegs({legorder[s][0], legorder[s][1], legorder[s][2]}, false, raise, raise_speed);
		}

	}
}

// tripod gait. setup so it moves +/-stride/2 around the home position
void tripodGait(int reps, float stridex, float stridey, float speed)
{
	static float last_stridex = 0, last_stridey = 0;
	float half_stridex = stridex / 2; // half stride in mm
	float half_stridey = stridey / 2; // half stride in mm
	float raise = 25;
	float raise_speed = 200;

	const uint8_t legorder[][3] { {0, 2, 4}, {1, 3, 5} };

	if(reps == -1) {
		// set legs to initial positions from home positions
		// should not matter where legs actually are
		std::vector<Pos2> v;
		for (int i = 0; i < 2; ++i) {
			for (int j = 0; j < 3; ++j) {
				uint8_t l = legorder[i][j];
				float x, y, z;
				std::tie(x, y, z) = legs[l].getHomeCoordinates();
				if(i == 0)
					v.push_back(Pos2(l, x - half_stridex, y - half_stridey));
				else
					v.push_back(Pos2(l, x + half_stridex, y + half_stridey));
			}
		}
		initLegs(v, false); // absolute positions
		reps = 1;
		last_stridex = stridex;
		last_stridey = stridey;
	}

	for (int i = 0; i < reps; ++i) {
		float dx, dy;
		// need to check if stride has changed since last full step at the start of the step phase
		if(last_stridex != stridex || last_stridey != stridey) {
			// if so adjust the legs by an appropriate offset on first phase of step
			dx = (stridex - last_stridex) / 2;
			dy = (stridey - last_stridey) / 2;

			last_stridex = stridex;
			last_stridey = stridey;

		} else {
			dx = 0;
			dy = 0;
		}

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
		speed, true);
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
		speed, true);
		raiseLegs({legorder[1][0], legorder[1][1], legorder[1][2]}, false, raise, raise_speed);
	}
}

// wave gait. setup so it moves +/-stride/2 around the home position
void waveGait(int reps, float stridex, float stridey, float speed)
{
	const uint8_t legorder[] {2, 1, 0, 3, 4, 5};
	static float last_stridex = 0, last_stridey = 0;
	float half_stridex = stridex / 2; // half stride in mm
	float half_stridey = stridey / 2; // half stride in mm
	float stridex_inc = stridex / 5; // the amount it moves per step
	float stridey_inc = stridey / 5; // the amount it moves per step
	float raise = 25;
	float raise_speed = 200;

	// set legs to initial positions from home positions at start of new gait
	// should not matter where legs actually are
	if(reps == -1) {
std: std::vector<Pos2> v;
		for (int i = 0; i < 6; ++i) { // for each leg
			uint8_t l = legorder[i];
			float x, y, z;
			std::tie(x, y, z) = legs[l].getHomeCoordinates();
			v.push_back(Pos2(l, x - (half_stridex - stridex_inc * i), y - (half_stridey - stridey_inc * i)));
		}

		initLegs(v, false); // absolute positions
		reps = 1;
		last_stridex = stridex;
		last_stridey = stridey;
	}

	for (int i = 0; i < reps; ++i) {

		for (int s = 0; s < 6; ++s) { // foreach step
			uint8_t leg = legorder[s];
			std::vector<Pos3> v;

			// reset the leg
			v.push_back(Pos3(leg, stridex, stridey, 0));

			// setup for moving other legs backwards
			for (int l = 0; l < 6; ++l) { // for each leg
				if(l != leg) v.push_back(Pos3(l, -stridex_inc, -stridey_inc, 0));
			}

			// need to check if stride has changed since last full step at the start of the step phase
			if(s == 0 && (last_stridex != stridex || last_stridey != stridey)) {
				// if so adjust the legs to appropriate positions
				// just adjust the currently queued movements for each leg
				float dx = stridex - last_stridex;
				float dy = stridey - last_stridey;

				// adjust all the legs
				for(auto &j : v) {
					// float nx= std::get<1>(j) - dx/2;
					// float ny= std::get<2>(j) - dx/2;
					// std::get<1>(j)= nx;
					// std::get<2>(j)= ny;
					std::get<1>(j) -= dx / 2;
					std::get<2>(j) -= dy / 2;
				}
				last_stridex = stridex;
				last_stridey = stridey;
			}

			// execute actual step
			raiseLeg(leg, true, raise, raise_speed);
			interpolatedMoves(v, speed, true);
			raiseLeg(leg, false, raise, raise_speed);
		}
	}
}

// being controoled via joystick over MQTT
void joystickControl()
{
	GAIT last_gait = gait;
	bool running = true;
	bool gait_changed = false;

	printf("Remote joystick control...\n");
	while(running) {
		gait_changed = (gait != last_gait);

		if(std::abs(current_x) > 0.0001F || std::abs(current_y) > 0.0001F)  {
			// calculate the vector of movement, then set the speed based on that
			float d = sqrtf(powf(current_x, 2) + powf(current_y, 2)); // vector size
			float x = current_stride * current_x / d; // normalize for proportion move in X
			float y = current_stride * current_y / d; // normalize for proportion move in Y
			float speed = max_speed * (d / 100.0F);
			printf("x: %f, y: %f, d: %f, speed: %f\n", x, y, d, speed);
			if(speed < 30) speed = 30;
			if(gait_changed) {
				// start a new gait
				switch(gait) {
					case WAVE:   waveGait(  -1, x, y, speed); break;
					case TRIPOD: tripodGait(-1, x, y, speed); break;
				}

			} else {
				// continue gait
				switch(gait) {
					case WAVE:   waveGait(  1, x, y, speed); break;
					case TRIPOD: tripodGait(1, x, y, speed); break;
				}
			}

		} else if(std::abs(current_rotate) > 0.001F) {
			float current_speed = max_speed * std::abs(current_rotate) / 100.0F ;
			float a = current_stride; // use stride for angle to turn as well
			if(a > 45) a = 45;
			else if(a < -45) a = -45;

			if(gait_changed) {
				// start a new gait
				switch(gait) {
					case WAVE_ROTATE: rotateRippleGait(-1, a, current_speed); break;
					case TRIPOD_ROTATE: rotateTripodGait(-1, a, current_speed); break;
				}

			} else {
				// continue gait
				switch(gait) {
					case WAVE_ROTATE: rotateRippleGait(1, a, current_speed); break;
					case TRIPOD_ROTATE:  rotateTripodGait(1, a, current_speed); break;
				}
			}
		}

		if(gait_changed) last_gait = gait;

		usleep(10); // just to give things a break
	}

	printf("Exited joystick control\n");
}

extern int mqtt_start(const char *, std::function<bool(const char *)>);

// handle a request from MQTT
bool handle_request(const char *req)
{
	int v;
	float x, y, z;
	size_t p1, p2, p3;

	std::string cmd(req);
	char c = cmd.front();
	cmd.erase(0, 1);

	switch(c) {
		case 'G':
			v = std::stoi(cmd, &p1);
			switch(v) {
				case 0: gait = NONE; break;
				case 1: gait = WAVE; break;
				case 2: gait = TRIPOD; break;
				default: printf("Unknown gait: %d\n", v);
			}
			break;

		case 'X':
			current_x = std::stof(cmd, &p1); // x is the proportional speed in X
			printf("x set to: %f\n", current_x.load());
			break;

		case 'Y':
			current_y = std::stof(cmd, &p1); // y is the proportional speed in Y
			printf("y set to: %f\n", current_y.load());
			break;

		case 'S': // stride
			x = std::stof(cmd, &p1) + 100; // we now have 0 - 200
			current_stride = max_stride * x / 200; // take percentage of max stride
			printf("stride set to: %f\n", current_stride.load());
			break;

		case 'R': // Rotate using current gait
			x = std::stof(cmd, &p1); // rotation -100 to 100
			if(std::abs(x) < 20) {
				current_rotate = 0;
				if(gait == WAVE_ROTATE) gait = WAVE;
				else if(gait == TRIPOD_ROTATE) gait = TRIPOD;
				printf("set rotate to: %f\n", current_rotate.load());
				break; // need some dead space to avoid accidental trigger
			}

			// stop all other movements
			current_y = current_x = 0;
			current_rotate.store(x);
			if(gait != NONE && gait != WAVE_ROTATE && gait != TRIPOD_ROTATE) {
				gait = (gait == WAVE) ? WAVE_ROTATE : TRIPOD_ROTATE;
			}
			printf("set rotate to: %f\n", current_rotate.load());
			break;


		default: printf("Unknown MQTT command: %c\n", c);
	}

#if 0
	if(c == 'A') {
		// set servos to given Angle
		leg = std::stoi(cmd, &p1);
		x = std::stof(cmd.substr(p1), &p2);
		y = std::stof(cmd.substr(p1 + p2), &p3);
		z = std::stof(cmd.substr(p1 + p2 + p3));
		printf("request %c: leg: %d, x: %f, y: %f, z: %f\n", c, leg, x, y, z);

		servo.updateServo((leg * 3) + 0, z); // ankle
		servo.updateServo((leg * 3) + 1, y); // knee
		servo.updateServo((leg * 3) + 2, x); // hip

	} else if(c == 'P') {
		// set Position to given XYZ
		leg = std::stoi(cmd, &p1);
		x = std::stof(cmd.substr(p1), &p2);
		y = std::stof(cmd.substr(p1 + p2), &p3);
		z = std::stof(cmd.substr(p1 + p2 + p3));
		printf("request %c: leg: %d, x: %f, y: %f, z: %f\n", c, leg, x, y, z);

		legs[leg].move(x, y, z);

	} else if(c == 'X') {
		quit = true;
		return false;
	}
#endif

	return true;
}

void rawMove(int leg, int joint, float x, float y, float z, bool absol = false)
{
	if(leg >= 0 && joint < 0) {
		if(absol) legs[leg].move(x, y, z);
		else legs[leg].moveBy(x, y, z);
	} else if(leg >= 0 && joint >= 0) {
		servo.move((leg * 3) + joint, (x * M_PI / 180.0) - M_PI_2);
	}
}

int main(int argc, char *argv[])
{
	int reps = 0;
	int c;
	int leg = -1, joint = -1;
	float x = 0, y = 0, z = 0;
	float speed = 10; // 10mm/sec default speed
	bool absol = false;
	bool do_walk = false;
	uint8_t gait = 0;

	// setup an array of legs, using this as they are not copyable.
	//  position angle, home angle, ankle, knee, hip
	legs.emplace_back(   60,  -60,  0,  1,  2, servo); // front left
	legs.emplace_back(    0,    0,  3,  4,  5, servo); // middle left
	legs.emplace_back(  -60,   60,  6,  7,  8, servo); // back left
	legs.emplace_back( -120,  120,  9, 10, 11, servo); // back right
	legs.emplace_back(  180,  180, 12, 13, 14, servo); // middle right
	legs.emplace_back(  120, -120, 15, 16, 17, servo); // front right

	while ((c = getopt (argc, argv, "hH:RDamc:l:j:f:x:y:z:s:S:TIL:W:JP:")) != -1) {
		switch (c) {
			case 'h':
				printf("Usage:\n -m home\n");
				printf(" -x n Set x to n\n");
				printf(" -y n Set y to n\n");
				printf(" -z n Set z to n\n");
				printf(" -l n Set leg to n\n");
				printf(" -s n Set speed to n mm/sec\n");
				printf(" -a Set absolute mode\n");
				printf(" -f n Set frequency to n Hz\n");
				printf(" -c n Set cycle count to n\n");
				printf(" -S n Set servo n to angle x\n");
				printf(" -H host set MQTT host\n");
				printf(" -D Daemon mode\n");
				printf(" -R raw move to x y z\n");
				printf(" -I interpolated move to xyz for leg at speed mm/sec\n");
				printf(" -L n raise leg or lower leg based on n\n");
				printf(" -W n walk with stride set by -y, speed set by -s, using gait n where 0: wave, 1: tripod, 2: rotate ripple, 3: rotate tripod\n");
				printf(" -J joystick control over MQTT\n");
				return 1;

			case 'H': mqtt_start(optarg, handle_request); break;

			case 'D': printf("Hit any key...\n"); getchar(); return 0;
			case 'J': joystickControl(); return 0;

			case 'S': servo.updateServo(atoi(optarg), x); break;
			case 'P': c = atoi(optarg); usleep(c * 1000); c = 0; break;

			case 'f': update_frequency = atof(optarg); break;

			case 'a': absol = true; break;
			case 's': speed = atof(optarg); break;
			case 'm':
				home(leg);
				break;
			case 'c':
				reps = atoi(optarg);
				break;
			case 'W':
				do_walk = true;
				gait = atoi(optarg);
				break;

			case 'l':
				leg = atoi(optarg);
				break;
			case 'j':
				joint = atoi(optarg);
				break;
			case 'x':
				x = atof(optarg);
				break;
			case 'y':
				y = atof(optarg);
				break;
			case 'z':
				z = atof(optarg);
				break;

			case 'R':
				rawMove(leg, joint, x, y, z, absol);
				break;

			case 'T':
				//           leg, x, y
				initLegs({Pos2(0, x, y), Pos2(5, x, y)}, !abs);
				//testConcurrentQueue();
				break;

			case 'I':
				//interpolatedMoves({Pos3(leg, x, y, z)}, speed, !abs);
				for (int i = 0; i <= reps; ++i) {
					interpolatedMoves({Pos3(0, x, y, z), Pos3(1, x, y, z), Pos3(2, x, y, z), Pos3(3, x, y, z), Pos3(4, x, y, z), Pos3(5, x, y, z)}, speed, !abs);
					interpolatedMoves({Pos3(0, -x, -y, -z), Pos3(1, -x, -y, -z), Pos3(2, -x, -y, -z), Pos3(3, -x, -y, -z), Pos3(4, -x, -y, -z), Pos3(5, -x, -y, -z)}, speed, !abs);
				}
				break;

			case'L':
				raiseLeg(leg, atoi(optarg) == 1);
				break;

			case '?':
				if (optopt == 'w')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
				return 1;
			default:
				abort ();
		}
	}

	if(do_walk) {
		switch(gait) {
			case 0: waveGait(-1, x, y, speed); waveGait(reps, x, y, speed); break;
			case 1: tripodGait(-1, x, y, speed); tripodGait(reps, x, y, speed); break;
			case 2: rotateRippleGait(-1, x, speed); rotateRippleGait(reps, x, speed); break;
			case 3: rotateTripodGait(-1, x, speed); rotateTripodGait(reps, x, speed); break;
			default: printf("Unknown Gait %d\n", gait);
		}
	}

	//printf("Hit any key...\n"); getchar();
	return 0;
}


#if 0
#include <tuple>
#include <utility>

template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type
for_each(std::tuple<Tp...> &, FuncT) // Unused arguments are given no names.
{ }

template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if < I < sizeof...(Tp), void>::type
for_each(std::tuple<Tp...> &t, FuncT f)
{
	f(std::get<I>(t));
	for_each < I + 1, FuncT, Tp... > (t, f);
}

#include "ConcurrentQueue.h"
#include <iostream>

void produce(ConcurrentQueue<int> &q)
{
	for (int i = 0; i < 10000; ++i) {
		std::cout << "Pushing " << i << "\n";
		q.push(i);
	}
}

void consume(ConcurrentQueue<int> &q, unsigned int id)
{
	for (int i = 0; i < 2500; ++i) {
		auto item = q.pop();
		std::cout << "Consumer " << id << " popped " << item << "\n";
	}
}

#include <thread>
int testConcurrentQueue()
{
	ConcurrentQueue<int> q;

	using namespace std::placeholders;

	// producer thread
	std::thread prod1(std::bind(produce, std::ref(q)));

	// consumer threads
	std::thread consumer1(std::bind(&consume, std::ref(q), 1));
	std::thread consumer2(std::bind(&consume, std::ref(q), 2));
	std::thread consumer3(std::bind(&consume, std::ref(q), 3));
	std::thread consumer4(std::bind(&consume, std::ref(q), 4));

	prod1.join();
	consumer1.join();
	consumer2.join();
	consumer3.join();
	consumer4.join();
}
#endif
