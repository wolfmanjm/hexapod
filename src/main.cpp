#include "Servo.h"
#include "Leg.h"

#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <functional>
#include <vector>


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

	0---------------3
	|               |
	|               |
	|               |
	|               |
	|               |
	1---------------2

	at Home position the legs are out at a diagonal

*/

// array of legs
static std::vector<Leg> legs;
static Servo servo;

static bool quit= false;

/* Walking speed. */
float creep_dx = 0.0;          // Sideways.
float creep_dy = 1.0;          // Forward.
float creep_rotation = 0.0;    // Rotation.
float creep_height = TIBIA; // Body height.

/* Creep gait parameters. */
#define SPEED 1
/* How far forward to move them when making a step? */
#define STRIDE  12.0F
#define STRIDE_STEPS  4
/* How much to shift the body? */
#define SHIFT 20.0F
/* In how many steps should the body be shifted? */
#define SHIFT_STEPS 4
/* How fast to raise the leg. */
#define RAISE 8.0F
/* In how many steps. */
#define RAISE_STEPS 4

#define FPS 15
const int TICK = 1000 / FPS; // tick in ms

void walk();

void home(int8_t l= -1)
{
	if(l < 0) {
		for(auto &i : legs) {
			i.home();
		}
	}else{
		legs[l].home();
	}
}


// Gait stuff

/* Given current leg, which leg to move next? */
const unsigned char _NEXT_LEG[6][4] = {
	{2, 0, 3, 1},   // forward
	{1, 3, 0, 2},   // backward
	{1, 2, 3, 0},   // clockwise
	{3, 0, 1, 2},   // counter-clockwise
	{2, 3, 1, 0},   // left
	{3, 2, 0, 1},   // right
};

/* Which legs are touching the ground? */
static bool _on_ground[4] = {true, true, true, true};

/*
Move all legs that are on the ground backwards, pushing the robot forward with
the current walking speed. This is called every tick, to make the robot move
continuously.
*/
void _creep_move()
{
	for (unsigned char leg = 0; leg < 4; ++leg) {
		if (_on_ground[leg]) {
			legs[leg].moveBy(-creep_dx * SPEED,	-creep_dy * SPEED, 0);
			//legs[leg].rotateBy(-creep_rotation * legs[leg].getX() * legs[leg].getY() / 2);
		}
	}
}

/* Called every frame. */
void _creep_tick()
{
	//_creep_move();
	usleep(TICK * 1000);
}

/* Shifts the whole body by defined amount. */
void _shift_body_by(float dx, float dy)
{
	for (unsigned char leg = 0; leg < 4; ++leg) {
		legs[leg].moveBy(dx, dy, 0);
	}
}

/* Shifts the body away from the given leg, for balance. */
void _shift_body(unsigned char leg, bool back= false)
{
	float dx = SHIFT / SHIFT_STEPS;
	float dy = SHIFT / SHIFT_STEPS;
	if(back) {
		dx= -dx;
		dy= -dy;
	}
	// moves legs in a diagonal towards the leg (moves body in opposite direction)
	switch(leg) {
		case 1: dy= -dy; break;
		case 2: dx= -dx; dy= -dy; break;
		case 3: dx= -dx; break;
	}
	for (unsigned char i = 0; i < SHIFT_STEPS; ++i) {
		_shift_body_by(dx, dy);
		_creep_tick();
	}
}

#if 0
/* Perform a full step with a single leg. */
void _creep_step(unsigned char leg)
{
	// shift CoG
	_shift_body(leg);
	_on_ground[leg] = false;

	// raise leg
	for (unsigned char step = 0; step < RAISE_STEPS; ++step) {
		legs[leg].moveBy(0, 0, RAISE);
		_creep_tick();
	}

	// move raised leg forward
	for (int i = 0; i < STRIDE_STEPS; ++i) {
		legs[leg].moveBy( (creep_dx * STRIDE), (creep_dy * STRIDE), 0);
		_creep_tick();
	}

	// move body
	_creep_move();

	// lower leg
	for (unsigned char step = 0; step < RAISE_STEPS - 1; ++step) {
		legs[leg].moveBy(0, 0, -RAISE);
		_creep_tick();
	}

	_on_ground[leg] = true;
}

/* Called every frame. */
void _trot_tick()
{
	_creep_move();
	_creep_move();
	usleep(TICK * 1000);
}

/* Perform a full trot step. */
void _trot_step(unsigned char leg)
{
	unsigned char other_leg = (leg + 2) % 4;
	_on_ground[leg] = false;
	_on_ground[other_leg] = false;
	move_leg(
		leg,
		HOME + creep_dx * STRIDE * LEG_X[leg] + creep_spread,
		HOME + creep_dy * STRIDE * LEG_Y[leg] + creep_spread,
		RAISE * RAISE_STEPS - creep_height
	);
	move_leg(
		other_leg,
		HOME + creep_dx * STRIDE * LEG_X[other_leg] + creep_spread,
		HOME + creep_dy * STRIDE * LEG_Y[other_leg] + creep_spread,
		RAISE * RAISE_STEPS - creep_height
	);
	_trot_tick();
	_trot_tick();
	_trot_tick();
	_trot_tick();
	_on_ground[leg] = true;
	_on_ground[other_leg] = true;
	_trot_tick();
	_trot_tick();
	_trot_tick();
	_trot_tick();
}

unsigned char step_order()
{
	// Decide on the best leg order.
	if (std::abs(creep_dy) > std::abs(creep_dx)) {
		// forward/backward
		if (creep_dy > 0) {
			return 0; // forward
		} else {
			return 1; // backward
		}
	} else if (std::abs(creep_dx) > std::abs(creep_dy)) {
		// sideways
		if (creep_dx < 0) {
			return 4; // left
		} else {
			return 5; // right
		}
	} else if (std::abs(creep_rotation) > 0.001) {
		// rotate
		if (creep_rotation > 0) {
			return 2; // clockwise
		} else {
			return 3; // counter-clockwise
		}
	}
	return 0; // default to forward
}

/* Perform one step of a walk, selected depending on the speed. */
void walk()
{
	static unsigned char current_leg = 0;
	float max_speed = std::max(std::abs(creep_rotation), std::max(std::abs(creep_dx), std::abs(creep_dy)));

	// if (max_speed > 1.5) {
	//     _trot_step(leg);
	//     leg = (leg + 1) % 2;
	// } else

	if (max_speed > 0.01) {
		_creep_step(current_leg);
		current_leg = _NEXT_LEG[step_order()][current_leg];
	} else {
		_creep_tick();
	}
}
#endif

void move_legs(float stride)
{
	for (int i = 0; i < STRIDE_STEPS ; ++i) {
		for(auto& l : legs) {
			l.moveBy(0, -stride, 0);
		}
		_creep_tick();
	}
}

void raise_leg(int leg, bool back= false)
{
	for (unsigned char step = 0; step < RAISE_STEPS; ++step) {
		if(back)
			legs[leg].moveBy(0, 0, -RAISE);
		else
			legs[leg].moveBy(0, 0, RAISE);
		_creep_tick();
	}
}

void slow_walk(float stride)
{
	uint8_t leg_order[]= {0, 3, 2, 1};

	// move body forward
	move_legs(stride);

	usleep(10000);
	// reset legs
	for (int i = 0; i < 4; ++i) {
		int leg= leg_order[i];
		_shift_body(leg);
		usleep(10000);
		// raise leg
		raise_leg(leg);

		// reset leg
		legs[leg].moveBy(0, stride*STRIDE_STEPS, 0);
		usleep(50000);

		// lower leg
		raise_leg(leg, true);

		// undo shift
		_shift_body(leg, true);

		usleep(10000);
	}
}

void test_shift(int leg)
{
	if(leg >= 0) {
		_shift_body(leg);
		usleep(1000000);
		// raise leg
		raise_leg(leg);
	}else{
		for (int l = 0; l < 4; ++l) {
			_shift_body(l);
			usleep(1000000);
			// raise leg
			raise_leg(l);
			usleep(1000000);
			raise_leg(l, true);
			usleep(1000000);
			_shift_body(l, true);
			usleep(1000000);
		}
	}
}


void test_move_x(int leg, int nsteps, int reps)
{
	float x= 1;
	for(int i=0; i<reps; i++) {
		for (unsigned char step = 0; step < nsteps; ++step) {
			if(!legs[leg].moveBy(x, 0, 0)) {
				printf("out of range\n");
				return;
			}
			usleep(TICK * 1000);
		}
		for (unsigned char step = 0; step < nsteps*2; ++step) {
			if(!legs[leg].moveBy(-x, 0, 0)) {
				printf("out of range\n");
				return;
			}
			usleep(TICK * 1000);
		}
	}
}

void test_move_y(int leg, float stride)
{
	// for (int i = 0; i < STRIDE_STEPS; ++i) {
	// 	for(auto& l : legs) {
	// 		l.moveBy(0, -stride, 0);
	// 	}
	//  	//legs[leg].moveBy(0,-stride,0);
 // 	}
	// for (int i = 0; i < STRIDE_STEPS; ++i) {
	// 	for(auto& l : legs) {
	// 		l.moveBy(0, stride, 0);
	// 	}
 // 		//legs[leg].moveBy(0,stride,0);
 // 	}

	move_legs(stride);
	move_legs(-stride);
}

// handle a request from MQTT
bool handle_request(const char *req)
{
	int leg;
	float x, y, z;
	size_t p1, p2, p3;

	std::string cmd(req);
	char c= cmd.front();
	cmd.erase(0, 1);
	if(c == 'A') {
		// set servos to given Angle
		leg= std::stoi(cmd, &p1);
		x= std::stof(cmd.substr(p1), &p2);
		y= std::stof(cmd.substr(p1+p2), &p3);
		z= std::stof(cmd.substr(p1+p2+p3));
		printf("request %c: leg: %d, x: %f, y: %f, z: %f\n", c, leg, x, y, z);

		servo.rawMove((leg*3)+0, z); // ankle
		servo.rawMove((leg*3)+1, y); // knee
		servo.rawMove((leg*3)+2, x); // hip

	} else if(c == 'P') {
		// set Position to given XYZ
		leg= std::stoi(cmd, &p1);
		x= std::stof(cmd.substr(p1), &p2);
		y= std::stof(cmd.substr(p1+p2), &p3);
		z= std::stof(cmd.substr(p1+p2+p3));
		printf("request %c: leg: %d, x: %f, y: %f, z: %f\n", c, leg, x, y, z);

		legs[leg].move(x, y, z);

	}else if(c == 'X') {
		quit= true;
		return false;
	}

	return true;
}

extern int mqtt_start(const char *, std::function<bool(const char *)>);

int main(int argc, char *argv[])
{
	int reps = 0;
	int c;
	int leg= -1, joint= -1;
	float x=0, y=0, z=0;
	bool rawmove= false;
	bool abs= false;
	bool remote_move= false;
	bool do_walk= false;
	float stride= STRIDE;

	// setup an array of legs, using this as they are not copyable.
	//                 position angle, X, Y, ankle, knee, hip
	legs.emplace_back(   0,  48,  48, 0,  1,  2, servo); // front left
	legs.emplace_back( -90,  48,   0, 3,  4,  5, servo); // hind left
	legs.emplace_back( 180, -48,   0, 6,  7,  8, servo); // hind right
	legs.emplace_back(  90, -48,  48, 9, 10, 11, servo); // front right

	while ((c = getopt (argc, argv, "hSH:Damw:r:b:s:c:l:j:x:y:z:t:X:Y:R")) != -1) {
		switch (c) {
			case 'h':
				printf("Usage:\n -m home\n -w n walk n\n -r n rotate n degrees\n -b n set body height to n\n -s n set leg spread to n\n -c n number of reps\n");
				printf("--\n -l n leg n\n -j n joint n\n -x n -y n -z n\n -t n 0:test selected leg by raising it, 1: shift\n");
				return 1;

			case 'H': mqtt_start(optarg, handle_request); break;

			case 'D': while(!quit) { usleep(1000000); } return 0;

			case 'S': test_shift(leg); return 0;

			case 'a': abs= true; break;
			case 'm':
				home(leg);
				return 0;
			case 'c':
				reps = atoi(optarg);
				break;
			case 'w':
				stride= atof(optarg);
				do_walk= true;;
				break;
			case 'r':
				creep_dx = 0;
				creep_dy = 0;
				creep_rotation = atof(optarg) * M_PI / 180.0;
				//legs[leg].rotateBy(atof(optarg) * M_PI / 180.0);
				//return 0;
				break;
			case 'b':
				creep_height = atof(optarg);
				for(auto& l : legs) {
					l.moveBy(0, 0, l.getPosition()[2] - (-creep_height));
				}
				break;

			case 'l':
				leg = atoi(optarg);
				break;
			case 'j':
				joint = atoi(optarg);
				break;
			case 'x':
				x= atof(optarg);
				rawmove= true;
				break;
			case 'y':
				y= atof(optarg);
				rawmove= true;
				break;
			case 'z':
				z= atof(optarg);
				rawmove= true;
				break;
			case 'X':
				test_move_x(leg, atoi(optarg), reps);
				return 0;
			case 'Y':
				test_move_y(leg, atof(optarg));
				return 0;

			case 't':
				switch(atoi(optarg)) {
					case 0: // test one leg by raising it
						raise_leg(leg);
						raise_leg(leg, true);
						break;
					case 1: // test shift
						_shift_body(leg);
						_shift_body(leg, true);
						break;
				}

				return 0;

			case 'R':
				remote_move= true;
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

	if(remote_move) {
		servo.move((leg*3)+0, RADIANS(z)-M_PI_2); // ankle
		servo.move((leg*3)+1, RADIANS(y)-M_PI_2); // knee
		servo.move((leg*3)+2, RADIANS(x)-M_PI_2); // hip
		return 0;
	}

	if(reps > 0) home();

	if(rawmove) {
		if(leg >= 0 && joint < 0){
			if(abs) legs[leg].move(x, y, z);
			else legs[leg].moveBy(x, y, z);
		}else if(leg >= 0 && joint >= 0) {
			servo.move((leg*3)+joint, (x * M_PI / 180.0) - M_PI_2);
		}
		return 0;
	}

	// walk
	if(do_walk) {
		for (int i = 0; i < reps; ++i) {
			slow_walk(stride);
			//usleep(2000000);
		}
	}

	//if(reps > 0) home();

	return 0;
}
