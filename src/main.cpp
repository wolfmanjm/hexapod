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

	0---------------5
	|               |
	|               |
	|               |
	|               |
	|               |
	1---------------4
	|               |
	|               |
	|               |
	|               |
	|               |
	2---------------3

*/

// array of legs
static std::vector<Leg> legs;
static Servo servo;

static bool quit= false;

float body_height = TIBIA; // Body height.

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

		servo.updateServo((leg*3)+0, z); // ankle
		servo.updateServo((leg*3)+1, y); // knee
		servo.updateServo((leg*3)+2, x); // hip

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
	legs.emplace_back(   0,  48,  48,  0,  1,  2, servo); // front left
	legs.emplace_back( -45,  48,  0,  3,  4,  5, servo);  // middle left
	legs.emplace_back( -90,  48, -48,  6,  7,  8, servo); // back left
	legs.emplace_back( 180, -48, -48,  9, 10, 11, servo); // back right
	legs.emplace_back( 135, -48,   0, 12, 13, 14, servo); // middle right
	legs.emplace_back(  90, -48,  48, 15, 16, 17, servo); // front right

	while ((c = getopt (argc, argv, "hH:Damc:l:j:x:y:z:S:")) != -1) {
		switch (c) {
			case 'h':
				printf("Usage:\n -m home\n");
				printf(" -x n Set x to n\n");
				printf(" -y n Set y to n\n");
				printf(" -z n Set z to n\n");
				printf(" -a Set absolute mode\n");
				printf(" -c n Set cycle count to n\n");
				printf(" -S n Set servo n to angle x\n");
				printf(" -H host set MQTT host\n");
				printf(" -D Daemon mode\n");
				return 1;

			case 'H': mqtt_start(optarg, handle_request); break;

			case 'D': printf("Hit any key...\n"); getchar(); return 0;

			case 'S': servo.updateServo(atoi(optarg), x); break;

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

	return 0;
}
