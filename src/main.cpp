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
#include <csignal>

#define RADIANS(a) ((a) * M_PI / 180.0F)
#define DEGREES(r) ((r) * 180.0F / M_PI)

// defined in gaits.cpp
extern void rotateWaveGait(int reps, float angle, float speed, bool init);
extern void rotateTripodGait(int reps, float angle, float speed, bool init);
extern void tripodGait(int reps, float stridex, float stridey, float speed, bool init);
extern void waveGait(int reps, float stridex, float stridey, float speed, bool init);
extern void raiseLeg(int leg, bool lift = true, int raise = 16, float speed = 60);

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

float update_frequency = 61.5; // 60Hz update frequency we need to be a little faster to make up for overhead
Timed timed(update_frequency); // timer that repeats a given function at the given frequency (also provides micros())
float MAX_RAISE = 30; // 35 is safe too

// array of legs
std::vector<Leg> legs;

// used locally only

static Servo servo;
static float optimal_stride = 60;
static float optimal_angle = 20;
static float max_stride = 70;
static float max_angle = 38;
static float min_stride = 5;
static float max_speed = 200;
static float stride_time = 0.300; // time for each stride in seconds for distance mode instead of speed mode

enum GAIT { NONE, WAVE, TRIPOD, WAVE_ROTATE, TRIPOD_ROTATE };
static std::atomic<GAIT>  gait {NONE};
static std::atomic<float> current_x {0};
static std::atomic<float> current_y {0};
static std::atomic<float> current_stride {optimal_stride};
static std::atomic<float> current_angle {optimal_angle};
static std::atomic<float> current_rotate {0};
static std::atomic<float> body_height {TIBIA}; // Body height.

static volatile bool doSafeHome= false;
static volatile bool doIdlePosition= false;
static volatile bool doStandUp= false;
static volatile bool velocity_mode= false; // determines if XY control speed or stride

// Interpolate a list of moves within the given time in seconds and issue to servos at the update rate
void interpolatedMoves(std::vector<Pos3> pos, float time, bool relative = true)
{
	std::vector<Pos3> moves;

	if(!relative) {
		// get the delta moves for each leg, we need deltas so we can interpolate them over time
		for(auto &l : pos) {
			int leg;
			float x, y, z;
			float cx, cy, cz;
			std::tie(leg, x, y, z) = l;
			std::tie(cx, cy, cz) = legs[leg].getPosition();
			moves.push_back(Pos3(leg, x-cx, y-cy, z-cz));
		}

	} else {
		moves = pos;
	}

	// calculate iterations and the amount to slice each move
	uint32_t iterations= roundf(time * update_frequency);
	if(iterations == 0) return;

	float slice = 1.0F / iterations; // the fraction amount each move makes
	//printf("time: %f secs, iterations= %lu\n", time, iterations);
	//uint32_t s = timed.micros();
	// execute the lambda iterations times at the specified frequency for timed
	timed.run(iterations, [moves, slice]() {
		for(auto &p : moves) {
			float x, y, z;
			int leg;
			std::tie(leg, x, y, z) = p;
			legs[leg].moveBy(x * slice, y * slice, z * slice);
		}
	});
	//uint32_t e = timed.micros();
	//printf("update rate %lu us for %d iterations= %fHz\n", e - s, iterations, iterations * 1000000.0F / (e - s));
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

// home all legs but raise them first
void safeHome()
{
	uint8_t legorder[] { 0, 3, 1, 4, 2, 5 };
	float time= 0.1;
	for (int i = 0; i < 6; ++i) { // for each leg
		int l= legorder[i];
		float x, y, z;
		std::tie(x, y, z) = legs[l].getHomeCoordinates();
		interpolatedMoves({Pos3(l, x, y, z + MAX_RAISE)}, time, false);
		interpolatedMoves({Pos3(l, x, y, z)}, time, false);
	}
}

// set the legs to a known initial position after powerup or after servos are disabled
void initialPosition()
{
	// this is the position we want in leg coordinates
	// {:pos=>[112.69999999999752, 0.0, 30.500000000000746], :rot=>0, :pos_rot=>0, :hip=>90.0, :knee=>159.8416795945004, :ankle=>69.84977038335144}
	float x, y, z;
	for(auto& l : legs) {
		//l.setAngle(90, 160, 90);
		std::tie(x, y, z) = l.getCoordinates(112.7, 0.0, 30.5); // equivalent to setting angles to ankle 70, knee 160, hip 90
		l.move(x, y, z);
	}
}

// lower to ground and raise all legs and power down servos
// NOTE presumes we know where the legs are, cannot use after powerup
void idlePosition()
{
	if(!servo.isEnabled()) {
		// after powerup or disabled servos we don't know where the legs are so set them to a known position
		initialPosition();
		// allow time for that to happen
		usleep(500000);
	}

	// now move slowly into a lower position
	std::vector<Pos3> v;
	for (int l = 0; l < 6; ++l) { // for each leg
		float x, y, z;
		std::tie(x, y, z) = legs[l].getCoordinates(112.7, 0.0, 30.5); // gets idle coordinates with knee up 45°
		v.push_back(Pos3(l, x, y, z));
	}
	interpolatedMoves(v, 1.0, false);

	// allow time for that to happen
	usleep(500000);

	servo.enableServos(false);
}

// process to initially standup presuming servos are not powered initially
// servos do not have enough torque to stand up usually so need to do a special
// sequence to get the legs into the home position
void standUp()
{
	float x, y, z;
	// for each leg move knee joint up and angle at 90°
	// TODO enable each servo as we go.
	initialPosition();

	// allow time for that to happen
	usleep(500000);

	// now move slowly into a low position hopefully standing up
	std::vector<Pos3> v;
	for (int l = 0; l < 6; ++l) { // for each leg
		std::tie(x, y, z) = legs[l].getCoordinates(58, 0, -41); // gets idle coordinates with knee up 45°
		v.push_back(Pos3(l, x, y, z));
	}
	interpolatedMoves(v, 0.5, false);

	// now slowly move into home position
	v.clear();
	for (int l = 0; l < 6; ++l) { // for each leg
		std::tie(x, y, z) = legs[l].getHomeCoordinates();
		v.push_back(Pos3(l, x, y, z));
	}
	interpolatedMoves(v, 0.5, false);

	// now move check each leg to home position
	safeHome();
}


// raises or lowers the body height by a delta
void changeBodyHeight(float dz)
{
	std::vector<Pos3> v;
	for (int i = 0; i < 6; ++i) {
		v.push_back(Pos3(i, 0, 0, -dz)); // negative delta as z is actually foot position so heigher body is lower leg
	}
	interpolatedMoves(v, 0.1); // relative move
}

volatile bool doabort= false;
void signalHandler( int signum )
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    if(signum == SIGTERM){
    	doabort= true;
    	printf("Exiting\n");
    }
}

#pragma GCC diagnostic ignored "-Wswitch"
// being controlled via joystick or GUI over MQTT
void joystickControl()
{
	GAIT last_gait = NONE;
	bool running = true;
	bool first_time= true;
	bool gait_changed = false;
	float last_body_height = body_height;

	// register signal and signal handler
	signal(SIGTERM, signalHandler);
	signal(SIGHUP, signalHandler);

	printf("Remote joystick control...\n");

	while(running) {
		try {
			if(first_time) {
				first_time= false;
				//standUp();
				idlePosition();
			}

			if(doIdlePosition) {
				doIdlePosition= false;
				idlePosition();
				continue;
			}
			if(doSafeHome) {
				doSafeHome= false;
				safeHome();
				continue;
			}
			if(doStandUp) {
				doStandUp= false;
				standUp();
				continue;
			}

			gait_changed = (gait_changed || gait != last_gait);
			last_gait = gait;

			if(gait != NONE && gait <  WAVE_ROTATE && (std::abs(current_x) > 0.0001F || std::abs(current_y) > 0.0001F))  {
				// current_x and current_y are speed percentage in that direction
				// calculate the vector of movement in percentage
				float d = sqrtf(powf(current_x, 2) + powf(current_y, 2)); // vector size

				float x = current_stride * current_x / d; // normalize for proportion move in X, this is stride in mm in X
				float y = current_stride * current_y / d; // normalize for proportion move in Y, this is stride in mm in Y

				float speed = max_speed * (d / 100.0F); // adjust speed based on size of movement vector
				if(speed < 20) speed = 20;
				//printf("x: %f, y: %f, d: %f, speed: %f\n", x, y, d, speed);

				// execute one entire step which will move body over the ground by current_stride mm
				switch(gait) {
					case WAVE:
						waveGait(1, x, y, speed, gait_changed);
						break;
					case TRIPOD:
						tripodGait(1, x, y, speed, gait_changed);
						break;
				}
				gait_changed = false;

			} else if(gait >= WAVE_ROTATE && std::abs(current_rotate) > 0.001F) {
				float speed = max_speed * std::abs(current_rotate) / 100.0F ;
				if(speed < 10) speed = 10;
				float a = current_angle;
				if(current_rotate < 0) a = -a; // direction of rotate

				switch(gait) {
					case WAVE_ROTATE:
						rotateWaveGait(1, a, speed, gait_changed);
						break;
					case TRIPOD_ROTATE:
						rotateTripodGait(1, a, speed, gait_changed);
						break;
				}
				gait_changed = false;

			} else {
				if(body_height != last_body_height) {
					changeBodyHeight(body_height - last_body_height);
					last_body_height = body_height;
				}
			}

			usleep(10); // just to give things a break

		}catch(std::range_error& e) {
			// we don't want to die when this happens
			printf("Continuing after: range error - %s\n", e.what());
		}

		if(doabort) running= false;
	}

	printf("Exited joystick control\n");
}

extern int mqtt_start(const char *, std::function<bool(const char *)>);

// handle a request from MQTT
bool handle_request(const char *req)
{
	int v;
	float x;
	size_t p1, p2;

	std::string cmd(req);
	char c = cmd.front();
	cmd.erase(0, 1);

	switch(c) {
		case 'G':
			v = std::stoi(cmd, &p1);
			debug_printf("gait set to: %d\n", v);
			switch(v) {
				case 1: gait = NONE; break;
				case 2: gait = WAVE; break;
				case 3: gait = TRIPOD; break;
				case 4: gait = TRIPOD_ROTATE; break;
				case 5: gait = WAVE_ROTATE; break;
				case 6: gait = NONE; doSafeHome= true; break;
				case 7: gait= NONE; doIdlePosition= true; break;
				case 8: gait= NONE; doStandUp= true; break;
				default: printf("Unknown button: %d\n", v);
			}
			break;

		case 'X':
			current_x = std::stof(cmd, &p1); // x is the proportional speed in X 0 - 100
			debug_printf("x set to: %f\n", current_x.load());
			break;

		case 'Y':
			current_y = std::stof(cmd, &p1); // y is the proportional speed in Y 0 - 100
			debug_printf("y set to: %f\n", current_y.load());
			break;

		case 'S': // stride
			x = std::stof(cmd, &p1); // we now have 0 - 100
			current_stride = max_stride * x / 100; // take percentage of max stride
			current_angle = max_angle * x / 100; // take percentage of max angle
			debug_printf("stride set to: %f%% - %f, angle set to: %f\n", x, current_stride.load(), current_angle.load());
			break;

		case 'R': // Rotate using current gait if using a rotate gait
			x = std::stof(cmd, &p1); // rotation -100 to 100
			current_rotate = x;
			debug_printf("set rotate to: %f\n", current_rotate.load());
			break;

		case 'T': // right and left analog triggers +100 is right -100 is left
			x = std::stof(cmd, &p1); // rotation -100 to 100
			if(std::abs(x) > 0) {
				gait = TRIPOD_ROTATE;
				current_rotate = x;
			}else{
				gait = TRIPOD;
				current_rotate= 0;
				current_x= 0;
				current_y= 0;
			}
			debug_printf("set rotate to: %f\n", current_rotate.load());
			break;

		case 'U': // up or down
			v = std::stoi(cmd, &p1);
			if(v > 0) {
				body_height = body_height + 1;
			} else if(v < 0) {
				body_height = body_height - 1;
			}else {
				body_height = TIBIA;
			}
			debug_printf("Set body height to %f\n", body_height.load());
			break;

		case 'L': // left or right
			v = std::stoi(cmd, &p1);
			if(v > 0) {
				current_stride= current_stride + 1;
			} else if(v < 0) {
				current_stride= current_stride - 1;
			} else {
				current_stride= optimal_stride;
			}

			if(current_stride > max_stride) current_stride= max_stride;
			else if(current_stride < min_stride) current_stride= min_stride;

			debug_printf("Set stride to %f\n", current_stride.load());
			break;

		case 'H': // up or down -100% to 100%
			v = std::stoi(cmd, &p1);
			body_height = TIBIA * (100 + v) / 100.0;
			debug_printf("Set body height to %f\n", body_height.load());
			break;

		case 'A':
			// set servos to given Angle - parameters: servo angle
			v = std::stoi(cmd, &p1);
			x = std::stof(cmd.substr(p1), &p2);
			servo.updateServo(v, x);
			debug_printf("Set Servo %d to %f°\n", v, x);
			break;

		case 'B':
			// set time for each stride - parameters: time in seconds
			stride_time= std::stof(cmd.substr(p1), &p2);
			debug_printf("Set Stride time to %f secs\n", stride_time);
			break;

		case 'C':
			// set velocity mode or stride mode
			velocity_mode= std::stoi(cmd.substr(p1), &p2) == 0;
			debug_printf("Set velocity mode to %d\n", velocity_mode);
			break;

		case 'E':
			// enable or disable servos
			v = std::stoi(cmd, &p1);
			servo.enableServos(v == 1);
			debug_printf("Servos %s\n", v == 1 ? "Enabled" : "Disabled");
			break;

		default: printf("Unknown MQTT command: %c\n", c);
	}

#if 0
	if(c == 'P') {
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
	bool do_test = false;

	// setup an array of legs, using this as they are not copyable.
	//  position angle, home angle, ankle, knee, hip
	legs.emplace_back("front left",    60,  -60,  0,  1,  2, servo); // front left
	legs.emplace_back("middle left",    0,    0,  3,  4,  5, servo); // middle left
	legs.emplace_back("back left",    -60,   60,  6,  7,  8, servo); // back left
	legs.emplace_back("back right",  -120,  120,  9, 10, 11, servo); // back right
	legs.emplace_back("middle right", 180,  180, 12, 13, 14, servo); // middle right
	legs.emplace_back("front right",  120, -120, 15, 16, 17, servo); // front right

	try{
	while ((c = getopt (argc, argv, "hH:RDaAmMc:l:j:f:x:y:z:s:S:TIL:W:JP:vE:b:B:")) != -1) {
		switch (c) {
			case 'h':
				printf("Usage:\n");
				printf(" -m home\n");
				printf(" -M stand up\n");
				printf(" -x n Set x to n\n");
				printf(" -y n Set y to n\n");
				printf(" -z n Set z to n\n");
				printf(" -l n Set leg to n\n");
				printf(" -s n Set speed to n mm/sec\n");
				printf(" -a Set absolute mode\n");
				printf(" -A Safe Home\n");
				printf(" -b n Set leg left delta to n\n");
				printf(" -B n Set body height to n\n");
				printf(" -f n Set frequency to n Hz\n");
				printf(" -c n Set cycle count to n\n");
				printf(" -S n Set servo n to angle x\n");
				printf(" -H host set MQTT host\n");
				printf(" -D Daemon mode\n");
				printf(" -R raw move to x y z\n");
				printf(" -I interpolated move to xyz for leg at speed mm/sec\n");
				printf(" -L n raise leg or lower leg based on n\n");
				printf(" -W n walk with stride set by -y, speed set by -s, using gait n where 0: wave, 1: tripod, 2: rotate Wave, 3: rotate tripod\n");
				printf(" -J joystick control over MQTT\n");
				printf(" -P m pause m milliseconds\n");
				printf(" -E n enable or disable servos\n");
				printf(" -T run test\n");
				printf(" -v verbose debug\n");
				return 1;

			case 'H': mqtt_start(optarg, handle_request); break;
			case 'v': debug_verbose = true;  break;

			case 'D': printf("Hit any key...\n"); getchar(); return 0;

			case 'S': servo.updateServo(atoi(optarg), x); break;
			case 'P': usleep(atoi(optarg) * 1000); break;
			case 'E': servo.enableServos(atoi(optarg) == 1); break;

			case 'f': update_frequency = atof(optarg); break;

			case 'a': absol = true; break;
			case 'b': MAX_RAISE = atof(optarg); break;
			case 'B': { float dz = atof(optarg); changeBodyHeight(dz); body_height= body_height + dz; } break;
			case 's': speed = atof(optarg); break;
			case 'm': home(leg); break;
			case 'M': standUp(); break;
			case 'A': safeHome(); break;
			case 'c': reps = atoi(optarg);  break;

			case 'J':
				joystickControl();
				return 0;

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
				do_test = true;
				break;

			case 'I':
				//interpolatedMoves({Pos3(leg, x, y, z)}, speed, !abs);
				for (int i = 0; i <= reps; ++i) {
					interpolatedMoves({Pos3(0, x, y, z), Pos3(1, x, y, z), Pos3(2, x, y, z), Pos3(3, x, y, z), Pos3(4, x, y, z), Pos3(5, x, y, z)}, speed, !absol);
					interpolatedMoves({Pos3(0, -x, -y, -z), Pos3(1, -x, -y, -z), Pos3(2, -x, -y, -z), Pos3(3, -x, -y, -z), Pos3(4, -x, -y, -z), Pos3(5, -x, -y, -z)}, speed, !absol);
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
				fprintf(stderr, "unimplemented option: `-%c'.\n", c);
				abort ();
		}
	}

	if(do_test) {
		waveGait(1, x, y, speed, true);

		waveGait(1, x, y + 80, speed, false);
		waveGait(1, x, y, speed, false);

	} else if(do_walk) {
		switch(gait) {
			case 0: waveGait(reps, x, y, speed, true); break;
			case 1: tripodGait(reps, x, y, speed, true); break;
			case 2: rotateWaveGait(reps, x, speed, true); break;
			case 3: rotateTripodGait(reps, x, speed, true); break;
			default: printf("Unknown Gait %d\n", gait);
		}
	}

	}catch(...) {
		fprintf(stderr, "Caught unhandled exception... Exiting\n");
	}
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
