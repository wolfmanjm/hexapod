#include "Servo.h"
#include "Leg.h"

#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <functional>
#include <vector>
#include <time.h>

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

using Pos3= std::tuple<int,float,float,float>;
using Pos2= std::tuple<int,float,float>;

// array of legs
static std::vector<Leg> legs;
static Servo servo;

static bool quit= false;

float body_height = TIBIA; // Body height.
static float update_frequency= 50; // 50Hz update frequency

static inline uint64_t rdtsc(void)
{
    uint32_t lo, hi;
    uint64_t returnVal;
    /* We cannot use "=A", since this would use %rax on x86_64 */
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    returnVal = hi;
    returnVal <<= 32;
    returnVal |= lo;

    return returnVal;
}

#if 0
static uint64_t tsc_init = 0;
static float cpufreq = 0;
static float clocks_per_ns = 0;

#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/sysinfo.h>
#include <memory.h>

/* TSC snapshot */
int timeInit(void)
{
    int cpufreq_fd, ret;
    char buf[0x400];
    char * str = 0, * str2 = 0;
    const char * mhz_str = "cpu MHz\t\t: ";

    /* Grab initial TSC snapshot */
    tsc_init = rdtsc();

    cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0){
        fprintf(stderr, "unable to open /proc/cpuinfo\n");
        return -1;
    }
    memset(buf, 0x00, sizeof(buf));
    ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 ){
        fprintf(stderr, "unable to read cpuinfo !\n");
        close(cpufreq_fd);
        return -1;
    }
    close(cpufreq_fd);
    str = strstr(buf, mhz_str);
    if (!str){
        fprintf(stderr, "Buffer %s does not contain CPU frequency info !\n", buf);
        return -1;
    }

    str += strlen(mhz_str);
    str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n'){
        str2++;
    }
    if(str2 == buf + sizeof(buf-1) && *str2 !='\n'){
        fprintf(stderr, "malformed cpufreq string %s\n", str);
        return -1;
    }
    *str2 = '\0';
    cpufreq = atof(str);


    printf("cpufrequency is %f mhz\n", cpufreq);

    /* Calculate nanoseconds per clock */
    clocks_per_ns = 1000/cpufreq;

    printf("nanoseconds per clock %f\n", clocks_per_ns);
}

unsigned long micros( void )
{
    uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;

    divisor = (cpufreq );
    diff = tsc_cur - tsc_init;

    return (unsigned long) (diff / divisor);
}
#endif

timespec time_diff(timespec& start, timespec& end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

// Interpolate a move at update rate and issue to servos at the update rate]
void interpolatedMove(Pos3 pos, float speed, bool relative= true)
{
	// body_pos_v3 = (end_v3 - start_v3) * (current_time - start_time) / move_time + start_v3
	int leg;
	float x, y, z;
	std::tie(leg, x, y, z) = pos;
	if(!relative) {
		x -= std::get<0>(legs[leg].getPosition());
		y -= std::get<1>(legs[leg].getPosition());
		z -= std::get<2>(legs[leg].getPosition());
	}
	float dist= sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
	float slice= speed/(dist*update_frequency); // the slice period based on speed and update frequency
	float dx= x*slice;
	float dy= y*slice;
	float dz= z*slice;

	int iterations= roundf(update_frequency * dist/speed); // iterations at update frequency to complete the move
	for (int i = 0; i < iterations; ++i) {
		legs[leg].moveBy(dx, dy, dz);
		usleep(1000000/update_frequency);
	}
}

// Interpolate a list of moves at update rate and issue to servos at the update rate]
void interpolatedMoves(std::vector<Pos3> pos, float speed, bool relative= true)
{
	int leg;
	float x, y, z;
	float maxdist= 0;
	std::vector<Pos3> moves;

	for(auto &l : pos) {
		std::tie(leg, x, y, z) = l;
		if(!relative) {
			x -= std::get<0>(legs[leg].getPosition());
			y -= std::get<1>(legs[leg].getPosition());
			z -= std::get<2>(legs[leg].getPosition());
			moves.push_back(Pos3(leg, x, y, z));
		}
		float dist= sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
		if(dist > maxdist) maxdist= dist;
	}

	if(relative) moves= pos;

	float slice= speed/(maxdist*update_frequency); // the slice period based on speed and update frequency
	int iterations= roundf(update_frequency * maxdist/speed); // iterations at update frequency to complete the move

	// for each specified leg
	for (int i = 0; i < iterations; ++i) {
		timespec time1, time2;
		int temp;
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
		//unsigned long s= micros();
		for(auto& p : moves) {
			std::tie(leg, x, y, z) = p;
			legs[leg].moveBy(x*slice, y*slice, z*slice);
		}
		//unsigned long e= micros();
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
		timespec diff= time_diff(time1, time2);
		printf("%d:%d\n", diff.tv_sec, diff.tv_nsec);
		//printf("%d:%d - %lu\n", diff.tv_sec, diff.tv_nsec, e-s);
		usleep(1000000/update_frequency);
	}
}


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

void raiseLeg(int leg, bool lift= true, int raise=32, float speed= 60)
{
	interpolatedMove(Pos3(leg, 0, 0, lift ? raise : -raise), speed);
}

// initialize legs to the specified positions
void initLegs(std::vector<Pos2> pos, bool relative= true)
{
	// lift leg, move to position, lower leg
	for(auto& i : pos) {
		int leg;
		float x, y;
		std::tie(leg, x, y) = i;
		//printf("move leg: %d, x: %f, y: %f relative %d\n", leg, x, y, relative);
		raiseLeg(leg);
		if(relative)
			legs[leg].moveBy(x, y, 0);
		else
			legs[leg].move(x, y);

		// lower leg
		raiseLeg(leg, false);
	}

}

extern int mqtt_start(const char *, std::function<bool(const char *)>);

int main(int argc, char *argv[])
{
	int reps = 0;
	int c;
	int leg= -1, joint= -1;
	float x=0, y=0, z=0;
	float speed= 10; // 10mm/sec default speed
	bool rawmove= false;
	bool abs= false;
	bool do_walk= false;

	//timeInit();

	// setup an array of legs, using this as they are not copyable.
	//  position angle, home angle, ankle, knee, hip
	legs.emplace_back(   60,  -60,  0,  1,  2, servo); // front left
	legs.emplace_back(    0,    0,  3,  4,  5, servo); // middle left
	legs.emplace_back(  -60,   60,  6,  7,  8, servo); // back left
	legs.emplace_back( -120,  120,  9, 10, 11, servo); // back right
	legs.emplace_back(  180,  180, 12, 13, 14, servo); // middle right
	legs.emplace_back(  120, -120, 15, 16, 17, servo); // front right

	while ((c = getopt (argc, argv, "hH:RDamc:l:j:f:x:y:z:s:S:TIL:")) != -1) {
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
				return 1;

			case 'H': mqtt_start(optarg, handle_request); break;

			case 'D': printf("Hit any key...\n"); getchar(); return 0;

			case 'S': servo.updateServo(atoi(optarg), x); break;

			case 'f': update_frequency= atof(optarg); break;

			case 'a': abs= true; break;
			case 's': speed= atof(optarg); break;
			case 'm':
				home(leg);
				break;
			case 'c':
				reps = atoi(optarg);
				break;
			case 'w':
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
				break;
			case 'y':
				y= atof(optarg);
				break;
			case 'z':
				z= atof(optarg);
				break;

			case 'R': rawmove= true; break;

			case 'T':
				//           leg, x, y
				initLegs({Pos2(0, x, y), Pos2(5, x, y)}, !abs);
				break;

			case 'I':
				//interpolatedMove(Pos3(leg, x, y, z), speed, !abs);
				interpolatedMoves({Pos3(leg, x, y, z)}, speed, !abs);
				break;

			case'L':
				raiseLeg(leg, atoi(optarg)==1);
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

	if(rawmove) {
		if(leg >= 0 && joint < 0){
			if(abs) legs[leg].move(x, y, z);
			else legs[leg].moveBy(x, y, z);
		}else if(leg >= 0 && joint >= 0) {
			servo.move((leg*3)+joint, (x * M_PI / 180.0) - M_PI_2);
		}
	}

	printf("Hit any key...\n"); getchar();
	return 0;
}
