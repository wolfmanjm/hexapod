#include <stdint.h>

#include <mosquittopp.h>
#include <linux/input.h>

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <stdarg.h>
#include <csignal>

#ifndef EV_SYN
#define EV_SYN 0
#endif

#define SIXAD
#ifdef SIXAD

#define     LEFTX              0
#define     LEFTY              1
#define     RIGHTX             2
#define     RIGHTY             3

#define     LEFTTRIGGER       12
#define     RIGHTTRIGGER      13

#define     triangle         300
#define     circle           301
#define     cross            302
#define     square           303

#define     select          288
#define     left_thumb      289
#define     right_thumb     290
#define     start           291

#define     HatLeft         295
#define     HatUp           292
#define     HatRight        293
#define     HatDown         294

#define     left_bottom_trigger 296
#define     right_bottomt_rigger    297
#define     LEFTTOPTRIGGER  298
#define     RIGHTTOPTRIGGER 299

#define     P3          304

#else

#define LEFTX  0
#define LEFTY  1
#define RIGHTX 2
#define RIGHTY 5

#define RIGHTTRIGGER 49
#define LEFTTRIGGER 48


#define     LeftX              0
#define     LeftY              1
#define     RightX             2
#define     RightY             5

#define     RightTrigger      48
#define     LeftTrigger       49

#define     triangle         300
#define     circle           301
#define     cross            302
#define     square           303

#define     select          288
#define     start           291

#define     HatLeft         295
#define     HatUp           292
#define     HatRight        293
#define     HatDown         294

#define     left_bottom_trigger 296
#define     right_bottomt_rigger    297
#define     LEFTTOPTRIGGER  298
#define     RIGHTTOPTRIGGER 299

#define     P3          704

#define LEFTX  0
#define LEFTY  1
#define RIGHTX 2
#define RIGHTY 5

#define RIGHTTRIGGER 49
#define LEFTTRIGGER 48
#endif

#ifdef DEBUG
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF(...)
#endif

using namespace mosqpp;

class Mqtt : public mosquittopp
{
private:
	const char         *host;
	const char         *id;
	const char         *topic;
	int                port;
	int                keepalive;

	void on_connect(int rc)
	{
		if ( rc == 0 ) {
			std::cout << ">> Mqtt - connected with server" << std::endl;
		} else {
			std::cout << ">> Mqtt - Impossible to connect with server(" << rc << ")" << std::endl;
		}
	}
	void on_disconnect(int rc)
	{
		std::cout << ">> Mqtt - disconnection(" << rc << ")" << std::endl;
	}
	void on_publish(int mid)
	{
#ifdef DEBUG
		std::cout << ">> Mqtt - Message (" << mid << ") succeed to be published " << std::endl;
#endif
	}

public:
	Mqtt(const char *id, const char *topic, const char *host, int port = 1883) : mosquittopp(id)
	{
		lib_init();      // Mandatory initialization for mosquitto library
		this->keepalive = 60;    // Basic configuration setup for Mqtt class
		this->id = id;
		this->port = port;
		this->host = host;
		this->topic = topic;
		connect_async(host,     // non blocking connection to broker request
					  port,
					  keepalive);
		loop_start();            // Start thread managing connection / publish / subscribe
	}

	~Mqtt()
	{
		loop_stop();      // Kill the thread
		lib_cleanup();    // Mosquitto library cleanup
	}

	bool send_message(const char *_message)
	{
		int ret = publish(NULL, this->topic, strlen(_message), _message, 1, false);
		return ( ret == MOSQ_ERR_SUCCESS );
	}

	bool print_message(const char *format, ...)
	{
	    va_list args;
	    va_start(args, format);
	    char buffer[132]; // max length
	    int n= vsnprintf(buffer, sizeof(buffer), format, args);
	    va_end(args);
		int ret = publish(NULL, this->topic, n, buffer, 1, false);
		return ( ret == MOSQ_ERR_SUCCESS );
	}
};

#define sgn(x) (((x) > 0) - ((x) < 0))

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

volatile bool doabort= false;
int fd= -1;
void signalHandler( int signum )
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    if(signum == SIGTERM){
    	doabort= true;
    	printf("Exiting\n");
    	close(fd);
    	fd= -1;
    }
}

std::map<int, int> button_map = { {right_thumb, 1}, {left_thumb, 1}, {triangle, 3}, {circle, 4}, {cross, 2}, {square, 5}, {select, 6}, {P3,  7}, {start, 8} };
int main (int argc, char **argv)
{
	int rd, i, j, k;
	struct input_event ev[64];
	int version;
	unsigned short id[4];
	char name[256] = "Unknown";
	int abs[5];
	Mqtt *mqtt = nullptr;
	bool connected= false;

	if (argc < 2) {
		printf("Usage: evtest /dev/input/eventX [mqtt host]\n");
		printf("Where X = input device number\n");
		return 1;
	}

tryagain:
	// register signal and signal handler
	signal(SIGTERM, signalHandler);
	signal(SIGHUP, signalHandler);

	while(!connected) {
	 	if ((fd = open(argv[1], O_RDONLY)) < 0) {
	 		for (int i = 0; i < 10; ++i) {
	 			usleep(500000); // wait a little longer
	 			if(doabort) goto aborted;
	 		}
	 		continue;
		}

		printf("Input driver version is %d.%d.%d\n", version >> 16, (version >> 8) & 0xff, version & 0xff);

		ioctl(fd, EVIOCGID, id);
		printf("Input device ID: bus 0x%x vendor 0x%x product 0x%x version 0x%x\n",  id[ID_BUS], id[ID_VENDOR], id[ID_PRODUCT], id[ID_VERSION]);

		ioctl(fd, EVIOCGNAME(sizeof(name)), name);
		printf("Input device name: \"%s\"\n", name);
		if(strstr(name, "PLAYSTATION") == nullptr) {
			fprintf(stderr, "device %s is not a Playstation GamePad: %s\n", argv[1], name);
			exit(1);
		}
		connected= true;
	}

	if(argc > 2) {
		mqtt = new Mqtt("hexapod control", "quadruped/commands", argv[2]);
		//mqtt->send_message("hello");
	}

	printf("Running ... send SIGTERM to exit\n");

	while (!doabort) {
		rd = read(fd, ev, sizeof(struct input_event) * 64);

		if (rd < (int) sizeof(struct input_event)) {
			if(doabort) break;

			perror("\nunexpected error reading event");
			break;
		}

		for (i = 0; i < rd / sizeof(struct input_event); i++) {
			if(doabort) break;

			if(ev[i].type == EV_KEY) {
				//printf("Button: code %d, value %d\n", ev[i].code, ev[i].value);
				if(ev[i].value == 0) continue; // ignore button up

				if(mqtt != nullptr && button_map.find(ev[i].code) != button_map.end()) {
					mqtt->print_message("G %d", button_map[ev[i].code]);

				}else{

					switch(ev[i].code) {
						case triangle:
							DEBUG_PRINTF("triangle\n");
							break;
						case circle:
							DEBUG_PRINTF("circle\n");
							break;
						case cross:
							DEBUG_PRINTF("cross\n");
							break;
						case square:
							DEBUG_PRINTF("square\n");
							break;
						case select:
							DEBUG_PRINTF("select\n");
							break;
						case start:
							DEBUG_PRINTF("start\n");
							break;
						case P3:
							DEBUG_PRINTF("P3\n");
							break;

						case HatUp:
							DEBUG_PRINTF("HatUp\n");
							if(mqtt != nullptr) mqtt->print_message("U 100");
							break;
						case HatDown:
							DEBUG_PRINTF("HatDown\n");
							if(mqtt != nullptr) mqtt->print_message("U -100");
							break;
						case HatLeft:
							DEBUG_PRINTF("HatLeft\n");
							if(mqtt != nullptr) mqtt->print_message("L -100");
							break;
						case HatRight:
							DEBUG_PRINTF("HatRight\n");
							if(mqtt != nullptr) mqtt->print_message("L 100");
							break;
						case LEFTTOPTRIGGER:
							DEBUG_PRINTF("left top trigger\n");
							// reset stride
							if(mqtt != nullptr) mqtt->print_message("L 0");
							break;
						case RIGHTTOPTRIGGER:
							DEBUG_PRINTF("right top trigger\n");
							// reset height
							if(mqtt != nullptr) mqtt->print_message("U 0");
							break;
					}
				}

			} else if(ev[i].type == EV_ABS) {
				//printf("Joystick: code %d, value %d\n", ev[i].code, ev[i].value);
#ifdef SIXAD
				// sixad returns +/- 10 --> 128 as 10 is the dead zone, map it to 1 --> 100
				int iv= ev[i].value;
				int v= 0;
				if(iv > 0) {
					v =  map(std::abs(iv), 10, 128, 1, 100);
				}else if(iv < 0) {
					v =  -map(std::abs(iv), 10, 128, 1, 100);
				}
				//printf("value: %d\n", v);
				// v= map(std::abs(iv), 10, 128, 1, 100) * sgn(iv);
#else
				int v = ev[i].value - 128;
				if(ev[i].code == RIGHTTRIGGER || ev[i].code == LEFTTRIGGER) {
					v = (v*100) / 128;

				} else {
					if(v > -10 && v < 10){
						v = 0;
					}else{
						v = (v*100) / 128;
					}
				}
#endif
				switch(ev[i].code) {
					case LEFTX:
						DEBUG_PRINTF("left X %d\n", v);
						if(mqtt != nullptr) mqtt->print_message("R %d", v);
						break;
					case LEFTY:
						DEBUG_PRINTF("left Y %d\n", v);
						break;
					case RIGHTX:
						DEBUG_PRINTF("right X %d\n", v);
						if(mqtt != nullptr) mqtt->print_message("X %d", -v);
						break;
					case RIGHTY:
						DEBUG_PRINTF("right Y %d\n", v);
						if(mqtt != nullptr) mqtt->print_message("Y %d", -v);
						break;
					case RIGHTTRIGGER:
						DEBUG_PRINTF("right trigger %d\n", v);
						if(mqtt != nullptr) mqtt->print_message("T %d", v);
						break;
					case LEFTTRIGGER:
						DEBUG_PRINTF("left trigger %d\n", v);
						if(mqtt != nullptr) mqtt->print_message("T %d", -v);
						break;
				}
			}
		}
	}

aborted:
	printf("Exiting cleanly\n");
	if(fd != -1) close(fd);
	return(1);
}

/*
Raw...
joystick code:
0 LeftX
1 LeftY
2 RightX

5 RightY

48 RightTrigger
49 LeftTrigger

button code:
300 triangle
301 circle
302 cross
303 square

288 select
291 start

295 HatLeft
292 HatUp
293 HatRight
294 HatDown

296 left bottom trigger
297 right bottom trigger
298 left top trigger
299 right top trigger

704 P3

sixa-raw...
joystick code:
+/- 128
2 RightX
3 RightY

12 LeftTrigger
13 RightTrigger


button code:
304 P3

*/

