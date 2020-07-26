#include "Timed.h"

#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/sysinfo.h>
#include <memory.h>
#include <math.h>

Timed::Timed(float update_frequency)
{
	timeInit();
	this->usleep_time= round(1000000.0/update_frequency);
}

Timed::~Timed()
{
}

#if !USE_CHRONO
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
#endif

/* TSC snapshot */
bool Timed::timeInit(void)
{
#if USE_CHRONO
    bot = std::chrono::steady_clock::now();
#else
    int cpufreq_fd, ret;
    char buf[0x400];
    char * str = 0, * str2 = 0;
    const char * mhz_str = "cpu MHz\t\t: ";

    /* Grab initial TSC snapshot */
    tsc_init = rdtsc();

    cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0){
        fprintf(stderr, "unable to open /proc/cpuinfo\n");
        return false;
    }
    memset(buf, 0x00, sizeof(buf));
    ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 ){
        fprintf(stderr, "unable to read cpuinfo !\n");
        close(cpufreq_fd);
        return false;
    }
    close(cpufreq_fd);
    str = strstr(buf, mhz_str);
    if (!str){
        fprintf(stderr, "Buffer %s does not contain CPU frequency info !\n", buf);
        return false;
    }

    str += strlen(mhz_str);
    str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n'){
        str2++;
    }
    if(str2 == buf + sizeof(buf-1) && *str2 !='\n'){
        fprintf(stderr, "malformed cpufreq string %s\n", str);
        return false;
    }
    *str2 = '\0';
    cpufreq = atof(str);


    //printf("cpufrequency is %f mhz\n", cpufreq);

    /* Calculate nanoseconds per clock */
    clocks_per_ns = 1000/cpufreq;

    //printf("nanoseconds per clock %f\n", clocks_per_ns);
#endif
    return true;
}

uint32_t Timed::micros( void )
{
#if USE_CHRONO
    std::chrono::steady_clock::time_point now= std::chrono::steady_clock::now();

    return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now-bot).count();

#else

    uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;

    divisor = (cpufreq );
    diff = tsc_cur - tsc_init;

    return (uint32_t) (diff / divisor);
#endif
}

void Timed::run(uint32_t iterations, std::function<void(void)> fnc)
{
	for (uint32_t j = 0; j < iterations; ++j) {
	    uint32_t t1= micros();
	    fnc();
		uint32_t t2= micros();
		uint32_t t3= (t2>t1)?t2-t1:0;

		// adjust for length of time it took above
        if(usleep_time > t3) {
            usleep(usleep_time - t3);
		}
	}
}
