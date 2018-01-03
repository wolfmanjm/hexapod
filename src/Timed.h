/**
	Will execute the given function at the frequency specified
*/

#pragma once

#include <functional>

class Timed
{
public:
	Timed(float update_frequency);
	~Timed();
	void setFrequency(float update_frequency);

	// execute the lambda iterations times at the update frequency
	void run(uint32_t iterations, std::function<void(void)> fnc);
	uint32_t micros( void );

private:
	bool timeInit(void);
	uint64_t tsc_init;
	float cpufreq;
	float clocks_per_ns;
	uint32_t usleep_time;
};
