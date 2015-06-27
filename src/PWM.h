#pragma once

#include <vector>
#include <stdint.h>

namespace mraa {
	class Pwm;
};

class PWM
{
public:
	PWM(uint8_t type);
	~PWM();
	void setAngle(int channel, float a);
	void setFrequency(int channel, float f);
	void enable(int channel, bool f);

private:
	float dutyCycle(float angle);

	std::vector<mraa::Pwm*> pwm;

	uint8_t type;
	int period_us;
};
