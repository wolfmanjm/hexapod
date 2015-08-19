#include "helpers.h"

bool debug_verbose = false;

void debug_printf(const char *format, ...)
{
	if(!debug_verbose) return;
	va_list args;
  	va_start (args, format);
  	vprintf (format, args);
  	va_end (args);
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
