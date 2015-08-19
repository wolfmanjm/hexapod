#pragma once

#include <iostream>
#include <stdarg.h>
#include <tuple>

extern bool debug_verbose;

// helper function to print a tuple of any size
template<class Tuple, std::size_t N>
struct TuplePrinter {
	static void print(const Tuple &t)
	{
		TuplePrinter < Tuple, N - 1 >::print(t);
		std::cout << ", " << std::get < N - 1 > (t);
	}
};

template<class Tuple>
struct TuplePrinter<Tuple, 1> {
	static void print(const Tuple &t)
	{
		std::cout << std::get<0>(t);
	}
};

template<class... Args>
void print(const std::tuple<Args...> &t)
{
	std::cout << "(";
	TuplePrinter<decltype(t), sizeof...(Args)>::print(t);
	std::cout << ")\n";
}

void debug_printf(const char *format, ...);
int map(int x, int in_min, int in_max, int out_min, int out_max);

#define sgn(x) (((x) > 0) - ((x) < 0))
