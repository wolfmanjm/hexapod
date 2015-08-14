#include <iostream>
#include <stdarg.h>

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

void debug_printf(const char *format, ...)
{
	if(!debug_verbose) return;
	va_list args;
  	va_start (args, format);
  	vprintf (format, args);
  	va_end (args);
}
