#ifndef UTILS_H
#define UTILS_H

#include "RuntimeExceptionWithBacktrace.h"

// macht aus blah "blah"
#define     _STR(x)   _VAL(x)
#define     _VAL(x)   #x

  #define ANSI_DEFAULT "\x1b[0m"
  #define ANSI_RED "\x1b[31m"
  #define ANSI_REDBOLD "\x1b[31;1m"

  #define ANSI_GREEN "\x1b[32m"
  #define ANSI_GREENBOLD "\x1b[32;1m"

extern const std::string NOT_SET;

extern bool cfg_debug;
extern int cfg_max_channels;

double getCurrentTime();

void printHex(const char *s, int len);

struct bufferEntry {
	double time;
	int value[8];
};
const int maxBufferEntries=1000;
extern bufferEntry ADCbuffer[];
extern int bufferPos;

#include <string>
#include <sstream>

namespace utils
{

	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << n ;
		return stm.str() ;
	}

	std::string format(const char* fmt, ...);
#ifdef NO_STD_STOD
	double stod(const std::string &in);
#endif
	bool stobool(const std::string &in);
	bool startsWith(const std::string &str, const char *with);
}


#endif
