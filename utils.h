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


extern bool cfg_debug;

double getCurrentTime();

void printHex(const char *s, int len);

struct bufferEntry {
	double time;
	int value;
};
const int maxBufferEntries=1000;
extern bufferEntry buffer[];
extern int bufferPos;


#endif
