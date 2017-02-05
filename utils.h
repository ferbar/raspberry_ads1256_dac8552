#ifndef UTILS_H
#define UTILS_H

// macht aus blah "blah"
#define     _STR(x)   _VAL(x)
#define     _VAL(x)   #x


double getCurrentTime();

void printHex(const char *s, int len);

#endif
