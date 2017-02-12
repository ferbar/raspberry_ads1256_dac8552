#include <time.h>
#include <stdio.h>
#include <string>
#include <cstdarg>
#include "utils.h"

const std::string NOT_SET="__NOT_SET";

double getCurrentTime() {
	double ret;
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);
	ret  = spec.tv_sec + ( spec.tv_nsec / 1.0e9 );
	return ret;
}

void printHex(const char *s, int len) {
	const char *pos=s;
	for(int i=0; i < len; i++) {
		// if(*pos < 31 || *pos > 127) {
			printf("\\x%02x",*pos);
		/*
		} else {
			printf("%c",*pos);
		}
		*/
		pos++;
	}
}

#ifdef NO_STD_STOD
double utils::stod(const std::string &in)  {
	if(in == NOT_SET) {
		throw std::runtime_error("NOT SET");
	}
	size_t end=0;
	double ret=std::stod(in, &end);
	if(end != in.length()) {
		throw std::runtime_error("error converting number ["+in+"]");
	}
	return ret;
}
#endif

bool utils::stobool(const std::string &in) {
	if(in=="false") return false;
	if(in=="true") return true;
	throw new std::runtime_error("error: invalid in ["+in+"]");
}


std::string utils::format(const char* fmt, ...){
    int size = 512;
    char* buffer = 0;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    int nsize = vsnprintf(buffer, size, fmt, vl);
    if(size<=nsize){ //fail delete buffer and try again
        delete[] buffer;
        buffer = 0;
        buffer = new char[nsize+1]; //+1 for /0
        nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
}

bool utils::startsWith(const std::string &str, const char *with) {
	return str.find(with) == 0;
}
