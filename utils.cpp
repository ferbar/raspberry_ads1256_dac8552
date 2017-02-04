#include <time.h>
#include <stdio.h>

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
	
