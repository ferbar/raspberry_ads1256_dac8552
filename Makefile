.SUFFIXES:
.PHONY: clean

all: ads1256_dac8552

CFLAGS=-g -O0 -Wall -Wformat -std=c++11
CFLAGS_DEP=-MMD -MF $(dir $@).$(notdir $(basename $@)).d

LDFLAGS=-lwiringPi -lpthread
CPP=g++

ads1256_dac8552: RuntimeExceptionWithBacktrace.o dac8532.o ads1256.o utils.o tcpclient.o clientthread.o server.o main.o
	${CPP} ${LDFLAGS} $+ -o ads1256_dac8552

%.o : %.cpp %.h
	${CPP} ${CFLAGS_DEP} ${CFLAGS} -c -o $@  $<

main.o : main.cpp
	${CPP} ${CFLAGS_DEP} ${CFLAGS} -c -o $@  $< -DSVNVERSION="git$$(git log --oneline | wc -l)"

clean:
	rm -f *.o .*.d

-include .*.d
