all: ads1256

ads1256: main.cpp
	g++ -o ads1256_dac8552 -lwiringPi main.cpp
