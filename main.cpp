#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <stdexcept>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>

#include "utils.h"
#include "ads1256.h"
#include "dac8532.h"

#include <getopt.h>

/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/

bool cfg_debug=false;
bool cfg_dump=false;
bool cfg_led_test=false;


int main(int argc, char*argv[] ) {
	int spiSpeed=500000;
	while (1) {
		int c;
		int option_index = 0;
		static struct option long_options[] = {
			{"help", 0, NULL, 'h'},
			{"debug", 0, NULL, 'd'},
			{"dump", 0, NULL, 'u'},
			{"version", 0, NULL, 'v'},
			{"spi-speed", 1, NULL, 's'},
			{"led-test", 0, NULL, 'l'},
			{0, 0, 0, 0}
		};
		c = getopt_long(argc, argv, "hduvs:l", long_options, &option_index);
		if (c == -1)
			break;
		switch (c) {
			case 'v':
				printf("version %s\n", _STR(SVNVERSION));
				exit(0);
			case 'd':
				cfg_debug=1;
			case 'u':
				cfg_dump=1;
				/* no break */
			case 'l':
				cfg_led_test=1;
			case 'h':
			default:
				printf("raspberry pi waveshare High-Precision AD/DA Board test\n");
				exit(1);
		}
	}


	DAC8532_initPins();
	ADS1256_initPins();
	int spiHandle=ADS1256_openSPI(spiSpeed);
	ADS1256_init(spiHandle);
	if(cfg_led_test) {
		fadeLeds(spiHandle);
		exit(0);
	}
	
	while(true) {
		ADS1256_WaitDRDY();
		digitalWrite(ADS1256_CS, LOW); // pi.write(22, 0)    # ADS1256 /CS low
		// set register01(MUX) reg.01,one byte,AIN0/AINCOM:0x51,0x00,0x08
		// send commands 0xfc:sync ,0x00:wakeup, 0x01:read data      
		// and read one ADC-value (3 bytes):
 		// (count, databyte) = pi.spi_xfer(ad_da, b'\x51\x00\x08\xfc\x00\x01\x00\x00\x00')
		unsigned char databyte[6] = {(CMD_WREG | REG_MUX), '\x00', '\x08', CMD_SYNC, CMD_WAKEUP, CMD_RDATA};
		// int count=wiringPiSPIDataRW(SPICHANNEL, databyte, sizeof(databyte));
		write(spiHandle, databyte, 6);

		ADS1256_DelayDATA();
		// write(spiChannel,"\x51\x00\x08\xfc\x00\x01\x00\x00\x00", 9);
		//char databyte[10];
		//bzero(databyte, sizeof(databyte));
		int count=read(spiHandle, databyte, 3);

		digitalWrite(ADS1256_CS, HIGH); // pi.write(22, 1)    # ADS1256 /CS high
		printf("%x %x %x \n",databyte[0], databyte[1], databyte[2]);
		// discard databyte[0:5]
		// concatenate 3 bytes of databyte[6:8], [6] is MSB:
		int  data = (databyte[0]<<16 | databyte[1]<<8 | databyte[2]);
		double volt = ((double)data / 0x7fffff) *5;
		printf("data: %8d => %01.4f count=%d\n", data, volt, count);
	}
}
