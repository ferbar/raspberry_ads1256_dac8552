#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <time.h>
#include <stdexcept>

#include <assert.h>
#include "dac8552.h"

#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include <wiringPi.h>

#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


// da hängt der DAC8532 dran
#define CS1_1() bcm2835_gpio_write(SPICS1,HIGH)
#define CS1_0() bcm2835_gpio_write(SPICS1,LOW)


void initiDAC8532Pins() {
	wiringPiSetup();
	printf("setting pins\n");
	// set GPIO data direction          ("pin*" = 40pin only):
	// pi.set_mode(0, pigpio.INPUT)     # pin*27 (Raspi-hat ID_SD)
	// pi.set_mode(1, pigpio.INPUT)     # pin*28 (Raspi-hat ID_SC) 
	// pi.set_mode(2, pigpio.INPUT)     # pin  3 (I2C SDA, fixed 1k8 pull-up)
	// pi.set_mode(3, pigpio.INPUT)     # pin  5 (I2C SCL, fixed 1k8 pull-up)
	// pi.set_mode(4, pigpio.INPUT)     # pin  7 (free)
	// pi.set_mode(5, pigpio.INPUT)     # pin*29 (free)
	// pi.set_mode(6, pigpio.INPUT)     # pin*31 (free)
	// pi.set_mode(7, pigpio.INPUT)     # pin 26 (main SPI CE1)
	// pi.set_mode(8, pigpio.OUTPUT)    # pin 24 AD9952 /CS (main SPI CE0) 
//	pi.set_mode(9, pigpio.INPUT)       // pin 21 (ADS1256 DOUT)
//	pi.set_mode(10, pigpio.OUTPUT)     // pin 19 (ADS1256 DIN)
//	pi.set_mode(11, pigpio.OUTPUT)     // pin 23 (ADS1256 SCLK)
	// pi.set_mode(12, pigpio.INPUT)    # pin*32 (free)
	// pi.set_mode(13, pigpio.INPUT)    # pin*33 (free)
	// pi.set_mode(14, pigpio.OUTPUT)   # pin  8 (Serial TXD)
	// pi.set_mode(15, pigpio.INPUT)    # pin 10 (Serial RXD)
	// pi.set_mode(16, pigpio.INPUT)    # pin*36 (aux SPI ce2)
//	pinMode(ADS1256_DRDY, INPUT);   //pi.set_mode(17, pigpio.INPUT)      // pin 11 ADS1256 /DRDY (aux SPI ce1)
//	pinMode(ADS1256_RESET, OUTPUT); //pi.set_mode(18, pigpio.OUTPUT)     // pin 12 ADS1256 /RESET (aux SPI ce0)
	// pi.set_mode(19, pigpio.INPUT)    # pin*35 (aux SPI miso)
	// pi.set_mode(20, pigpio.INPUT)    # pin*38 (aux SPI mosi)
	// pi.set_mode(21, pigpio.INPUT)    # pin*40 (aux SPI sclk)
//	pinMode(ADS1256_CS, OUTPUT); //pi.set_mode(22, pigpio.OUTPUT)     // pin 15 ADS1256 /CS input
	pinMode(DAC8532_CS, OUTPUT); //pi.set_mode(23, pigpio.OUTPUT)     // pin 16 DAC8532 /CS input
	// pi.set_mode(24, pigpio.OUTPUT)   # pin 18 AD9952 Reset)
	// pi.set_mode(25, pigpio.OUTPUT)   # pin 22 AD9952 I/O-Update)
	// pi.set_mode(26, pigpio.INPUT)    # pin*37 (free)
//	pinMode(ADS1256_PDWN, OUTPUT); //pi.set_mode(27, pigpio.OUTPUT)     // pin 13 ADS1256 /PDWN input

	// set GPIO outputs to start values
	digitalWrite(DAC8532_CS, HIGH); // pi.write(23, 1)    // DAC8532 /CS deselected
//	digitalWrite(ADS1256_CS, HIGH); // pi.write(22, 1)    // ADS1256 /CS deselected
// pi.write(11, 0)    // ADS1256 SCLK low (later overridden by spi_open)
// pi.write(10, 0)    // ADS1256 DIN  low (later overridden by spi_open)
//	digitalWrite(ADS1256_PDWN, HIGH); // pi.write(27, 1)    // ADS1256 /PDWN high
//	digitalWrite(ADS1256_RESET, HIGH); // pi.write(18, 1)    // ADS1256 /RESET high
	// pi.write(8, 1)     # AD9952  /CS high
	// pi.write(24, 0)    # AD9952  RESET low
	// pi.write(25, 0)    # AD9952  I/O-Update low

//	digitalWrite(ADS1256_RESET, LOW);//pi.write(18, 0)    # ADS1256 /RESET low
//	usleep(1000); // wait 1 msec
//	digitalWrite(ADS1256_RESET, HIGH); //pi.write(18, 1)    # ADS1256 /RESET high
//	usleep(500 * 1000); // wait 0.5 sec

}


/*
*********************************************************************************************************
*       name: Write_DAC8552
*       function:  DAC send data 
*       parameter: channel : output channel number (0x30 oder 0x34)
*                          data : output DAC value 
*       The return value:  NULL
*********************************************************************************************************
*/
void Write_DAC8552(int spiHandle, uint8_t channel, uint16_t data)
{
	uint8_t i;

	unsigned char buffer[3];
	buffer[0] = channel;
	buffer[1] = data>>8;
	buffer[2] = data & 0xff;
	/*
	CS1_1() ;
	CS1_0() ;
	*/
/*
	digitalWrite(DAC8532_CS, HIGH);
	printf(".");
	digitalWrite(DAC8532_CS, LOW);
*/
	/*
	bcm2835_spi_transfer(channel);
	bcm2835_spi_transfer((Data>>8));
	bcm2835_spi_transfer((Data&0xff));
	*/
	//if(write(spiChannel, buffer, sizeof(buffer)) != sizeof(buffer)) {
int rc;
// while(1) {
	//printf("+");
	digitalWrite(DAC8532_CS, HIGH);
	//printf("-");
	digitalWrite(DAC8532_CS, LOW);
	// write(spiChannel, buffer, sizeof(buffer));
	buffer[0] = channel;
	buffer[1] = data>>8;
	buffer[2] = data & 0xff;
	// rc=wiringPiSPIDataRW(spiChannel, buffer, sizeof(buffer));
	rc=write(spiHandle, buffer, sizeof(buffer));
	if(rc != sizeof(buffer) ) {
		perror("blah");
		exit(1);
	}
// break;
// }
//	 rc=wiringPiSPIDataRW(SPICHANNEL, buffer, sizeof(buffer));
	//printf("rc: %d writing data\n", rc);
	//CS1_1() ;
	digitalWrite(DAC8532_CS, HIGH);
}

/**
 * @param spiChannel - file handle
 */
void fadeLeds(int spiChannel) {
	while(1) {
		for(int i=0; i < 6400; i++) {
			Write_DAC8552(spiChannel, 0x30, i*10);
			Write_DAC8552(spiChannel, 0x34, (6400-i)*10);
			if(i%10 == 0 ) {
				printf("."); fflush(stdout);
			} else {
				usleep(100);
			}
		}
	}

}

