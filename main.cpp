#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <time.h>
#include <stdexcept>

#include <assert.h>

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

#include <stdio.h>
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

#include "utils.h"
#include "ads1256.h"


int main(int argc, char*argv[] ) {
	ADS1256_initPins();
	int spiHandle=ADS1256_openSPI(500000);
	ADS1256_init(spiHandle);
/*
	int id=ADS1256_ReadChipID(spiHandle);
	printf("ADS1256 chip ID: %d\n", id);

	// select ADC:
	digitalWrite(ADS1256_CS, LOW); //pi.write(22, 0)    # ADS1256 /CS low
	ADS1256_WaitDRDY();

	write(spiHandle,"\xfe", 1); // pi.spi_write(ad_da, b'\xfe') # command 0xfe: soft-reset command
	usleep(500 * 1000);     // wait 0.5 sec

	// set register 00 (STATUS) reg.00,one byte,no autocal,no buffer
	ADS1256_WaitDRDY();
	digitalWrite(ADS1256_CS, LOW); //pi.write(22, 0)    # ADS1256 /CS low
	const char status[5]= { '\xfc', '\x00' ,(CMD_WREG | REG_STATUS) , '\x00', 1 };
	write(spiHandle, status, 5); // pi.spi_write(ad_da, b'\xfc\x00\x50\x00\x01')
	digitalWrite(ADS1256_CS, HIGH); //pi.write(22, 1)    # ADS1256 /CS high
	usleep(100); // wait 0.1 msec

	printf("status initialized1\n");

	// set register 02 (ADCON)
	ADS1256_WaitDRDY();
	digitalWrite(ADS1256_CS, LOW); //pi.write(22, 0)    # ADS1256 /CS low
	const char adcon[5]= { '\xfc', '\x00' ,(CMD_WREG | REG_ADCON) , '\x00', ADS1256_GAIN_1 };
	printf("adcon: >>"); printHex(adcon,5); printf("<<\n");
	write(spiHandle, adcon,5); //pi.spi_write(ad_da, b'\xfc\x00\x52\x00\x00')
	digitalWrite(ADS1256_CS, HIGH); //pi.write(22, 1)    # ADS1256 /CS high
	usleep(100); // wait 0.1 msec

	// pi.set register 03 (DRATE) reg.03,one byte,10 samples per secondc
	ADS1256_WaitDRDY();
	digitalWrite(ADS1256_CS, LOW); //pi.write(22, 0)    # ADS1256 /CS low
	const char drate[5]= { '\xfc', '\x00', (CMD_WREG | REG_DRATE) ,'\x00', ADS1256_1000SPS};
	printf("drate: >>"); printHex(drate,5); printf("<<\n");
	write(spiHandle, drate, 5); //pi.spi_write(ad_da, b'\xfc\x00\x53\x00\x23')
	digitalWrite(ADS1256_CS, HIGH); //pi.write(22, 1)    # ADS1256 /CS high
	usleep(100);  // wait 0.1 msec

	printf("adcon + drate initialized\n");
*/
	
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
