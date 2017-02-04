#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
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

#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <wiringPi.h>

#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "utils.h"
#include "ads1256.h"

int cfg_spiSpeed=-1;

void ADS1256_initPins() {
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
	pinMode(ADS1256_DRDY, INPUT);   //pi.set_mode(17, pigpio.INPUT)      // pin 11 ADS1256 /DRDY (aux SPI ce1)
	pinMode(ADS1256_RESET, OUTPUT); //pi.set_mode(18, pigpio.OUTPUT)     // pin 12 ADS1256 /RESET (aux SPI ce0)
	// pi.set_mode(19, pigpio.INPUT)    # pin*35 (aux SPI miso)
	// pi.set_mode(20, pigpio.INPUT)    # pin*38 (aux SPI mosi)
	// pi.set_mode(21, pigpio.INPUT)    # pin*40 (aux SPI sclk)
	pinMode(ADS1256_CS, OUTPUT); //pi.set_mode(22, pigpio.OUTPUT)     // pin 15 ADS1256 /CS input
//	pinMode(DAC8532_CS, OUTPUT); //pi.set_mode(23, pigpio.OUTPUT)     // pin 16 DAC8532 /CS input
	// pi.set_mode(24, pigpio.OUTPUT)   # pin 18 AD9952 Reset)
	// pi.set_mode(25, pigpio.OUTPUT)   # pin 22 AD9952 I/O-Update)
	// pi.set_mode(26, pigpio.INPUT)    # pin*37 (free)
	pinMode(ADS1256_PDWN, OUTPUT); //pi.set_mode(27, pigpio.OUTPUT)     // pin 13 ADS1256 /PDWN input

	// set GPIO outputs to start values
//	digitalWrite(DAC8532_CS, HIGH); // pi.write(23, 1)    // DAC8532 /CS deselected
	digitalWrite(ADS1256_CS, HIGH); // pi.write(22, 1)    // ADS1256 /CS deselected
// pi.write(11, 0)    // ADS1256 SCLK low (later overridden by spi_open)
// pi.write(10, 0)    // ADS1256 DIN  low (later overridden by spi_open)
	digitalWrite(ADS1256_PDWN, HIGH); // pi.write(27, 1)    // ADS1256 /PDWN high
	digitalWrite(ADS1256_RESET, HIGH); // pi.write(18, 1)    // ADS1256 /RESET high
	// pi.write(8, 1)     # AD9952  /CS high
	// pi.write(24, 0)    # AD9952  RESET low
	// pi.write(25, 0)    # AD9952  I/O-Update low

	digitalWrite(ADS1256_RESET, LOW);//pi.write(18, 0)    # ADS1256 /RESET low
	usleep(1000); // wait 1 msec
	digitalWrite(ADS1256_RESET, HIGH); //pi.write(18, 1)    # ADS1256 /RESET high
	usleep(500 * 1000); // wait 0.5 sec

}

/**
 *  according to the datasheet t6 must be at least 50*1/CLKIN = 6.5µs
 *  at a clock rate of 200kHz the time between clock low and next clock rise would be 5µs which means we would have to wait for just 1.5µs
 */
void ADS1256_DelayDATA(void)
{
	/*
	   Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
	   min  50   CLK = 50 * 0.13uS = 6.5uS
	 */
	//bsp_DelayUS(10);        /* The minimum time delay 6.5us */
	usleep(7);
}


/**
 * delay time wait for automatic calibration
 *       parameter:  NULL
 *       The return value:  NULL
 * wait until ADS1256_DRDY goes to zero. using a timeout of 0.5s
 */
void ADS1256_WaitDRDY() {
	double start = getCurrentTime();
	// Waits for DRDY to go to zero or TIMEOUT seconds to pass
	while (getCurrentTime()-start < 0.5) {
		bool drdy_level = digitalRead(ADS1256_DRDY); //  drdy_level = pi.read(17) # ADS1256 /DRDY
		if(drdy_level == LOW) {
			return;
		}
		usleep(1000);
	}
	throw std::runtime_error("ADS1256_WaitDRDY timeout");

}

/**
 * read ADC value
 * @param spiHandle
 * @return sample value
 */
int32_t ADS1256_ReadData(int spiHandle)
{
	uint32_t ret = 0;
	static uint8_t buf[3];

	digitalWrite(ADS1256_CS, LOW) ; //CS_0(); /* SPI  cs  = 0 */

	//ADS1256_Send8Bit(CMD_RDATA);    /* read ADC command  */
	buf[0]=CMD_RDATA;
	assert(write(spiHandle, buf, 1) == 1);

	ADS1256_DelayDATA();    /*delay time  */

	/*Read the sample results 24bit*/
	//buf[0] = ADS1256_Recive8Bit();
	//buf[1] = ADS1256_Recive8Bit();
	//buf[2] = ADS1256_Recive8Bit();
	assert(read(spiHandle, buf, 3) == 3);

	ret = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	ret |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
	ret |= buf[2];

	digitalWrite(ADS1256_CS, HIGH); // CS_1(); /* SPIƬѡ = 1 */

	/* Extend a signed number*/
	if (ret & 0x800000)
	{
		ret |= 0xFF000000;
	}

	return (int32_t)ret;
}

/**
 * Read the corresponding register
 * @param spiHandle
 * @param _RegID: register  ID
 * @return read register value
 */
uint8_t ADS1256_ReadReg(int spiHandle, uint8_t _regID)
{

	digitalWrite(ADS1256_CS, LOW) ; //CS_0(); /* SPI  cs  = 0 */
/*
	ADS1256_Send8Bit(CMD_RREG | _regID);    // Write command register 
	ADS1256_Send8Bit(0x00); // Write the register number
*/
	unsigned char buffer[2];
	buffer[0] = CMD_RREG | _regID;
	buffer[1] = 0;
	write(spiHandle, buffer, sizeof(buffer));

	ADS1256_DelayDATA();    //delay time

	// uint8_t read = ADS1256_Recive8Bit();    /* Read the register values */
	if(read(spiHandle, buffer, 1) != 1) {
		perror("error reading data");
		exit(1);
	}
	digitalWrite(ADS1256_CS, HIGH); // CS_1(); /* SPI   cs  = 1 */

	return buffer[0];
}

/**
 * ADS1256_ReadChipID - Read the chip ID
 * @param spiHandle
 * @return four high status register
 */
uint8_t ADS1256_ReadChipID(int spiHandle)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(spiHandle, REG_STATUS);
	return (id >> 4);
}

int ADS1256_openSPI(int spiSpeed) {
	cfg_spiSpeed=spiSpeed;
	int fd;
	if ((fd = open ("/dev/spidev0.0", O_RDWR)) < 0) {
		printf("error opening spi\n");
		exit(1);
	}
	printf("fd: %d\n", fd);
	__u8 mode=-1;
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		printf("error reading spi mode\n");
		exit(1);
	}
	printf("current mode: %d\n", mode);

	int speed=-1;
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		printf("error reading spi speed\n");
		exit(1);
	}
	printf("current speed: %d\n", speed);
	close(fd);

	int spiHandle=wiringPiSPISetupMode(/* channel */ SPICHANNEL , /* speed */ spiSpeed, SPI_MODE_1) ;
	//int spiChannel=wiringPiSPISetup(/* channel */ SPICHANNEL , /* speed */ 200000) ;

	if(spiHandle < 0) {
		fprintf(stderr, "error: %s\n", strerror(errno));
		exit(1);
	}

	printf("spiHandle: %d\n", spiHandle);
	if(ioctl(spiHandle, SPI_IOC_RD_MODE, &mode) < 0) {
		printf("error reading spi mode\n");
		exit(1);
	}
	printf("current mode: %d\n", mode);
	return spiHandle;

}

void ADS1256_init(int spiHandle) {

// exit(1);
	

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
/*	
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
*/
}
