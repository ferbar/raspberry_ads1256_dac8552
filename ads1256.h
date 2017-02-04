#ifndef ADS1256_H
#define ADS1256_H

#include <inttypes.h>

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

#define ADS1256_DRDY  0
#define ADS1256_RESET 1
#define ADS1256_CS 3
#define ADS1256_PDWN 2

#define SPICHANNEL 1

//CS      -----   SPICS  
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset


#define  DRDY  RPI_GPIO_P1_11    //P0
#define  RST  RPI_GPIO_P1_12     //P1
#define  SPICS  RPI_GPIO_P1_15   //P3

// da hängt der ADC dran
#define CS_1()  bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)

// da hängt der DAC8532 dran
#define CS1_1() bcm2835_gpio_write(SPICS1,HIGH)
#define CS1_0() bcm2835_gpio_write(SPICS1,LOW)

#define DRDY_IS_LOW()  ((bcm2835_gpio_lev(DRDY)==0))

#define RST_1()   bcm2835_gpio_write(RST,HIGH);
#define RST_0()   bcm2835_gpio_write(RST,LOW);


/* gain channelî */
typedef enum
{
	ADS1256_GAIN_1      = 0,  /* GAIN   1 */
	ADS1256_GAIN_2      = 1,  /* GAIN   2 */
	ADS1256_GAIN_4      = 2,  /* GAIN   4 */
	ADS1256_GAIN_8      = 3,  /* GAIN   8 */
	ADS1256_GAIN_16     = 4,  /* GAIN  16 */
	ADS1256_GAIN_32     = 5,  /* GAIN  32 */
	ADS1256_GAIN_64     = 6,  /* GAIN  64 */
} ADS1256_GAIN_E;

/* Sampling speed choice*/
/* 
  11110000 = 30,000SPS (default)
  11100000 = 15,000SPS
  11010000 = 7,500SPS
  11000000 = 3,750SPS
  10110000 = 2,000SPS
  10100001 = 1,000SPS
  10010010 = 500SPS
  10000010 = 100SPS
  01110010 = 60SPS
  01100011 = 50SPS
  01010011 = 30SPS
  01000011 = 25SPS
  00110011 = 15SPS
  00100011 = 10SPS
  00010011 = 5SPS
  00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0xF0, // reset the default values
	ADS1256_15000SPS = 0xE0,
	ADS1256_7500SPS = 0xD0,
	ADS1256_3750SPS = 0xC0,
	ADS1256_2000SPS = 0xB0,
	ADS1256_1000SPS = 0xA1,
	ADS1256_500SPS = 0x92,
	ADS1256_100SPS = 0x82,
	ADS1256_60SPS = 0x72,
	ADS1256_50SPS = 0x63,
	ADS1256_30SPS = 0x53,
	ADS1256_25SPS = 0x43,
	ADS1256_15SPS = 0x33,
	ADS1256_10SPS = 0x20,
	ADS1256_5SPS = 0x13,
	ADS1256_2d5SPS = 0x03,
} ADS1256_DRATE_E;

/*
typedef struct
{
	ADS1256_GAIN_E Gain;    // GAIN
	ADS1256_DRATE_E DataRate;  // DATA output  speed
	int32_t AdcNow[8];      // ADC  Conversion value
	uint8_t Channel;        // The current channel
	uint8_t ScanMode;       // Scanning mode,   0  Single-ended input  8 channel£¬ 1 Differential input  4 chann
} ADS1256_VAR_T;
*/


/*Register definition£º Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,  // x1H
	REG_MUX    = 1,  // 01H
	REG_ADCON  = 2,  // 20H
	REG_DRATE  = 3,  // F0H
	REG_IO     = 4,  // E0H
	REG_OFC0   = 5,  // xxH
	REG_OFC1   = 6,  // xxH
	REG_OFC2   = 7,  // xxH
	REG_FSC0   = 8,  // xxH
	REG_FSC1   = 9,  // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition£º TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP  = 0x00, // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

void ADS1256_initPins();

void ADS1256_init(int spiHandle);
int ADS1256_openSPI(int spiSpeed);

void ADS1256_DelayDATA(void);

void ADS1256_WaitDRDY();

int32_t ADS1256_ReadData(int spiHandle);

uint8_t ADS1256_ReadReg(int spiHandle, uint8_t _regID);

uint8_t ADS1256_ReadChipID(int spiHandle);


#endif
