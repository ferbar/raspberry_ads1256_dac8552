#ifndef DAC8552_H
#define DAC8552_H

#include <inttypes.h>

#define DAC8532_CS 4

void DAC8532_initPins();
void DAC8532_write(int spiHandle, uint8_t channel, uint16_t data);
void fadeLeds(int spiChannel);

#endif
