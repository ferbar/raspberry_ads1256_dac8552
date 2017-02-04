#ifndef DAC8552_H
#define DAC8552_H

#define DAC8532_CS 4


void Write_DAC8552(int spiHandle, uint8_t channel, uint16_t data);
void fadeLeds(int spiChannel);

#endif
