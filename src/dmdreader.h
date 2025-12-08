// dmd_reader.h
#ifndef DMD_READER_H
#define DMD_READER_H

#include <Arduino.h>
#include "hardware/pio.h"

void dmdreader_init(PIO pio);

void dmdreader_spi_init(PIO pio);
bool dmdreader_spi_send();

void dmdreader_loopback_init(uint8_t *buffer1, uint8_t *buffer2);
bool dmdreader_loopback_render();

#endif
