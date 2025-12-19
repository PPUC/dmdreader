// dmd_reader.h
#ifndef DMD_READER_H
#define DMD_READER_H

#include <Arduino.h>

enum class Color : uint8_t {
  ORANGE,
  RED,
  YELLOW,
  GREEN,
  BLUE,
  PURPLE,
  PINK,
  WHITE
};

void dmdreader_init();

void dmdreader_spi_init();
bool dmdreader_spi_send();

void dmdreader_loopback_init(uint8_t *buffer1, uint8_t *buffer2, Color color);
void dmdreader_loopback_stop();
uint8_t *dmdreader_loopback_render();

void dmdreader_error_blink(bool no_error);

#endif
