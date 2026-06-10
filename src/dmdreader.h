// dmd_reader.h
#ifndef DMD_READER_H
#define DMD_READER_H

#include <Arduino.h>

enum class Color : uint8_t {
  DMD_ORANGE,
  DMD_RED,
  DMD_YELLOW,
  DMD_GREEN,
  DMD_BLUE,
  DMD_PURPLE,
  DMD_PINK,
  DMD_WHITE
};

// DMD types
enum DmdType : uint8_t {
  DMD_UNKNOWN,
  DMD_WPC,
  DMD_DOTMATION,
  DMD_WHITESTAR,
  DMD_SPIKE1,
  DMD_SAM,
  DMD_DE_X16_V1,
  DMD_DE_X16_V2,
  DMD_DESEGA,
  DMD_SEGA_HD,
  DMD_GOTTLIEB,
  DMD_ALVING,
  DMD_ROMSTAR,
  DMD_ISLAND,
  DMD_HOMEPIN,
  DMD_SPINBALL,
  DMD_SLEIC,
  // CAPCOM needs to be the last two entries:
  DMD_CAPCOM,
  DMD_CAPCOM_HD,
};

bool dmdreader_init(bool return_on_no_detection = false);

void dmdreader_spi_init();
bool dmdreader_spi_send();

void dmdreader_loopback_init(uint8_t *buffer1, uint8_t *buffer2, Color color);
void dmdreader_loopback_stop();
uint8_t *dmdreader_loopback_render();

void dmdreader_error_blink(bool no_error);

uint16_t dmdreader_get_source_width();
uint16_t dmdreader_get_source_height();

#endif
