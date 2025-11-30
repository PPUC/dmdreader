// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include <Arduino.h>

#include "dmdreader.h"
#include "hardware/clocks.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // overclock to achieve higher SPI transfer speed
  set_sys_clock_khz(SYS_CLK_MHZ * 1000, true);

  dmdreader_init();

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() { dmdreader_read(); }