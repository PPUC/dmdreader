// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include <Arduino.h>

#include "dmdreader.h"
#include "hardware/clocks.h"

void setup() {
  // overclock to achieve higher SPI transfer speed
  set_sys_clock_khz(SYS_CLK_MHZ * 1000, true);

  pinMode(LED_BUILTIN, OUTPUT);

  dmdreader_init();
}

void loop() {
  // Everything is triggered by interrupts, so nothing to do here
  delay(10);
}

void loop1() {
  if (!dmdreader_send()) {
    // @todo use interrupt to be as fats as possible
    delayMicroseconds(100);
  }
}