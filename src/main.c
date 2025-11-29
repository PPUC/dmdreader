// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include <Arduino.h>

#include "dmd_reader.h"
#include "hardware/clocks.h"

void setup() {
  // overclock to achieve higher SPI transfer speed
  set_sys_clock_khz(SYS_CLK_MHZ * 1000, true);

  init();
}

void loop() { read_dmd(); }