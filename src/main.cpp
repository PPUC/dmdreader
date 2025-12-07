#ifdef RP2350
// Set the official 200 MHz system clock
// For RP2350, PLL parameters must be provided
#define PLL_SYS_VCO_FREQ_HZ (1600ul * 1000ul * 1000ul)
#define PLL_SYS_POSTDIV1    4
#define PLL_SYS_POSTDIV2    2
#endif

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

  dmdreader_init(pio0);
}

void setup1() {
  dmdreader_spi_init(pio0);
}

void loop() {
  // Everything is triggered by interrupts, so nothing to do here
  delay(10);
}

void loop1() {
  if (!dmdreader_spi_send()) {
    tight_loop_contents();
  }
}