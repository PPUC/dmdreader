// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include <stdbool.h>

#include "dmd_reader.h"
#include "hardware/clocks.h"

int main() {
  // overclock to achieve higher SPI transfer speed
  set_sys_clock_khz(SYS_CLK_MHZ * 1000, true);

  int result = read_dmd();

  return result;
}
