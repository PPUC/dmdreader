// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include "dmd_reader.h"

int main() {
  // overclock to achieve higher SPI transfer speed
  set_sys_clock_khz(SYS_CLK_MHZ * 1000, true);
  uint32_t freq = clock_get_hz(clk_sys);
  printf("System clock: %.2f MHz\n", freq / 1e6);

  int result = read_dmd();

  return result;
}
