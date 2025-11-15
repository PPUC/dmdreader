// set to officially supported 200MHz clock
// @see SYS_CLK_MHZ https://github.com/raspberrypi/pico-sdk/releases/tag/2.1.1
#define SYS_CLK_MHZ 200

#include <stdbool.h>

#include "dmd_reader.h"
#include "hardware/gpio.h"
#include "logic_analyzer.h"
#include "pico/stdlib.h"

int main() {
  int result = read_dmd();

  if (result == -1) {
    analyze();
    return 0;
  }

  return result;
}
