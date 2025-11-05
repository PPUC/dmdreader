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
