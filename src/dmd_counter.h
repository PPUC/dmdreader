#ifndef DMD_COUNTER_H
#define DMD_COUNTER_H

#include "dmd_counter.pio.h"
#include "dmd_reader_pins.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

void dmd_counter_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c = dmd_count_dotclk_program_get_default_config(offset);

  // Set the IN base pin
  sm_config_set_in_pins(&c, DE);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, DOTCLK, 1, false);
  pio_sm_set_consecutive_pindirs(pio, sm, RCLK, 3, false);  // RCLK, RDATA, DE

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, DOTCLK);
  pio_gpio_init(pio, DE);
  pio_gpio_init(pio, RDATA);
  pio_gpio_init(pio, RCLK);

  // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(&c,
                         false,  // Shift-to-right = false
                         true,   // Autopull enabled
                         32      // Autopull threshold = 32
  );

  // We only send, so disable the TX FIFO to make the RX FIFO deeper.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  // For the 200MHz clock
  sm_config_set_clkdiv(&c, 1.6f);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_COUNTER_H
