#ifndef DMD_INTERFACE_WHITESTAR_H
#define DMD_INTERFACE_WHITESTAR_H

#include "dmd_interface_whitestar.pio.h"
#include "dmd_reader_pins.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

void dmd_reader_whitestar_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c = dmd_reader_whitestar_program_get_default_config(offset);

  // Set the IN pin, we don't use any other
  sm_config_set_in_pins(&c, SDATA);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, SDATA, 2, false);  // SDATA, DOTCLK

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, DOTCLK);
  pio_gpio_init(pio, SDATA);

  // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(&c,
                         false,  // Shift-to-right = false
                         true,   // Autopull enabled
                         32      // Autopull threshold
  );

  // We only send, so disable the TX FIFO to make the RX FIFO deeper.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

void dmd_framedetect_whitestar_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c =
      dmd_framedetect_whitestar_program_get_default_config(offset);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, RDATA, 1, false);

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, RDATA);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_INTERFACE_WHITESTAR_H
