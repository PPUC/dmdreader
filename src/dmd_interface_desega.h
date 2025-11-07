#ifndef DMD_INTERFACE_DESEGA_H
#define DMD_INTERFACE_DESEGA_H

#include "dmd_interface_desega.pio.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

static inline void dmd_reader_desega_program_init(PIO pio, uint sm,
                                                  uint offset) {
  pio_sm_config c = dmd_reader_desega_program_get_default_config(offset);

  // Set the IN pin, we don't use any other
  sm_config_set_in_pins(&c, desega_SDATA);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, desega_SDATA, 2,
                                 false);  // SDATA, DOTCLK

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, desega_DOTCLK);
  pio_gpio_init(pio, desega_SDATA);

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

static inline void dmd_framedetect_desega_program_init(PIO pio, uint sm,
                                                       uint offset) {
  pio_sm_config c = dmd_framedetect_desega_program_get_default_config(offset);
  // DE is used for jump control
  sm_config_set_jmp_pin(&c, desega_DE);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, desega_DE, 1, false);

  // Connect this GPIO to this PIO block
  pio_gpio_init(pio, desega_DE);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_INTERFACE_DESEGA_H
