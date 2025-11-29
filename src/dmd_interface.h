#ifndef DMD_INTERFACE_H
#define DMD_INTERFACE_H

#include "dmd_interface_desega.pio.h"
#include "dmd_interface_sam.pio.h"
#include "dmd_interface_spike.pio.h"
#include "dmd_interface_whitestar.pio.h"
#include "dmd_interface_wpc.pio.h"
#include "dmd_reader_pins.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

// Init the DMD reader (dots) PIO program, common for all DMD types.
void dmd_reader_program_init(PIO pio, uint sm, uint offset, pio_sm_config c) {
  // Set the IN pin, we don't use any other
  sm_config_set_in_pins(&c, SDATA);

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, DOTCLK);
  pio_gpio_init(pio, SDATA);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, SDATA, 2, false);  // SDATA, DOTCLK

  // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(&c,
                         false,  // Shift-to-right = false
                         true,   // Autopull enabled
                         32      // Autopull threshold
  );

  // We only send, so disable the TX FIFO to make the RX FIFO deeper.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  // For the 200MHz clock
  sm_config_set_clkdiv(&c, 1.6f);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

// Init the framedetect PIO program.
void dmd_framedetect_program_init(PIO pio, uint sm, uint offset,
                                  pio_sm_config c, const uint* input_pins,
                                  uint num_input_pins, uint jump_pin) {
  //if (jump_pin > 0) {
    // Pin is used for jump control
  sm_config_set_jmp_pin(&c, 6);
  //}

    // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(
      &c,
      false, // Shift-to-right = false
      false, // No autopull, we don't read data from this SM
      32     // Autopull threshold
  );
  //for (uint i = 0; i < num_input_pins; i++) {
    // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, 6, 1, false);
    // Connect that GPIO to this PIO block
  pio_gpio_init(pio, 6);
  //}

  // For the 200MHz clock
  sm_config_set_clkdiv(&c, 1.6f);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_INTERFACE_H
