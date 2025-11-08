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

void dmd_reader_program_init(PIO pio, uint sm, pio_sm_config c) {
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

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

void dmd_framedetect_desega_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c = dmd_framedetect_desega_program_get_default_config(offset);
  // DE is used for jump control
  sm_config_set_jmp_pin(&c, DE);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, DE, 1, false);

  // Connect this GPIO to this PIO block
  pio_gpio_init(pio, DE);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

void dmd_framedetect_whitestar_sam_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c =
      dmd_framedetect_whitestar_program_get_default_config(offset);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, RDATA, 1, false);

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, RDATA);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

void dmd_framedetect_spike_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c = dmd_framedetect_spike_program_get_default_config(offset);
  // RDATA is used for jump control
  sm_config_set_jmp_pin(&c, RDATA);

  // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(&c,
                         false,  // Shift-to-right = false
                         false,  // No autopull, we don't read data from this SM
                         32      // Autopull threshold
  );

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, RCLK, 2, false);  // RCLK, RDATA

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, RDATA);
  pio_gpio_init(pio, RCLK);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

void dmd_framedetect_wpc_program_init(PIO pio, uint sm, uint offset) {
  pio_sm_config c = dmd_framedetect_wpc_program_get_default_config(offset);

  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, DE);
  pio_gpio_init(pio, RDATA);
  pio_gpio_init(pio, DOTCLK);

  // Set the pin direction at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, RDATA, 2, false);  // RDATA, DE
  pio_sm_set_consecutive_pindirs(pio, sm, DOTCLK, 1, false);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_INTERFACE_H
