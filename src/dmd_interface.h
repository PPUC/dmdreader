#ifndef DMD_INTERFACE_H
#define DMD_INTERFACE_H

#ifdef ALPHADMD
#include "dmd_interface_sam_alphadmd.pio.h"
#include "dmd_interface_spike_alphadmd.pio.h"
#else
#include "dmd_interface_sam.pio.h"
#include "dmd_interface_spike.pio.h"
#endif
#include "dmd_interface_alving.pio.h"
#include "dmd_interface_capcom.pio.h"
#include "dmd_interface_capcom_hd.pio.h"
#include "dmd_interface_de_x16.pio.h"
#include "dmd_interface_desega.pio.h"
#include "dmd_interface_gottlieb.pio.h"
#include "dmd_interface_sega_hd.pio.h"
#include "dmd_interface_whitestar.pio.h"
#include "dmd_interface_wpc.pio.h"
#include "dmdreader_pins.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

// Init the DMD reader (dots) PIO program, common for all DMD types.
void dmd_reader_program_init(PIO pio, uint sm, uint offset, pio_sm_config c, uint in_base_pin) {
 
  if(in_base_pin == SDATA_X16) {
    // We need to set DOTCLK as jump pin + additional SDATA line as base in pin
    sm_config_set_jmp_pin(&c, DOTCLK);
    sm_config_set_in_pins(&c, in_base_pin);

    pio_gpio_init(pio, SDATA_X16);         // Extra data line for Data East X16
    pio_gpio_init(pio, SDATA_X16_PADDING); // used as a padding 0 bit
    // pio_sm_exec(pio, sm, pio_encode_out(pio_osr, 8192));
    // pio_encode_mov();

    pio_sm_set_consecutive_pindirs(pio, sm, SDATA_X16, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, SDATA_X16_PADDING, 1, false);
  } else {
    sm_config_set_in_pins(&c, in_base_pin);
  }
  // Connect these GPIOs to this PIO block
  pio_gpio_init(pio, SDATA);
  pio_gpio_init(pio, DOTCLK);

  // Set the pin direction at the PIO, handle pins seprately to support alphaDMD
  // as well
  pio_sm_set_consecutive_pindirs(pio, sm, SDATA, 1, false);
  pio_sm_set_consecutive_pindirs(pio, sm, DOTCLK, 1, false);

  // Shifting to left matches the customary MSB-first ordering of SPI.
  sm_config_set_in_shift(&c,
                         false,  // shift-to-right = false
                         true,   // autopush enabled
                         32      // autopush threshold
  );

  // We only send, so disable the TX FIFO to make the RX FIFO deeper.
  // sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

// Init the framedetect PIO program.
void dmd_framedetect_program_init(PIO pio, uint sm, uint offset,
                                  pio_sm_config c, const uint* input_pins,
                                  uint num_input_pins, uint jump_pin) {
  if (jump_pin > 0) {
    // Pin is used for jump control
    sm_config_set_jmp_pin(&c, jump_pin);
  }

  for (uint i = 0; i < num_input_pins; i++) {
    // Connect that GPIO to this PIO block
    pio_gpio_init(pio, input_pins[i]);
    // Set the pin direction at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, input_pins[i], 1, false);
  }

  sm_config_set_in_shift(&c,
                         false,  // shift-to-right = false
                         false,  // no autopush
                         0);

  // Derive divider from current clock so PIO runs at ~125 MHz reference
  uint32_t sys_hz = clock_get_hz(clk_sys);    // e.g. 125/200/266 MHz
  float target_hz = 125000000.0f;             // PIO code designed for 125 MHz
  float divider = (float)sys_hz / target_hz;  // scales automatically
  sm_config_set_clkdiv(&c, divider);

  // Load our configuration, do not yet start the program
  pio_sm_init(pio, sm, offset, &c);
}

#endif  // DMD_INTERFACE_H
