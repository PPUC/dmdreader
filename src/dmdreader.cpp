#include "dmdreader.h"

#include <array>

#include "crc32.h"
#include "dmd_counter.h"
#include "dmd_interface.h"
#include "dmdreader_pins.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "loopback_renderer.h"
#include "spi_slave_sender.pio.h"

// should CRC32 checksum be caculated and sent with each frame
#define USE_CRC

// supress duplicate frames (implies USE_CRC)
#define SUPRESS_DUPLICATES

/**
 * Glossary
 *
 * Plane
 *  image with one bit data per pixel. This doesn't NOT mean it is stored with
 * 1bit/pixel
 *
 * Frame
 *  image with potentially more than one bit per pixel
 *
 */

#ifdef SUPRESS_DUPLICATES
#define USE_CRC
#endif

typedef struct buf32_t {
  uint8_t byte0;
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte3;
} buf32_t;

// SPI data types and header blocks
// header block length should always be a multiple of 32bit
#define SPI_BLOCK_PIX 0xcc33      // DMD frame
#define SPI_BLOCK_PIX_CRC 0x44ee  // DMD frame with CRC32 checksum

typedef struct __attribute__((__packed__)) block_header_t {
  uint16_t block_type;  // block type
  uint16_t len;         // length of the whole data including header in bytes
} block_header_t __attribute__((aligned(4)));

typedef struct __attribute__((__packed__)) block_pix_header_t {
  uint16_t columns;       // number of columns
  uint16_t rows;          // number of rows
  uint16_t bitsperpixel;  // bits per pixel
  uint16_t padding;
} block_pix_header_t __attribute__((aligned(4)));

typedef struct __attribute__((__packed__)) block_pix_crc_header_t {
  uint16_t columns;       // number of columns
  uint16_t rows;          // number of rows
  uint16_t bitsperpixel;  // bits per pixel
  uint16_t padding;
  uint32_t crc32;  // crc32 of the pixel data
} block_pix_crc_header_t __attribute__((aligned(4)));

// DMD types
enum DmdType {
  DMD_UNKNOWN,
  DMD_WPC,
  DMD_WHITESTAR,
  DMD_SPIKE1,
  DMD_SAM,
  DMD_DESEGA,
  DMD_SEGA_HD,
  DMD_GOTTLIEB,
  // CAPCOM need to be the last entries:
  DMD_CAPCOM,
  DMD_CAPCOM_HD,
};

DmdType dmd_type;

// Line oversampling
#define LINEOVERSAMPLING_NONE 1
#define LINEOVERSAMPLING_2X 2
#define LINEOVERSAMPLING_4X 4

// Merging multiple planes
#define MERGEPLANES_NONE 0
#define MERGEPLANES_ADD 0
#define MERGEPLANES_ADDSHIFT 1

// data buffer
#define MAX_WIDTH 256
#define MAX_HEIGHT 64
#define MAX_BITSPERPIXEL 4
#define MAX_PLANESPERFRAME 6
#define MAX_OVERSAMPLING LINEOVERSAMPLING_4X

// Use uint16_t for all of these variables to erase calculations:
uint16_t source_width;
uint16_t source_height;
uint16_t source_bitsperpixel;
uint16_t target_bitsperpixel;
uint16_t source_pixelsperbyte;
uint16_t source_pixelsperdword;
uint16_t source_bytes;
uint16_t target_bytes;
uint16_t source_dwords;
uint16_t source_pixelsperframe;
uint16_t source_dwordsperplane;
uint16_t source_bytesperplane;
uint16_t source_planesperframe;
uint16_t source_dwordsperframe;
uint16_t source_bytesperframe;
uint16_t source_lineoversampling;
uint16_t source_dwordsperline;
uint16_t source_mergeplanes;

// the buffers need to be aligned to 4 byte because we work with uint32_t
// pointers later. raw data read from DMD
uint8_t planebuf1[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL *
                  MAX_PLANESPERFRAME / 8] __attribute__((aligned(4)));
uint8_t planebuf2[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL *
                  MAX_PLANESPERFRAME / 8] __attribute__((aligned(4)));
uint8_t *currentPlaneBuffer = planebuf2;

// processed frame (merged planes)
uint8_t framebuf1[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL / 8 *
                  MAX_OVERSAMPLING] __attribute__((aligned(8)));
uint8_t framebuf2[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL / 8 *
                  MAX_OVERSAMPLING] __attribute__((aligned(8)));
uint8_t framebuf3[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL / 8 *
                  MAX_OVERSAMPLING] __attribute__((aligned(8)));
uint8_t *current_framebuf = framebuf1;
uint8_t *framebuf_to_send = framebuf2;

uint32_t frame_crc;
uint32_t crc_previous_frame = 0;
bool detected_0_1_0_1 = false;
bool detected_1_0_0_0 = false;
bool locked_in = false;
bool plane0_shifted = false;
bool loopback = false;

// SPI PIO
PIO spi_pio;
uint spi_sm;

// DMD reader PIO
PIO dmd_pio;
uint dmd_sm;
uint dmd_offset;
PIO frame_pio;
uint frame_sm;
uint frame_offset;

// DMA
uint dmd_dma_channel;
uint spi_dma_channel;

dma_channel_config dmd_dma_channel_cfg;
dma_channel_config spi_dma_channel_cfg;

volatile bool spi_dma_running = false;

// Interrupts
uint dmd_int = 0;

volatile bool frame_received = false;

uint8_t *renderbuf1;
uint8_t *renderbuf2;
uint8_t *current_renderbuf;
Color monochromeColor;

/**
 * @brief Send data via SPI, transfer data via DMA
 *
 * @param buf a byte buffer
 * @param len
 */
void spi_send_dma(uint32_t *buf, uint16_t len) {
  spi_dma_running = true;
  // SET DMA source address and immediately start transfer
  dma_channel_set_read_addr(spi_dma_channel, buf, false);
  dma_channel_set_trans_count(spi_dma_channel, len / 4, true);
}

/**
 * @brief Send data via SPI, using blocking IO
 *
 * @param buf a byte buffer
 * @param len
 */
void spi_send_blocking(uint32_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i += 4) {
    pio_sm_put_blocking(spi_pio, spi_sm, *buf);
    buf++;
  }
}

/**
 * @brief Check if there is still an active SPI data transfer
 *
 * @return true if there is still data in the TX FIFO
 * @return false if there is no data in the TX FIFO
 */
bool spi_busy() {
  if (!(pio_sm_is_tx_fifo_empty(spi_pio, spi_sm))) {
    return true;
  }

  if (dma_channel_is_busy(spi_dma_channel)) {
    return true;
  }

  if (spi_dma_running) {
    return true;
  }

  return false;
}

/**
 * @brief Abort running SPI transfers. This can be necessary in case the SPI
 * master hangs
 *
 */
void spi_abort() {
  if (dma_channel_is_busy(spi_dma_channel)) {
    dma_channel_abort(spi_dma_channel);
  }

  if (!(pio_sm_is_tx_fifo_empty(spi_pio, spi_sm))) {
    pio_sm_clear_fifos(spi_pio, spi_sm);
  }

  spi_dma_running = false;
}

/**
 * @brief Notify on pin SPI0_CS that data are ready on SPI
 *
 * The SPI master (the Pico is slave) should start a data transfer when this
 * signal is received It toggles pin SPI0_CS to H
 *
 */
void start_spi() { digitalWrite(SPI0_CS, HIGH); }

/**
 * @brief Set pin SPI0_CS to L to signal that there is no active SPI data
 * transfer
 *
 */
void finish_spi() { digitalWrite(SPI0_CS, LOW); }

/**
 * @brief Send a pix buffer via SPI
 *
 * @param pixbuf a frame to send
 */
bool spi_send_pix(uint8_t *pixbuf, uint32_t crc32, bool skip_when_busy) {
#ifdef USE_CRC
  block_header_t h = {.block_type = SPI_BLOCK_PIX_CRC};
  block_pix_crc_header_t ph = {};
#else
  block_header_t h = {.block_type = SPI_BLOCK_PIX};
  block_pix_header_t ph = {};
#endif

  // round length to 4-byte blocks
  h.len = (((target_bytes + 3) / 4) * 4) + sizeof(h) + sizeof(ph);
  ph.columns = source_width;
  ph.rows = source_height;
  ph.bitsperpixel = target_bitsperpixel;
#ifdef USE_CRC
  ph.crc32 = crc32;
#endif

  if (skip_when_busy) {
    if (spi_busy()) return false;
  }

  spi_send_blocking((uint32_t *)&h, sizeof(h));
  spi_send_blocking((uint32_t *)&ph, sizeof(ph));
  spi_send_dma((uint32_t *)pixbuf, target_bytes);
  start_spi();

  return true;
}

/**
 * @brief Is being called when SPI DMA transfer has finished
 *
 */
void spi_dma_handler() {
  // Clear the interrupt request
  dma_hw->ints1 = 1u << spi_dma_channel;

  finish_spi();
  spi_dma_running = false;
}

/**
 * @brief Count a clock using different PIO programs defined in dmd_counter.pio
 *
 * @return uint32_t Number of clocks per second
 */
uint32_t count_clock(uint pin) {
  uint offset;
  pio_claim_free_sm_and_add_program_for_gpio_range(
      &dmd_count_signal_program, &dmd_pio, &dmd_sm, &offset, pin, 1, true);
  dmd_counter_program_init(dmd_pio, dmd_sm, offset, pin);
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);
  delay(500);
  pio_sm_exec(dmd_pio, dmd_sm, pio_encode_in(pio_x, 32));
  uint32_t count = ~pio_sm_get(dmd_pio, dmd_sm);
  pio_sm_set_enabled(dmd_pio, dmd_sm, false);
  pio_remove_program_and_unclaim_sm(&dmd_count_signal_program, dmd_pio, dmd_sm,
                                    offset);

  return count * 2;
}

DmdType detect_dmd() {
  uint32_t dotclk = count_clock(DOTCLK);
  uint32_t de = count_clock(DE);
  uint32_t rdata = count_clock(RDATA);

  // By checking DOTCLK, DE and RDATA we can identify system types
  // All values are based on a 500ms sample of data, multiplied by 2

  // SPIKE1 -> DOTCLK: 1040000 | DE: 8150 | RDATA: 255
  if ((dotclk > 1015000) && (dotclk < 1065000) && (de > 8000) && (de < 8300) &&
      (rdata > 245) && (rdata < 265)) {
    return DMD_SPIKE1;

    // SAM -> DOTCLK: 1025000 | DE: 8000 | RDATA: 60
  } else if ((dotclk > 1000000) && (dotclk < 1050000) && (de > 7900) &&
             (de < 8100) && (rdata > 55) && (rdata < 65)) {
    return DMD_SAM;
  }
#ifndef ALPHADMD
  // WPC: DOTCLK: 500000 | DE: 3900 | RDATA: 120
  else if ((dotclk > 450000) && (dotclk < 550000) && (de > 3800) &&
           (de < 4000) && (rdata > 115) && (rdata < 130)) {
    return DMD_WPC;

    // Data East: DOTCLK: 640000 | DE: 5000 | RDATA: 80
  } else if ((dotclk > 630000) && (dotclk < 650000) && (de > 4930) &&
             (de < 5070) && (rdata > 75) && (rdata < 85)) {
    return DMD_DESEGA;

    // SEGA: DOTCLK: 640000 | DE: 5000 | RDATA: 2580
  } else if ((dotclk > 630000) && (dotclk < 650000) && (de > 4930) &&
             (de < 5070) && (rdata > 2530) && (rdata < 2630)) {
    return DMD_DESEGA;

    // SEGA HD: DOTCLK: 1836000 | DE: 14350 | RDATA: 75
  } else if ((dotclk > 1750000) && (dotclk < 1900000) && (de > 14250) &&
             (de < 14450) && (rdata > 70) && (rdata < 80)) {
    return DMD_SEGA_HD;

    // Whitestar -> DOTCLK: 657000 | DE: 5140 | RDATA: 80
  } else if ((dotclk > 645000) && (dotclk < 669000) && (de > 5075) &&
             (de < 5200) && (rdata > 75) && (rdata < 85)) {
    return DMD_WHITESTAR;

    // Gottlieb -> DOTCLK: 1647000 | DE: 12930 | RDATA: 390
  } else if ((dotclk > 1550000) && (dotclk < 1750000) && (de > 12700) &&
             (de < 13100) && (rdata > 370) && (rdata < 410)) {
    return DMD_GOTTLIEB;

    // Capcom -> DOTCLK: 4168000 | DE: 16280 | RDATA: 510
  } else if ((dotclk > 4000000) && (dotclk < 4300000) && (de > 16000) &&
             (de < 16500) && (rdata > 490) && (rdata < 530)) {
    return DMD_CAPCOM;

    // Capcom HD -> DOTCLK: 4168000 | DE: 16280 | RDATA: 255
  } else if ((dotclk > 4000000) && (dotclk < 4300000) && (de > 16000) &&
             (de < 16500) && (rdata > 240) && (rdata < 270)) {
    return DMD_CAPCOM_HD;
  }
#endif

  return DMD_UNKNOWN;
}

uint64_t convert_2bit_to_4bit_fast(uint32_t input) {
  static const uint64_t lut[4] = {0x0, 0x5, 0xA, 0xF};
  uint64_t result = 0;
  // Map pixel 0 to the most significant nibble of the first 32-bit word
  // so the nibble order matches the MSB-first converters.
  for (uint8_t i = 0; i < 8; ++i) {
    uint64_t val = lut[(input >> (30 - i * 2)) & 0x3];
    result |= val << (28 - i * 4);
  }

  // Pixels 8-15 go into the upper 32 bits, again MSB-first.
  for (uint8_t i = 8; i < 16; ++i) {
    uint64_t val = lut[(input >> (30 - i * 2)) & 0x3];
    result |= val << (60 - (i - 8) * 4);
  }

  return result;
}

// ---------------------------------
// convert_4bit_to_2bit_fast() BEGIN
// ---------------------------------

static constexpr uint8_t map_nibble(uint8_t p) { return (p > 3) ? 3 : p; }

static constexpr uint8_t make_lut_entry(uint16_t b) {
  uint8_t lo = map_nibble(uint8_t(b & 0x0F));
  uint8_t hi = map_nibble(uint8_t((b >> 4) & 0x0F));
  return uint8_t((lo << 0) | (hi << 2));  // 2 nibbles -> 4 bits (2x2bit)
}

static constexpr std::array<uint8_t, 256> kByteLut = [] {
  std::array<uint8_t, 256> a{};
  for (uint16_t i = 0; i < 256; ++i) a[i] = make_lut_entry(i);
  return a;
}();

static inline __attribute__((always_inline)) uint16_t
convert_4bit_to_2bit_fast(uint32_t input) {
  uint32_t b0 = (input >> 0) & 0xFF;
  uint32_t b1 = (input >> 8) & 0xFF;
  uint32_t b2 = (input >> 16) & 0xFF;
  uint32_t b3 = (input >> 24) & 0xFF;

  uint16_t r0 = kByteLut[b0];
  uint16_t r1 = kByteLut[b1];
  uint16_t r2 = kByteLut[b2];
  uint16_t r3 = kByteLut[b3];

  return uint16_t((r0 << 0) | (r1 << 4) | (r2 << 8) | (r3 << 12));
}

// -------------------------------
// convert_4bit_to_2bit_fast() END
// -------------------------------

void switch_buffers() {
  // Switch to next plane and frame buffers
  if (currentPlaneBuffer == planebuf1) {
    currentPlaneBuffer = planebuf2;
    current_framebuf = framebuf2;
    framebuf_to_send = framebuf1;
  } else {
    currentPlaneBuffer = planebuf1;
    current_framebuf = framebuf1;
    framebuf_to_send = framebuf2;
  }
}

/**
 * @brief
 *
 */
void dmd_set_and_enable_new_dma_target() {
  // Set the other plane buffer as new DMA transfer target
  dma_channel_set_write_addr(
      dmd_dma_channel,
      (currentPlaneBuffer == planebuf1) ? planebuf2 : planebuf1, true);

  // Clear the interrupt request, enable a new transfer
#ifdef RP2350
  dma_irqn_acknowledge_channel(3, dmd_dma_channel);
#else
  dma_channel_acknowledge_irq0(dmd_dma_channel);
#endif
}

/**
 * @brief Handles DMD DMA requests by switching between the buffers
 *
 */
void dmd_dma_handler() {
  dmd_set_and_enable_new_dma_target();

  // Required as long as CAPCOM is not locked-in:
  plane0_shifted = false;
  detected_0_1_0_1 = false;
  detected_1_0_0_0 = false;

  // Fix byte order within the buffer
  uint32_t *planebuf = (uint32_t *)currentPlaneBuffer;
  buf32_t *v;
  uint32_t res;
  for (int i = 0; i < source_dwordsperframe; i++) {
    v = (buf32_t *)planebuf;
    res = (v->byte3 << 24) | (v->byte2 << 16) | (v->byte1 << 8) | (v->byte0);
    *planebuf = res;
    planebuf++;
  }

  // Merge multiple planes to get the frame data.
  // Calculate offsets for the first pixel of each plane and cache these.
  uint16_t offset[MAX_PLANESPERFRAME];
  for (int i = 0; i < MAX_PLANESPERFRAME; i++) {
    offset[i] = i * source_dwordsperplane;
  }

  // Get a 32bit pointer to the frame buffer to handle more pixels at once.
  uint32_t *framebuf = (uint32_t *)current_framebuf;

  bool source_shiftplanesatmerge = (source_mergeplanes == MERGEPLANES_ADDSHIFT);

  planebuf = (uint32_t *)currentPlaneBuffer;
  // px represents a group of pixels stored in a double word. 8 pixels of 4bit
  // or 16 pixels of 2bit depth.
  for (int px = 0; px < source_dwordsperplane; px++) {
    uint32_t pixval = 0;
    for (int plane = 0; plane < source_planesperframe; plane++) {
      uint32_t v = planebuf[offset[plane] + px];
      if (source_shiftplanesatmerge) {
        v <<= plane;
      }
      pixval += v;
    }

    // CAPCOM is only using these patterns for four planes:
    //       0/0/0/0
    //       1/0/0/0
    //       0/1/0/1
    //       1/1/1/1
    //
    // Just two examples for false positives when searching for 1/0/0/0:
    // 0/1/0/1 0/0/0/0
    //       1/0/0/0
    // 1/1/1/1 0/0/0/0
    //       1/0/0/0
    //
    // We can be sure to be in sync if no illegal pattern occurs and if 0/1/0/1
    // and 1/0/0/0 are present. If an illegal pattern occures for a pixel, the
    // planes are out of sync and need to be shifted and no further check is
    // required for this frame.
    if (DMD_CAPCOM >= dmd_type && !locked_in && !plane0_shifted) {
      for (uint8_t p = 0; p < 32; p += 4) {
        uint8_t value = (pixval >> p) & 0x0F;
        if (value == 2 && (planebuf[px] & 0x0F) != 1 &&
            (planebuf[offset[2] + px] & 0x0F) != 1) {
          detected_0_1_0_1 = true;
        } else if (value == 1 && (planebuf[px] & 0x0F) == 1) {
          detected_1_0_0_0 = true;
        }
        // Check for illegal patterns that can happen when not in sync:
        //   1/1/1/0 => 3
        //   0/1/1/1 => 3
        //   1/0/1/1 => 3
        //   1/1/0/1 => 3
        //
        //   0/0/0/1 => 1
        //   0/0/1/0 => 1
        //   0/1/0/0 => 1
        //
        //   1/0/1/0 => 2
        //   1/1/0/0 => 2
        //   0/0/1/1 => 2
        else if (value == 3 || value > 4 ||
                 (value == 1 && (planebuf[px] & 0x0F) != 1) ||
                 (value == 2 && ((planebuf[px] & 0x0F) == 1 ||
                                 planebuf[offset[2] + px] & 0x0F) == 1)) {
          // Stop the state machine that detects frames.
          pio_sm_set_enabled(dmd_pio, frame_sm, false);
          // Start state machine again. The PIO program will skip at least one
          // plane as it is waiting for RDATA at the beginning.
          pio_sm_set_enabled(dmd_pio, frame_sm, true);
          plane0_shifted = true;
          break;
        }
      }
    }

    if (source_bitsperpixel == target_bitsperpixel || loopback) {
      framebuf[px] = pixval;
    } else if (4 == source_bitsperpixel && 2 == target_bitsperpixel) {
      uint16_t v16 = convert_4bit_to_2bit_fast(pixval);
      uint32_t out = px >> 1;  // Shifting leeds to that index steps: 0, 0, 1,
                               // 1, 2, 2, 3, 3, 4, ...
      if ((px & 1) == 0) {
        // Write first 8 pixel in upper 16 Bit.
        framebuf[out] = (uint32_t)v16 << 16;
      } else {
        // Write second 8 pixel in lower 16 Bit.
        framebuf[out] |= v16;
      }
    }
  }

  if (DMD_CAPCOM >= dmd_type && !locked_in && !plane0_shifted &&
      detected_0_1_0_1 && detected_1_0_0_0) {
    locked_in = true;
  }

  // The code below doesn't work if we reduced the bit depth above. But at the
  // moment there's no system with oversampling and bit depth reduction.
  if (source_bitsperpixel == target_bitsperpixel) {
    // deal with whitestar line oversampling directly within framebuf
    if (source_lineoversampling == LINEOVERSAMPLING_2X) {
      uint16_t i = 0;
      uint32_t *dst, *src1, *src2;
      dst = src1 = framebuf;
      src2 = src1 + source_dwordsperline;
      uint32_t v;

      for (int l = 0; l < source_height; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          v = src1[w] * 2 + src2[w];
          dst[w] = v;
        }
        src1 += source_dwordsperline * 2;  // source skips 2 lines forward
        src2 += source_dwordsperline * 2;
        dst += source_dwordsperline;  // destination skips only one line
      }
    } else if (source_lineoversampling == LINEOVERSAMPLING_4X) {
      uint16_t i = 0;
      uint32_t *dst, *src1, *src2, *src3, *src4;
      dst = src1 = framebuf;
      src2 = src1 + source_dwordsperline;
      src3 = src2 + source_dwordsperline;
      src4 = src3 + source_dwordsperline;
      uint32_t v;

      for (int l = 0; l < source_height; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          // On SAM line order is really messed up :-(
          v = src4[w] * 8 + src3[w] * 1 + src2[w] * 4 + src1[w] * 2;
          dst[w] = v;
        }
        src1 += source_dwordsperline * 4;  // source skips 4 lines forward
        src2 += source_dwordsperline * 4;
        src3 += source_dwordsperline * 4;
        src4 += source_dwordsperline * 4;
        dst += source_dwordsperline;  // destination skips only one line
      }
    }
  }

#ifdef USE_CRC
  frame_crc = crc32(0, current_framebuf, target_bytes);
#endif

  switch_buffers();

  frame_received = true;
}

void dmdreader_error_blink(bool no_error) {
  while (!no_error) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

using DmdConfigGetter = pio_sm_config (*)(uint);

void dmdreader_programs_init(const pio_program_t *dmd_reader_program,
                             DmdConfigGetter reader_get_default_config,
                             const pio_program_t *dmd_framedetect_program,
                             DmdConfigGetter framedetect_get_default_config,
                             uint *input_pins, uint8_t num_input_pins,
                             uint8_t jump_pin) {
  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      dmd_reader_program, &dmd_pio, &dmd_sm, &dmd_offset,
      (DE < SDATA) ? DE : SDATA, 5, true));
  pio_sm_config dmd_config = reader_get_default_config(dmd_offset);
  dmd_reader_program_init(dmd_pio, dmd_sm, dmd_offset, dmd_config);

  // The framedetect program just runs and detects the beginning of a new
  // frame
  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      dmd_framedetect_program, &frame_pio, &frame_sm, &frame_offset,
      (DE < SDATA) ? DE : SDATA, 5, true));
  pio_sm_config frame_config = framedetect_get_default_config(frame_offset);
  dmd_framedetect_program_init(frame_pio, frame_sm, frame_offset, frame_config,
                               input_pins, num_input_pins, jump_pin);
  pio_sm_set_enabled(frame_pio, frame_sm, true);
}

void dmdreader_init() {
  dmd_type = DMD_UNKNOWN;
  // Loop until the DMD is detected as it might need some time to be available
  // on power-on
  while (dmd_type == DMD_UNKNOWN) {
    dmd_type = detect_dmd();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  // Delay is still needed when blink gets removed above.
  // delay(1000);

  // Debug blinking to indicate the detected system:
  /*
    for (uint8_t i = 0; i < (dmd_type * 3); i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  */

  // Initialize DMD reader
  switch (dmd_type) {
    case DMD_WPC: {
      uint input_pins[] = {RDATA, DE, DOTCLK};
      dmdreader_programs_init(
          &dmd_reader_wpc_program, dmd_reader_wpc_program_get_default_config,
          &dmd_framedetect_wpc_program,
          dmd_framedetect_wpc_program_get_default_config, input_pins, 3, 0);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;
      target_bitsperpixel = 2;
      source_planesperframe = 3;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_WHITESTAR: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_whitestar_program,
          dmd_reader_whitestar_program_get_default_config,
          &dmd_framedetect_whitestar_program,
          dmd_framedetect_whitestar_program_get_default_config, input_pins, 1,
          0);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Whitestar is 2bpp
      target_bitsperpixel = 2;
      // in Whitestar, there's only one plane, containg
      // one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      // in Whitestar each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SPIKE1: {
      uint input_pins[] = {RCLK, RDATA};
      dmdreader_programs_init(&dmd_reader_spike_program,
                              dmd_reader_spike_program_get_default_config,
                              &dmd_framedetect_spike_program,
                              dmd_framedetect_spike_program_get_default_config,
                              input_pins, 2, RDATA);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 4;  // in Spike there are 4 planes
      source_lineoversampling = LINEOVERSAMPLING_NONE;  // no line oversampling
      source_mergeplanes = MERGEPLANES_ADDSHIFT;
      break;
    }

    case DMD_SAM: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_sam_program, dmd_reader_sam_program_get_default_config,
          &dmd_framedetect_sam_program,
          dmd_framedetect_sam_program_get_default_config, input_pins, 1, 0);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 1;  // in SAM there is one plane
      // with 4x line oversampling
      source_lineoversampling = LINEOVERSAMPLING_4X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_DESEGA: {
      uint input_pins[] = {DE};
      dmdreader_programs_init(&dmd_reader_desega_program,
                              dmd_reader_desega_program_get_default_config,
                              &dmd_framedetect_desega_program,
                              dmd_framedetect_desega_program_get_default_config,
                              input_pins, 1, DE);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Data East and Sega are 2bpp
      target_bitsperpixel = 2;
      // in DE-Sega, there's only one plane,
      // containg one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      // in DE-Sega each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SEGA_HD: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_sega_hd_program,
          dmd_reader_sega_hd_program_get_default_config,
          &dmd_framedetect_sega_hd_program,
          dmd_framedetect_sega_hd_program_get_default_config, input_pins, 1, 0);

      source_width = 192;
      source_height = 64;
      source_bitsperpixel = 2;  // Data East and Sega are 2bpp
      target_bitsperpixel = 2;
      // in DE-Sega, there's only one plane,
      // containg one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      // in DE-Sega each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_GOTTLIEB: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_gottlieb_program,
          dmd_reader_gottlieb_program_get_default_config,
          &dmd_framedetect_gottlieb_program,
          dmd_framedetect_gottlieb_program_get_default_config, input_pins, 1,
          0);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 6;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_CAPCOM: {
      uint input_pins[] = {RDATA, RCLK};
      dmdreader_programs_init(&dmd_reader_capcom_program,
                              dmd_reader_capcom_program_get_default_config,
                              &dmd_framedetect_capcom_program,
                              dmd_framedetect_capcom_program_get_default_config,
                              input_pins, 2, 0);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 4;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_CAPCOM_HD: {
      uint input_pins[] = {RDATA, RCLK};
      dmdreader_programs_init(
          &dmd_reader_capcom_hd_program,
          dmd_reader_capcom_hd_program_get_default_config,
          &dmd_framedetect_capcom_hd_program,
          dmd_framedetect_capcom_hd_program_get_default_config, input_pins, 2,
          0);

      source_width = 256;
      source_height = 64;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 4;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }
  }

  // Calculate display parameters
  source_pixelsperbyte = 8 / source_bitsperpixel;
  source_pixelsperdword = 4 * source_pixelsperbyte;
  source_bytes = source_width * source_height * source_bitsperpixel / 8;
  target_bytes = source_width * source_height * target_bitsperpixel / 8;
  source_pixelsperframe = source_width * source_height;
  source_dwords = source_bytes / 4;
  source_dwordsperplane = source_bytes / 4;
  if (source_lineoversampling == LINEOVERSAMPLING_2X) {
    source_dwordsperplane *= 2;
  } else if (source_lineoversampling == LINEOVERSAMPLING_4X) {
    source_dwordsperplane *= 4;
  }
  source_bytesperplane = source_bytes;
  source_dwordsperframe = source_dwordsperplane * source_planesperframe;
  source_bytesperframe = source_bytesperplane * source_planesperframe;
  source_dwordsperline = source_width * source_bitsperpixel / 32;

  // DMA for DMD reader
  dmd_dma_channel = dma_claim_unused_channel(true);
  dmd_dma_channel_cfg = dma_channel_get_default_config(dmd_dma_channel);
  channel_config_set_read_increment(&dmd_dma_channel_cfg, false);
  channel_config_set_write_increment(&dmd_dma_channel_cfg, true);
  channel_config_set_dreq(&dmd_dma_channel_cfg,
                          pio_get_dreq(dmd_pio, dmd_sm, false));

  // Configure the DMA channel. As soon as the PIO pushed a specified number
  // of words to its RX FIFO, the DMA transfer will be triggered. The amount
  // of words to transfer is source_dwordsperframe.
  dma_channel_configure(dmd_dma_channel, &dmd_dma_channel_cfg,
                        NULL,  // Destination pointer, needs to be set later
                        &dmd_pio->rxf[dmd_sm],  // Source pointer
                        source_dwordsperframe,  // Number of transfers
                        false                   // Do not yet start
  );

  // Enable DMA interrupt to be triggered when the transfer is done.
#ifdef RP2350
  dma_irqn_set_channel_enabled(3, dmd_dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_3, dmd_dma_handler);
  irq_set_enabled(DMA_IRQ_3, true);
#else
  dma_channel_set_irq0_enabled(dmd_dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dmd_dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);
#endif

  // Finally start DMD reader PIO program and DMA
  dmd_set_and_enable_new_dma_target();
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);
}

void dmdreader_spi_init() {
  loopback = false;
  // this is used to notify the Pi that data is available
  pinMode(SPI0_CS, OUTPUT);
  digitalWrite(SPI0_CS, LOW);

  // initialize SPI slave PIO
  uint offset;
  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      &clocked_output_program, &spi_pio, &spi_sm, &offset, SPI_BASE, 4, true));
  clocked_output_program_init(spi_pio, spi_sm, offset, SPI_BASE);

  // DMA for SPI
  spi_dma_channel = dma_claim_unused_channel(true);
  spi_dma_channel_cfg = dma_channel_get_default_config(spi_dma_channel);
  channel_config_set_read_increment(&spi_dma_channel_cfg, true);
  channel_config_set_write_increment(&spi_dma_channel_cfg, false);
  channel_config_set_dreq(&spi_dma_channel_cfg,
                          pio_get_dreq(spi_pio, spi_sm, true));

  dma_channel_configure(spi_dma_channel, &spi_dma_channel_cfg,
                        &spi_pio->txf[spi_sm],  // Destination pointer
                        NULL,                   // Source pointer
                        0,                      // Number of transfers
                        false                   // Do not yet start
  );

  dma_channel_set_irq1_enabled(spi_dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_handler);
  irq_set_enabled(DMA_IRQ_1, true);
}

bool dmdreader_spi_send() {
  if (frame_received) {
    frame_received = false;

#ifdef SUPRESS_DUPLICATES
    if (frame_crc != crc_previous_frame) {
      spi_send_pix(framebuf_to_send, frame_crc, true);
      crc_previous_frame = frame_crc;
    }
#else
    spi_send_pix(framebuf_to_send, frame_crc, true);
#endif

    return true;
  }

  return false;
}

void dmdreader_loopback_init(uint8_t *buffer1, uint8_t *buffer2, Color color) {
  renderbuf1 = buffer1;
  renderbuf2 = buffer2;
  current_renderbuf = renderbuf1;
  monochromeColor = color;
  loopback = true;
}

uint8_t *dmdreader_loopback_render() {
  uint64_t *frame4bit = (uint64_t *)framebuf3;

  if (loopback && frame_received) {
    frame_received = false;
    if (frame_crc != crc_previous_frame) {
      if (current_renderbuf == renderbuf1) {
        current_renderbuf = renderbuf2;
      } else {
        current_framebuf = renderbuf1;
      }

      auto func =
          get_optimized_converter(source_width, source_height, monochromeColor);
      if (func) {
        if (2 == source_bitsperpixel) {
          for (uint16_t i = 0; i < source_dwords; i++) {
            frame4bit[i] =
                convert_2bit_to_4bit_fast(((uint32_t *)framebuf_to_send)[i]);
          }
          func((uint32_t *)frame4bit, current_renderbuf);
        } else {
          func((uint32_t *)framebuf_to_send, current_renderbuf);
        }
      }

      crc_previous_frame = frame_crc;

      return current_renderbuf;
    }
  }

  return nullptr;
}
