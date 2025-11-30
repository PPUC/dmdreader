#include "dmdreader.h"

#include "crc32.h"
#include "dmd_counter.h"
#include "dmd_interface.h"
#include "dmd_reader_pins.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
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
} block_header_t;

typedef struct __attribute__((__packed__)) block_pix_header_t {
  uint16_t columns;       // number of columns
  uint16_t rows;          // number of rows
  uint16_t bitsperpixel;  // bits per pixel
  uint16_t padding;
} block_pix_header_t;

typedef struct __attribute__((__packed__)) block_pix_crc_header_t {
  uint16_t columns;       // number of columns
  uint16_t rows;          // number of rows
  uint16_t bitsperpixel;  // bits per pixel
  uint16_t padding;
  uint32_t crc32;  // crc32 of the pixel data
} block_pix_crc_header_t;

// DMD types
#define DMD_UNKNOWN 0
#define DMD_WPC 1
#define DMD_WHITESTAR 2
#define DMD_SPIKE1 3
#define DMD_SAM 4
#define DMD_DESEGA 5

// Line oversampling
#define LINEOVERSAMPLING_NONE 1
#define LINEOVERSAMPLING_2X 2
#define LINEOVERSAMPLING_4X 4

// Merging multiple planes
#define MERGEPLANES_NONE 0
#define MERGEPLANES_ADD 0
#define MERGEPLANES_ADDSHIFT 1

// data buffer
#define MAX_WIDTH 192
#define MAX_HEIGHT 64
#define MAX_BITSPERPIXEL 4
#define MAX_PLANESPERFRAME 4
#define MAX_MEMORY_OVERHEAD \
  4  // reserve additional memory in framebuf for line oversampling

uint16_t source_width;
uint16_t source_height;
uint16_t source_bitsperpixel;
uint16_t source_pixelsperbyte;
uint16_t source_bytes;
uint16_t source_pixelsperframe;
uint16_t source_wordsperplane;
uint16_t source_bytesperplane;
uint16_t source_planesperframe;
uint16_t source_wordsperframe;
uint16_t source_bytesperframe;
uint16_t source_lineoversampling;
uint16_t source_wordsperline;
uint8_t source_mergeplanes;

// raw data read from DMD
uint8_t planebuf1[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL *
                  MAX_PLANESPERFRAME / 8];
uint8_t planebuf2[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL *
                  MAX_PLANESPERFRAME / 8];
uint8_t *lastplane = planebuf2;

// processed frames (merged planes)
uint8_t framebuf1[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL / 8 *
                  MAX_MEMORY_OVERHEAD];
uint8_t framebuf2[MAX_WIDTH * MAX_HEIGHT * MAX_BITSPERPIXEL / 8 *
                  MAX_MEMORY_OVERHEAD];
uint32_t crc1;
uint32_t crc2;
uint8_t *lastframe = framebuf1;
uint32_t *lastcrc;

uint32_t stat_frames_received = 0;
uint32_t stat_spi_skipped = 0;

// SPI PIO
PIO spi_pio;
uint spi_sm;

// DMD reader PIO
PIO dmd_pio;
uint dmd_sm;

// Frame detection PIO
PIO frame_pio;
uint frame_sm;

// DMA
uint dmd_dma_chan = 0;
uint spi_dma_chan = 1;

dma_channel_config dmd_dma_chan_cfg;
dma_channel_config spi_dma_chan_cfg;

volatile bool spi_dma_running = false;

// Interrupts
uint dmd_int = 0;

volatile bool frame_received = false;

/**
 * @brief Send data via SPI, transfer data via DMA
 *
 * @param buf a byte buffer
 * @param len
 */
void spi_send_dma(uint32_t *buf, uint16_t len) {
  spi_dma_running = true;
  // SET DMA source address and immediately start transfer
  dma_channel_set_read_addr(spi_dma_chan, buf, false);
  dma_channel_set_trans_count(spi_dma_chan, len / 4, true);
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

  if (dma_channel_is_busy(spi_dma_chan)) {
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
  if (dma_channel_is_busy(spi_dma_chan)) {
    dma_channel_abort(spi_dma_chan);
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
  h.len = (((source_bytes + 3) / 4) * 4) + sizeof(h) + sizeof(ph);
  ph.columns = source_width;
  ph.rows = source_height;
  ph.bitsperpixel = source_bitsperpixel;
#ifdef USE_CRC
  ph.crc32 = crc32;
#endif

  if (skip_when_busy) {
    if (spi_busy()) return false;
  }

  spi_send_blocking((uint32_t *)&h, sizeof(h));
  spi_send_blocking((uint32_t *)&ph, sizeof(ph));
  spi_send_dma((uint32_t *)pixbuf, source_bytes);
  start_spi();

  return true;
}

/**
 * @brief Count a clock using different PIO programs defined in dmd_counter.pio
 *
 * @return uint32_t Number of clocks per second
 */
uint32_t count_clock(uint pin) {
  dmd_pio = pio0;
  uint offset = pio_add_program(dmd_pio, &dmd_count_signal_program);
  dmd_sm = pio_claim_unused_sm(dmd_pio, true);
  dmd_counter_program_init(dmd_pio, dmd_sm, offset, pin);
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);
  delay(500);
  pio_sm_exec(dmd_pio, dmd_sm, pio_encode_in(pio_x, 32));
  uint32_t count = ~pio_sm_get(dmd_pio, dmd_sm);
  pio_sm_set_enabled(dmd_pio, dmd_sm, false);
  pio_remove_program(dmd_pio, &dmd_count_signal_program, offset);
  pio_sm_unclaim(dmd_pio, dmd_sm);

  return count * 2;
}

int detect_dmd() {
  uint32_t dotclk = count_clock(DOTCLK);
  uint32_t de = count_clock(DE);
  uint32_t rdata = count_clock(RDATA);

  // By checking DOTCLK, DE and RDATA we can identify system types
  // All values are based on a 500ms sample of data, multiplied by 2

  // WPC: DOTCLK: 500000 | DE: 3900 | RDATA: 120
  if ((dotclk > 450000) && (dotclk < 550000) && (de > 3800) && (de < 4000) &&
      (rdata > 115) && (rdata < 130)) {
    return DMD_WPC;

    // Data East: DOTCLK: 640000 | DE: 5000 | RDATA: 80
  } else if ((dotclk > 630000) && (dotclk < 650000) && (de > 4930) &&
             (de < 5070) && (rdata > 75) && (rdata < 85)) {
    return DMD_DESEGA;

    // SEGA: DOTCLK: 640000 | DE: 5000 | RDATA: 2580
  } else if ((dotclk > 630000) && (dotclk < 650000) && (de > 4930) &&
             (de < 5070) && (rdata > 2530) && (rdata < 2630)) {
    return DMD_DESEGA;

    // Whitestar -> DOTCLK: 657000 | DE: 5140 | RDATA: 80
  } else if ((dotclk > 645000) && (dotclk < 669000) && (de > 5075) &&
             (de < 5200) && (rdata > 75) && (rdata < 85)) {
    return DMD_WHITESTAR;

    // SPIKE1 -> DOTCLK: 1040000 | DE: 8150 | RDATA: 255
  } else if ((dotclk > 1015000) && (dotclk < 1065000) && (de > 8000) &&
             (de < 8300) && (rdata > 245) && (rdata < 265)) {
    return DMD_SPIKE1;

    // SAM -> DOTCLK: 1025000 | DE: 8000 | RDATA: 60
  } else if ((dotclk > 1000000) && (dotclk < 1050000) && (de > 7900) &&
             (de < 8100) && (rdata > 55) && (rdata < 65)) {
    return DMD_SAM;
  }

  return DMD_UNKNOWN;
}

/**
 * @brief Is being called when SPI DMA transfer has finished
 *
 */
void spi_dma_handler() {
  // Clear the interrupt request
  dma_hw->ints1 = 1u << spi_dma_chan;

  finish_spi();
  spi_dma_running = false;
}

/**
 * @brief Handles DMD DMA requests by switching between the buffers
 *
 */
void dmd_dma_handler() {
  uint8_t *target;
  uint32_t *targetcrc;
  uint32_t *planebuf;

  // Switch between buffers
  if (lastplane == planebuf1) {
    target = planebuf1;
    lastplane = planebuf2;
    lastframe = framebuf2;
    lastcrc = &crc2;
    targetcrc = &crc1;
  } else {
    target = planebuf2;
    lastplane = planebuf1;
    lastframe = framebuf1;
    lastcrc = &crc1;
    targetcrc = &crc2;
  }

  // Clear the interrupt request
  dma_hw->ints0 = 1u << dmd_dma_chan;

  // Start a new DMA transfer to the new buffer
  dma_channel_set_write_addr(dmd_dma_chan, target, true);

  // Just for debugging purposes
  stat_frames_received++;

  // Fix byte order within the buffer
  planebuf = (uint32_t *)lastplane;
  buf32_t *v;
  uint32_t res;
  for (int i = 0; i < source_wordsperframe; i++) {
    v = (buf32_t *)planebuf;
    res = (v->byte3 << 24) | (v->byte2 << 16) | (v->byte1 << 8) | (v->byte0);
    *planebuf = res;
    planebuf++;
  }

  // Merge multiple planes

  // add all planes to get the frame data
  uint32_t *framebuf = (uint32_t *)lastframe;
  // calculate offsets for each plane and cache these
  uint16_t offset[MAX_PLANESPERFRAME];
  for (int i = 0; i < MAX_PLANESPERFRAME; i++) {
    offset[i] = i * source_wordsperplane;
  }

  bool source_shiftplanesatmerge = (source_mergeplanes == MERGEPLANES_ADDSHIFT);

  planebuf = (uint32_t *)lastplane;
  for (int px = 0; px < source_wordsperplane; px++) {
    uint32_t pixval = 0;
    for (int plane = 0; plane < source_planesperframe; plane++) {
      uint32_t v = planebuf[offset[plane] + px];
      if (source_shiftplanesatmerge) {
        v <<= plane;
      }
      pixval += v;
    }
    framebuf[px] = pixval;
  }

  // deal with whitestar line oversampling directly within framebuf
  if (source_lineoversampling == LINEOVERSAMPLING_2X) {
    uint16_t i = 0;
    uint32_t *dst, *src1, *src2;
    dst = src1 = framebuf;
    src2 = framebuf + source_wordsperline;
    uint32_t v;

    for (int l = 0; l < source_height; l++) {
      for (int w = 0; w < source_wordsperline; w++) {
        v = src1[w] * 2 + src2[w];
        dst[w] = v;
      }
      src1 += source_wordsperline * 2;  // source skips 2 lines forward
      src2 += source_wordsperline * 2;
      dst += source_wordsperline;  // destination skips only one line
    }
  } else if (source_lineoversampling == LINEOVERSAMPLING_4X) {
    uint16_t i = 0;
    uint32_t *dst, *src1, *src2, *src3, *src4;
    dst = src1 = framebuf;
    src2 = src1 + source_wordsperline;
    src3 = src2 + source_wordsperline;
    src4 = src3 + source_wordsperline;
    uint32_t v;

    for (int l = 0; l < source_height; l++) {
      for (int w = 0; w < source_wordsperline; w++) {
        // On SAM line order is really messed up :-(
        v = src4[w] * 8 + src3[w] * 1 + src2[w] * 4 + src1[w] * 2;
        dst[w] = v;
      }
      src1 += source_wordsperline * 4;  // source skips 4 lines forward
      src2 += source_wordsperline * 4;
      src3 += source_wordsperline * 4;
      src4 += source_wordsperline * 4;
      dst += source_wordsperline;  // destination skips only one line
    }
  }
#ifdef USE_CRC
  *lastcrc = crc32(0, (const uint8_t *)framebuf, source_bytes);
#endif

  frame_received = true;
}

void dmdreader_init() {
  // this is used to notify the Pi that data is available
  pinMode(SPI0_CS, OUTPUT);
  digitalWrite(SPI0_CS, LOW);

  int dmd_type = DMD_UNKNOWN;
  // Loop until the DMD is detected as it might need some time to be available
  // on power-on
  while (dmd_type == DMD_UNKNOWN) {
    dmd_type = detect_dmd();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  // Delay is still needed if blink gets removed above.
  delay(1000);

  for (uint8_t i = 0; i < dmd_type; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  uint offset;

  // Initialize DMD reader
  switch (dmd_type) {
    case DMD_WPC: {
      dmd_pio = pio0;
      offset = pio_add_program(dmd_pio, &dmd_reader_wpc_program);
      dmd_sm = pio_claim_unused_sm(dmd_pio, true);
      pio_sm_config dmd_config =
          dmd_reader_wpc_program_get_default_config(offset);
      dmd_reader_program_init(dmd_pio, dmd_sm, offset, dmd_config);

      // The framedetect program just runs and detects the beginning of a new
      // frame
      uint input_pins[] = {RDATA, DE, DOTCLK};
      frame_pio = pio0;
      offset = pio_add_program(frame_pio, &dmd_framedetect_wpc_program);
      frame_sm = pio_claim_unused_sm(frame_pio, true);
      pio_sm_config frame_config =
          dmd_framedetect_wpc_program_get_default_config(offset);
      dmd_framedetect_program_init(frame_pio, frame_sm, offset, frame_config,
                                   input_pins, 3, 0);
      pio_sm_set_enabled(frame_pio, frame_sm, true);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;
      source_pixelsperbyte = 8 / source_bitsperpixel;
      source_planesperframe = 3;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_WHITESTAR: {
      dmd_pio = pio0;
      offset = pio_add_program(dmd_pio, &dmd_reader_whitestar_program);
      dmd_sm = pio_claim_unused_sm(dmd_pio, true);
      pio_sm_config dmd_config =
          dmd_reader_whitestar_program_get_default_config(offset);
      dmd_reader_program_init(dmd_pio, dmd_sm, offset, dmd_config);

      // The framedetect program just runs and detects the beginning of a new
      // frame
      uint input_pins[] = {RDATA};
      frame_pio = pio0;
      offset = pio_add_program(frame_pio, &dmd_framedetect_whitestar_program);
      frame_sm = pio_claim_unused_sm(frame_pio, true);
      pio_sm_config frame_config =
          dmd_framedetect_whitestar_program_get_default_config(offset);
      dmd_framedetect_program_init(frame_pio, frame_sm, offset, frame_config,
                                   input_pins, 1, 0);
      pio_sm_set_enabled(frame_pio, frame_sm, true);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Whitestar is 2bpp
      source_pixelsperbyte = 8 / source_bitsperpixel;
      // in Whitestar, there's only one plane, containg
      // one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      // in Whitestar each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SPIKE1: {
      dmd_pio = pio0;
      offset = pio_add_program(dmd_pio, &dmd_reader_spike_program);
      dmd_sm = pio_claim_unused_sm(dmd_pio, true);
      pio_sm_config dmd_config =
          dmd_reader_spike_program_get_default_config(offset);
      dmd_reader_program_init(dmd_pio, dmd_sm, offset, dmd_config);

      // The framedetect program just runs and detects the beginning of a new
      // frame
      uint input_pins[] = {RCLK, RDATA};
      frame_pio = pio0;
      offset = pio_add_program(frame_pio, &dmd_framedetect_spike_program);
      frame_sm = pio_claim_unused_sm(frame_pio, true);
      pio_sm_config frame_config =
          dmd_framedetect_spike_program_get_default_config(offset);
      dmd_framedetect_program_init(frame_pio, frame_sm, offset, frame_config,
                                   input_pins, 2, RDATA);
      pio_sm_set_enabled(frame_pio, frame_sm, true);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      source_pixelsperbyte = 8 / source_bitsperpixel;
      source_planesperframe = 4;  // in Spike there are 4 planes
      source_lineoversampling = LINEOVERSAMPLING_NONE;  // no line oversampling
      source_mergeplanes = MERGEPLANES_ADDSHIFT;
      break;
    }

    case DMD_SAM: {
      dmd_pio = pio0;
      offset = pio_add_program(dmd_pio, &dmd_reader_sam_program);
      dmd_sm = pio_claim_unused_sm(dmd_pio, true);
      pio_sm_config dmd_config =
          dmd_reader_sam_program_get_default_config(offset);
      dmd_reader_program_init(dmd_pio, dmd_sm, offset, dmd_config);

      // The framedetect program just runs and detects the beginning of a new
      // frame
      uint input_pins[] = {RDATA};
      frame_pio = pio0;
      offset = pio_add_program(frame_pio, &dmd_framedetect_sam_program);
      frame_sm = pio_claim_unused_sm(frame_pio, true);
      pio_sm_config frame_config =
          dmd_framedetect_sam_program_get_default_config(offset);
      dmd_framedetect_program_init(frame_pio, frame_sm, offset, frame_config,
                                   input_pins, 1, 0);
      pio_sm_set_enabled(frame_pio, frame_sm, true);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      source_pixelsperbyte = 8 / source_bitsperpixel;
      source_planesperframe = 1;  // in SAM there is one plane
      // with 4x line oversampling
      source_lineoversampling = LINEOVERSAMPLING_4X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_DESEGA: {
      dmd_pio = pio0;
      offset = pio_add_program(dmd_pio, &dmd_reader_desega_program);
      dmd_sm = pio_claim_unused_sm(dmd_pio, true);
      pio_sm_config dmd_config =
          dmd_reader_desega_program_get_default_config(offset);
      dmd_reader_program_init(dmd_pio, dmd_sm, offset, dmd_config);

      // The framedetect program just runs and detects the beginning of a new
      // frame
      uint input_pins[] = {DE};
      frame_pio = pio0;
      offset = pio_add_program(frame_pio, &dmd_framedetect_desega_program);
      frame_sm = pio_claim_unused_sm(frame_pio, true);
      pio_sm_config frame_config =
          dmd_framedetect_desega_program_get_default_config(offset);
      dmd_framedetect_program_init(frame_pio, frame_sm, offset, frame_config,
                                   input_pins, 1, DE);
      pio_sm_set_enabled(frame_pio, frame_sm, true);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Data East and Sega are 2bpp
      source_pixelsperbyte = 8 / source_bitsperpixel;
      // in DE-Sega, there's only one plane,
      // containg one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      // in DE-Sega each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }
  }

  // Calculate display parameters
  source_bytes = source_width * source_height * source_bitsperpixel / 8;
  source_pixelsperframe = source_width * source_height;
  source_wordsperplane = source_bytes / 4;
  if (source_lineoversampling == LINEOVERSAMPLING_2X) {
    source_wordsperplane *= 2;
  } else if (source_lineoversampling == LINEOVERSAMPLING_4X) {
    source_wordsperplane *= 4;
  }
  source_bytesperplane = source_bytes;
  source_wordsperframe = source_wordsperplane * source_planesperframe;
  source_bytesperframe = source_bytesperplane * source_planesperframe;
  source_wordsperline = source_width * source_bitsperpixel / 32;

  // DMA for DMD reader
  dmd_dma_chan_cfg = dma_channel_get_default_config(dmd_dma_chan);
  channel_config_set_read_increment(&dmd_dma_chan_cfg, false);
  channel_config_set_write_increment(&dmd_dma_chan_cfg, true);
  channel_config_set_dreq(&dmd_dma_chan_cfg,
                          pio_get_dreq(dmd_pio, dmd_sm, false));

  // Configure the DMA channel. As soon as the PIO pushed a specified number of
  // words to its RX FIFO, the DMA transfer will be triggered.
  // The amount of words to transfer is source_wordsperframe.
  dma_channel_configure(dmd_dma_chan, &dmd_dma_chan_cfg,
                        NULL,  // Destination pointer, needs to be set later
                        &dmd_pio->rxf[dmd_sm],  // Source pointer
                        source_wordsperframe,   // Number of transfers
                        false                   // Do not yet start
  );
  // Enable DMA interrupt 0 to be triggered when the transfer is done.
  dma_channel_set_irq0_enabled(dmd_dma_chan, true);
  // Set the IRQ handler function.
  irq_set_exclusive_handler(DMA_IRQ_0, dmd_dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  // initialize SPI slave PIO
  spi_pio = pio0;
  offset = pio_add_program(spi_pio, &clocked_output_program);
  spi_sm = pio_claim_unused_sm(spi_pio, true);
  clocked_output_program_init(spi_pio, spi_sm, offset, SPI_BASE);

  // DMA for SPI
  spi_dma_chan_cfg = dma_channel_get_default_config(spi_dma_chan);
  channel_config_set_read_increment(&spi_dma_chan_cfg, true);
  channel_config_set_write_increment(&spi_dma_chan_cfg, false);
  channel_config_set_dreq(&spi_dma_chan_cfg,
                          pio_get_dreq(spi_pio, spi_sm, true));

  dma_channel_configure(spi_dma_chan, &spi_dma_chan_cfg,
                        &spi_pio->txf[spi_sm],  // Destination pointer
                        NULL,                   // Source pointer
                        0,                      // Number of transfers
                        false                   // Do not yet start
  );
  digitalWrite(LED_BUILTIN, HIGH);

  dma_channel_set_irq1_enabled(spi_dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_handler);
  irq_set_enabled(DMA_IRQ_1, true);

  // Finally start DMD reader PIO program and DMA
  dmd_dma_handler();
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);
}

void dmdreader_read() {
  uint32_t crc_previous_frame = 0;

  while (true) {
    // Wait for the next frame
    while (!frame_received) {
      // @todo use an interrupt to avoid waiting
      delay(1);
    }
    frame_received = false;

#ifdef SUPRESS_DUPLICATES
    if (*lastcrc != crc_previous_frame) {
      spi_send_pix(lastframe, *lastcrc, true);
      crc_previous_frame = *lastcrc;
    }
#else
    spi_send_pix(lastframe, *lastcrc, true);
#endif
  }
}
