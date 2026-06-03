#include "dmdreader.h"

#include <array>
#include <cstdint>
#include <cstdlib>

#include "crc32.h"
#include "dmd_counter.h"
#include "dmd_interface.h"
#include "dmdreader_pins.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "loopback_renderer.h"
#include "pio/spi_slave_sender.pio.h"
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
uint16_t source_planehistoryperframe;
uint16_t source_dwordsperframe;
uint16_t source_bytesperframe;
uint16_t source_lineoversampling;
uint16_t source_dwordsperline;
uint16_t source_mergeplanes;
uint16_t offset[MAX_PLANESPERFRAME];

static uint8_t *alloc_aligned_buffer(size_t size, size_t alignment,
                                     void **base_out) {
  size_t effective_alignment =
      (alignment < alignof(void *)) ? alignof(void *) : alignment;
  void *base = malloc(size + effective_alignment - 1);
  if (!base) {
    return nullptr;
  }
  uintptr_t raw = reinterpret_cast<uintptr_t>(base);
  uintptr_t aligned = (raw + effective_alignment - 1) &
                      ~(static_cast<uintptr_t>(effective_alignment) - 1);
  if (base_out) {
    *base_out = base;
  }
  return reinterpret_cast<uint8_t *>(aligned);
}

// the buffers need to be aligned to 4 byte because we work with uint32_t
// pointers later. raw data read from DMD
uint8_t *planebuf1;
uint8_t *planebuf2;
uint8_t *currentPlaneBuffer;

// tmp buffer for oversampling etc.
uint8_t *processingbuf;

// processed frame (merged planes)
uint8_t *framebuf1;
uint8_t *framebuf2;
uint8_t *framebuf3;
uint8_t *current_framebuf;
uint8_t *framebuf_to_send;

uint32_t frame_crc = 0;
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
  block_header_t h = {.block_type = SPI_BLOCK_PIX_CRC};
  block_pix_crc_header_t ph = {};

  // round length to 4-byte blocks
  h.len = (((target_bytes + 3) / 4) * 4) + sizeof(h) + sizeof(ph);
  ph.columns = source_width;
  ph.rows = source_height;
  ph.bitsperpixel = target_bitsperpixel;
  ph.crc32 = crc32;

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
  uint pio_offset;
  pio_claim_free_sm_and_add_program_for_gpio_range(
      &dmd_count_signal_program, &dmd_pio, &dmd_sm, &pio_offset, pin, 1, true);
  dmd_counter_program_init(dmd_pio, dmd_sm, pio_offset, pin);
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);
  delay(250);
  pio_sm_exec(dmd_pio, dmd_sm, pio_encode_in(pio_x, 32));
  uint32_t count = ~pio_sm_get(dmd_pio, dmd_sm);
  pio_sm_set_enabled(dmd_pio, dmd_sm, false);
  pio_remove_program_and_unclaim_sm(&dmd_count_signal_program, dmd_pio, dmd_sm,
                                    pio_offset);

  return count * 4;
}

DmdType detect_dmd() {
  uint32_t dotclk = count_clock(DOTCLK);
  uint32_t rclk = count_clock(RCLK);
  uint32_t rdata = count_clock(RDATA);

  // By checking DOTCLK, RCLK and RDATA we can identify system types
  // All values are based on a 1000ms sample of data

  // SPIKE1 -> DOTCLK: 1040000 | RCLK: 8150 | RDATA: 255
  if ((dotclk > 1015000) && (dotclk < 1065000) && (rclk > 8000) &&
      (rclk < 8300) && (rdata > 245) && (rdata < 265)) {
    return DMD_SPIKE1;

    // SAM -> DOTCLK: 1025000 | RCLK: 2000 | RDATA: 60
  } else if ((dotclk > 1000000) && (dotclk < 1050000) && (rclk > 1950) &&
             (rclk < 2050) && (rdata > 55) && (rdata < 65)) {
    return DMD_SAM;

    // WPC: DOTCLK: 500000 | RCLK: 3900 | RDATA: 120
  } else if ((dotclk > 450000) && (dotclk < 550000) && (rclk > 3800) &&
             (rclk < 4000) && (rdata > 115) && (rdata < 130)) {
    return DMD_WPC;

    // Data East X16 V1: DOTCLK: 121000 or 60544 | RCLK: 3905 | RDATA: 120
  } else if ((dotclk > 55000) && (dotclk < 125000) && (rclk > 3880) &&
             (rclk < 3930) && (rdata > 110) && (rdata < 125)) {
    return DMD_DE_X16_V1;

    // Data East X16 V2: DOTCLK: 121000 or 60544 | RCLK: 3850 | RDATA: 120
  } else if ((dotclk > 55000) && (dotclk < 125000) && (rclk > 3825) &&
             (rclk < 3875) && (rdata > 110) && (rdata < 125)) {
    return DMD_DE_X16_V2;

    // Data East X32: DOTCLK: 640000 | RCLK: 2500 | RDATA: 80
  } else if ((dotclk > 630000) && (dotclk < 650000) && (rclk > 2450) &&
             (rclk < 2550) && (rdata > 75) && (rdata < 85)) {
    return DMD_DESEGA;

    // SEGA: DOTCLK: 640000 | RCLK: 2500 | RDATA: 2580
  } else if ((dotclk > 630000) && (dotclk < 650000) && (rclk > 2450) &&
             (rclk < 2550) && (rdata > 2530) && (rdata < 2630)) {
    return DMD_DESEGA;

    // SEGA HD: DOTCLK: 1836000 | RCLK: 4785 | RDATA: 75
  } else if ((dotclk > 1750000) && (dotclk < 1900000) && (rclk > 4700) &&
             (rclk < 4850) && (rdata > 70) && (rdata < 80)) {
    return DMD_SEGA_HD;

    // Whitestar -> DOTCLK: 657000 | RCLK: 2568 | RDATA: 80
  } else if ((dotclk > 645000) && (dotclk < 669000) && (rclk > 2500) &&
             (rclk < 2620) && (rdata > 75) && (rdata < 85)) {
    return DMD_WHITESTAR;

    // Gottlieb -> DOTCLK: 1647000 | RCLK: 13160 | RDATA: 390
  } else if ((dotclk > 1550000) && (dotclk < 1750000) && (rclk > 13000) &&
             (rclk < 13300) && (rdata > 370) && (rdata < 410)) {
    return DMD_GOTTLIEB;

    // Alvin G -> DOTCLK: 1192000 | RCLK: 2340 | RDATA: 73
  } else if ((dotclk > 1150000) && (dotclk < 1250000) && (rclk > 2300) &&
             (rclk < 2380) && (rdata > 65) && (rdata < 80)) {
    return DMD_ALVING;

    // Island -> DOTCLK: 2323000 | RCLK: 18100 | RDATA: 565
  } else if ((dotclk > 2200000) && (dotclk < 2450000) && (rclk > 17700) &&
             (rclk < 18500) && (rdata > 540) && (rdata < 590)) {
    return DMD_ISLAND;

    // Homepin -> DOTCLK: 837400 | RCLK: 1635 | RDATA: 50
  } else if ((dotclk > 800000) && (dotclk < 870000) && (rclk > 1580) &&
             (rclk < 1690) && (rdata > 45) && (rdata < 55)) {
    return DMD_HOMEPIN;

    // Spinball -> DOTCLK: 544000 | RCLK: 4250 | RDATA: 130
  } else if ((dotclk > 520000) && (dotclk < 570000) && (rclk > 4100) &&
             (rclk < 4400) && (rdata > 125) && (rdata < 140)) {
    return DMD_SPINBALL;

    // Sleic -> DOTCLK: 599000 | RCLK: 4700 | RDATA: 145
  } else if ((dotclk > 570000) && (dotclk < 630000) && (rclk > 4550) &&
             (rclk < 4850) && (rdata > 135) && (rdata < 155)) {
    return DMD_SLEIC;

    // ROMSTAR -> DOTCLK: 4000000 | RCLK: 15625 | RDATA: 245
  } else if ((dotclk > 3900000) && (dotclk < 4100000) && (rclk > 15300) &&
             (rclk < 15900) && (rdata > 230) && (rdata < 260)) {
    return DMD_ROMSTAR;

    // Capcom -> DOTCLK: 4168000 | RCLK: 16280 | RDATA: 510
  } else if ((dotclk > 4000000) && (dotclk < 4300000) && (rclk > 16000) &&
             (rclk < 16500) && (rdata > 490) && (rdata < 530)) {
    return DMD_CAPCOM;

    // Capcom HD -> DOTCLK: 4168000 | RCLK: 16280 | RDATA: 255
  } else if ((dotclk > 4100000) && (dotclk < 4300000) && (rclk > 15900) &&
             (rclk < 16500) && (rdata > 240) && (rdata < 270)) {
    return DMD_CAPCOM_HD;
  }

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
// convert_4bit_to_2bit_de_x16() BEGIN
// ---------------------------------

static constexpr uint8_t map_nibble_de_x16(uint8_t p) {
  return (p <= 3) ? p : (p == 4) ? 1 : (p == 8) ? 2 : (p == 10) ? 0 : 3;
}

static constexpr uint8_t make_lut_entry_de_x16(uint16_t b) {
  uint8_t lo = map_nibble_de_x16(uint8_t(b & 0x0F));
  uint8_t hi = map_nibble_de_x16(uint8_t((b >> 4) & 0x0F));
  return uint8_t((lo << 0) | (hi << 2));  // 2 nibbles -> 4 bits (2x2bit)
}

static constexpr std::array<uint8_t, 256> kByteLut_de_x16 = [] {
  std::array<uint8_t, 256> a{};
  for (uint16_t i = 0; i < 256; ++i) a[i] = make_lut_entry_de_x16(i);
  return a;
}();

static inline __attribute__((always_inline)) uint16_t
convert_4bit_to_2bit_de_x16(uint32_t input) {
  uint32_t b0 = (input >> 0) & 0xFF;
  uint32_t b1 = (input >> 8) & 0xFF;
  uint32_t b2 = (input >> 16) & 0xFF;
  uint32_t b3 = (input >> 24) & 0xFF;

  uint16_t r0 = kByteLut_de_x16[b0];
  uint16_t r1 = kByteLut_de_x16[b1];
  uint16_t r2 = kByteLut_de_x16[b2];
  uint16_t r3 = kByteLut_de_x16[b3];

  return uint16_t((r0 << 0) | (r1 << 4) | (r2 << 8) | (r3 << 12));
}

// -------------------------------
// convert_4bit_to_2bit_de_x16() END
// -------------------------------

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

// ---------------------------------
// upscale_4bit_0_4_to_0_15() BEGIN
// ---------------------------------

static constexpr uint8_t map_nibble_0_4_to_0_15(uint8_t p) {
  return (p == 0) ? 0 : (p >= 4 ? 15 : uint8_t((p << 2) - 1));
}

static constexpr uint8_t make_nibble_upscale_entry(uint16_t b) {
  uint8_t lo = map_nibble_0_4_to_0_15(uint8_t(b & 0x0F));
  uint8_t hi = map_nibble_0_4_to_0_15(uint8_t((b >> 4) & 0x0F));
  return uint8_t(lo | (hi << 4));  // 2 nibbles -> 2 nibbles
}

static constexpr std::array<uint8_t, 256> kNibbleUpscaleLut = [] {
  std::array<uint8_t, 256> a{};
  for (uint16_t i = 0; i < 256; ++i) a[i] = make_nibble_upscale_entry(i);
  return a;
}();

static inline __attribute__((always_inline)) uint32_t
upscale_4bit_0_4_to_0_15(uint32_t input) {
  uint32_t b0 = (input >> 0) & 0xFF;
  uint32_t b1 = (input >> 8) & 0xFF;
  uint32_t b2 = (input >> 16) & 0xFF;
  uint32_t b3 = (input >> 24) & 0xFF;

  uint32_t r0 = kNibbleUpscaleLut[b0];
  uint32_t r1 = kNibbleUpscaleLut[b1];
  uint32_t r2 = kNibbleUpscaleLut[b2];
  uint32_t r3 = kNibbleUpscaleLut[b3];

  return uint32_t((r0 << 0) | (r1 << 8) | (r2 << 16) | (r3 << 24));
}

// -------------------------------
// upscale_4bit_0_4_to_0_15() END
// -------------------------------

void switch_buffers() {
  uint8_t *previousPlaneBuffer = currentPlaneBuffer;
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
  if (source_planehistoryperframe > 0) {
    memcpy(&currentPlaneBuffer[source_bytesperplane *
                               (source_planesperframe -
                                source_planehistoryperframe)],
           previousPlaneBuffer,
           source_bytesperplane * source_planehistoryperframe);
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

// Resets the DMD DMA data collection starting point.
void dmd_dma_reset() {
#ifdef RP2350
  dma_irqn_set_channel_enabled(3, dmd_dma_channel, false);
  irq_set_enabled(DMA_IRQ_3, false);
#else
  dma_channel_set_irq0_enabled(dmd_dma_channel, false);
  irq_set_enabled(DMA_IRQ_0, false);
#endif

  dma_channel_abort(dmd_dma_channel);
  dma_channel_set_trans_count(dmd_dma_channel, source_dwordsperframe, false);

#ifdef RP2350
  dma_irqn_set_channel_enabled(3, dmd_dma_channel, true);
  irq_set_enabled(DMA_IRQ_3, true);
#else
  dma_channel_set_irq0_enabled(dmd_dma_channel, true);
  irq_set_enabled(DMA_IRQ_0, true);
#endif
  dmd_set_and_enable_new_dma_target();
}

/**
 * @brief Handles DMD DMA requests by switching between the buffers
 *
 */
void dmd_dma_handler() {
  dmd_set_and_enable_new_dma_target();

  if (dmd_type == DMD_DE_X16_V2) {
    // Due to the complexity of x16 v2, we use this way to re-sync
    // if the signals are noisy, or whatever else could happen.
    pio_sm_set_enabled(dmd_pio, dmd_sm, false);
    // clear the interrupt for FRAME_START_IRQ 5 in the pio
    dmd_pio->irq = (1u << 5);
    pio_sm_exec_wait_blocking(dmd_pio, dmd_sm, pio_encode_mov(pio_y, pio_null));
    dmd_dma_reset();
    pio_sm_exec(dmd_pio, dmd_sm, pio_encode_jmp(dmd_offset));
    pio_sm_set_enabled(dmd_pio, dmd_sm, true);
  }

  // Required as long as CAPCOM is not locked-in:
  plane0_shifted = false;
  detected_0_1_0_1 = false;
  detected_1_0_0_0 = false;

  // Used for Data East 128x16 to correctly align 64x16 + 64x16.
  // Also stores initial data in the hidden part of framebuf.
  int16_t diff_x16 = 0;
  uint16_t offset_x16 = 511;

  // Fix byte order within the buffer
  uint32_t *planebuf = (uint32_t *)currentPlaneBuffer;
  //buf32_t *v;
  //uint32_t res;
  // source_dwordsperframe is not the entire frame buffer if plane history is
  // used. So only the new plane data is fixed here.
  //for (int i = 0; i < source_dwordsperframe; i++) {
  //  v = (buf32_t *)planebuf;
  //  res = (v->byte3 << 24) | (v->byte2 << 16) | (v->byte1 << 8) | (v->byte0);
  //  *planebuf = res;
  //  planebuf++;
  //}

  // Get a 32bit pointer to the frame buffer to handle more pixels at once.
  uint32_t *framebuf = (uint32_t *)processingbuf;

  bool source_shiftplanesatmerge = (source_mergeplanes == MERGEPLANES_ADDSHIFT);

  //planebuf = (uint32_t *)currentPlaneBuffer;
  // px represents a group of pixels stored in a double word. 8 pixels of 4bit
  // or 16 pixels of 2bit depth.
  for (int px = 0; px < source_dwordsperplane; px++) {
    uint32_t pixval = 0;
    for (int plane = 0; plane < source_planesperframe; plane++) {
      uint32_t v = planebuf[offset[plane] + px];
      if (source_shiftplanesatmerge) {
        if (dmd_type == DMD_SPIKE1) {
          v <<= plane;
        } else if (dmd_type == DMD_SLEIC) {
          v <<= (source_planesperframe - 1) - plane;
        }
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
    // It seems to be sufficient to check every 8th pixel for these patterns to
    // detect sync. So we could avoid bitschifiting of the uint32_t value to
    // check every single pixel.
    if (dmd_type >= DMD_CAPCOM && !locked_in && !plane0_shifted) {
      digitalWrite(LED_BUILTIN, HIGH);
      uint8_t value = pixval & 0x0F;
      if (value == 2 && (planebuf[px] & 0x0F) != 1 &&
          (planebuf[offset[2] + px] & 0x0F) != 1) {
        detected_0_1_0_1 = true;
      } else if (value == 1 && (planebuf[px] & 0x0F) == 1) {
        detected_1_0_0_0 = true;
      }
      // Check for illegal patterns that can happen when not in sync:
      //   0/1/1/1 => 3
      //   1/0/1/1 => 3
      //   1/1/0/1 => 3
      //   1/1/1/0 => 3 (checking whether value 3 appears is actually enough)
      //
      //   0/0/0/1 => 1
      //   0/0/1/0 => 1
      //   0/1/0/0 => 1
      //
      //   1/0/1/0 => 2
      //   1/1/0/0 => 2
      //   0/0/1/1 => 2
      else if (value == 3 || value > 4) {
        // An unsynchronized frame has been found.
        // Disable the state machine, clean the DMA channel and restart.
        // As a result, we will skip exactly one plane.
        pio_sm_set_enabled(dmd_pio, dmd_sm, false);
        dmd_dma_reset();
        pio_sm_exec(dmd_pio, dmd_sm, pio_encode_jmp(dmd_offset));
        pio_sm_set_enabled(dmd_pio, dmd_sm, true);
        plane0_shifted = true;
      }
    }

    if (source_bitsperpixel == target_bitsperpixel ||
        (loopback && dmd_type != DMD_DE_X16_V1 && dmd_type != DMD_DE_X16_V2)) {
      framebuf[px] = pixval;
    } else if (4 == source_bitsperpixel && 2 == target_bitsperpixel) {
      if (dmd_type != DMD_DE_X16_V1 && dmd_type != DMD_DE_X16_V2) {
        uint32_t out = px >> 1;  // Shifting leads to index steps 0, 0, 1,
                                 // 1, 2, 2, 3, 3, 4, 4 ...
        uint16_t v16 = convert_4bit_to_2bit_fast(pixval);
        if ((px & 1) == 0) {
          // Write first 8 pixel in upper 16 Bit.
          framebuf[out] = (uint32_t)v16 << 16;
        } else {
          // Write second 8 pixel in lower 16 Bit.
          framebuf[out] |= v16;
        }
      } else {  // Data East 128x16 case
        framebuf[px + diff_x16 + offset_x16] = pixval;
        // increase diff everytime we cross one MSB or LSB row (64 pixels wide)
        // px is based on 4bpp, so we increase every 8 px.
        if ((px & 7) == 7) {
          diff_x16 += 8;
          // When we have processed half of the pixels in an entire frame,
          // turn the diff into a - value, it will start at the top again and
          // work its way back to 0. This way we prepare framebuf for
          // oversampling
          if (px == ((source_dwordsperplane / 2) - 1)) {
            diff_x16 -= 8;  // immediately decrement once -> select correct row
            diff_x16 *= -1;
          }
        }
      }
    } else if (2 == source_bitsperpixel && 4 == target_bitsperpixel) {
      // There's no system using this conversion yet, but let's have it ready
    }
  }

  if (dmd_type >= DMD_CAPCOM && !locked_in && !plane0_shifted &&
      detected_0_1_0_1 && detected_1_0_0_0) {
    digitalWrite(LED_BUILTIN, LOW);
    locked_in = true;
  }

  if (dmd_type == DMD_DE_X16_V1 || dmd_type == DMD_DE_X16_V2) {
    // merge the rows and convert from 4bpp to 2bpp with a LUT
    uint32_t *dst, *src1, *src2;
    dst = framebuf + 64; // start in the middle of 128x32 frame
    src1 = framebuf + 511; // everything is stored from here onwards
    src2 = src1 + source_dwordsperline;

    if (dmd_type == DMD_DE_X16_V1) {
      for (int l = 0; l < source_height / 2; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          uint32_t out = w >> 1;  // Shifting leads to 0, 0, 1, 1, etc
          uint16_t v16 = convert_4bit_to_2bit_de_x16((src1[w] * 3));
          if ((w & 1) == 0) {
            // Write first 8 pixel in upper 16 Bit.
            dst[out] = (uint32_t)v16 << 16;
          } else {
            // Write second 8 pixel in lower 16 Bit.
            dst[out] |= v16;
          }
        }
        src1 += source_dwordsperline * 2;  // source skips 2 lines forward
        dst += source_dwordsperline / 2;   // 4bbp -> 2bpp
      }
    } else {
      // DMD_DE_X16_V2 case
      for (int l = 0; l < source_height / 2; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          uint32_t out = w >> 1;  // Shifting leads to 0, 0, 1, 1, etc
          uint16_t v16 = convert_4bit_to_2bit_de_x16((src1[w] + src2[w] * 2));
          if ((w & 1) == 0) {
            // Write first 8 pixel in upper 16 Bit.
            dst[out] = (uint32_t)v16 << 16;
          } else {
            // Write second 8 pixel in lower 16 Bit.
            dst[out] |= v16;
          }
        }
        src1 += source_dwordsperline * 2;  // source skips 2 lines forward
        src2 += source_dwordsperline * 2;
        dst += source_dwordsperline / 2;  // 4bbp -> 2bpp
      }
    }
  }

  if (source_bitsperpixel == target_bitsperpixel) {
    // deal with line oversampling directly within framebuf
    if (source_lineoversampling == LINEOVERSAMPLING_2X) {
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
      uint32_t *dst, *src1, *src2, *src3, *src4;
      dst = src1 = framebuf;
      src2 = src1 + source_dwordsperline;
      src3 = src2 + source_dwordsperline;
      src4 = src3 + source_dwordsperline;
      uint32_t v;

      for (int l = 0; l < source_height; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          switch (dmd_type) {
            case DMD_SAM:
              // On SAM line order is really messed up :-(
              v = src4[w] * 8 + src3[w] * 1 + src2[w] * 4 + src1[w] * 2;
              break;
            case DMD_ALVING:
              v = upscale_4bit_0_4_to_0_15(src4[w] + src3[w] + src2[w] +
                                           src1[w]);
              break;
            case DMD_HOMEPIN:
            default:
              v = src4[w] * 8 + src3[w] * 4 + src2[w] * 2 + src1[w];
          }
          dst[w] = v;
        }
        src1 += source_dwordsperline * 4;  // source skips 4 lines forward
        src2 += source_dwordsperline * 4;
        src3 += source_dwordsperline * 4;
        src4 += source_dwordsperline * 4;
        dst += source_dwordsperline;  // destination skips only one line
      }
    } else if (dmd_type == DMD_ISLAND) {
      // processed as 4bpp, but we need to increase the brightness
      uint32_t *dst;
      dst = framebuf;

      for (int l = 0; l < source_height; l++) {
        for (int w = 0; w < source_dwordsperline; w++) {
          dst[w] = upscale_4bit_0_4_to_0_15(dst[w]);
        }
        dst += source_dwordsperline;  // destination skips only one line
      }
    }
  }

  memcpy(current_framebuf, processingbuf,
         loopback ? source_bytes : target_bytes);

  frame_crc =
      crc32(0, current_framebuf, loopback ? source_bytes : target_bytes);

  switch_buffers();

  if (frame_crc != crc_previous_frame) {
    crc_previous_frame = frame_crc;
    frame_received = true;
  }
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
                             uint8_t jump_pin, uint8_t in_base_pin) {
  uint32_t sys_hz = clock_get_hz(clk_sys);  // e.g. 125/200/266 MHz
  float target_hz = 125000000.0f;           // PIO code designed for 125 MHz
  float dmd_clkdiv = (float)sys_hz / target_hz;  // scales automatically

  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      dmd_reader_program, &dmd_pio, &dmd_sm, &dmd_offset,
      (DE < SDATA_X16) ? DE : SDATA_X16, 8, true));
  pio_sm_config dmd_config = reader_get_default_config(dmd_offset);
  dmd_reader_program_init(dmd_clkdiv, dmd_pio, dmd_sm, dmd_offset, dmd_config,
                          in_base_pin);

  // The framedetect program just runs and detects the beginning of a new
  // frame
  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      dmd_framedetect_program, &frame_pio, &frame_sm, &frame_offset,
      (DE < SDATA_X16) ? DE : SDATA_X16, 8, true));
  pio_sm_config frame_config = framedetect_get_default_config(frame_offset);
  dmd_framedetect_program_init(dmd_clkdiv, frame_pio, frame_sm, frame_offset,
                               frame_config, input_pins, num_input_pins,
                               jump_pin);
}

bool dmdreader_init(bool return_on_no_detection) {
  dmd_type = DMD_UNKNOWN;
  // Loop until the DMD is detected as it might need some time to be available
  // on power-on
  do {
    dmd_type = detect_dmd();
    if (dmd_type == DMD_UNKNOWN && return_on_no_detection) {
      return false;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  } while (dmd_type == DMD_UNKNOWN);

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
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(&dmd_reader_2bpp_program,
                              dmd_reader_2bpp_program_get_default_config,
                              &dmd_framedetect_generic_program,
                              dmd_framedetect_generic_program_get_default_config,
                              input_pins, 1, 0, SDATA);

      // load 4096 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 4095);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;
      target_bitsperpixel = 2;
      source_planesperframe = 3;
      source_planehistoryperframe = 2;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_WHITESTAR: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_2bpp_program,
          dmd_reader_2bpp_program_get_default_config,
          &dmd_framedetect_generic_program,
          dmd_framedetect_generic_program_get_default_config, input_pins, 1,
          0, SDATA);

      // load 8192 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 8191);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Whitestar is 2bpp
      target_bitsperpixel = 2;
      // in Whitestar, there's only one plane, containg
      // one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      // in Whitestar each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SPIKE1: {
      uint input_pins[] = {COLLAT};
      dmdreader_programs_init(&dmd_reader_4bpp_program,
                              dmd_reader_4bpp_program_get_default_config,
                              &dmd_framedetect_spike_program,
                              dmd_framedetect_spike_program_get_default_config,
                              input_pins, 1, COLLAT, SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 4;  // in Spike there are 4 planes
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_NONE;  // no line oversampling
      source_mergeplanes = MERGEPLANES_ADDSHIFT;
      break;
    }

    case DMD_SAM: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(&dmd_reader_4bpp_program,
                              dmd_reader_4bpp_program_get_default_config,
                              &dmd_framedetect_generic_program,
                              dmd_framedetect_generic_program_get_default_config,
                              input_pins, 1, 0, SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 1;  // in SAM there is one plane
      source_planehistoryperframe = 0;
      // with 4x line oversampling
      source_lineoversampling = LINEOVERSAMPLING_4X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_DE_X16_V1: {
      uint input_pins[] = {DE, RDATA};
      dmdreader_programs_init(
          &dmd_reader_de_x16_v1_program,
          dmd_reader_de_x16_v1_program_get_default_config,
          &dmd_framedetect_de_x16_v1_program,
          dmd_framedetect_de_x16_v1_program_get_default_config, input_pins, 2,
          DE, SDATA_X16);
      gpio_set_inover(DOTCLK, GPIO_OVERRIDE_INVERT);  // invert DOTCLK signal
      // we need it to sample data on the rising edge

      source_width = 128;
      source_height = 32; // is actually 16, but we process as 32
      source_bitsperpixel = 4;  // recorded as 4bpp in the pio
      target_bitsperpixel = 2;  // max pixel value is 3
      // in DE-Sega, there's only one plane,
      // containg one MSB row followed by one LSB row and so on
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_DE_X16_V2: {
      uint input_pins[] = {DE};
      dmdreader_programs_init(
          &dmd_reader_de_x16_v2_program,
          dmd_reader_de_x16_v2_program_get_default_config,
          &dmd_framedetect_de_x16_v2_program,
          dmd_framedetect_de_x16_v2_program_get_default_config, input_pins, 1,
          DE, SDATA_X16);
      gpio_set_inover(DOTCLK, GPIO_OVERRIDE_INVERT);  // invert DOTCLK signal
      // we need it to sample data on the rising edge

      // initialise Y register to zero
      pio_sm_exec_wait_blocking(dmd_pio, dmd_sm,
                                pio_encode_mov(pio_y, pio_null));

      // load 4096 delay cycles directly to TX fifo (32uS)
      pio_sm_put(dmd_pio, dmd_sm, 4096);
      // load 2500 delay cycles directly to TX fifo (20uS)
      pio_sm_put(frame_pio, frame_sm, 2500);

      source_width = 128;
      source_height = 32; // is actually 16, but we process as 32
      source_bitsperpixel = 4;  // recorded as 4bpp in the pio
      target_bitsperpixel = 2;  // max pixvalues are 0, 1, 2, 3
      // in DE-Sega, there's only one plane,
      // containg one MSB row followed by one LSB row and so on
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_DESEGA: {
      uint input_pins[] = {DE};
      dmdreader_programs_init(&dmd_reader_2bpp_program,
                              dmd_reader_2bpp_program_get_default_config,
                              &dmd_framedetect_desega_program,
                              dmd_framedetect_desega_program_get_default_config,
                              input_pins, 1, DE, SDATA);

      // load 8192 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 8191);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;  // Data East and Sega are 2bpp
      target_bitsperpixel = 2;
      // in DE-Sega, there's only one plane,
      // containg one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      // in DE-Sega each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SEGA_HD: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_2bpp_program,
          dmd_reader_2bpp_program_get_default_config,
          &dmd_framedetect_generic_program,
          dmd_framedetect_generic_program_get_default_config, input_pins, 1, 0,
          SDATA);

      // load 24576 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 24575);

      source_width = 192;
      source_height = 64;
      source_bitsperpixel = 2;  // Data East and Sega are 2bpp
      target_bitsperpixel = 2;
      // in DE-Sega, there's only one plane,
      // containg one LSB row followed by one MSB row and so on
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      // in DE-Sega each line is sent twice
      source_lineoversampling = LINEOVERSAMPLING_2X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_GOTTLIEB: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_4bpp_program,
          dmd_reader_4bpp_program_get_default_config,
          &dmd_framedetect_generic_program,
          dmd_framedetect_generic_program_get_default_config, input_pins, 1, 0,
          SDATA);

      // load 4096 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 4095);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 6;
      source_planehistoryperframe = 3;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_ALVING: {
      uint input_pins[] = {RDATA, COLLAT};
      dmdreader_programs_init(&dmd_reader_4bpp_program,
                              dmd_reader_4bpp_program_get_default_config,
                              &dmd_framedetect_alving_program,
                              dmd_framedetect_alving_program_get_default_config,
                              input_pins, 2, 0, SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);
      // load 128 - 1 COLLAT edges directly to TX fifo
      pio_sm_put(frame_pio, frame_sm, 127);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 1;  // in Alvin G there is one plane
      source_planehistoryperframe = 0;
      // with 4x line oversampling
      source_lineoversampling = LINEOVERSAMPLING_4X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_ISLAND: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_4bpp_program,
          dmd_reader_4bpp_program_get_default_config,
          &dmd_framedetect_generic_program,
          dmd_framedetect_generic_program_get_default_config, input_pins, 1, 0,
          SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 4;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_HOMEPIN: {
      uint input_pins[] = {RDATA};
      dmdreader_programs_init(
          &dmd_reader_4bpp_program,
          dmd_reader_4bpp_program_get_default_config,
          &dmd_framedetect_homepin_program,
          dmd_framedetect_homepin_program_get_default_config, input_pins, 1, 0,
          SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 4;
      source_planesperframe = 1;
      source_planehistoryperframe = 0;
      // 4x line oversampling for Homepin, similar to SAM
      source_lineoversampling = LINEOVERSAMPLING_4X;
      source_mergeplanes = MERGEPLANES_NONE;
      break;
    }

    case DMD_SPINBALL: {
      uint input_pins[] = {RDATA, RCLK};
      dmdreader_programs_init(&dmd_reader_2bpp_program,
                              dmd_reader_2bpp_program_get_default_config,
                              &dmd_framedetect_capcom_program,
                              dmd_framedetect_capcom_program_get_default_config,
                              input_pins, 2, 0, SDATA);
      // Spinball uses the WPC rendering method (timings are very close actually)
      // The Capcom framedetect method is used to find the start of a frame

      // load 4096 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 4095);
      // load 32 - 1 rows directly to TX fifo
      pio_sm_put(frame_pio, frame_sm, 31);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;
      target_bitsperpixel = 2;
      source_planesperframe = 3;
      source_planehistoryperframe = 2;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_SLEIC: {
      uint input_pins[] = {DE, RDATA};
      dmdreader_programs_init(
          &dmd_reader_2bpp_program,
          dmd_reader_2bpp_program_get_default_config,
          &dmd_framedetect_sleic_program,
          dmd_framedetect_sleic_program_get_default_config, input_pins, 2,
          DE, SDATA);

      // load 8192 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 8191);
      // load 6144 delay cycles directly to TX fifo
      pio_sm_put(frame_pio, frame_sm, 6144);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 2;
      target_bitsperpixel = 2;
      source_planesperframe = 2;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADDSHIFT;
      break;
    }

    case DMD_CAPCOM: {
      uint input_pins[] = {RDATA, RCLK};
      dmdreader_programs_init(&dmd_reader_capcom_program,
                              dmd_reader_capcom_program_get_default_config,
                              &dmd_framedetect_capcom_program,
                              dmd_framedetect_capcom_program_get_default_config,
                              input_pins, 2, 0, SDATA);

      // load 127 directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 127);
      // load 32 - 1 rows directly to TX fifo
      pio_sm_put(frame_pio, frame_sm, 31);

      source_width = 128;
      source_height = 32;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 4;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }

    case DMD_ROMSTAR:
    case DMD_CAPCOM_HD: {
      uint input_pins[] = {RDATA, RCLK};
      dmdreader_programs_init(
          &dmd_reader_4bpp_program,
          dmd_reader_4bpp_program_get_default_config,
          &dmd_framedetect_capcom_program,
          dmd_framedetect_capcom_program_get_default_config, input_pins, 2,
          0, SDATA);

      // load 16384 - 1 pixels directly to TX fifo
      pio_sm_put(dmd_pio, dmd_sm, 16383);
      // load 64 - 1 rows directly to TX fifo
      pio_sm_put(frame_pio, frame_sm, 63);

      source_width = 256;
      source_height = 64;
      source_bitsperpixel = 4;
      target_bitsperpixel = 2;
      source_planesperframe = 4;
      source_planehistoryperframe = 0;
      source_lineoversampling = LINEOVERSAMPLING_NONE;
      source_mergeplanes = MERGEPLANES_ADD;
      break;
    }
  }

  // pull 32 bits of data (if configured) from the TX fifo into osr
  pio_sm_exec(dmd_pio, dmd_sm, pio_encode_pull(false, false));
  pio_sm_exec(frame_pio, frame_sm, pio_encode_pull(false, false));

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
  source_dwordsperframe = source_dwordsperplane *
                          (source_planesperframe - source_planehistoryperframe);
  source_bytesperframe = source_bytesperplane * source_planesperframe;
  source_dwordsperline = source_width * source_bitsperpixel / 32;

  if (!planebuf1) {
    size_t plane_bytes = source_bytesperplane * source_planesperframe;
    size_t dma_bytes = source_dwordsperframe * sizeof(uint32_t);
    if (dma_bytes > plane_bytes) {
      plane_bytes = dma_bytes;
    }

    size_t processing_bytes = source_bytes * source_lineoversampling;

    planebuf1 = alloc_aligned_buffer(plane_bytes, 4, nullptr);
    planebuf2 = alloc_aligned_buffer(plane_bytes, 4, nullptr);
    processingbuf = alloc_aligned_buffer(processing_bytes, 8, nullptr);
    framebuf1 = alloc_aligned_buffer(source_bytes, 8, nullptr);
    framebuf2 = alloc_aligned_buffer(source_bytes, 8, nullptr);
    size_t framebuf3_bytes = target_bytes;
    size_t loopback_render_bytes =
        source_width * source_height * 4 / 8;  // 4bpp render buffer
    if (loopback_render_bytes > framebuf3_bytes) {
      framebuf3_bytes = loopback_render_bytes;
    }
    framebuf3 = alloc_aligned_buffer(framebuf3_bytes, 8, nullptr);

    dmdreader_error_blink(planebuf1 && planebuf2 && processingbuf &&
                          framebuf1 && framebuf2 && framebuf3);

    memset(planebuf1, 0, plane_bytes);
    memset(planebuf2, 0, plane_bytes);
    memset(processingbuf, 0, processing_bytes);
    memset(framebuf1, 0, source_bytes);
    memset(framebuf2, 0, source_bytes);
    memset(framebuf3, 0, framebuf3_bytes);
  }

  currentPlaneBuffer = planebuf2;
  current_framebuf = framebuf1;
  framebuf_to_send = framebuf2;

  // Merge multiple planes to get the frame data.
  // Calculate offsets for the first pixel of each plane and cache these.
  for (int i = 0; i < MAX_PLANESPERFRAME; i++) {
    offset[i] = i * source_dwordsperplane;
  }

  // Read a 128x16 frame but process as 128x32, so, change the number of
  // transfers at this stage to trigger a dma transfer at the right time.
  if (dmd_type == DMD_DE_X16_V1 || dmd_type == DMD_DE_X16_V2) {
      source_dwordsperframe /= 2;
      source_dwordsperplane /= 2;
  }

  // DMA for DMD reader
  dmd_dma_channel = dma_claim_unused_channel(true);
  dmd_dma_channel_cfg = dma_channel_get_default_config(dmd_dma_channel);
  channel_config_set_read_increment(&dmd_dma_channel_cfg, false);
  channel_config_set_write_increment(&dmd_dma_channel_cfg, true);
  channel_config_set_dreq(&dmd_dma_channel_cfg,
                          pio_get_dreq(dmd_pio, dmd_sm, false));

  channel_config_set_transfer_data_size(&dmd_dma_channel_cfg, DMA_SIZE_32);
  channel_config_set_bswap(&dmd_dma_channel_cfg, true);

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
  pio_sm_set_enabled(frame_pio, frame_sm, true);
  pio_sm_set_enabled(dmd_pio, dmd_sm, true);

  return true;
}

void dmdreader_spi_init() {
  // this is used to notify the Pi that data is available
  pinMode(SPI0_CS, OUTPUT);
  digitalWrite(SPI0_CS, LOW);

  // initialize SPI slave PIO
  uint pio_offset;
  dmdreader_error_blink(pio_claim_free_sm_and_add_program_for_gpio_range(
      &clocked_output_program, &spi_pio, &spi_sm, &pio_offset, SPI_BASE, 4,
      true));
  clocked_output_program_init(spi_pio, spi_sm, pio_offset, SPI_BASE);

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
  if (!loopback && frame_received) {
    frame_received = false;
    spi_send_pix(framebuf_to_send, frame_crc, true);

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

void dmdreader_loopback_stop() {
  free(framebuf3);
  loopback = false;
}

uint8_t *dmdreader_loopback_render() {
  uint64_t *frame4bit = (uint64_t *)framebuf3;

  if (loopback && frame_received) {
    frame_received = false;
    if (current_renderbuf == renderbuf1) {
      current_renderbuf = renderbuf2;
    } else {
      current_renderbuf = renderbuf1;
    }

    auto func = get_optimized_converter(source_width, source_height,
                                        monochromeColor, dmd_type);
    if (func) {
      if (2 == source_bitsperpixel) {
        for (uint16_t i = 0; i < source_dwords; i++) {
          frame4bit[i] =
              convert_2bit_to_4bit_fast(((uint32_t *)framebuf_to_send)[i]);
        }
        func((uint32_t *)frame4bit, current_renderbuf);
      } else if (dmd_type == DMD_DE_X16_V1 || dmd_type == DMD_DE_X16_V2) {
        for (uint16_t i = 0; i < source_dwords / 2; i++) {
          frame4bit[i] =
              convert_2bit_to_4bit_fast(((uint32_t *)framebuf_to_send)[i]);
        }
        func((uint32_t *)frame4bit, current_renderbuf);
      } else {
        func((uint32_t *)framebuf_to_send, current_renderbuf);
      }
    }

    return current_renderbuf;
  }

  return nullptr;
}

uint16_t dmdreader_get_source_width() { return source_width; }
uint16_t dmdreader_get_source_height() { return source_height; }
