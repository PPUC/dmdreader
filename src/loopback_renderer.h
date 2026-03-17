#ifndef LOOPBACK_RENDERER_H
#define LOOPBACK_RENDERER_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "dmdreader.h"

// RGB888 type definition
struct RGB888 {
  uint8_t r, g, b;

  // Default constructor
  constexpr RGB888() : r(0), g(0), b(0) {}

  // Constructor with values
  constexpr RGB888(uint8_t red, uint8_t green, uint8_t blue)
      : r(red), g(green), b(blue) {}
};

// Forward declarations of conversion functions
void convert_to_orange(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count);
void convert_to_red(const uint32_t* src, uint8_t* dst, std::size_t pixel_count);
void convert_to_yellow(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count);
void convert_to_green(const uint32_t* src, uint8_t* dst,
                      std::size_t pixel_count);
void convert_to_blue(const uint32_t* src, uint8_t* dst,
                     std::size_t pixel_count);
void convert_to_purple(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count);
void convert_to_pink(const uint32_t* src, uint8_t* dst,
                     std::size_t pixel_count);
void convert_to_white(const uint32_t* src, uint8_t* dst,
                      std::size_t pixel_count);

// Forward declarations of base color functions
constexpr RGB888 orange_base(uint8_t brightness);
constexpr RGB888 red_base(uint8_t brightness);
constexpr RGB888 yellow_base(uint8_t brightness);
constexpr RGB888 green_base(uint8_t brightness);
constexpr RGB888 blue_base(uint8_t brightness);
constexpr RGB888 purple_base(uint8_t brightness);
constexpr RGB888 pink_base(uint8_t brightness);
constexpr RGB888 white_base(uint8_t brightness);

// Basic implementation with template for different colors
template <typename ColorFunc>
void convert_color_template(const uint32_t* src, uint8_t* dst,
                            std::size_t pixel_count, ColorFunc color_func) {
  const std::size_t words = (pixel_count + 7) / 8;

  for (std::size_t word_idx = 0; word_idx < words; ++word_idx) {
    uint32_t packed = src[word_idx];

    // Process 8 pixels per loop iteration
    for (int i = 0; i < 8; ++i) {
      if (word_idx * 8 + i >= pixel_count) break;

      uint8_t brightness = (packed >> (28 - i * 4)) & 0x0F;
      color_func(brightness, dst);
      dst += 3;  // Next RGB position
    }
  }
}

// LUT management
template <RGB888 (*BaseColor)(uint8_t)>
class LUTManager {
 private:
  static std::array<RGB888, 16>* lut;

  static std::array<RGB888, 16>* get_lut() {
    if (!lut) {
      lut = new std::array<RGB888, 16>();
      for (uint8_t i = 0; i < 16; ++i) {
        (*lut)[i] = BaseColor(i);
      }
    }
    return lut;
  }

 public:
  static void convert(const uint32_t* src, uint8_t* dst,
                      std::size_t pixel_count) {
    auto* local_lut = get_lut();

    convert_color_template(src, dst, pixel_count,
                           [local_lut](uint8_t brightness, uint8_t* dst_ptr) {
                             const RGB888& color = (*local_lut)[brightness];
                             dst_ptr[0] = color.r;
                             dst_ptr[1] = color.g;
                             dst_ptr[2] = color.b;
                           });
  }
};

// Initialize template instances
template <>
std::array<RGB888, 16>* LUTManager<orange_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<red_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<yellow_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<green_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<blue_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<purple_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<pink_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<white_base>::lut = nullptr;

constexpr uint8_t scale_brightness(uint8_t value, uint8_t max_value) {
  const uint8_t clamped = (value > max_value) ? max_value : value;
  return static_cast<uint8_t>((clamped * 15 + max_value / 2) / max_value);
}

constexpr std::array<uint8_t, 16> make_brightness_map(uint8_t max_value) {
  std::array<uint8_t, 16> map = {};
  for (uint8_t i = 0; i < map.size(); ++i) {
    map[i] = scale_brightness(i, max_value);
  }
  return map;
}

static constexpr std::array<uint8_t, 16> kCapcomBrightnessMap =
    make_brightness_map(4);
static constexpr std::array<uint8_t, 16> kGottliebBrightnessMap =
    make_brightness_map(6);

// ------------- Color base functions definitions -------------
// Color values are done in a way that they are less harsh to the eye.
// We do this by mixing in some lower value colors with the primary colors.
// Rule of thumb: avoid fully saturated extremes. Instead add warmth and depth
// → makes it look more organic.
constexpr RGB888 orange_base(uint8_t brightness) {
  // Orange: R=255, G=165, B=0 scaled with brightness
  const uint16_t scale = brightness * 17;  // 0-15 -> 0-255 (≈17 per step)
  return RGB888(static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 128 / 255), 0);
}

constexpr RGB888 red_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale * 30 / 255));
}

constexpr RGB888 green_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 30 / 255));
}

constexpr RGB888 blue_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale));
}

constexpr RGB888 yellow_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale), static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 30 / 255));
}

constexpr RGB888 purple_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale * 128 / 255),
                static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale));
}

constexpr RGB888 pink_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 30 / 255),
                static_cast<uint8_t>(scale));
}

constexpr RGB888 white_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale), static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale));
}

// Public interfaces
void convert_to_orange(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count) {
  LUTManager<orange_base>::convert(src, dst, pixel_count);
}

void convert_to_red(const uint32_t* src, uint8_t* dst,
                    std::size_t pixel_count) {
  LUTManager<red_base>::convert(src, dst, pixel_count);
}

void convert_to_yellow(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count) {
  LUTManager<yellow_base>::convert(src, dst, pixel_count);
}

void convert_to_green(const uint32_t* src, uint8_t* dst,
                      std::size_t pixel_count) {
  LUTManager<green_base>::convert(src, dst, pixel_count);
}

void convert_to_blue(const uint32_t* src, uint8_t* dst,
                     std::size_t pixel_count) {
  LUTManager<blue_base>::convert(src, dst, pixel_count);
}

void convert_to_purple(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count) {
  LUTManager<purple_base>::convert(src, dst, pixel_count);
}

void convert_to_pink(const uint32_t* src, uint8_t* dst,
                     std::size_t pixel_count) {
  LUTManager<pink_base>::convert(src, dst, pixel_count);
}

void convert_to_white(const uint32_t* src, uint8_t* dst,
                      std::size_t pixel_count) {
  LUTManager<white_base>::convert(src, dst, pixel_count);
}

// Macro for generating optimized functions
#define DEFINE_OPTIMIZED_CONVERTER(COLOR_NAME, BASE_FUNC, FUNC_SUFFIX, \
                                   PIXEL_COUNT)                        \
  void convert_to_##COLOR_NAME##_##FUNC_SUFFIX(const uint32_t* src,    \
                                               uint8_t* dst) {         \
    constexpr std::size_t WORDS = (PIXEL_COUNT + 7) / 8;               \
    static auto* lut = []() -> std::array<RGB888, 16>* {               \
      auto* l = new std::array<RGB888, 16>();                          \
      for (uint8_t i = 0; i < 16; ++i) {                               \
        (*l)[i] = BASE_FUNC(i);                                        \
      }                                                                \
      return l;                                                        \
    }();                                                               \
    for (std::size_t word_idx = 0; word_idx < WORDS; ++word_idx) {     \
      uint32_t packed = src[word_idx];                                 \
      const RGB888& c0 = (*lut)[(packed >> 28) & 0x0F];                \
      const RGB888& c1 = (*lut)[(packed >> 24) & 0x0F];                \
      const RGB888& c2 = (*lut)[(packed >> 20) & 0x0F];                \
      const RGB888& c3 = (*lut)[(packed >> 16) & 0x0F];                \
      const RGB888& c4 = (*lut)[(packed >> 12) & 0x0F];                \
      const RGB888& c5 = (*lut)[(packed >> 8) & 0x0F];                 \
      const RGB888& c6 = (*lut)[(packed >> 4) & 0x0F];                 \
      const RGB888& c7 = (*lut)[(packed >> 0) & 0x0F];                 \
      dst[0] = c0.r;                                                   \
      dst[1] = c0.g;                                                   \
      dst[2] = c0.b;                                                   \
      dst[3] = c1.r;                                                   \
      dst[4] = c1.g;                                                   \
      dst[5] = c1.b;                                                   \
      dst[6] = c2.r;                                                   \
      dst[7] = c2.g;                                                   \
      dst[8] = c2.b;                                                   \
      dst[9] = c3.r;                                                   \
      dst[10] = c3.g;                                                  \
      dst[11] = c3.b;                                                  \
      dst[12] = c4.r;                                                  \
      dst[13] = c4.g;                                                  \
      dst[14] = c4.b;                                                  \
      dst[15] = c5.r;                                                  \
      dst[16] = c5.g;                                                  \
      dst[17] = c5.b;                                                  \
      dst[18] = c6.r;                                                  \
      dst[19] = c6.g;                                                  \
      dst[20] = c6.b;                                                  \
      dst[21] = c7.r;                                                  \
      dst[22] = c7.g;                                                  \
      dst[23] = c7.b;                                                  \
      dst += 24;                                                       \
    }                                                                  \
  }

#define DEFINE_OPTIMIZED_CONVERTER_MAPPED(COLOR_NAME, BASE_FUNC,         \
                                          FUNC_SUFFIX, PIXEL_COUNT,     \
                                          MAP, MAP_SUFFIX)              \
  void convert_to_##COLOR_NAME##_##FUNC_SUFFIX##_##MAP_SUFFIX(          \
      const uint32_t* src, uint8_t* dst) {                              \
    constexpr std::size_t WORDS = (PIXEL_COUNT + 7) / 8;                \
    static auto* lut = []() -> std::array<RGB888, 16>* {                \
      auto* l = new std::array<RGB888, 16>();                           \
      for (uint8_t i = 0; i < 16; ++i) {                                \
        (*l)[i] = BASE_FUNC(i);                                         \
      }                                                                 \
      return l;                                                         \
    }();                                                                \
    const uint8_t* map = MAP.data();                                    \
    for (std::size_t word_idx = 0; word_idx < WORDS; ++word_idx) {      \
      uint32_t packed = src[word_idx];                                  \
      const RGB888& c0 = (*lut)[map[(packed >> 28) & 0x0F]];             \
      const RGB888& c1 = (*lut)[map[(packed >> 24) & 0x0F]];             \
      const RGB888& c2 = (*lut)[map[(packed >> 20) & 0x0F]];             \
      const RGB888& c3 = (*lut)[map[(packed >> 16) & 0x0F]];             \
      const RGB888& c4 = (*lut)[map[(packed >> 12) & 0x0F]];             \
      const RGB888& c5 = (*lut)[map[(packed >> 8) & 0x0F]];              \
      const RGB888& c6 = (*lut)[map[(packed >> 4) & 0x0F]];              \
      const RGB888& c7 = (*lut)[map[(packed >> 0) & 0x0F]];              \
      dst[0] = c0.r;                                                    \
      dst[1] = c0.g;                                                    \
      dst[2] = c0.b;                                                    \
      dst[3] = c1.r;                                                    \
      dst[4] = c1.g;                                                    \
      dst[5] = c1.b;                                                    \
      dst[6] = c2.r;                                                    \
      dst[7] = c2.g;                                                    \
      dst[8] = c2.b;                                                    \
      dst[9] = c3.r;                                                    \
      dst[10] = c3.g;                                                   \
      dst[11] = c3.b;                                                   \
      dst[12] = c4.r;                                                   \
      dst[13] = c4.g;                                                   \
      dst[14] = c4.b;                                                   \
      dst[15] = c5.r;                                                   \
      dst[16] = c5.g;                                                   \
      dst[17] = c5.b;                                                   \
      dst[18] = c6.r;                                                   \
      dst[19] = c6.g;                                                   \
      dst[20] = c6.b;                                                   \
      dst[21] = c7.r;                                                   \
      dst[22] = c7.g;                                                   \
      dst[23] = c7.b;                                                   \
      dst += 24;                                                        \
    }                                                                   \
  }

// Define optimized functions for all resolutions
DEFINE_OPTIMIZED_CONVERTER(orange, orange_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(orange, orange_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(orange, orange_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(orange, orange_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(red, red_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(red, red_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(red, red_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(red, red_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(yellow, yellow_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(yellow, yellow_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(yellow, yellow_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(yellow, yellow_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(green, green_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(green, green_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(green, green_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(green, green_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(blue, blue_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(blue, blue_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(blue, blue_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(blue, blue_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(purple, purple_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(purple, purple_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(purple, purple_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(purple, purple_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(white, white_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 256x64, 256 * 64)

// Define CAPCOM-mapped optimized functions for all resolutions
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 128x16, 128 * 16,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 128x32, 128 * 32,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 192x64, 192 * 64,
                                  kCapcomBrightnessMap, capcom)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 256x64, 256 * 64,
                                  kCapcomBrightnessMap, capcom)

// Define GOTTLIEB-mapped optimized functions for all resolutions
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(orange, orange_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(red, red_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(yellow, yellow_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(green, green_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(blue, blue_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(purple, purple_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(pink, pink_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 128x16, 128 * 16,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 128x32, 128 * 32,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 192x64, 192 * 64,
                                  kGottliebBrightnessMap, gottlieb)
DEFINE_OPTIMIZED_CONVERTER_MAPPED(white, white_base, 256x64, 256 * 64,
                                  kGottliebBrightnessMap, gottlieb)

// Dispatcher for optimized functions
enum class Resolution { RES_128x16, RES_128x32, RES_192x64, RES_256x64 };

constexpr Resolution get_resolution_from_dimensions(uint16_t width,
                                                    uint16_t height) {
  if (width == 128 && height == 16) return Resolution::RES_128x16;
  if (width == 192 && height == 64) return Resolution::RES_192x64;
  if (width == 256 && height == 64) return Resolution::RES_256x64;
  return Resolution::RES_128x32;
}

// Type definition for conversion functions
using ConvertFunction = void (*)(const uint32_t*, uint8_t*);

constexpr bool is_capcom_type(DmdType dmd_type) {
  return dmd_type == DMD_CAPCOM || dmd_type == DMD_CAPCOM_HD;
}

inline ConvertFunction select_converter(ConvertFunction normal,
                                        ConvertFunction capcom,
                                        ConvertFunction gottlieb,
                                        DmdType dmd_type) {
  if (is_capcom_type(dmd_type)) return capcom;
  if (dmd_type == DMD_GOTTLIEB) return gottlieb;
  return normal;
}

// Dispatcher for optimized functions
ConvertFunction get_optimized_converter(uint16_t width, uint16_t height,
                                        Color color, DmdType dmd_type) {
  switch (get_resolution_from_dimensions(width, height)) {
    case Resolution::RES_128x16:
      switch (color) {
        case Color::DMD_ORANGE:
          return select_converter(convert_to_orange_128x16,
                                  convert_to_orange_128x16_capcom,
                                  convert_to_orange_128x16_gottlieb, dmd_type);
        case Color::DMD_RED:
          return select_converter(convert_to_red_128x16,
                                  convert_to_red_128x16_capcom,
                                  convert_to_red_128x16_gottlieb, dmd_type);
        case Color::DMD_YELLOW:
          return select_converter(convert_to_yellow_128x16,
                                  convert_to_yellow_128x16_capcom,
                                  convert_to_yellow_128x16_gottlieb,
                                  dmd_type);
        case Color::DMD_GREEN:
          return select_converter(convert_to_green_128x16,
                                  convert_to_green_128x16_capcom,
                                  convert_to_green_128x16_gottlieb, dmd_type);
        case Color::DMD_BLUE:
          return select_converter(convert_to_blue_128x16,
                                  convert_to_blue_128x16_capcom,
                                  convert_to_blue_128x16_gottlieb, dmd_type);
        case Color::DMD_PURPLE:
          return select_converter(convert_to_purple_128x16,
                                  convert_to_purple_128x16_capcom,
                                  convert_to_purple_128x16_gottlieb,
                                  dmd_type);
        case Color::DMD_PINK:
          return select_converter(convert_to_pink_128x16,
                                  convert_to_pink_128x16_capcom,
                                  convert_to_pink_128x16_gottlieb, dmd_type);
        case Color::DMD_WHITE:
          return select_converter(convert_to_white_128x16,
                                  convert_to_white_128x16_capcom,
                                  convert_to_white_128x16_gottlieb, dmd_type);
      }
      break;

    case Resolution::RES_128x32:
      switch (color) {
        case Color::DMD_ORANGE:
          return select_converter(convert_to_orange_128x32,
                                  convert_to_orange_128x32_capcom,
                                  convert_to_orange_128x32_gottlieb, dmd_type);
        case Color::DMD_RED:
          return select_converter(convert_to_red_128x32,
                                  convert_to_red_128x32_capcom,
                                  convert_to_red_128x32_gottlieb, dmd_type);
        case Color::DMD_YELLOW:
          return select_converter(convert_to_yellow_128x32,
                                  convert_to_yellow_128x32_capcom,
                                  convert_to_yellow_128x32_gottlieb,
                                  dmd_type);
        case Color::DMD_GREEN:
          return select_converter(convert_to_green_128x32,
                                  convert_to_green_128x32_capcom,
                                  convert_to_green_128x32_gottlieb, dmd_type);
        case Color::DMD_BLUE:
          return select_converter(convert_to_blue_128x32,
                                  convert_to_blue_128x32_capcom,
                                  convert_to_blue_128x32_gottlieb, dmd_type);
        case Color::DMD_PURPLE:
          return select_converter(convert_to_purple_128x32,
                                  convert_to_purple_128x32_capcom,
                                  convert_to_purple_128x32_gottlieb,
                                  dmd_type);
        case Color::DMD_PINK:
          return select_converter(convert_to_pink_128x32,
                                  convert_to_pink_128x32_capcom,
                                  convert_to_pink_128x32_gottlieb, dmd_type);
        case Color::DMD_WHITE:
          return select_converter(convert_to_white_128x32,
                                  convert_to_white_128x32_capcom,
                                  convert_to_white_128x32_gottlieb, dmd_type);
      }
      break;

    case Resolution::RES_192x64:
      switch (color) {
        case Color::DMD_ORANGE:
          return select_converter(convert_to_orange_192x64,
                                  convert_to_orange_192x64_capcom,
                                  convert_to_orange_192x64_gottlieb, dmd_type);
        case Color::DMD_RED:
          return select_converter(convert_to_red_192x64,
                                  convert_to_red_192x64_capcom,
                                  convert_to_red_192x64_gottlieb, dmd_type);
        case Color::DMD_YELLOW:
          return select_converter(convert_to_yellow_192x64,
                                  convert_to_yellow_192x64_capcom,
                                  convert_to_yellow_192x64_gottlieb,
                                  dmd_type);
        case Color::DMD_GREEN:
          return select_converter(convert_to_green_192x64,
                                  convert_to_green_192x64_capcom,
                                  convert_to_green_192x64_gottlieb, dmd_type);
        case Color::DMD_BLUE:
          return select_converter(convert_to_blue_192x64,
                                  convert_to_blue_192x64_capcom,
                                  convert_to_blue_192x64_gottlieb, dmd_type);
        case Color::DMD_PURPLE:
          return select_converter(convert_to_purple_192x64,
                                  convert_to_purple_192x64_capcom,
                                  convert_to_purple_192x64_gottlieb,
                                  dmd_type);
        case Color::DMD_PINK:
          return select_converter(convert_to_pink_192x64,
                                  convert_to_pink_192x64_capcom,
                                  convert_to_pink_192x64_gottlieb, dmd_type);
        case Color::DMD_WHITE:
          return select_converter(convert_to_white_192x64,
                                  convert_to_white_192x64_capcom,
                                  convert_to_white_192x64_gottlieb, dmd_type);
      }
      break;

    case Resolution::RES_256x64:
      switch (color) {
        case Color::DMD_ORANGE:
          return select_converter(convert_to_orange_256x64,
                                  convert_to_orange_256x64_capcom,
                                  convert_to_orange_256x64_gottlieb, dmd_type);
        case Color::DMD_RED:
          return select_converter(convert_to_red_256x64,
                                  convert_to_red_256x64_capcom,
                                  convert_to_red_256x64_gottlieb, dmd_type);
        case Color::DMD_YELLOW:
          return select_converter(convert_to_yellow_256x64,
                                  convert_to_yellow_256x64_capcom,
                                  convert_to_yellow_256x64_gottlieb,
                                  dmd_type);
        case Color::DMD_GREEN:
          return select_converter(convert_to_green_256x64,
                                  convert_to_green_256x64_capcom,
                                  convert_to_green_256x64_gottlieb, dmd_type);
        case Color::DMD_BLUE:
          return select_converter(convert_to_blue_256x64,
                                  convert_to_blue_256x64_capcom,
                                  convert_to_blue_256x64_gottlieb, dmd_type);
        case Color::DMD_PURPLE:
          return select_converter(convert_to_purple_256x64,
                                  convert_to_purple_256x64_capcom,
                                  convert_to_purple_256x64_gottlieb,
                                  dmd_type);
        case Color::DMD_PINK:
          return select_converter(convert_to_pink_256x64,
                                  convert_to_pink_256x64_capcom,
                                  convert_to_pink_256x64_gottlieb, dmd_type);
        case Color::DMD_WHITE:
          return select_converter(convert_to_white_256x64,
                                  convert_to_white_256x64_capcom,
                                  convert_to_white_256x64_gottlieb, dmd_type);
      }
      break;
  }
  return nullptr;
}

#endif
