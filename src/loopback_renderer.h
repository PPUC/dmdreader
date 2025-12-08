#ifndef LOOPBACK_RENDERER_H
#define LOOPBACK_RENDERER_H

#include <array>
#include <cstddef>
#include <cstdint>

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
void convert_to_violet(const uint32_t* src, uint8_t* dst,
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
constexpr RGB888 violet_base(uint8_t brightness);
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

      uint8_t brightness = (packed >> (i * 4)) & 0x0F;
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
std::array<RGB888, 16>* LUTManager<violet_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<pink_base>::lut = nullptr;

template <>
std::array<RGB888, 16>* LUTManager<white_base>::lut = nullptr;

// Color base functions definitions
constexpr RGB888 orange_base(uint8_t brightness) {
  // Orange: R=255, G=165, B=0 scaled with brightness
  const uint16_t scale = brightness * 17;  // 0-15 -> 0-255 (≈17 per step)
  return RGB888(static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 165 / 255), 0);
}

constexpr RGB888 red_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale), 0, 0);
}

constexpr RGB888 yellow_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale), static_cast<uint8_t>(scale), 0);
}

constexpr RGB888 green_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(0, static_cast<uint8_t>(scale), 0);
}

constexpr RGB888 blue_base(uint8_t brightness) {
  const uint16_t scale = brightness * 17;
  return RGB888(0, 0, static_cast<uint8_t>(scale));
}

constexpr RGB888 violet_base(uint8_t brightness) {
  // Violet: R=128, G=0, B=128
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale * 128 / 255), 0,
                static_cast<uint8_t>(scale * 128 / 255));
}

constexpr RGB888 pink_base(uint8_t brightness) {
  // Pink: R=255, G=192, B=203
  const uint16_t scale = brightness * 17;
  return RGB888(static_cast<uint8_t>(scale),
                static_cast<uint8_t>(scale * 192 / 255),
                static_cast<uint8_t>(scale * 203 / 255));
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

void convert_to_violet(const uint32_t* src, uint8_t* dst,
                       std::size_t pixel_count) {
  LUTManager<violet_base>::convert(src, dst, pixel_count);
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
      const RGB888& c0 = (*lut)[(packed >> 0) & 0x0F];                 \
      const RGB888& c1 = (*lut)[(packed >> 4) & 0x0F];                 \
      const RGB888& c2 = (*lut)[(packed >> 8) & 0x0F];                 \
      const RGB888& c3 = (*lut)[(packed >> 12) & 0x0F];                \
      const RGB888& c4 = (*lut)[(packed >> 16) & 0x0F];                \
      const RGB888& c5 = (*lut)[(packed >> 20) & 0x0F];                \
      const RGB888& c6 = (*lut)[(packed >> 24) & 0x0F];                \
      const RGB888& c7 = (*lut)[(packed >> 28) & 0x0F];                \
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

DEFINE_OPTIMIZED_CONVERTER(violet, violet_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(violet, violet_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(violet, violet_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(violet, violet_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(pink, pink_base, 256x64, 256 * 64)

DEFINE_OPTIMIZED_CONVERTER(white, white_base, 128x16, 128 * 16)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 128x32, 128 * 32)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 192x64, 192 * 64)
DEFINE_OPTIMIZED_CONVERTER(white, white_base, 256x64, 256 * 64)

// Dispatcher for optimized functions
enum class Resolution { RES_128x16, RES_128x32, RES_192x64, RES_256x64 };

constexpr Resolution get_resolution_from_dimensions(uint16_t width,
                                                    uint16_t height) {
  if (width == 128 && height == 16) return Resolution::RES_128x16;
  if (width == 192 && height == 64) return Resolution::RES_192x64;
  if (width == 256 && height == 64) return Resolution::RES_256x64;
  return Resolution::RES_128x32;
}

enum class Color { ORANGE, RED, YELLOW, GREEN, BLUE, VIOLET, PINK, WHITE };

// Type definition for conversion functions
using ConvertFunction = void (*)(const uint32_t*, uint8_t*);

// Dispatcher for optimized functions
ConvertFunction get_optimized_converter(uint16_t width, uint16_t height,
                                        Color color) {
  switch (get_resolution_from_dimensions(width, height)) {
    case Resolution::RES_128x16:
      switch (color) {
        case Color::ORANGE:
          return convert_to_orange_128x16;
        case Color::RED:
          return convert_to_red_128x16;
        case Color::YELLOW:
          return convert_to_yellow_128x16;
        case Color::GREEN:
          return convert_to_green_128x16;
        case Color::BLUE:
          return convert_to_blue_128x16;
        case Color::VIOLET:
          return convert_to_violet_128x16;
        case Color::PINK:
          return convert_to_pink_128x16;
        case Color::WHITE:
          return convert_to_white_128x16;
      }
      break;

    case Resolution::RES_128x32:
      switch (color) {
        case Color::ORANGE:
          return convert_to_orange_128x32;
        case Color::RED:
          return convert_to_red_128x32;
        case Color::YELLOW:
          return convert_to_yellow_128x32;
        case Color::GREEN:
          return convert_to_green_128x32;
        case Color::BLUE:
          return convert_to_blue_128x32;
        case Color::VIOLET:
          return convert_to_violet_128x32;
        case Color::PINK:
          return convert_to_pink_128x32;
        case Color::WHITE:
          return convert_to_white_128x32;
      }
      break;

    case Resolution::RES_192x64:
      switch (color) {
        case Color::ORANGE:
          return convert_to_orange_192x64;
        case Color::RED:
          return convert_to_red_192x64;
        case Color::YELLOW:
          return convert_to_yellow_192x64;
        case Color::GREEN:
          return convert_to_green_192x64;
        case Color::BLUE:
          return convert_to_blue_192x64;
        case Color::VIOLET:
          return convert_to_violet_192x64;
        case Color::PINK:
          return convert_to_pink_192x64;
        case Color::WHITE:
          return convert_to_white_192x64;
      }
      break;

    case Resolution::RES_256x64:
      switch (color) {
        case Color::ORANGE:
          return convert_to_orange_256x64;
        case Color::RED:
          return convert_to_red_256x64;
        case Color::YELLOW:
          return convert_to_yellow_256x64;
        case Color::GREEN:
          return convert_to_green_256x64;
        case Color::BLUE:
          return convert_to_blue_256x64;
        case Color::VIOLET:
          return convert_to_violet_256x64;
        case Color::PINK:
          return convert_to_pink_256x64;
        case Color::WHITE:
          return convert_to_white_256x64;
      }
      break;
  }
  return nullptr;
}

#endif
