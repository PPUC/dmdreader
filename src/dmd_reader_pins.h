#ifndef DMD_READER_PINS_H
#define DMD_READER_PINS_H

#define DE 7
#define RDATA 6
#define RCLK 5
#define COLLAT 4
#define DOTCLK 3
#define SDATA 2

#define LED1_PIN 27
#define LED2_PIN 28

// SPI Defines
#define SPI0 spi0
#define SPI_BASE 16
#define SPI0_MISO SPI_BASE        // 16
#define SPI0_CS (SPI_BASE + 1)    // 17
#define SPI0_SCK (SPI_BASE + 2)   // 18
#define SPI0_MOSI (SPI_BASE + 3)  // 19

#endif  // DMD_READER_PINS_H
