#ifndef DMD_READER_PINS_H
#define DMD_READER_PINS_H

#ifndef DE
#define DE 7
#endif
#ifndef RDATA
#define RDATA 6
#endif
#ifndef RCLK
#define RCLK 5
#endif
#ifndef COLLAT
#define COLLAT 4
#endif
#ifndef DOTCLK
#define DOTCLK 3
#endif
#ifndef SDATA
#define SDATA 2
#endif

// SPI Defines
#define SPI0 spi0
#define SPI_BASE 16
#define SPI0_MISO SPI_BASE        // 16
#define SPI0_CS (SPI_BASE + 1)    // 17
#define SPI0_SCK (SPI_BASE + 2)   // 18
#define SPI0_MOSI (SPI_BASE + 3)  // 19

#endif  // DMD_READER_PINS_H
