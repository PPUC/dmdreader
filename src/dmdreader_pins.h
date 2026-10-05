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
#ifndef SDATA_X16_PADDING
#define SDATA_X16_PADDING 1
#endif
#ifndef SDATA_X16
#define SDATA_X16 0
#endif

// SPI Defines
#ifndef DMDREADER_SPI_BASE
#define DMDREADER_SPI_BASE 12  // 16 is the default for backwards compatibility
#endif
#define DMDREADER_SPI_MISO SPI_BASE
#define DMDREADER_SPI_CS 17  // Is locked to 17 for all PPUC/DMD revisions
#define DMDREADER_SPI_SCK (DMDREADER_SPI_BASE + 2)
#define DMDREADER_SPI_MOSI (DMDREADER_SPI_BASE + 3)

#endif  // DMD_READER_PINS_H