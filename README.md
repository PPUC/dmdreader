# Read and display pinball DMD data

This project allows to read the contents of a pinball DMD using the Raspberry Pi Pico.

## Hardware

The Pi Pico is directly connected to the 6 DMD data lines. Communication between the Pi Pico and the consumer is implemented via SPI with an additional IRQ line.
On this IRQ line, the Pico signals that new data is available and consumer must start the data transfer.
Since not any consumer (especially the Raspberry Pi) can act as an SPI slave this method is used.

## Officially supported hardware systems

* WPC95 & WPC -> 128x32
* Data East -> 128x32 & 128x16
* Sega -> 128x32 & 192x64
* Stern Whitestar -> 128x32
* Stern SAM -> 128x32
* Stern SPIKE 1 -> 128x32
* Capcom -> 128x32 & 256x64
* Gottlieb/Premier -> 128x32
* Alvin G. & Co -> 128x32
* Homepin -> 128x32
* Spinball -> 128x32
* Sleic -> 128x32

## Reading data

When reading data, we assume the data is sent correctly.
This means we can read a full frame, containing a predefined amount of bits per pixel.

The process is as follows:
 - Wait for a frame to start (DMD frame detect program)
 - Read frame (Pixel loop)
 - Construct frame based on pixel loop data and system specific code

## License

This project has been forked from https://github.com/pinballpower/code_dmd after that project changed its license from MIT to GPL v3 on 2022-05-02.
So, the license of this fork is GPL v3.