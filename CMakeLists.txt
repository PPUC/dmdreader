# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

file(READ version.txt VERSION_STRING)
string(STRIP "${VERSION_STRING}" VERSION_STRING)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(${PICO_SDK_PATH}/tools/CMakeLists.txt)

project(dmdreader C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.2.0
add_executable(${PROJECT_NAME}
        src/dmdreader.c
        src/crc32.c
)

pico_set_program_name(${PROJECT_NAME} "dmdreader")
pico_set_program_version(${PROJECT_NAME} "${VERSION_STRING}")

pico_generate_pio_header(${PROJECT_NAME} src/spi_slave_sender.pio)
pico_generate_pio_header(${PROJECT_NAME} src/dmd_counter.pio)
pico_generate_pio_header(${PROJECT_NAME} src/dmd_interface_wpc.pio)
pico_generate_pio_header(${PROJECT_NAME} src/dmd_interface_whitestar.pio)
pico_generate_pio_header(${PROJECT_NAME} src/dmd_interface_spike.pio)
pico_generate_pio_header(${PROJECT_NAME} src/dmd_interface_sam.pio)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME} pico_stdlib)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_timer
        hardware_watchdog
        hardware_clocks
        )

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
