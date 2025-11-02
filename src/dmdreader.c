#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "dmd_reader.h"
#include "logic_analyzer.h"

int main()
{
    stdio_init_all();

    gpio_init(SPI_IRQ_PIN);
    gpio_set_dir(SPI_IRQ_PIN, GPIO_IN);
    gpio_disable_pulls(SPI_IRQ_PIN);

    if (pin_is_stably_high(SPI_IRQ_PIN, 100, 5, 1500)) {
        analyze();
        return 0;
    }

    return read_dmd();
}
