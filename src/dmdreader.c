#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "dmd_reader.h"
#include "logic_analyzer.h"

int main()
{
    int result = read_dmd();

    if (result == -1) {
        analyze();
        return 0;
    }

    return result;
}
