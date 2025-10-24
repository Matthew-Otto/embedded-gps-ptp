#include "mcu.h"

int main(void) {
    while (1) {
        // Timer 2 interrupts every ~1 sec and will wake the core here
        __WFI();
    }

    return 0;
}