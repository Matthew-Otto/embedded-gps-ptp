#include <stdint.h>
#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"


#include "gpio.h"

int main(void) {

    GPIO_init();

    //BSP_LED_Init(LED_YELLOW);
    //BSP_LED_On(LED_YELLOW);

    volatile int x = 1;
    while(x) {}

    return 0;
}