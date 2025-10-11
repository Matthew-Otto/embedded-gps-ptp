#include <stdint.h>
#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"

#include "gpio.h"
#include "ethernet.h"

int main(void) {

    //GPIO_init();
    //ETH_init();

    //BSP_LED_Init(LED_YELLOW);
    //BSP_LED_On(LED_YELLOW);

    uint8_t data;
    uint32_t len;

    GPIO_TypeDef *GPIOx = GPIOB;
    while(1) {
        GPIOx->BSRR = 1;
        for (volatile int i = 0; i < 1000000; ++i);
        
        GPIOx->BSRR = 0x1 << 16;
        for (volatile int i = 0; i < 1000000; ++i);

        if (eth_receive(&data, &len)) {
            volatile int x = 5;
            break;
        }
    }

    return 0;
}