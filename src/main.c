#include <stdint.h>
#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"

#include "gpio.h"
#include "ethernet.h"

/*
//// TODO
periodically poll PHY for linkup/linkdown
reconfigure MAC when new link autonegotiate finishes

multiple descriptors/buffers
*/



int main(void) {

    //GPIO_init();
    //ETH_init();

    //BSP_LED_Init(LED_YELLOW);
    //BSP_LED_On(LED_YELLOW);

    uint32_t len;
    volatile uint32_t datac;

    GPIO_TypeDef *GPIOx = GPIOB;
    ETH_TypeDef *eth = ETH;
    while(1) {
        GPIOx->BSRR = 1;
        for (volatile int i = 0; i < 1000000; ++i);
        
        GPIOx->BSRR = 0x1 << 16;
        for (volatile int i = 0; i < 1000000; ++i);

        datac = *(volatile uint32_t *)((uint32_t)ETH + 0x7c4);

        ETH_receive_frame();
    }

    return 0;
}