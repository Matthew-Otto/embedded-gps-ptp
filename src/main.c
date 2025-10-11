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

    uint8_t data[1500];

    for (int i = 0; i < 1500; i++)
        data[i] = 0;

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
        volatile uint32_t y = READ_REG(eth->DMADSR);
        volatile uint32_t x = READ_REG(eth->DMACSR);

        volatile uint32_t pp = READ_REG(eth->DMACRDLAR);
        volatile uint32_t ps = READ_REG(eth->DMACTDLAR);

        if (eth_receive(data, &len)) {
            volatile int x = 5;
            break;
        }
    }

    return 0;
}