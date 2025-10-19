#include <stdint.h>
#include "mcu.h"

#include "gpio.h"
#include "ethernet.h"

/*
//// TODO
periodically poll PHY for linkup/linkdown
reconfigure MAC when new link autonegotiate finishes
*/


int main(void) {

    //GPIO_init();
    //ETH_init();

    //BSP_LED_Init(LED_YELLOW);
    //BSP_LED_On(LED_YELLOW);

    
    GPIO_TypeDef *GPIOx = GPIOB;
    ETH_TypeDef *eth = ETH;


    WRITE_REG(eth->DMACSR, 0xffffffff); // BOZO
    while(1) {
        GPIOx->BSRR = 1;
        for (volatile int i = 0; i < 1000000; ++i);
        
        GPIOx->BSRR = 0x1 << 16;
        for (volatile int i = 0; i < 1000000; ++i);

        // send frame on button press
        /* if (((GPIO_TypeDef *)GPIOC)->IDR & GPIO_PIN_13) {
            ((GPIO_TypeDef *)GPIOF)->BSRR = (uint32_t)(GPIO_PIN_4<<16);
            ETH_send_frame();
        } */
    }

    return 0;
}