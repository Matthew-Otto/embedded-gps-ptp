#include <stdint.h>
#include "mcu.h"

#include "time.h"
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


    while(1) {
        volatile uint32_t time = get_timer_val();
        
        GPIOx->BSRR = 1;
        for (volatile int i = 0; i < 1000000; ++i);
        
        GPIOx->BSRR = 0x1 << 16;
        for (volatile int i = 0; i < 1000000; ++i);

        volatile uint32_t ptp_sec = READ_REG(eth->MACSTSR);
        volatile uint32_t ptp_nsec = READ_REG(eth->MACSTNR);

        //ETH_receive_frame();

        if (((GPIO_TypeDef *)GPIOC)->IDR & GPIO_PIN_13) {
            //((GPIO_TypeDef *)GPIOF)->BSRR = (uint32_t)(GPIO_PIN_4<<16);
            //ETH_update_PTP_TS(178943);
        }
    }

    return 0;
}