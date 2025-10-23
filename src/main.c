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
    while (1) {
        __WFI();
    }

    /*while(1) {
        // if button press
        if (((GPIO_TypeDef *)GPIOC)->IDR & GPIO_PIN_13) {
            //((GPIO_TypeDef *)GPIOF)->BSRR = (uint32_t)(GPIO_PIN_4<<16);
            //ETH_update_PTP_TS(178943);
        }
    } */

    return 0;
}