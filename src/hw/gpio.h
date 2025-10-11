#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "stm32h563xx.h"

void GPIO_init();
void GPIO_clk_en(char);
void configure_pin(GPIO_TypeDef *GPIO_bank, uint16_t pin, uint32_t mode, uint32_t pupd, uint32_t ospeed, uint8_t alternate);

#endif // GPIO_H