#ifndef TIME_H
#define TIME_H

#include <stdint.h>
#include "mcu.h"

void TIM2_IRQHandler(void);
void TIME_init();
void TIME_update(uint32_t new_val);
uint32_t get_timer_val(void);

#endif // TIME_H