#include "mcu.h"


static TIM_TypeDef *timer = TIM2;

// Use TIMER2 as the main time-keeping timer operating the the core clock frequency of 250MHz

void TIME_timer_init(void) {
    // enable peripheral clock
    SET_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM2EN);
    (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM2EN);

    // enable timer
    SET_BIT(timer->CR1, TIM_CR1_CEN);


    // reset timer
    SET_BIT(timer->EGR, TIM_EGR_UG);
}


void TIME_init(void) {
    TIME_timer_init();
}

uint32_t get_timer_val(void) {
    return READ_REG(timer->CNT);
}