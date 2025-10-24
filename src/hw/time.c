// Embedded IoT Ethernet Lab
// Matthew Otto
// October 2025

#include "mcu.h"

static TIM_TypeDef *timer = TIM2;

// Use TIMER2 as the main loop timer

static uint8_t light_on = 0;
GPIO_TypeDef *GPIOx = GPIOB;

void TIM2_IRQHandler(void) {
    WRITE_REG(timer->SR, 0);
    if (light_on) {
        ((GPIO_TypeDef *)GPIOB)->BSRR = 0x1 << 16;
        light_on = 0;
    } else {
        ((GPIO_TypeDef *)GPIOB)->BSRR = 1;
        light_on = 1;
    }
}

void TIME_timer_init(void) {
    // enable peripheral clock
    SET_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM2EN);
    (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM2EN);

    // enable autoload
    SET_BIT(timer->CR1, TIM_CR1_ARPE);
    // configure timer prescaler
    //WRITE_REG(timer->PSC, 250-1); // 1MHz
    // configure timer overflow
    WRITE_REG(timer->ARR, 250000000-1); // 1 second

    // enable timer interrupt
    SET_BIT(timer->DIER, TIM_DIER_UIE);

    // enable TIM2 interrupts in NVIC
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    // enable timer
    SET_BIT(timer->CR1, TIM_CR1_CEN);

    // reset timer
    SET_BIT(timer->EGR, TIM_EGR_UG);
}

void TIME_update(uint32_t offset) {
    timer->CNT = (timer->CNT + offset) % (250000000-1);
}


void TIME_init(void) {
    TIME_timer_init();
}

uint32_t get_timer_val(void) {
    return READ_REG(timer->CNT);
}