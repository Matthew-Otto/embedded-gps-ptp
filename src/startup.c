// 2nd stage bootloader
// copies initialized data from flash to ram

#include <stdint.h>

extern uint32_t __stack_top;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

// forward declaration of ISRs
extern void main(void);
void reset_handler(void);
void NMI_handler(void);
void hardfault_handler(void);
void memmanage_handler(void);
void busfault_handler(void);
void usagefault_handler(void);
void securefault_handler(void);


/* Vector table */
__attribute__((section(".isr_vector")))
void (* const vector_table[])(void) = {
    ((void (*)(void))(&__stack_top)),
    reset_handler,
    NMI_handler,
    hardfault_handler,
    memmanage_handler,
    busfault_handler,
    usagefault_handler,
    securefault_handler,
    0,
    0,
    0,

    // BOZO put fault handlers and stuff here, real ISRs go in a separate file
    /*
    SVC_handler,
    0,
    0,
    pendSV_handler,
    systick_handler,
    WWDG_IRQHandler,
    PVD_AVD_IRQHandler,
    RTC_IRQHandler,
    RTC_S_IRQHandler,
    TAMP_IRQHandler,
    RAMCFG_IRQHandler,
    FLASH_IRQHandler,
    FLASH_S_IRQHandler,
    GTZC_IRQHandler,
    RCC_IRQHandler,
    RCC_S_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    EXTI5_IRQHandler,
    EXTI6_IRQHandler,
    EXTI7_IRQHandler,
    EXTI8_IRQHandler,
    EXTI9_IRQHandler,
    EXTI10_IRQHandler,
    EXTI11_IRQHandler,
    EXTI12_IRQHandler,
    EXTI13_IRQHandler,
    EXTI14_IRQHandler,
    EXTI15_IRQHandler,
    GPDMA1_Channel0_IRQHandler,
    GPDMA1_Channel1_IRQHandler,
    GPDMA1_Channel2_IRQHandler,
    GPDMA1_Channel3_IRQHandler,
    GPDMA1_Channel4_IRQHandler,
    GPDMA1_Channel5_IRQHandler,
    GPDMA1_Channel6_IRQHandler,
    GPDMA1_Channel7_IRQHandler,
    IWDG_IRQHandler,
    0,
    ADC1_IRQHandler,
    DAC1_IRQHandler,
    FDCAN1_IT0_IRQHandler,
    FDCAN1_IT1_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    TIM5_IRQHandler,
    TIM6_IRQHandler,
    TIM7_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    SPI3_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    UART4_IRQHandler,
    UART5_IRQHandler,
    LPUART1_IRQHandler,
    LPTIM1_IRQHandler,
    TIM8_BRK_IRQHandler,
    TIM8_UP_IRQHandler,
    TIM8_TRG_COM_IRQHandler,
    TIM8_CC_IRQHandler,
    ADC2_IRQHandler,
    LPTIM2_IRQHandler,
    TIM15_IRQHandler,
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    USB_DRD_FS_IRQHandler,
    CRS_IRQHandler,
    UCPD1_IRQHandler,
    FMC_IRQHandler,
    OCTOSPI1_IRQHandler,
    SDMMC1_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    SPI4_IRQHandler,
    SPI5_IRQHandler,
    SPI6_IRQHandler,
    USART6_IRQHandler,
    USART10_IRQHandler,
    USART11_IRQHandler,
    SAI1_IRQHandler,
    SAI2_IRQHandler,
    GPDMA2_Channel0_IRQHandler,
    GPDMA2_Channel1_IRQHandler,
    GPDMA2_Channel2_IRQHandler,
    GPDMA2_Channel3_IRQHandler,
    GPDMA2_Channel4_IRQHandler,
    GPDMA2_Channel5_IRQHandler,
    GPDMA2_Channel6_IRQHandler,
    GPDMA2_Channel7_IRQHandler,
    UART7_IRQHandler,
    UART8_IRQHandler,
    UART9_IRQHandler,
    UART12_IRQHandler,
    SDMMC2_IRQHandler,
    FPU_IRQHandler,
    ICACHE_IRQHandler,
    DCACHE1_IRQHandler,
    ETH_IRQHandler,
    ETH_WKUP_IRQHandler,
    DCMI_PSSI_IRQHandler,
    FDCAN2_IT0_IRQHandler,
    FDCAN2_IT1_IRQHandler,
    CORDIC_IRQHandler,
 	.word   FMAC_IRQHandler,
    DTS_IRQHandler,
    RNG_IRQHandler,
    0,
    0,
    HASH_IRQHandler,
    0,
    CEC_IRQHandler,
    TIM12_IRQHandler,
    TIM13_IRQHandler,
    TIM14_IRQHandler,
    I3C1_EV_IRQHandler,
    I3C1_ER_IRQHandler,
    I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler,
    LPTIM3_IRQHandler,
    LPTIM4_IRQHandler,
    LPTIM5_IRQHandler,
    LPTIM6_IRQHandler,
    */
};

void reset_handler(void) {
    // Copy .data section from FLASH to SRAM
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;

    // Zero initialize .bss
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // TODO initialize system here

    // jump to main
    main();
}

void NMI_handler(void)         __attribute__((weak, alias("default_isr")));
void hardfault_handler(void)   __attribute__((weak, alias("default_isr")));
void memmanage_handler(void)   __attribute__((weak, alias("default_isr")));
void busfault_handler(void)    __attribute__((weak, alias("default_isr")));
void usagefault_handler(void)  __attribute__((weak, alias("default_isr")));
void securefault_handler(void) __attribute__((weak, alias("default_isr")));

void default_isr(void) {
    while (1);
}