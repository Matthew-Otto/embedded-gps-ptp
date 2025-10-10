#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"

#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINBANK(pin) (pin >> 8)


void GPIO_clk_en(char port) {
    // port letter A - I
    // convert to uppercase
    if (port >= 'a' && port <= 'z')
        port -= 32;
    if (port < 'A' || port > 'I')
        return;

    uint32_t reg_addr = 0x1UL << (port - 'A');
    SET_BIT(RCC->AHB2ENR, reg_addr);
    volatile uint32_t t = READ_BIT(RCC->AHB2ENR, reg_addr);
    (void)t;
}


void configure_pin(char port, uint16_t pin, uint32_t mode, uint32_t pupd, uint32_t ospeed) {
    if (port >= 'a' && port <= 'z')
        port -= 32;
    if (port < 'A' || port > 'I')
        return;

    GPIO_TypeDef *GPIO_bank = (GPIO_TypeDef *)(((port - 'A') << 10) + GPIOA_BASE_NS);
    uint32_t pin_ofst = pin;
    uint32_t pin_mode_ofst = pin << 1;

    MODIFY_REG(GPIO_bank->MODER, 0x3 << pin_mode_ofst, (mode & GPIO_MODE) << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OTYPER, 0x1 << pin_ofst, ((mode & GPIO_OUTPUT_TYPE) >> 4U) << pin_ofst);
    MODIFY_REG(GPIO_bank->PUPDR, 0x3 << pin_mode_ofst, pupd << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OSPEEDR, 0x3 << pin_mode_ofst, ospeed << pin_mode_ofst);
};


void GPIO_init() {

    GPIO_clk_en('A');
    GPIO_clk_en('B');
    GPIO_clk_en('C');
    GPIO_clk_en('D');
    GPIO_clk_en('E');
    GPIO_clk_en('F');
    GPIO_clk_en('G');
    GPIO_clk_en('H');
    GPIO_clk_en('I');

    configure_pin('B', 0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    configure_pin('F', 4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    configure_pin('G', 4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);


    GPIO_TypeDef *GPIOx = GPIOF;
    GPIOx->BSRR = (uint32_t)GPIO_PIN_4;

    GPIOx = GPIOG;
    GPIOx->BSRR = (uint32_t)GPIO_PIN_4;

    GPIOx = GPIOB;
    GPIOx->BSRR = (uint32_t)GPIO_PIN_0;
}
