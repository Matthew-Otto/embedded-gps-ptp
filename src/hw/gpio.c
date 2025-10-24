// Embedded IoT Ethernet Lab
// Matthew Otto
// October 2025

#include "gpio.h"

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
    (void)READ_BIT(RCC->AHB2ENR, reg_addr);
}


void configure_pin(GPIO_TypeDef *GPIO_bank, uint16_t pin, uint32_t mode, uint32_t pupd, uint32_t ospeed, uint8_t alternate) {
    uint32_t pin_num = 0;
    while ((pin >> pin_num) != 0) pin_num++;
    pin_num -= 1;
    uint32_t pin_mode_ofst = pin_num << 1;
    uint32_t pin_alt_ofst = (pin_num & 0x7) << 2;

    MODIFY_REG(GPIO_bank->MODER, 0x3 << pin_mode_ofst, (mode & GPIO_MODE) << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OTYPER, 0x1 << pin_num, ((mode & GPIO_OUTPUT_TYPE) >> 4U) << pin_num);
    MODIFY_REG(GPIO_bank->PUPDR, 0x3 << pin_mode_ofst, pupd << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OSPEEDR, 0x3 << pin_mode_ofst, ospeed << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->AFR[pin_num >> 3], 0xF << pin_alt_ofst, alternate << pin_alt_ofst);
};

void configure_button() {
    GPIO_TypeDef *GPIO_bank = GPIOC;
    uint16_t pin = GPIO_PIN_13;
    uint32_t pin_num = 0;
    while ((pin >> pin_num) != 0) pin_num++;
    pin_num -= 1;
    uint32_t pin_mode_ofst = pin_num << 1;

    MODIFY_REG(GPIO_bank->MODER, 0x3 << pin_mode_ofst, (GPIO_MODE_INPUT & GPIO_MODE) << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OTYPER, 0x1 << pin_num, ((GPIO_MODE_INPUT & GPIO_OUTPUT_TYPE) >> 4U) << pin_num);
    MODIFY_REG(GPIO_bank->PUPDR, 0x3 << pin_mode_ofst, GPIO_PULLDOWN << pin_mode_ofst);
    MODIFY_REG(GPIO_bank->OSPEEDR, 0x3 << pin_mode_ofst, GPIO_SPEED_FREQ_HIGH << pin_mode_ofst);
}


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

    configure_pin(GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    configure_pin(GPIOF, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    configure_pin(GPIOG, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    configure_button();

    GPIO_TypeDef *GPIOx = GPIOF;
    GPIOx->BSRR = (uint32_t)GPIO_PIN_4;

    GPIOx = GPIOG;
    GPIOx->BSRR = (uint32_t)GPIO_PIN_4;

    GPIOx = GPIOB;
    //GPIOx->BSRR = (uint32_t)GPIO_PIN_0;
}
