#include "mcu.h"
#include "fifo.h"

static FIFO8_t *tx_fifo;
static FIFO8_t *rx_fifo;

void configure_basic_usart(USART_TypeDef *uart, uint16_t baudrate) {
    // Clear config and disable usart
    WRITE_REG(uart->CR1, 0);
    SET_BIT(uart->CR1, USART_CR1_FIFOEN); // enable FIFO
    SET_BIT(uart->CR1, USART_CR1_TE);     // enable transmit
    SET_BIT(uart->CR1, USART_CR1_RE);     // enable receive
    
    WRITE_REG(uart->BRR, 250000000 / baudrate);

    SET_BIT(uart->CR1, USART_CR1_RXNEIE_RXFNEIE); // RX fifo not empty
    //SET_BIT(uart->CR1, USART_CR1_TXFNFIE); // TX fifo not full

    SET_BIT(uart->CR1, USART_CR1_UE);     // enable UART
}

void init_uart(uint8_t uart_idx, uint16_t baudrate) {
    switch (uart_idx) {
        case 2:
            // Configure USART2 clock
            SET_BIT(RCC->APB1LENR, RCC_APB1LENR_USART2EN);
            (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_USART2EN);
            // set basic configuration
            configure_basic_usart(USART2, baudrate);
            // enable interrupts
            NVIC_SetPriority(USART2_IRQn, 5);
            NVIC_EnableIRQ(USART2_IRQn);
            break;
    }

    // TODO FIFO pair per uart
    tx_fifo = fifo8_init(256);
    rx_fifo = fifo8_init(256);
}

void USART2_IRQHandler(void) {
    uint8_t data;
    // if data in rx hw fifo, put it in software fifo
    while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
        data = (0xFF & USART2->RDR);
        fifo8_put(rx_fifo, data);
    }
}

void uart_in_string(char *buff, uint32_t buff_size) {
    uint32_t length = 0;
    uint8_t inchar;
    do {
        fifo8_get_blocking(rx_fifo, &inchar);
        
        if (inchar == '\n')
            continue;
        if (inchar == '\r') {
            break;
        } else if (length < buff_size) {
            *buff++ = inchar;
            length++;
        }
    } while (length < buff_size);
    *buff = '\0';
}

int uart_in_string_nonblocking(char *buff, uint32_t buff_size) {
    uint32_t length = 0;
    uint8_t inchar;
    do {
        if (fifo8_get(rx_fifo, &inchar) < 0) {
            *buff = '\0';
            return 1;
        }
        
        if (inchar == '\n')
            continue;
        if (inchar == '\r') {
            break;
        } else if (length < buff_size) {
            *buff++ = inchar;
            length++;
        }
    } while (length < buff_size);
    *buff = '\0';

    return 0;
}
