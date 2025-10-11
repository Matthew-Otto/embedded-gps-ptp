#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"
#include "stm32h563xx_eth.h"
#include "gpio.h"
#include "ethernet.h"

static uint8_t MACAddr[6] = {0x00,0x00,0x00,0xE1,0x80,0x00}; // Big Endian

#define BUFFER_SIZE 1524

__attribute__((aligned(32)))
__attribute__((section(".eth_rx_buffer")))
volatile static uint8_t eth_rx_buffer[2][BUFFER_SIZE];
volatile static uint8_t eth_tx_buffer[2][BUFFER_SIZE];

volatile static ETH_DMA_desc_t DMA_tx_descriptor;
volatile static ETH_DMA_desc_t DMA_rx_descriptor;

static ETH_TypeDef *eth = ETH;

void ETH_IO_init(){
    // Configures bus / IO pins connected to Ethernet PHY

    // Enable clocks
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHTXEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHTXEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHRXEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHRXEN);

    // ETH GPIO Configuration
    // PA1     ------> ETH_REF_CLK
    // PC1     ------> ETH_MDC
    // PA2     ------> ETH_MDIO
    // PA7     ------> ETH_CRS_DV
    // PC4     ------> ETH_RXD0
    // PC5     ------> ETH_RXD1
    // PG13    ------> ETH_TXD0
    // PB15    ------> ETH_TXD1
    // PG11    ------> ETH_TX_EN

    configure_pin(GPIOC, RMII_MDC_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOC, RMII_RXD0_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOC, RMII_RXD1_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    
    configure_pin(GPIOA, RMII_REF_CLK_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOA, RMII_MDIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOA, RMII_CRS_DV_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);

    configure_pin(RMII_TXD1_GPIO_Port, RMII_TXD1_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
   
    configure_pin(GPIOG, RMII_TXT_EN_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOG, RMI_TXD0_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
}

void ETH_HW_init() {
    // Initialize hardware interrupts and timers
}


void ETH_PHY_init(){
    // Configure physical link parameters

    // enable SBS clock
    SET_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);
    (void)READ_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);

    // select RMII PHY interface (change from MII)
    MODIFY_REG(SBS->PMCR, SBS_PMCR_ETH_SEL_PHY, (uint32_t)(SBS_ETH_RMII));
    (void)SBS->PMCR; // dummy read to sync with ETH

    // Ethernet Software reset
    SET_BIT(eth->DMAMR, ETH_DMAMR_SWR);
    while (READ_BIT(eth->DMAMR, ETH_DMAMR_SWR) != 0) {};

    // set mdio clock
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_CR, ETH_MACMDIOAR_CR_DIV124); // BOZO TODO find correct div
}


void ETH_MAC_init(){
    uint32_t cfg;
    //////// Configure MAC ////////
    // configure MAC address
    WRITE_REG(eth->MACA0HR, ((uint32_t)MACAddr[5] << 8 | (uint32_t)MACAddr[4]));
    WRITE_REG(eth->MACA0LR, ((uint32_t)MACAddr[3] << 24 | (uint32_t)MACAddr[2] << 16 | 
                             (uint32_t)MACAddr[1] << 8 | (uint32_t)MACAddr[0]));

    // configure IPv4 address
    WRITE_REG(eth->MACARPAR, 0x01010101);

    // configure filtering
    // Promiscuous Mode
    WRITE_REG(eth->MACPFR, 0x1);

    // interrupts
    // cfg = eth->MACIER register

    // operating mode config
    cfg = eth->MACCR;
    cfg |= 0x1 << 31; // ARP offloading
    //cfg |= 0b010 << 28; // automatic source MAC address insertion
    cfg |= 0x1 << 27; // checksum offload
    cfg |= 0x1 << 21; // CRC stripping for Type packets
    cfg |= 0x1 << 20; // automatic pad/crc stripping
    cfg |= 0x1 << 14; // 100 Mbps
    cfg |= 0x1 << 13; // full duplex
    WRITE_REG(eth->MACCR, cfg);


    //////// Configure DMA ////////
    // cfg = eth->DMAMR;
    
    // cfg = eth->DMASBMR

    // set DSL to 64 bits
    MODIFY_REG(eth->DMACCR, ETH_DMACCR_DSL, ETH_DMACCR_DSL_64BIT);

    // Set Receive Buffers Length
    MODIFY_REG(eth->DMACRCR, ETH_DMACRCR_RBSZ, BUFFER_SIZE << 1);


    // TX descriptors
    DMA_tx_descriptor.status = 0x0;
    DMA_tx_descriptor.control = 0x0;
    DMA_tx_descriptor.buffer1_addr = (uint32_t)eth_tx_buffer;
    DMA_tx_descriptor.buffer2_next_desc = (uint32_t)&DMA_tx_descriptor;

    // Set Transmit Descriptor Ring Length
    WRITE_REG(eth->DMACTDRLR, 0);
    // Set Transmit Descriptor List Address
    WRITE_REG(eth->DMACTDLAR, (uint32_t)&DMA_tx_descriptor);
    // Set Transmit Descriptor Tail pointer
    WRITE_REG(eth->DMACTDTPR, (uint32_t)&DMA_tx_descriptor);


    // RX descriptors
    DMA_rx_descriptor.status = 0x80000000U;
    DMA_rx_descriptor.control = BUFFER_SIZE & 0x7FF; // buffer size in lower 11 bits;
    DMA_rx_descriptor.buffer1_addr = (uint32_t)eth_rx_buffer;
    DMA_rx_descriptor.buffer2_next_desc = (uint32_t)&DMA_rx_descriptor; // self-link

    // Set Receive Descriptor Ring Length
    WRITE_REG(eth->DMACRDRLR, 4);
    // Set Receive Descriptor List Address
    WRITE_REG(eth->DMACRDLAR, (uint32_t)&DMA_rx_descriptor);
    // Set Receive Descriptor Tail pointer Address
    WRITE_REG(eth->DMACRDTPR, (uint32_t)&DMA_rx_descriptor);

    SET_BIT(eth->DMACRCR, ETH_DMACRCR_SR); // start receive
    SET_BIT(eth->DMACTCR, ETH_DMACTCR_ST); // start transmit

    // Enable MAC transmitter and receiver
    SET_BIT(eth->MACCR, 0x3);
}


void ETH_init(){
    ETH_IO_init();
    ETH_HW_init();
    ETH_PHY_init(ETH);
    ETH_MAC_init(ETH);
}



void ETH_construct_frame(ethernet_frame_t *frame, uint8_t *dest_mac, uint8_t *src_mac, uint16_t eth_type, uint8_t *payload, uint16_t payload_len){
    memcpy(frame->dest_mac, dest_mac, 6);
    memcpy(frame->src_mac, src_mac, 6);
    frame->eth_type = eth_type;
    memcpy(frame->payload, payload, payload_len);
}

void ETH_send_frame(ethernet_frame_t *frame, uint16_t payload_len){
    // Send an Ethernet frame using DMA.

    //ETH_buffer_t tx_buffer;

    //ETH_construct_frame(&frame);

    //tx_buffer.data = (uint8_t*)&frame;
    //tx_buffer.len = 14 + payload_len;
    //tx_buffer.next = NULL;
    //tx_config.buffer = &tx_buffer;

}

int eth_receive(uint8_t *out, uint32_t *len)
{
    if (DMA_rx_descriptor.status & (1U << 31))
        return 0; // still owned by DMA, nothing received yet

    uint32_t frame_len = (DMA_rx_descriptor.status >> 16) & 0x3FFF;
    memcpy(out, (void *)DMA_rx_descriptor.buffer1_addr, frame_len);
    *len = frame_len;

    // Invalidate D-cache here if enabled

    // Return ownership to DMA
    DMA_rx_descriptor.status = (1U << 31);

    // Update tail pointer so DMA can reuse descriptor
    eth->DMACRDTPR = (uint32_t)&DMA_rx_descriptor;

    return 1;
}
