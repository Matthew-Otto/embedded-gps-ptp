#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "stm32h563xx.h"
#include "stm32h563xx_gpio.h"
#include "stm32h563xx_eth.h"
#include "gpio.h"
#include "ethernet.h"

static uint8_t MACAddr[6] = {0x00,0x80,0xE1,0x00,0x00,0x00};
static uint8_t IPv4_ADDR[4] = {10, 1, 123, 1};

#define BUFFER_SIZE 1524
#define RX_DSC_CNT 4
#define TX_DSC_CNT 4

__attribute__((section(".eth_rx_buffer")))
volatile static uint8_t eth_rx_buffer[RX_DSC_CNT][BUFFER_SIZE];
__attribute__((section(".eth_tx_buffer")))
volatile static uint8_t eth_tx_buffer[TX_DSC_CNT][BUFFER_SIZE];

volatile static ETH_rx_desc_u dma_rx_desc[RX_DSC_CNT];
volatile static ETH_tx_desc_t dma_tx_desc[TX_DSC_CNT];
static uint32_t current_rx_desc_idx = 0;
static uint32_t current_tx_desc_idx = 0;


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
    // PA2     ------> ETH_MDIO
    // PC1     ------> ETH_MDC
    // PA7     ------> ETH_CRS_DV
    // PC4     ------> ETH_RXD0
    // PC5     ------> ETH_RXD1
    // PG11    ------> ETH_TX_EN
    // PG13    ------> ETH_TXD0
    // PB15    ------> ETH_TXD1

    
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


uint16_t read_PHY_reg(uint8_t phy_addr) {
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_MOC_RD, 0b11 << 2); // read mode
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_RDA, (phy_addr & 0x1F) << 16); // reg addr
    SET_BIT(eth->MACMDIOAR, ETH_MACMDIOAR_MB);

    while (READ_BIT(eth->MACMDIOAR, ETH_MACMDIOAR_MB));
    return (uint16_t)READ_BIT(eth->MACMDIODR, ETH_MACMDIODR_MD);
}

void write_PHY_reg(uint8_t phy_addr, uint16_t data) {
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_MOC_RD, 0b01 << 2); // write mode
    MODIFY_REG(eth->MACMDIODR, ETH_MACMDIODR_MD, data);
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_RDA, (phy_addr & 0x1F) << 16); // reg addr
    SET_BIT(eth->MACMDIOAR, ETH_MACMDIOAR_MB);

    while (READ_BIT(eth->MACMDIOAR, ETH_MACMDIOAR_MB));
}

void ETH_PHY_init(){
    // Configure physical link parameters

    // enable SBS clock
    SET_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);
    (void)READ_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);

    // select RMII PHY interface (change from MII)
    MODIFY_REG(SBS->PMCR, SBS_PMCR_ETH_SEL_PHY, (uint32_t)(SBS_ETH_RMII));
    (void)SBS->PMCR; // dummy read to sync with ETH

    // set MDIO clock
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_CR, ETH_MACMDIOAR_CR_DIV124);
}



void ETH_MAC_init(){
    uint32_t cfg;
    // Ethernet Software reset
    SET_BIT(eth->DMAMR, ETH_DMAMR_SWR);
    while (READ_BIT(eth->DMAMR, ETH_DMAMR_SWR) != 0) {};

    //////// Configure MAC ////////
    // configure MAC address
    WRITE_REG(eth->MACA0HR, ((uint32_t)MACAddr[0] << 8 | (uint32_t)MACAddr[1]));
    WRITE_REG(eth->MACA0LR, ((uint32_t)MACAddr[2] << 24 | (uint32_t)MACAddr[3] << 16 | 
                             (uint32_t)MACAddr[4] << 8 | (uint32_t)MACAddr[5]));

    // configure IPv4 address
    WRITE_REG(eth->MACARPAR, (IPv4_ADDR[0]<<24 | IPv4_ADDR[1]<<16 | IPv4_ADDR[2]<<8 | IPv4_ADDR[3]));

    volatile int x = READ_REG(eth->MACA0HR);

    // configure filtering
    // Promiscuous Mode
    WRITE_REG(eth->MACPFR, 0x1);

    // disable crc checking
    WRITE_REG(eth->MACECR, 0x1<<16);

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

    //////// Configure MTL ////////
    WRITE_REG(eth->MTLRQOMR, 0x7 << 4);
}


void ETH_DMA_init(void) {
    // TX descriptors
    for (int i = 0; i < TX_DSC_CNT; i++) {
        dma_tx_desc[i].buffer1_addr = (uint32_t)eth_tx_buffer[i];
        dma_tx_desc[i].buffer2_addr = 0;
        dma_tx_desc[i].TDES2 = 0x0;
        dma_tx_desc[i].TDES3 = 0x1 << 29 | 0x1 << 28;
    }
    // Set Transmit Descriptor Ring Length
    WRITE_REG(eth->DMACTDRLR, TX_DSC_CNT);
    // Set Transmit Descriptor List Address
    WRITE_REG(eth->DMACTDLAR, (uint32_t)&dma_tx_desc[0]);
    // Set Transmit Descriptor Tail pointer
    WRITE_REG(eth->DMACTDTPR, (uint32_t)&dma_tx_desc[TX_DSC_CNT-1]);


    // RX descriptors
    for (int i = 0; i < RX_DSC_CNT; i++) {
        ETH_rx_rd_desc_t *desc  = &dma_rx_desc[i].rd;
        desc->buffer1_addr = (uint32_t)eth_rx_buffer[i];
        desc->buffer2_addr = 0;
        desc->status = 0x81;
    }
    // Set Receive Buffers Length
    MODIFY_REG(eth->DMACRCR, ETH_DMACRCR_RBSZ, BUFFER_SIZE << 1);
    // Set Receive Descriptor Ring Length
    WRITE_REG(eth->DMACRDRLR, RX_DSC_CNT-1);
    // Set Receive Descriptor List Address
    WRITE_REG(eth->DMACRDLAR, (uint32_t)&dma_rx_desc[0]);
    // Set Receive Descriptor Tail pointer Address
    WRITE_REG(eth->DMACRDTPR, (uint32_t)&dma_rx_desc[RX_DSC_CNT-1]);

}


void ETH_init(){
    ETH_IO_init();
    ETH_HW_init();
    ETH_PHY_init();
    ETH_MAC_init();
    ETH_DMA_init();

    // Start DMA transmit and receive
    SET_BIT(eth->DMACTCR, ETH_DMACTCR_ST);
    SET_BIT(eth->DMACRCR, ETH_DMACRCR_SR);

    // Enable MAC transmitter and receiver
    SET_BIT(eth->MACCR, 0x3);
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

// Checks for valid RX descriptors
// If one exists, process it before resetting DMA descriptor
int ETH_receive_frame(){
    volatile ETH_rx_rd_desc_t *rd_desc = &dma_rx_desc[current_rx_desc_idx].rd;
    volatile ETH_rx_wb_desc_t *wb_desc = &dma_rx_desc[current_rx_desc_idx].wb;

    // check that DMA has released this descriptor
    if (wb_desc->status & (0x1<<15))
        return -1;

    // TODO process frame here


    // Configure descriptor for receive and release back to DMA
    rd_desc->buffer1_addr = (uint32_t)eth_rx_buffer[current_rx_desc_idx];
    rd_desc->status = 0x81; // set own bit and buffer1_valid
    // Update current RX descriptor idx
    current_rx_desc_idx = (current_rx_desc_idx + 1) % RX_DSC_CNT;
    // Update RX descriptor tail pointer;
    WRITE_REG(eth->DMACRDTPR, (uint32_t)&dma_rx_desc[current_rx_desc_idx]);

    return 0;
}

