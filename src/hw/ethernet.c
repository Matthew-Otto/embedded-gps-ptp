#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "mcu.h"
#include "gpio.h"
#include "ethernet.h"
#include "ip.h"
#include "ptp.h"

// TODO:
// filter mac address but allow PTP multi/broadcast

#ifdef MASTER
static uint8_t MACAddr[6] = {0x00,0x80,0xE1,0x00,0x00,0x00};
static uint8_t IPv4_ADDR[4] = {10, 1, 123, 1};
#else
static uint8_t MACAddr[6] = {0x00,0x80,0xE1,0x12,0x34,0x00};
static uint8_t IPv4_ADDR[4] = {10, 1, 123, 2};
#endif
//static uint8_t IPv4_ADDR[4] = {10, 0, 4, 123};

#define BUFFER_SIZE 1524
#define RX_DSC_CNT 4
#define TX_DSC_CNT 8

// Global variables
uint32_t rx_timestamp_sec;
uint32_t rx_timestamp_nsec;

// Local (static) variables
__attribute__((section(".eth_rx_buffer")))
volatile static uint8_t eth_rx_buffer[RX_DSC_CNT][BUFFER_SIZE];
__attribute__((section(".eth_tx_buffer")))
volatile static uint8_t eth_tx_buffer[TX_DSC_CNT][BUFFER_SIZE];

// BOZO context descriptors dont need frame buffers
volatile static ETH_rx_desc_u dma_rx_desc[RX_DSC_CNT];
volatile static ETH_tx_desc_u dma_tx_desc[TX_DSC_CNT];
static uint32_t current_rx_desc_idx = 0;
static uint32_t current_tx_desc_idx = 0;


static ETH_TypeDef *eth = ETH;

static volatile int cnt = 0;

void ETH_IRQHandler(void) {
    volatile uint32_t int_src = READ_REG(eth->DMAISR);
    volatile uint32_t isr;
    if (int_src & 0x1) {
        // DMA
        isr = READ_REG(eth->DMACSR);
        // receive interrupt
        if (isr & ETH_DMACIER_RIE) ETH_receive_frame();
    }
    /* else if (int_src & (0x1<<17)) {

    }
    else if (int_src & (0x1<<16)) {
        // MTL
        isr = READ_REG(eth->MTLISR);
    } */

    // clear all DMA interrupt bits
    // The interrupt is cleared only when all the bits of Interrupt status register (ETH_DMAISR) are cleared.
    WRITE_REG(eth->DMACSR, 0xFFFFFFFF);
}

void ETH_IO_init(){
    // Enable clocks
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHTXEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHTXEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHRXEN);
    (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHRXEN);

    // Configure GPIO pins connecting MCU to Ethernet PHY
    configure_pin(GPIOC, RMII_MDC_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOC, RMII_RXD0_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOC, RMII_RXD1_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    
    configure_pin(GPIOA, RMII_REF_CLK_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOA, RMII_MDIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOA, RMII_CRS_DV_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);

    configure_pin(GPIOB, RMII_TXD1_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);

    configure_pin(GPIOG, RMII_TXT_EN_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
    configure_pin(GPIOG, RMI_TXD0_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF11_ETH);
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
    // Configure physical link parameters to LAN8742A-CZ-TR

    // enable SBS clock
    SET_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);
    (void)READ_BIT(RCC->APB3ENR, RCC_APB3ENR_SBSEN);

    // select RMII PHY interface (change from MII)
    MODIFY_REG(SBS->PMCR, SBS_PMCR_ETH_SEL_PHY, (uint32_t)(SBS_ETH_RMII));
    (void)SBS->PMCR; // dummy read to sync with ETH

    // set MDIO clock
    MODIFY_REG(eth->MACMDIOAR, ETH_MACMDIOAR_CR, ETH_MACMDIOAR_CR_DIV124);

    // Wait until AutoNeg has completed
    volatile uint16_t scsr = read_PHY_reg(31);
    //while (!(scsr & (0x1 << 12))) scsr = read_PHY_reg(31);
    // Read PHY status (get link speed)
    volatile uint8_t speed = (scsr >> 2) & 0x7;
}


void ETH_MAC_init(){
    uint32_t cfg;
    // Ethernet Software reset
    SET_BIT(eth->DMAMR, ETH_DMAMR_SWR);
    while (READ_BIT(eth->DMAMR, ETH_DMAMR_SWR) != 0) {};

    //////// Configure MAC ////////
    // configure MAC address
    WRITE_REG(eth->MACA0HR, ((uint32_t)MACAddr[5] << 8 | (uint32_t)MACAddr[4]));
    WRITE_REG(eth->MACA0LR, ((uint32_t)MACAddr[3] << 24 | (uint32_t)MACAddr[2] << 16 | 
                             (uint32_t)MACAddr[1] << 8 | (uint32_t)MACAddr[0]));

    // configure IPv4 address
    WRITE_REG(eth->MACARPAR, (IPv4_ADDR[0]<<24 | IPv4_ADDR[1]<<16 | IPv4_ADDR[2]<<8 | IPv4_ADDR[3]));

    // configure filtering
    cfg = 0;
    cfg |= ETH_MACPFR_PR; // Promiscuous Mode
    cfg |= ETH_MACPFR_SAF; // Filter MAC address
    WRITE_REG(eth->MACPFR, cfg);

    // disable crc checking
    WRITE_REG(eth->MACECR, cfg);

    // interrupts
    // cfg = eth->MACIER register

    // operating mode config
    cfg = eth->MACCR;
    cfg |= ETH_MACCR_ARP; // ARP offloading
    cfg |= ETH_MACCR_SARC_REPADDR0; // automatic source MAC address 1 replacement
    cfg |= ETH_MACCR_IPC; // checksum offload
    cfg |= ETH_MACCR_CST; // CRC stripping for Type packets
    cfg |= ETH_MACCR_ACS; // automatic pad/crc stripping
    cfg |= ETH_MACCR_FES; // 100 Mbps
    cfg |= ETH_MACCR_DM; // full duplex
    WRITE_REG(eth->MACCR, cfg);

    //////// Configure MTL ////////
    WRITE_REG(eth->MTLRQOMR, 0x7 << 4); // configure receive MTL
    WRITE_REG(eth->MTLTQOMR, 0x2 << 2); // configure transmit MTL
}


static inline void init_read_descriptor(ETH_rx_rd_desc_t *desc, uint16_t idx) {
    // Configure descriptor for receive and release back to DMA
    desc->buffer1_addr = (uint32_t)eth_rx_buffer[idx];
    desc->status = 0xC1; // set own bit , enable interrupt, and buffer1_valid
}


void ETH_DMA_init(void) {
    // TX descriptors
    // Set Transmit Descriptor Ring Length
    WRITE_REG(eth->DMACTDRLR, TX_DSC_CNT-1);
    // Set Transmit Descriptor List Address
    WRITE_REG(eth->DMACTDLAR, (uint32_t)&dma_tx_desc[0]);

    // RX descriptors
    for (int i = 0; i < RX_DSC_CNT; i++) {
        ETH_rx_rd_desc_t *desc = &dma_rx_desc[i].rd;
        init_read_descriptor(desc, i);
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


void ETH_int_init() {
    // Initialize hardware interrupts and timers

    // Enable ETH DMA interrupts
    uint32_t dma_ints = 0;
    dma_ints |= ETH_DMACIER_NIE;  // Normal DMA ints bulk-enable
    //dma_ints |= ETH_DMACIER_AIE;  // Abnormal DMA ints bulk-enable
    //dma_ints |= ETH_DMACIER_ERIE; // Early receive interrupts
    dma_ints |= ETH_DMACIER_RIE;  // Receive interrupts
    dma_ints |= ETH_DMACIER_TIE; // Transmit interrupts
    SET_BIT(eth->DMACIER, dma_ints);

    // Enable ETH MAC interrupts
    uint32_t mac_ints = 0;
    //mac_ints |= ETH_MACIER_TSIE;  // Timestamp interrupts
    mac_ints |= ETH_MACIER_PHYIE; // PHY interrupts
    //SET_BIT(eth->MACIER, mac_ints);

    // enable ETH interrupts in NVIC
    NVIC_SetPriority(ETH_IRQn, 5);
    NVIC_EnableIRQ(ETH_IRQn);
}

void ETH_PTP_init() {
    uint32_t cfg;

    // Mask the Timestamp Trigger interrupt
    CLEAR_BIT(eth->MACIER, ETH_MACIER_TSIE);
    
    // Enable timestamping
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSENA);

    // configure subsecond increment value
    const int CLK_PERIOD = 4; // clk_ptp_i period (4ns for 250MHz)
    SET_BIT(eth->MACSSIR, 8 << ETH_MACMACSSIR_SSINC_Pos);
    //SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSCTRLSSR);

    // Update system time
    WRITE_REG(eth->MACSTSUR, 0);
    WRITE_REG(eth->MACSTNUR, 0);
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSINIT); // initialize timestamp value
    while (READ_BIT(eth->MACTSCR, ETH_MACTSCR_TSINIT)); // wait for completion

    // Enable PTP offloading features
#ifdef MASTER
    // Automatic PTP Sync messages
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_SNAPTYPSEL_Msk, 0 << ETH_MACTSCR_SNAPTYPSEL_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSMSTRENA_Msk, 1 << ETH_MACTSCR_TSMSTRENA_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSEVNTENA_Msk, 1 << ETH_MACTSCR_TSEVNTENA_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSIPV4ENA_Msk, 1 << ETH_MACTSCR_TSIPV4ENA_Pos);
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSIPENA);
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSVER2ENA);

    // Enable PTP offloading
    cfg = 0;
    cfg |= ETH_MACPOCR_PTOEN;
    cfg |= ETH_MACPOCR_ASYNCEN;
    //cfg |= domain_num << ETH_MACPOCR_DN_Pos;
    WRITE_REG(eth->MACPOCR, cfg);

    // Configure Source port identity
    WRITE_REG(eth->MACSPI0R, 0xdeadbeef);
    WRITE_REG(eth->MACSPI1R, 0xd066f00d);
    WRITE_REG(eth->MACSPI2R, 0x1234);

    // Set automatic sync message period
    MODIFY_REG(eth->MACLMIR, ETH_MACLMIR_LSI_Msk, 0 << ETH_MACLMIR_LSI_Pos);
#else
    // Automatic PTP Sync messages
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_SNAPTYPSEL_Msk, 0 << ETH_MACTSCR_SNAPTYPSEL_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSMSTRENA_Msk, 0 << ETH_MACTSCR_TSMSTRENA_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSEVNTENA_Msk, 1 << ETH_MACTSCR_TSEVNTENA_Pos);
    MODIFY_REG(eth->MACTSCR, ETH_MACTSCR_TSIPV4ENA_Msk, 1 << ETH_MACTSCR_TSIPV4ENA_Pos);
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSIPENA);
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSVER2ENA);

    // BOZO overwrite ts even if prev one was missed
    //SET_BIT(eth->MACTSCR, ETH_MACTSCR_TXTSSTSM);

    // Enable PTP offloading
    cfg = 0;
    cfg |= ETH_MACPOCR_PTOEN;
    //cfg |= domain_num << ETH_MACPOCR_DN_Pos;
    WRITE_REG(eth->MACPOCR, cfg);

    // Configure Source port identity
    WRITE_REG(eth->MACSPI0R, 0xdeadbeef);
    WRITE_REG(eth->MACSPI1R, 0xd066f00d);
    WRITE_REG(eth->MACSPI2R, 0x4321);

    // Number of sync messages received before sending DELAY_REQ
    MODIFY_REG(eth->MACLMIR, ETH_MACLMIR_DRSYNCR_Msk, 0 << ETH_MACLMIR_DRSYNCR_Pos);
#endif

    // BOZO drop transmit status from MAC in MTL
    //SET_BIT(eth->MTLOMR, ETH_MTLOMR_DTXSTS);

    // Enable timestamp interrupt
    //SET_BIT(eth->MACIER, ETH_MACIER_TSIE);
}


// Configure Ethernet PPS to PG8
void ETH_PPS_init(void) {
    // configure PPS pin PG8
    configure_pin(GPIOG, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF11_ETH);

    // Exceeding target time (0 unless set) triggers PPS output
    MODIFY_REG(eth->MACPPSCR, ETH_MACPPSCR_TRGTMODSEL0_Msk, 0x3 << ETH_MACPPSCR_TRGTMODSEL0_Pos);
    // Freq of PPS
    MODIFY_REG(eth->MACPPSCR, ETH_MACPPSCR_PPSCTRL_Msk, 0xA << ETH_MACPPSCR_PPSCTRL_Pos);
}


void ETH_update_PTP_TS(const int32_t offset_sec, const int32_t offset_nsec) {
    WRITE_REG(eth->MACSTSUR, offset_sec);
    WRITE_REG(eth->MACSTNUR, offset_nsec);
    // Update and wait for completion
    SET_BIT(eth->MACTSCR, ETH_MACTSCR_TSUPDT);
    while (READ_BIT(eth->MACTSCR, ETH_MACTSCR_TSUPDT));
}

void ETH_init(){
    ETH_IO_init();
    ETH_PHY_init();
    ETH_MAC_init();
    ETH_DMA_init();
    ETH_int_init();
    ETH_PTP_init();
    ETH_PPS_init();
    
    // Start DMA transmit and receive
    SET_BIT(eth->DMACTCR, ETH_DMACTCR_ST);
    SET_BIT(eth->DMACRCR, ETH_DMACRCR_SR);
    
    // Enable MAC transmitter and receiver
    SET_BIT(eth->MACCR, 0x3);
}


// finds a valid TX buffer, fills in the correct ethernet header and returns the buffer address
void ETH_build_header(uint8_t **buffer, const uint8_t *dst, const uint16_t ethertype) {
    *buffer = (uint8_t *)eth_tx_buffer[current_tx_desc_idx];

    memcpy(*buffer, dst, 6);
    memcpy(*buffer + 6, MACAddr, 6);
    *(*buffer + 12) = ethertype;
}


void ETH_send_timestamp_frame(uint8_t *data, uint16_t length) {
    // Set up context descriptor
    ETH_tx_ctx_desc_t *ctx_desc = &dma_tx_desc[current_tx_desc_idx].ctx;
    ctx_desc->ctrl = 0;
    ctx_desc->ctrl |= (0x1 << 30); // context type
    //ctx_desc->ctrl |= (0x1 << 27); // one-step correction
    ctx_desc->ctrl |= (0x1 << 31);

    // Update current TX descriptor idx
    current_tx_desc_idx = (current_tx_desc_idx + 1) % TX_DSC_CNT;
    // Update TX descriptor tail pointer;
    WRITE_REG(eth->DMACTDTPR, (uint32_t)&dma_tx_desc[current_tx_desc_idx]);

    ETH_send_frame(data, length);
}


void ETH_send_frame(uint8_t *data, uint16_t length){
    // Send an Ethernet frame using DMA.
    // TODO check for free descriptor

    // Set up packet descriptor
    ETH_tx_rd_desc_t *desc = &dma_tx_desc[current_tx_desc_idx].rd;
    ETH_tx_wb_desc_t *wb_desc = &dma_tx_desc[current_tx_desc_idx].wb;

    uint8_t *buffer = eth_tx_buffer[current_tx_desc_idx];
    memcpy(buffer, data, length);
    
    desc->buffer1_addr = (uint32_t)&eth_tx_buffer[current_tx_desc_idx];
    desc->buffer1_len = length & 0x3FFF;
    desc->buffer2_len = 0x1 << 14; // enable timestamp
    desc->ctrl = 0;
    desc->ctrl |= 0x1 << 29 | 0x1 << 28; // first and last descriptor
    desc->ctrl |= 0b10 << 23; // src addr insertion
    desc->ctrl |= 0x1 << 31; // set own bit

    // Update current TX descriptor idx
    current_tx_desc_idx = (current_tx_desc_idx + 1) % TX_DSC_CNT;
    // Update TX descriptor tail pointer;
    WRITE_REG(eth->DMACTDTPR, (uint32_t)&dma_tx_desc[current_tx_desc_idx]);
}


// Checks for valid RX descriptors
// If one exists, process it before resetting DMA descriptor
int ETH_receive_frame(){
    volatile ETH_rx_rd_desc_t *rd_desc = &dma_rx_desc[current_rx_desc_idx].rd;
    volatile ETH_rx_wb_desc_t *wb_desc = &dma_rx_desc[current_rx_desc_idx].wb;

    // Check that DMA has released this descriptor
    if (wb_desc->status & (0x1<<15))
        return -1;

    // Check for timestamp
    uint8_t clear_ctx_desc = 0;
    uint16_t ctx_desc_idx = (current_rx_desc_idx + 1) % RX_DSC_CNT;
    volatile ETH_rx_ctx_desc_t *ctx_desc = &dma_rx_desc[ctx_desc_idx].ctx;
    if ((ctx_desc->ctrl >> 30) == 0x1) { // DMA has released own bit and this descriptor is of context type
        rx_timestamp_sec = ctx_desc->timestamp_high;
        rx_timestamp_nsec = ctx_desc->timestamp_low;
        clear_ctx_desc = 1;
    }

    // If no errors, process packet
    if (!(wb_desc->pkt_len & (0x1<<15))){
        ETH_process_frame(eth_rx_buffer[current_rx_desc_idx]);

        // BOZO
        //ETH_send_frame(eth_rx_buffer[current_rx_desc_idx], wb_desc->pkt_len);
    }

    // Configure descriptor for receive and release back to DMA
    init_read_descriptor(rd_desc, current_rx_desc_idx);
    // Update current RX descriptor idx
    current_rx_desc_idx = (current_rx_desc_idx + 1) % RX_DSC_CNT;

    // Reconfigure context descriptor if it exists
    if (clear_ctx_desc) {
        // Configure descriptor for receive and release back to DMA
        init_read_descriptor((ETH_rx_rd_desc_t *)ctx_desc, current_rx_desc_idx);
        // Update current RX descriptor idx
        current_rx_desc_idx = (current_rx_desc_idx + 1) % RX_DSC_CNT;
    }

    // Update RX descriptor tail pointer;
    WRITE_REG(eth->DMACRDTPR, (uint32_t)&dma_rx_desc[current_rx_desc_idx]);

    // BOZO check next descriptor just in case
    volatile ETH_rx_wb_desc_t *wb_desc2 = &dma_rx_desc[current_rx_desc_idx].wb;
    if (!(wb_desc2->status & (0x1<<15))) {
        __asm volatile("BKPT #0");
    }

    return 0;
}

int ETH_process_frame(uint8_t *frame) {
    eth_header_t *header = (eth_header_t *)frame;
    uint16_t ethertype = ntohs(header->ethertype);

    uint8_t *payload = ((uint8_t *)frame + sizeof(eth_header_t));
    if (ethertype == ETHERTYPE_PTP) {
        process_ptp_message(payload);
    } else if (ethertype == ETHERTYPE_IPv4) {
        process_packet(frame);
    }
}

// dumps various status registers useful for debugging
/* void ETH_dump_SR(void) {
    volatile uint32_t global_ints_disabled = __get_PRIMASK();  // Should be 0 (interrupts enabled)

    // Ethernet interrupt category
    volatile uint32_t ETH_int_stat = READ_REG(eth->DMAISR);
    volatile uint8_t MACIS = (ETH_int_stat & ETH_DMAISR_MACIS) >> ETH_DMAISR_MACIS_Pos;     // MAC Interrupt Status
    volatile uint8_t MTLIS = (ETH_int_stat & ETH_DMAISR_MTLIS) >> ETH_DMAISR_MTLIS_Pos;     // MTL Interrupt Status
    volatile uint8_t DMACIS = (ETH_int_stat & ETH_DMAISR_DMACIS) >> ETH_DMAISR_DMACIS_Pos;  // DMA Channel Interrupt Status

    // DMA status bits
    volatile uint32_t DMA_stat = READ_REG(eth->DMACSR);
    volatile uint8_t REB = (DMA_stat & ETH_DMACSR_REB) >> ETH_DMACSR_REB_Pos;  // Rx DMA Error Bits
    volatile uint8_t TEB = (DMA_stat & ETH_DMACSR_TEB) >> ETH_DMACSR_TEB_Pos;  // Tx DMA Error Bits
    volatile uint8_t NIS = (DMA_stat & ETH_DMACSR_NIS) >> ETH_DMACSR_NIS_Pos;  // Normal Interrupt Summary
    volatile uint8_t AIS = (DMA_stat & ETH_DMACSR_AIS) >> ETH_DMACSR_AIS_Pos;  // Abnormal Interrupt Summary
    volatile uint8_t CDE = (DMA_stat & ETH_DMACSR_CDE) >> ETH_DMACSR_CDE_Pos;  // Context Descriptor Error
    volatile uint8_t FBE = (DMA_stat & ETH_DMACSR_FBE) >> ETH_DMACSR_FBE_Pos;  // Fatal Bus Error
    volatile uint8_t ERI = (DMA_stat & ETH_DMACSR_ERI) >> ETH_DMACSR_ERI_Pos;  // Early Receive Interrupt
    volatile uint8_t ETI = (DMA_stat & ETH_DMACSR_ETI) >> ETH_DMACSR_ETI_Pos;  // Early Transmit Interrupt
    volatile uint8_t RWT = (DMA_stat & ETH_DMACSR_RWT) >> ETH_DMACSR_RWT_Pos;  // Receive Watchdog Timeout
    volatile uint8_t RPS = (DMA_stat & ETH_DMACSR_RPS) >> ETH_DMACSR_RPS_Pos;  // Receive Process Stopped
    volatile uint8_t RBU = (DMA_stat & ETH_DMACSR_RBU) >> ETH_DMACSR_RBU_Pos;  // Receive Buffer Unavailable
    volatile uint8_t RI  = (DMA_stat & ETH_DMACSR_RI)  >> ETH_DMACSR_RI_Pos;   // Receive Interrupt
    volatile uint8_t TBU = (DMA_stat & ETH_DMACSR_TBU) >> ETH_DMACSR_TBU_Pos;  // Transmit Buffer Unavailable
    volatile uint8_t TPS = (DMA_stat & ETH_DMACSR_TPS) >> ETH_DMACSR_TPS_Pos;  // Transmit Process Stopped
    volatile uint8_t TI  = (DMA_stat & ETH_DMACSR_TI)  >> ETH_DMACSR_TI_Pos;   // Transmit Interrupt

    // DMA interrupt enable bits
    volatile uint32_t DMA_int_en = READ_REG(eth->DMACIER);
    volatile uint8_t NIE = (DMA_int_en & ETH_DMACIER_NIE) >> ETH_DMACIER_NIE_Pos;     // Normal Interrupt Summary Enable
    volatile uint8_t AIE = (DMA_int_en & ETH_DMACIER_AIE) >> ETH_DMACIER_AIE_Pos;     // Abnormal Interrupt Summary Enable
    volatile uint8_t CDEE = (DMA_int_en & ETH_DMACIER_CDEE) >> ETH_DMACIER_CDEE_Pos;  // Context Descriptor Error Enable
    volatile uint8_t FBEE = (DMA_int_en & ETH_DMACIER_FBEE) >> ETH_DMACIER_FBEE_Pos;  // Fatal Bus Error Enable
    volatile uint8_t ERIE = (DMA_int_en & ETH_DMACIER_ERIE) >> ETH_DMACIER_ERIE_Pos;  // Early Receive Interrupt Enable
    volatile uint8_t ETIE = (DMA_int_en & ETH_DMACIER_ETIE) >> ETH_DMACIER_ETIE_Pos;  // Early Transmit Interrupt Enable
    volatile uint8_t RWTE = (DMA_int_en & ETH_DMACIER_RWTE) >> ETH_DMACIER_RWTE_Pos;  // Receive Watchdog Timeout Enable
    volatile uint8_t RSE = (DMA_int_en & ETH_DMACIER_RSE) >> ETH_DMACIER_RSE_Pos;     // Receive Stopped Enable
    volatile uint8_t RBUE = (DMA_int_en & ETH_DMACIER_RBUE) >> ETH_DMACIER_RBUE_Pos;  // Receive Buffer Unavailable Enable
    volatile uint8_t RIE = (DMA_int_en & ETH_DMACIER_RIE) >> ETH_DMACIER_RIE_Pos;     // Receive Interrupt Enable
    volatile uint8_t TBUE = (DMA_int_en & ETH_DMACIER_TBUE) >> ETH_DMACIER_TBUE_Pos;  // Transmit Buffer Unavailable Enable
    volatile uint8_t TXSE = (DMA_int_en & ETH_DMACIER_TXSE) >> ETH_DMACIER_TXSE_Pos;  // Transmit Stopped Enable
    volatile uint8_t TIE = (DMA_int_en & ETH_DMACIER_TIE) >> ETH_DMACIER_TIE_Pos;     // Transmit Interrupt Enable

    // contains address that points to the start of the respective descriptor lists
    volatile uint32_t DMA_TX_descr_list_addr = READ_REG(eth->DMACTDLAR);
    volatile uint32_t DMA_RX_descr_list_addr = READ_REG(eth->DMACRDLAR);

    // points to the last valid DMA descriptor
    volatile uint32_t DMA_TX_descr_tail_ptr = READ_REG(eth->DMACTDTPR);
    volatile uint32_t DMA_RX_descr_tail_ptr = READ_REG(eth->DMACRDTPR);

    volatile uint32_t TX_queue_stat = READ_REG(eth->MTLTQDR);
    volatile uint8_t STXSTSF = (TX_queue_stat & ETH_MTLTQDR_STXSTSF) >> ETH_MTLTQDR_STXSTSF_Pos;        //  Number of Status Words in the Tx Status FIFO of Queue
    volatile uint8_t PTXQ = (TX_queue_stat & ETH_MTLTQDR_PTXQ) >> ETH_MTLTQDR_PTXQ_Pos;                 //  Number of Packets in the Transmit Queue
    volatile uint8_t TXSTSFSTS = (TX_queue_stat & ETH_MTLTQDR_TXSTSFSTS) >> ETH_MTLTQDR_TXSTSFSTS_Pos;  //  MTL Tx Status FIFO Full Status
    volatile uint8_t TXQSTS = (TX_queue_stat & ETH_MTLTQDR_TXQSTS) >> ETH_MTLTQDR_TXQSTS_Pos;           //  MTL Tx Queue Not Empty Status
    volatile uint8_t TWCSTS = (TX_queue_stat & ETH_MTLTQDR_TWCSTS) >> ETH_MTLTQDR_TWCSTS_Pos;           //  MTL Tx Queue Write Controller Status
    volatile uint8_t TRCSTS = (TX_queue_stat & ETH_MTLTQDR_TRCSTS) >> ETH_MTLTQDR_TRCSTS_Pos;           //  MTL Tx Queue Read Controller Status
    volatile uint8_t TXQPAUSED = (TX_queue_stat & ETH_MTLTQDR_TXQPAUSED) >> ETH_MTLTQDR_TXQPAUSED_Pos;  //  Transmit Queue in Pause


    volatile uint32_t TX_queue_underflow_stat = READ_REG(eth->MTLTQUR);
    volatile uint8_t Underflow_Counter_Overflow = (TX_queue_underflow_stat & ETH_MTLTQUR_UFCNTOVF) >> ETH_MTLTQUR_UFCNTOVF_Pos;  // Overflow Bit for Underflow Packet Counter
    volatile uint8_t Underflow_Pkt_Cnt = (TX_queue_underflow_stat & ETH_MTLTQUR_UFPKTCNT) >> ETH_MTLTQUR_UFPKTCNT_Pos;           // Underflow Packet Counter

    __asm volatile("BKPT #0");
} */
