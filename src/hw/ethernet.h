#ifndef ETH_H
#define ETH_H

#include <stdint.h>
#include "mcu.h"

#define ETHERTYPE_ARP   0x0806
#define ETHERTYPE_IPv4  0x0800
#define ETHERTYPE_IPV6  0x86DD
#define ETHERTYPE_PTP   0x88F7

extern uint32_t rx_timestamp_sec;
extern uint32_t rx_timestamp_nsec;

static inline uint16_t ntohs(uint16_t data) {
    return (data >> 8) | (data << 8);
}

static inline uint16_t pack4byte (uint8_t bytes[4]) {
    return ((uint32_t)bytes[3]) |
           ((uint32_t)bytes[2] << 8) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[0] << 24);
}

typedef struct {
    volatile uint32_t buffer1_addr;
    volatile uint32_t buffer2_addr;
    volatile uint16_t buffer1_len;
    volatile uint16_t buffer2_len;
    volatile uint32_t ctrl;
} ETH_tx_rd_desc_t;

typedef struct {
    volatile uint32_t timestamp_low;
    volatile uint32_t timestamp_high;
    volatile uint32_t reserved;
    volatile uint32_t status;
} ETH_tx_wb_desc_t;

typedef struct {
    volatile uint32_t timestamp_low;
    volatile uint32_t timestamp_high;
    volatile uint16_t max_seg_size;
    volatile uint16_t inner_vlan_tag;
    volatile uint16_t vlan_tag;
    volatile uint16_t ctrl;
} ETH_tx_ctx_desc_t;

typedef union {
    ETH_tx_rd_desc_t rd;
    ETH_tx_wb_desc_t wb;
    ETH_tx_ctx_desc_t ctx;
} ETH_tx_desc_u;

typedef struct {
    volatile uint32_t buffer1_addr;
    volatile uint32_t reserved1;
    volatile uint32_t buffer2_addr;
    volatile uint8_t  reserved2[3];
    volatile uint8_t  status;
} ETH_rx_rd_desc_t;

typedef struct {
    volatile uint16_t outer_vlan_tag;
    volatile uint16_t inner_vlan_tag;
    volatile uint16_t ext_stat;
    volatile uint16_t mac_ctrl_op;
    volatile uint16_t vlan_filter_status;
    volatile uint16_t mac_filter_stat;
    volatile uint16_t pkt_len;
    volatile uint16_t status;
} ETH_rx_wb_desc_t;

typedef struct {
    volatile uint32_t timestamp_low;
    volatile uint32_t timestamp_high;
    volatile uint32_t _;
    volatile uint32_t ctrl;
} ETH_rx_ctx_desc_t;

typedef union {
    ETH_rx_rd_desc_t rd;
    ETH_rx_wb_desc_t wb;
    ETH_rx_ctx_desc_t ctx;
} ETH_rx_desc_u;

typedef struct __attribute__((packed)) {
    uint8_t  dest[6];
    uint8_t  src[6];
    uint16_t ethertype;
} eth_header_t;


void ETH_IRQHandler(void);
void ETH_init(void);
void ETH_update_PTP_TS_coarse(const int32_t offset_sec, const int32_t offset_nsec);
void ETH_update_PTP_drift_comp(const int32_t comp);
void ETH_dump_SR(void);
void ETH_build_header(uint8_t **buffer, const uint8_t *dst, const uint16_t ethertype);
void ETH_send_timestamp_frame(uint8_t *data, uint16_t length);
void ETH_send_frame(uint8_t *data, uint16_t length);
int ETH_receive_frame(void);
int ETH_process_frame(uint8_t *frame);

#endif // ETH_H
