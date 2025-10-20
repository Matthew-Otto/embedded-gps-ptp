#ifndef ETH_H
#define ETH_H

#include <stdint.h>
#include "mcu.h"

typedef struct {
    uint8_t dest_mac[6];
    //uint8_t src_mac[6];
    uint16_t eth_type;
    uint8_t payload[1500];
} ethernet_frame_t;

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
  volatile uint16_t control;
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

typedef union {
    ETH_rx_rd_desc_t rd;
    ETH_rx_wb_desc_t wb;
} ETH_rx_desc_u;

typedef struct {
  uint32_t ts_low;
  uint32_t ts_high;
} ETH_timestamp_t;

void ETH_IRQHandler(void);
void ETH_init(void);
void ETH_dump_SR(void);

void ETH_send_frame(uint8_t *data, uint16_t length);
int ETH_receive_frame(void);

#endif // ETH_H
