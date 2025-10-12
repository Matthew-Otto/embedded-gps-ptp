#ifndef ETH_H
#define ETH_H

#include <stdint.h>
#include "stm32h563xx.h"

typedef struct {
    uint8_t dest_mac[6];
    uint8_t src_mac[6];
    uint16_t eth_type;
    uint8_t payload[1500];
} ethernet_frame_t;

typedef struct {
  volatile uint32_t buffer1_addr;
  volatile uint32_t buffer2_addr;
  volatile uint32_t TDES2;
  volatile uint32_t TDES3;
} ETH_tx_desc_t;

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
  volatile uint16_t idk;
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

void ETH_init();
void ETH_construct_frame(ethernet_frame_t *frame, uint8_t *dest_mac, uint8_t *src_mac, uint16_t eth_type, uint8_t *payload, uint16_t payload_len);
void ETH_send_frame(ethernet_frame_t *frame, uint16_t payload_len);

int ETH_receive_frame();

#endif // ETH_H