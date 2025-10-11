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

typedef struct
{
  volatile uint32_t status;            // OWN + flags (receive status / transmit status)
  volatile uint32_t control;           // buffer sizes, control bits
  volatile uint32_t buffer1_addr;      // data buffer pointer
  volatile uint32_t buffer2_next_desc; // buffer2 pointer or next descriptor pointer
} ETH_DMA_desc_t;

typedef struct
{
  uint32_t ts_low;
  uint32_t ts_high;

} ETH_timestamp_t;

void ETH_init();
void ETH_construct_frame(ethernet_frame_t *frame, uint8_t *dest_mac, uint8_t *src_mac, uint16_t eth_type, uint8_t *payload, uint16_t payload_len);
void ETH_send_frame(ethernet_frame_t *frame, uint16_t payload_len);

// bozo
int eth_receive(uint8_t *out, uint32_t *len);

#endif // ETH_H