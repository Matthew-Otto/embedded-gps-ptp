#ifndef IP_H
#define IP_H

#include <stdint.h>
#include "mcu.h"

void process_packet(uint8_t *data, uint16_t len);

typedef struct {
    uint8_t dst[6];
    uint8_t src[6];
    uint16_t ethertype;
} eth_header_t;

typedef struct {
    uint8_t  ver_ihl;
    uint8_t  tos;
    uint16_t total_length;
    uint16_t id;
    uint16_t flags_frag;
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t checksum;
    uint32_t src_ip;
    uint32_t dst_ip;
} ip4_header_t;

typedef struct {
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint16_t id;
    uint16_t seq;
    uint8_t  data[]; // variable length
} icmp_header_t;

#endif // IP_H
