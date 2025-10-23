#include <stdint.h>
#include <string.h>
#include "mcu.h"
#include "ip.h"
#include "ptp.h"


void process_packet(uint8_t *packet) {
    ipv4_header_t *ip = (ipv4_header_t *)packet;

    if ((ip->version_ihl >> 4) != 4)
        return; // Not IPv4

    uint16_t ip_header_len = (ip->version_ihl & 0x0F) * 4;
    uint16_t payload_len = ip->total_length - ip_header_len;
    uint8_t *payload = ((uint8_t *)ip + ip_header_len);

    switch (ip->protocol) {
        case PROTO_ICMP:
            process_icmp((icmp_header_t *)payload, payload_len);
            break;
        case PROTO_TCP:
            break;
        case PROTO_UDP:
            break;
    }
}

void process_icmp(icmp_header_t *icmp, uint16_t len) {
    if (icmp->type != 8)
        return; // Not Echo Request

    // --- Build Echo Reply ---
    icmp->type = 0; // Echo Reply
    icmp->checksum = 0;
    /* uint16_t icmp_len = ntohs(ip->total_length) - ip_header_len;
    icmp->checksum = checksum16(icmp, icmp_len);

    // Swap IP addresses
    uint32_t tmp_ip = ip->src_ip;
    ip->src_ip = ip->dst_ip;
    ip->dst_ip = tmp_ip;

    // Recompute IP header checksum
    ip->checksum = 0;
    ip->checksum = checksum16(ip, ip_header_len);

    // Swap Ethernet addresses
    uint8_t tmp_mac[6];
    memcpy(tmp_mac, frame->src, 6);
    memcpy(frame->src, frame->dst, 6);
    memcpy(frame->dst, tmp_mac, 6); */
}

void process_udp(udp_header_t *packet) {
    uint16_t port = packet->dst_port;

    switch(port) {
        case PORT_PTP_EVENT:
            break;
        case PORT_PTP_GENERAL:
            break;
    }
}

// compute 16-bit Internet checksum
static uint16_t checksum16(const void *data, uint16_t len) {
    uint32_t sum = 0;
    const uint16_t *ptr = data;

    while (len > 1) {
        sum += *ptr++;
        len -= 2;
    }

    if (len > 0)
        sum += *((uint8_t *)ptr);

    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);

    return (uint16_t)(~sum);
}
