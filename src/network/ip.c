#include <stdint.h>
#include <string.h>
#include "mcu.h"
#include "ip.h"

// convert a 16-bit number from network byte order (big-endian) to little-endian
static uint16_t ntohs(uint16_t data) {
    return (data >> 8) | (data << 8);
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

// This serves as a demo for now. Will respond to ICMP pings.
void process_packet(uint8_t *data, uint16_t len) {
    eth_header_t *frame = (eth_header_t *)data;
    if (ntohs(frame->ethertype) != 0x0800) // IPv4
        return;

    ip4_header_t *ip = (ip4_header_t *)(data + sizeof(eth_header_t));
    if ((ip->ver_ihl >> 4) != 4)
        return; // Not IPv4
    if (ip->protocol != 1)
        return; // Not ICMP

    uint16_t ip_header_len = (ip->ver_ihl & 0x0F) * 4;
    icmp_header_t *icmp = (icmp_header_t *)((uint8_t *)ip + ip_header_len);

    if (icmp->type != 8)
        return; // Not Echo Request



    // --- Build Echo Reply ---
    icmp->type = 0; // Echo Reply
    icmp->checksum = 0;
    uint16_t icmp_len = ntohs(ip->total_length) - ip_header_len;
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
    memcpy(frame->dst, tmp_mac, 6);
}