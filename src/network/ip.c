#include <stdint.h>
#include <string.h>
#include "mcu.h"
#include "ethernet.h"
#include "ip.h"

const uint8_t IPv4_ADDR[4] = {10, 1, 123, 1};

// forward declaration
static uint16_t checksum16(const void *data, uint16_t len);

///////////////////////////////////////////////////////////////////////////////
//// Receive //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void process_packet(uint8_t *packet, eth_header_t *frame_header) {
    ipv4_header_t *ip = (ipv4_header_t *)packet;

    if ((ip->version_ihl >> 4) != 4)
        return; // Not IPv4

    uint16_t ip_header_len = (ip->version_ihl & 0x0F) * 4;
    uint16_t payload_len = ntohs(ip->total_length) - ip_header_len;
    uint8_t *payload = ((uint8_t *)ip + ip_header_len);

    switch (ip->protocol) {
        case PROTO_ICMP:
            process_icmp((icmp_header_t *)payload, frame_header, ip, payload_len);
            break;
        case PROTO_TCP:
            break;
        case PROTO_UDP:
            break;
    }
}

void process_icmp(icmp_header_t *icmp, eth_header_t *frame_header, ipv4_header_t *ip_pkt, uint16_t pkt_len) {
    if (icmp->type != 8)
        return; // Not Echo Request

    // respond to ping
    // (we really shouldn't be doing this inside an interrupt handler)
    uint8_t *buffer = ETH_get_tx_buffer();
    if (buffer == NULL)
        return;

    uint16_t length = 0;
    length += build_icmp_reply(buffer, ntohs(icmp->id), ntohs(icmp->seq), icmp->data, (pkt_len - sizeof(icmp_header_t)));
    length += build_ipv4_header(buffer - length, pack4byte(IPv4_ADDR), ntohl(ip_pkt->src_addr), length, ip_pkt->protocol, ntohs(ip_pkt->id));
    length += ETH_build_header(buffer - length, frame_header->src, ntohs(frame_header->ethertype));
    ETH_send_frame(buffer - length, length);
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

///////////////////////////////////////////////////////////////////////////////
//// Transmit /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

uint16_t build_ipv4_header(uint8_t *buffer, uint32_t src_ip, 
                           uint32_t dst_ip, uint16_t payload_len, 
                           uint8_t protocol, uint16_t id) {

    uint16_t header_len = sizeof(ipv4_header_t);
    ipv4_header_t *ip = (ipv4_header_t *)(buffer - header_len);

    ip->version_ihl = (4 << 4) | 5; // IPv4, 5*4 = 20 bytes
    ip->tos = 0;
    ip->total_length = htons(header_len + payload_len);
    ip->id = htons(id);
    ip->flags_fragment = htons(0x4000); // Don't Fragment
    ip->ttl = 64;
    ip->protocol = protocol;
    ip->checksum = 0;
    ip->src_addr = htonl(src_ip);
    ip->dst_addr = htonl(dst_ip);
    ip->checksum = checksum16((uint8_t*)ip, header_len);

    return header_len;
}

uint16_t build_icmp_reply(uint8_t *buffer, uint16_t id, uint16_t seq_num, 
                          uint8_t *payload, uint16_t payload_len) {
    uint16_t pkt_len = sizeof(icmp_header_t) + payload_len;
    icmp_header_t *icmp = (icmp_header_t *)(buffer - pkt_len);

    icmp->type = ICMP_ECHO_RPLY;
    icmp->code = 0;
    icmp->checksum = 0;
    icmp->id = htons(id);
    icmp->seq = htons(seq_num);
    memcpy(icmp->data, payload, payload_len);
    icmp->checksum = checksum16((uint8_t*)icmp, pkt_len);

    return pkt_len;
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
