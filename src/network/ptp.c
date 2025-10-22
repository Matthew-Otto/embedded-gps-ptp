// IEEE 1588 Precision Time Protocol App

#include "mcu.h"
#include "ethernet.h"
#include "ip.h"

#define LEADER // BOZO

#ifdef LEADER
void ptp_main(void) {
    /* tasks:
    send syncs periodically
    respond to delay_req with delay_resp
    */
}

#else
void ptp_main(void) {
    /* tasks:
    wait for syncs
    send delay_req
    process delay_resp
    calibrate clock
    */
}
#endif


void send_sync_packet() {
    //port = 319 for event
    uint8_t *frame;
    uint8_t dst_mac[6] = {0x01,0x1B,0x19,0x00,0x00,0x00};
    uint32_t src_ip = (192<<24)|(168<<16)|(1<<8)|10;
    uint32_t dst_ip = (224<<24)|(0<<16)|(1<<8)|129; // PTPv2 multicast


    ETH_build_header(&frame, dst_mac, ntohs(0x0800));


    // construct frame
    //ETH_send_timestamp_frame();
}