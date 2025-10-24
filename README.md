# ECE382V IoT Ethernet Lab

### Hardware:
* 2x Nucleo-H563ZI boards
* An Ethernet cable


### Dependencies:

* arm-none-eabi-*
* openocd-stm


## Ethernet on STM32H563ZI

The application communicates with the Ethernet MAC via DMA.
During initialization, the DMA controller is programmed with the addresses of RX and TX descriptor rings.

A frame can be transmitted by writing the address of an existing Ethernet frame to a TX descriptor and then updating the descriptor ring tail pointer. This signals the DMA to copy a frame to the MAC which will automatically send it out onto the wire.

Frames will be received as they come in over the network. If there are valid RX descriptors that contain pointers to a receive buffer, the DMA will automatically copy frames received by the MAC to this buffer. If there are no valid RX descriptors, packets will be dropped.

### Receive

The DMA controller is already configured with receive interrupts enabled. the `ETH_IRQHandler()` function handles this interrupt. It will call `ETH_receive_frame()` which handling resetting the RX descriptors. `ETH_process_frame()` is where incomming packets are processed. This repo includes an example for handling ICMP pings.

### Transmit

There is preallocated memory for `<TX_DSC_CNT>` packet buffers and TX DMA descriptors.

When you want to send a packet over the network, first call `ETH_get_tx_buffer()` to get a pointer to an available packet buffer to construct your packet within.
`ETH_get_tx_buffer()` returns a pointer to the last byte in the buffer so packets can be crafted "bottom-up".

Once you have a valid Ethernet frame ready to send, call `ETH_send_frame()` with the pointer to the first byte of the frame and the length of the frame.

Eample (as seen in `ip.c/process_icmp()`):

``` c++
uint8_t *buffer = ETH_get_tx_buffer();
if (buffer == NULL)
    return;

uint16_t length = 0;
length += build_icmp_reply(buffer, ntohs(icmp->id), ntohs(icmp->seq), icmp->data, (pkt_len - sizeof(icmp_header_t)));
length += build_ipv4_header(buffer - length, pack4byte(IPv4_ADDR), ntohl(ip_pkt->src_addr), length, ip_pkt->protocol, ntohs(ip_pkt->id));
length += ETH_build_header(buffer - length, frame_header->src, ntohs(frame_header->ethertype));
ETH_send_frame(buffer - length, length);
```
### Changing MAC/IP Address
The MAC address if set using the constant `MACAddr` at the top of `ethernet.c`

The IP address is set using the constant `IPv4_ADDR` at the top of `ip.c`

If you will be connecting two boards together, the addresses for one of them should be changed.



## Debugging:

The VSCode config files useful for debugging are included in this repo under `.vscode`. After opening this repo in VSCode, simply install the Cortex-Debug addon. The integrated debugger should now be functional.

## References / Resources:

[STMicroelectronics/OpenOCD](https://github.com/STMicroelectronics/OpenOCD)

[STM32H563 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0481-stm32h52333xx-stm32h56263xx-and-stm32h573xx-armbased-32bit-mcus-stmicroelectronics.pdf) (RM0481)

[STM32H563 Datasheet](https://www.st.com/resource/en/datasheet/stm32h562ag.pdf) (DS14258)
