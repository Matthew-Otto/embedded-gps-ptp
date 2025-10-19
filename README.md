# Embedded GPS PTP Timeserver

A GPS referenced Stratum 1 network time server with suport for IEEE 1588 Precision Time Protocol (PTP).

Bare-metal implementation (using CMSIS register descriptions) written for the STMicroelectronics NUCLEO-H563ZI platform

---

### Dependencies:

* arm-none-eabi-*
* openocd-stm

---

### Ethernet Notes

The application sends data via Ethernet by assigning the address of an ethernet frame in memory to a DMA descriptor and triggering a transmission by updating the descriptor ring tail pointer.
To receive, the application assigns a pointer to an empty buffer to a DMA descriptor and waits for a packet to arrive via the network.

The MAC can output a PPS signal used to compare the synchronization between two devices. This function ETH_PPS_OUT can be assigned to pins PB5 and PG8

---

### Configuring Ethernet


#### Configuring Clocks
The Ethernet MAC uses three clocks connected to the AHB1 bus:
ETH
ETHTX
ETHRX


#### Configuring GPIO
The Ethernet MAC on the STM32H563ZI connects to the PHY via RMII interface. This interface includes the following pins which should be configured as high-speed alt-function 11 (Ethernet) GPIO:

ETH_REF_CLK -> PA1 \
ETH_MDC     -> PC1 \
ETH_MDIO    -> PA2 \
ETH_CRS_DV  -> PA7 \
ETH_RXD0    -> PC4 \
ETH_RXD1    -> PC5 \
ETH_TXD0    -> PG13 \
ETH_TXD1    -> PB15 \
ETH_TX_EN   -> PG11


#### Initialize PHY (via MDIO)
The LAN8742A-CZ-TR PHY is set to autonegotiate out of the box and likely doesn't need modification. However, the ETH <-> PHY interface must be configured to use RMII

1. Enable SBS clock (APB3ENR_SBS)
2. Set the PHY interface to RMII in the SBS_PMCR register \
(optional)
3. Set MDIO clock to a reasonable division of the system clock in the MACMDIOAR register


#### Configuring DMA
Memory dedicated for Rx DMA descriptors:
1524 bytes * 4 descriptors = 6096 -> 8192 bytes
TODO configure descriptors
* Enable DMA transmit and receive functions

#### Configuring MAC

TODO mac address \
TODO filtering rx mac address
* Enable MTL transmit and receive functions
* Enable MAC transmit and receive functions



---

### References / Resources:

[STMicroelectronics/OpenOCD](https://github.com/STMicroelectronics/OpenOCD)

[IEEE 1588-2012 Standard](https://standards.ieee.org/ieee/1588/4355/)

[STM32H563 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0481-stm32h52333xx-stm32h56263xx-and-stm32h573xx-armbased-32bit-mcus-stmicroelectronics.pdf) (RM0481)

[STM32H563 Datasheet](https://www.st.com/resource/en/datasheet/stm32h562ag.pdf) (DS14258)
