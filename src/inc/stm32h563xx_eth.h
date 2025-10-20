#include "stm32h563xx.h"


#define TRACE_CK_Pin GPIO_PIN_2
#define TRACE_CK_GPIO_Port GPIOE
#define TRACE_D0_Pin GPIO_PIN_3
#define TRACE_D0_GPIO_Port GPIOE
#define TRACE_D1_Pin GPIO_PIN_4
#define TRACE_D1_GPIO_Port GPIOE
#define TRACE_D2_Pin GPIO_PIN_5
#define TRACE_D2_GPIO_Port GPIOE
#define TRACE_D3_Pin GPIO_PIN_6
#define TRACE_D3_GPIO_Port GPIOE
#define STLK_MCO_Pin GPIO_PIN_0
#define STLK_MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_4
#define VBUS_SENSE_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define UCPD_CC1_Pin GPIO_PIN_13
#define UCPD_CC1_GPIO_Port GPIOB
#define UCPD_CC2_Pin GPIO_PIN_14
#define UCPD_CC2_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_15
#define RMII_TXD1_GPIO_Port GPIOB
#define UCPD_FLT_Pin GPIO_PIN_7
#define UCPD_FLT_GPIO_Port GPIOG
#define UCDP_DBn_Pin GPIO_PIN_9
#define UCDP_DBn_GPIO_Port GPIOA
#define USB_FS_N_Pin GPIO_PIN_11
#define USB_FS_N_GPIO_Port GPIOA
#define USB_FS_P_Pin GPIO_PIN_12
#define USB_FS_P_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define T_JTDI_Pin GPIO_PIN_15
#define T_JTDI_GPIO_Port GPIOA
#define RMII_TXT_EN_Pin GPIO_PIN_11
#define RMII_TXT_EN_GPIO_Port GPIOG
#define RMI_TXD0_Pin GPIO_PIN_13
#define RMI_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ARD_D1_TX_Pin GPIO_PIN_6
#define ARD_D1_TX_GPIO_Port GPIOB
#define ARD_D0_RX_Pin GPIO_PIN_7
#define ARD_D0_RX_GPIO_Port GPIOB

#define SBS_ETH_RMII            SBS_PMCR_ETH_SEL_PHY_2 


#define ETH_MACCR_MASK                0xFFFB7F7CU
#define ETH_MACECR_MASK               0x3F077FFFU
#define ETH_MACPFR_MASK               0x800007FFU
#define ETH_MACWTR_MASK               0x0000010FU
#define ETH_MACTFCR_MASK              0xFFFF00F2U
#define ETH_MACRFCR_MASK              0x00000003U
#define ETH_MTLTQOMR_MASK             0x00000072U
#define ETH_MTLRQOMR_MASK             0x0000007BU

#define ETH_DMAMR_MASK                0x00007802U
#define ETH_DMASBMR_MASK              0x0000D001U
#define ETH_DMACCR_MASK               0x00013FFFU
#define ETH_DMACTCR_MASK              0x003F1010U
#define ETH_DMACRCR_MASK              0x803F0000U
#define ETH_MACPCSR_MASK              (ETH_MACPCSR_PWRDWN | ETH_MACPCSR_RWKPKTEN | \
                                       ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_GLBLUCAST | \
                                       ETH_MACPCSR_RWKPFE)
