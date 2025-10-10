#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_ALL               ((uint16_t)0xFFFF)  /* All pins selected */

/*!< Input Floating Mode                                                 */
#define  GPIO_MODE_INPUT                        (0x00000000U)
/*!< Output Push Pull Mode                                               */
#define  GPIO_MODE_OUTPUT_PP                    (0x00000001U)
/*!< Output Open Drain Mode                                             */
#define  GPIO_MODE_OUTPUT_OD                    (0x00000011U)
/*!< Alternate Function Push Pull Mode                                   */
#define  GPIO_MODE_AF_PP                        (0x00000002U)
/*!< Alternate Function Open Drain Mode                                  */
#define  GPIO_MODE_AF_OD                        (0x00000012U)
/*!< Analog Mode                                                         */
#define  GPIO_MODE_ANALOG                       (0x00000003U)
/*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_RISING                    (0x10110000U)
/*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_FALLING                   (0x10210000U)
/*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
#define  GPIO_MODE_IT_RISING_FALLING            (0x10310000U)
/*!< External Event Mode with Rising edge trigger detection             */
#define  GPIO_MODE_EVT_RISING                   (0x10120000U)
/*!< External Event Mode with Falling edge trigger detection            */
#define  GPIO_MODE_EVT_FALLING                  (0x10220000U)
/*!< External Event Mode with Rising/Falling edge trigger detection      */
#define  GPIO_MODE_EVT_RISING_FALLING           (0x10320000U)

#define  GPIO_SPEED_FREQ_LOW        (0x00000000U)   /*!< Low speed       */
#define  GPIO_SPEED_FREQ_MEDIUM     (0x00000001U)   /*!< Medium speed    */
#define  GPIO_SPEED_FREQ_HIGH       (0x00000002U)   /*!< High speed      */
#define  GPIO_SPEED_FREQ_VERY_HIGH  (0x00000003U)   /*!< Very-high speed */

#define  GPIO_NOPULL        (0x00000000U)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        (0x00000001U)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      (0x00000002U)   /*!< Pull-down activation                */

#define GPIO_MODE             (0x00000003U)
#define EXTI_MODE             (0x10000000U)
#define GPIO_MODE_IT          (0x00010000U)
#define GPIO_MODE_EVT         (0x00020000U)
#define RISING_EDGE           (0x00100000U)
#define FALLING_EDGE          (0x00200000U)
#define GPIO_OUTPUT_TYPE      (0x00000010U)
#define GPIO_NUMBER           (16U)



// AF 0 selection
#define GPIO_AF0_RTC_50HZ      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */
#define GPIO_AF0_CSLEEP        ((uint8_t)0x00)  /* CSLEEP Alternate Function mapping                         */
#define GPIO_AF0_CSTOP         ((uint8_t)0x00)  /* CSTOP Alternate Function mapping                          */
#define GPIO_AF0_CRS           ((uint8_t)0x00)  /* CRS Alternate Function mapping                            */

// AF 1 selection
#define GPIO_AF1_TIM1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping                           */
#define GPIO_AF1_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping                           */
#define GPIO_AF1_TIM16         ((uint8_t)0x01)  /* TIM16 Alternate Function mapping                          */
#define GPIO_AF1_TIM17         ((uint8_t)0x01)  /* TIM17 Alternate Function mapping                          */

// AF 2 selection
#if defined(LPTIM3)
#define GPIO_AF2_LPTIM3        ((uint8_t)0x02)  /* LPTIM3 Alternate Function mapping                         */
#endif /* LPTIM3 */
#if defined(SAI1)
#define GPIO_AF2_SAI1          ((uint8_t)0x02)  /* SAI1 Alternate Function mapping                           */
#endif /* SAI1 */
#define GPIO_AF2_TIM3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping                           */
#if defined(TIM4)
#define GPIO_AF2_TIM4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping                           */
#endif /* TIM4 */
#if defined(TIM5)
#define GPIO_AF2_TIM5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping                           */
#endif /* TIM5 */
#if defined(TIM12)
#define GPIO_AF2_TIM12         ((uint8_t)0x02)  /* TIM12 Alternate Function mapping                          */
#endif /* TIM12 */
#if defined(TIM15)
#define GPIO_AF2_TIM15         ((uint8_t)0x02)  /* TIM15 Alternate Function mapping                          */
#endif /* TIM15 */

// AF 3 selection
#define GPIO_AF3_I3C1          ((uint8_t)0x03)  /* I3C1 Alternate Function mapping                           */
#define GPIO_AF3_LPTIM2        ((uint8_t)0x03)  /* LPTIM2 Alternate Function mapping                         */
#if defined(LPTIM3)
#define GPIO_AF3_LPTIM3        ((uint8_t)0x03)  /* LPTIM3 Alternate Function mapping                         */
#endif /* LPTIM3 */
#define GPIO_AF3_LPUART1       ((uint8_t)0x03)  /* LPUART1 Alternate Function mapping                        */
#if defined(OCTOSPI1)
#define GPIO_AF3_OCTOSPI1      ((uint8_t)0x03)  /* OCTOSPI1 Alternate Function mapping                       */
#endif /* OCTOSPI1 */
#if defined(TIM8)
#define GPIO_AF3_TIM8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping                           */
#endif /* TIM8 */

// AF 4 selection
#if defined(CEC)
#define GPIO_AF4_CEC           ((uint8_t)0x04)  /* CEC Alternate Function mapping                            */
#endif /* CEC */
#if defined(DCMI)
#define GPIO_AF4_DCMI          ((uint8_t)0x04)  /* DCMI Alternate Function mapping                           */
#endif /* DCMI */
#if defined(PSSI)
#define GPIO_AF4_PSSI          ((uint8_t)0x04)  /* PSSI Alternate Function mapping                           */
#endif /* PSSI */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping                           */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping                           */
#if defined(I2C3)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping                           */
#endif /* I2C3 */
#if defined(I2C4)
#define GPIO_AF4_I2C4          ((uint8_t)0x04)  /* I2C4 Alternate Function mapping                           */
#endif /* I2C4 */
#define GPIO_AF4_LPTIM1        ((uint8_t)0x04)  /* LPTIM1 Alternate Function mapping                         */
#define GPIO_AF4_LPTIM2        ((uint8_t)0x04)  /* LPTIM2 Alternate Function mapping                         */
#define GPIO_AF4_SPI1          ((uint8_t)0x04)  /* SPI1 Alternate Function mapping                           */
#if defined(TIM15)
#define GPIO_AF4_TIM15         ((uint8_t)0x04)  /* TIM15 Alternate Function mapping                          */
#endif /* TIM15 */
#define GPIO_AF4_USART1        ((uint8_t)0x04)  /* USART1 Alternate Function mapping                         */

// AF 5 selection
#if defined(CEC)
#define GPIO_AF5_CEC           ((uint8_t)0x05)  /* CEC Alternate Function mapping                            */
#endif /* CEC */
#define GPIO_AF5_LPTIM1        ((uint8_t)0x05)  /* LPTIM1 Alternate Function mapping                         */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping                           */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2 Alternate Function mapping                           */
#if defined(SPI4)
#define GPIO_AF5_SPI4          ((uint8_t)0x05)  /* SPI4 Alternate Function mapping                           */
#endif /* SPI4 */
#if defined(SPI5)
#define GPIO_AF5_SPI5          ((uint8_t)0x05)  /* SPI5 Alternate Function mapping                           */
#endif /* SPI5 */
#if defined(SPI6)
#define GPIO_AF5_SPI6          ((uint8_t)0x05)  /* SPI6 Alternate Function mapping                           */
#endif /* SPI6 */

// AF 6 selection
#if defined(I2C4)
#define GPIO_AF6_I2C4          ((uint8_t)0x06)  /* I2C4 Alternate Function mapping                           */
#endif /* I2C4 */
#define GPIO_AF6_OCTOSPI1      ((uint8_t)0x06)  /* OCTOSPI1 Alternate Function mapping                       */
#if defined(SAI1)
#define GPIO_AF6_SAI1          ((uint8_t)0x06)  /* SAI1 Alternate Function mapping                           */
#endif /* SAI1 */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3 Alternate Function mapping                           */
#if defined(SPI4)
#define GPIO_AF6_SPI4          ((uint8_t)0x06)  /* SPI4 Alternate Function mapping                           */
#endif /* SPI4 */
#if defined(UART4)
#define GPIO_AF6_UART4         ((uint8_t)0x06)  /* UART4 Alternate Function mapping                          */
#endif /* UART4 */
#if defined(UART12)
#define GPIO_AF6_UART12        ((uint8_t)0x06)  /* UART12 Alternate Function mapping                         */
#endif /* UART12 */
#if defined(USART10)
#define GPIO_AF6_USART10       ((uint8_t)0x06)  /* USART10 Alternate Function mapping                        */
#endif /* USART10 */
#if defined(UCPD1)
#define GPIO_AF6_UCPD1         ((uint8_t)0x06)  /* UCPD1 Alternate Function mapping                          */
#endif /* UCPD1 */

// AF 7 selection
#if defined(SDMMC1)
#define GPIO_AF7_SDMMC1        ((uint8_t)0x07)  /* SDMMC1 Alternate Function mapping                         */
#endif /* SDMMC1 */
#define GPIO_AF7_SPI2          ((uint8_t)0x07)  /* SPI2 Alternate Function mapping                           */
#define GPIO_AF7_SPI3          ((uint8_t)0x07)  /* SPI3 Alternate Function mapping                           */
#if defined(SPI6)
#define GPIO_AF7_SPI6          ((uint8_t)0x07)  /* SPI6 Alternate Function mapping                           */
#endif /* SPI6 */
#if defined(UART7)
#define GPIO_AF7_UART7         ((uint8_t)0x07)  /* UART7 Alternate Function mapping                          */
#endif /* UART7 */
#if defined(UART8)
#define GPIO_AF7_UART8         ((uint8_t)0x07)  /* UART8 Alternate Function mapping                          */
#endif /* UART8 */
#if defined(UART12)
#define GPIO_AF7_UART12        ((uint8_t)0x07)  /* UART12 Alternate Function mapping                         */
#endif /* UART12 */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping                         */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping                         */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping                         */
#if defined(USART6)
#define GPIO_AF7_USART6        ((uint8_t)0x07)  /* USART6 Alternate Function mapping                         */
#endif /* USART6 */
#if defined(USART10)
#define GPIO_AF7_USART10       ((uint8_t)0x07)  /* USART10 Alternate Function mapping                        */
#endif /* USART10 */
#if defined(USART11)
#define GPIO_AF7_USART11       ((uint8_t)0x07)  /* USART11 Alternate Function mapping                        */
#endif /* USART11 */

// AF 8 selection
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)  /* LPUART1 Alternate Function mapping                        */
#if defined(SAI2)
#define GPIO_AF8_SAI2          ((uint8_t)0x08)  /* SAI2 Alternate Function mapping                           */
#endif /* SAI2 */
#if defined(SDMMC1)
#define GPIO_AF8_SDMMC1        ((uint8_t)0x08)  /* SDMMC1 Alternate Function mapping                         */
#endif /* SDMMC1 */
#if defined(SPI6)
#define GPIO_AF8_SPI6          ((uint8_t)0x08)  /* SPI6 Alternate Function mapping                           */
#endif /* SPI6 */
#if defined(UART4)
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping                          */
#endif /* UART4 */
#if defined(UART5)
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping                          */
#endif /* UART5 */
#if defined(UART8)
#define GPIO_AF8_UART8         ((uint8_t)0x08)  /* UART8 Alternate Function mapping                          */
#endif /* UART8 */

// AF 9 selection
#define GPIO_AF9_FDCAN1        ((uint8_t)0x09)  /* FDCAN1 Alternate Function mapping                         */
#if defined(FDCAN2)
#define GPIO_AF9_FDCAN2        ((uint8_t)0x09)  /* FDCAN2 Alternate Function mapping                         */
#endif /* FDCAN2 */
#if defined(FMC_BANK1)
#define GPIO_AF9_FMC           ((uint8_t)0x09)  /* FMC Alternate Function mapping                            */
#endif /* FMC_BANK1 */
#if defined(OCTOSPI1)
#define GPIO_AF9_OCTOSPI1      ((uint8_t)0x09)  /* OCTOSPI1 Alternate Function mapping                       */
#endif /* OCTOSPI1 */
#if defined(SDMMC2)
#define GPIO_AF9_SDMMC2        ((uint8_t)0x09)  /* SDMMC2 Alternate Function mapping                         */
#endif /* SDMMC2 */
#if defined(TIM13)
#define GPIO_AF9_TIM13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping                          */
#endif /* TIM13 */
#if defined(TIM14)
#define GPIO_AF9_TIM14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping                          */
#endif /* TIM14 */

// AF 10 selection
#define GPIO_AF10_CRS          ((uint8_t)0x0A)  /* CRS Alternate Function mapping                            */
#if defined(FMC_BANK1)
#define GPIO_AF10_FMC          ((uint8_t)0x0A)  /* FMC Alternate Function mapping                            */
#endif /* FMC_BANK1 */
#if defined(OCTOSPI1)
#define GPIO_AF10_OCTOSPI1     ((uint8_t)0x0A)  /* OCTOSPI1 Alternate Function mapping                       */
#endif /* OCTOSPI1 */
#if defined(SAI2)
#define GPIO_AF10_SAI2         ((uint8_t)0x0A)  /* SAI2 Alternate Function mapping                           */
#endif /* SAI2 */
#if defined(SDMMC2)
#define GPIO_AF10_SDMMC2       ((uint8_t)0x0A)  /* SDMMC2 Alternate Function mapping                         */
#endif /* SDMMC2 */
#if defined(TIM8)
#define GPIO_AF10_TIM8         ((uint8_t)0x0A)  /* TIM8 Alternate Function mapping                           */
#endif /* TIM8 */
#if defined(USB_DRD_FS)
#define GPIO_AF10_USB          ((uint8_t)0x0A)  /* USB Alternate Function mapping                            */
#endif /* USB_DRD_FS */

// AF 11 selection
#if defined(ETH)
#define GPIO_AF11_ETH          ((uint8_t)0x0B)  /* ETH Alternate Function mapping                            */
#endif /* ETH */
#if defined(FMC_BANK1)
#define GPIO_AF11_FMC          ((uint8_t)0x0B)  /* FMC Alternate Function mapping                            */
#endif /* FMC_BANK1 */
#if defined(OCTOSPI1)
#define GPIO_AF11_OCTOSPI1     ((uint8_t)0x0B)  /* OCTOSPI1 Alternate Function mapping                       */
#endif /* OCTOSPI1 */
#if defined(SDMMC2)
#define GPIO_AF11_SDMMC2       ((uint8_t)0x0B)  /* SDMMC2 Alternate Function mapping                         */
#endif /* SDMMC2 */
#if defined(UART7)
#define GPIO_AF11_UART7        ((uint8_t)0x0B)  /* UART7 Alternate Function mapping                          */
#endif /* UART7 */
#if defined(UART9)
#define GPIO_AF11_UART9        ((uint8_t)0x0B)  /* UART9 Alternate Function mapping                          */
#endif /* UART9 */
#if defined(UCPD1)
#define GPIO_AF11_UCPD1        ((uint8_t)0x0B)  /* UCPD1 Alternate Function mapping                          */
#endif /* UCPD1 */

// AF 12 selection
#if defined(FMC_BANK1)
#define GPIO_AF12_FMC          ((uint8_t)0x0C)  /* FMC Alternate Function mapping                            */
#endif /* FMC_BANK1 */
#if defined(SDMMC1)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)  /* SDMMC1 Alternate Function mapping                         */
#endif /* SDMMC1 */

// AF 13 selection
#if defined(DCMI)
#define GPIO_AF13_DCMI         ((uint8_t)0x0D)  /* DCMI Alternate Function mapping                           */
#define GPIO_AF13_PSSI         ((uint8_t)0x0D)  /* PSSI Alternate Function mapping                           */
#endif /* DCMI */
#if defined(FMC_BANK1)
#define GPIO_AF13_FMC          ((uint8_t)0x0D)  /* FMC Alternate Function mapping                            */
#endif /* FMC_BANK1 */
#if defined(LPTIM5)
#define GPIO_AF13_LPTIM5       ((uint8_t)0x0D)  /* LPTIM5 Alternate Function mapping                         */
#endif /* LPTIM5 */

// AF 14 selection
#if defined(LPTIM3)
#define GPIO_AF14_LPTIM3       ((uint8_t)0x0E)  /* LPTIM3 Alternate Function mapping                         */
#endif /* LPTIM3 */
#if defined(LPTIM4)
#define GPIO_AF14_LPTIM4       ((uint8_t)0x0E)  /* LPTIM4 Alternate Function mapping                         */
#endif /* LPTIM4 */
#if defined(LPTIM5)
#define GPIO_AF14_LPTIM5       ((uint8_t)0x0E)  /* LPTIM5 Alternate Function mapping                         */
#endif /* LPTIM5 */
#if defined(LPTIM6)
#define GPIO_AF14_LPTIM6       ((uint8_t)0x0E)  /* LPTIM6 Alternate Function mapping                         */
#endif /* LPTIM6 */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)  /* TIM2 Alternate Function mapping                           */
#if defined(UART5)
#define GPIO_AF14_UART5        ((uint8_t)0x0E)  /* UART5 Alternate Function mapping                          */
#endif /* UART5 */

// AF 15 selection
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)