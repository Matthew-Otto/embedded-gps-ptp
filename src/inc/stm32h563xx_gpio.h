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
