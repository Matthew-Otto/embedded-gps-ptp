#ifndef CLOCKS_H
#define CLOCKS_H

#define RCC_CLOCKTYPE_SYSCLK           (0x00000001U)  /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK             (0x00000002U)  /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1            (0x00000004U)  /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2            (0x00000008U)  /*!< PCLK2 to configure */
#define RCC_CLOCKTYPE_PCLK3            (0x00000010U)  /*!< PCLK3 to configure */

#define RCC_SYSCLKSOURCE_HSI             (0x00000000U)                      /*!< HSI selection as system clock */
#define RCC_SYSCLKSOURCE_CSI             RCC_CFGR1_SW_0                     /*!< CSI selection as system clock */
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR1_SW_1                     /*!< HSE selection as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK          (RCC_CFGR1_SW_0 | RCC_CFGR1_SW_1)  /*!< PLL1 selection as system clock */

#define RCC_SYSCLK_DIV1                (0x00000000U)                                                                /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                RCC_CFGR2_HPRE_3                                                             /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                (RCC_CFGR2_HPRE_0 | RCC_CFGR2_HPRE_3)                                        /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                (RCC_CFGR2_HPRE_1 | RCC_CFGR2_HPRE_3)                                        /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16               (RCC_CFGR2_HPRE_0 | RCC_CFGR2_HPRE_1 | RCC_CFGR2_HPRE_3)                     /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64               (RCC_CFGR2_HPRE_2 | RCC_CFGR2_HPRE_3)                                        /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128              (RCC_CFGR2_HPRE_0 | RCC_CFGR2_HPRE_2 | RCC_CFGR2_HPRE_3)                     /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256              (RCC_CFGR2_HPRE_1 | RCC_CFGR2_HPRE_2 | RCC_CFGR2_HPRE_3)                     /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512              (RCC_CFGR2_HPRE_0 | RCC_CFGR2_HPRE_1  | RCC_CFGR2_HPRE_2 | RCC_CFGR2_HPRE_3) /*!< SYSCLK divided by 512 */

#define RCC_HCLK_DIV1                  (0x00000000U)                                               /*!< HCLK not divided */
#define RCC_HCLK_DIV2                  RCC_CFGR2_PPRE1_2                                           /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                  (RCC_CFGR2_PPRE1_0 | RCC_CFGR2_PPRE1_2)                     /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                  (RCC_CFGR2_PPRE1_1 | RCC_CFGR2_PPRE1_2)                     /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                 (RCC_CFGR2_PPRE1_0 | RCC_CFGR2_PPRE1_1 | RCC_CFGR2_PPRE1_2) /*!< HCLK divided by 16 */

void init_sysclk(void);

#endif // CLOCKS_H