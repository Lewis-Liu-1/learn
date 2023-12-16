#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#define __IO volatile /*!< Defines 'read / write' permissions */
#define __STATIC_INLINE static inline
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define UNUSED(X) (void)X /* To avoid gcc/g++ warnings */

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
} GPIO_TypeDef;
/**
 * @brief Reset and Clock Control
 */

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
} RCC_TypeDef;

#define FLASH_BASE            0x08000000UL              /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       0x0801FFFFUL /*!< FLASH END address of bank1 */
#define SRAM_BASE             0x20000000UL              /*!< SRAM base address in the alias region */
#define PERIPH_BASE           0x40000000UL              /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00UL)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x00001000UL)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400UL)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#define RCC_AHBENR_GPIOAEN_Pos                   (17U)
#define RCC_AHBENR_GPIOAEN_Msk                   (0x1UL << RCC_AHBENR_GPIOAEN_Pos) /*!< 0x00020000 */
#define RCC_AHBENR_GPIOAEN                       RCC_AHBENR_GPIOAEN_Msk        /*!< GPIOA clock enable */
#define RCC_AHBENR_GPIOBEN_Pos                   (18U)
#define RCC_AHBENR_GPIOBEN_Msk                   (0x1UL << RCC_AHBENR_GPIOBEN_Pos) /*!< 0x00040000 */
#define RCC_AHBENR_GPIOBEN                       RCC_AHBENR_GPIOBEN_Msk        /*!< GPIOB clock enable */
#define RCC_AHBENR_GPIOCEN_Pos                   (19U)
#define RCC_AHBENR_GPIOCEN_Msk                   (0x1UL << RCC_AHBENR_GPIOCEN_Pos) /*!< 0x00080000 */
#define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk        /*!< GPIOC clock enable */

#define RCC_APB2ENR_SYSCFGCOMPEN_Pos             (0U)
#define RCC_APB2ENR_SYSCFGCOMPEN_Msk             (0x1UL << RCC_APB2ENR_SYSCFGCOMPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_SYSCFGCOMPEN                 RCC_APB2ENR_SYSCFGCOMPEN_Msk  /*!< SYSCFG and comparator clock enable */
#define  RCC_APB2ENR_SYSCFGEN                RCC_APB2ENR_SYSCFGCOMPEN        /*!< SYSCFG clock enable */

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __HAL_RCC_GPIOC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
typedef struct
{
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
       uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08 */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                           Address offset: 0x18 */
} SYSCFG_TypeDef;

#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000UL)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

#define __HAL_RCC_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)


#define EXTI_BASE             (APBPERIPH_BASE + 0x00010400UL)

typedef struct
{
  __IO uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */
} EXTI_TypeDef;
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

typedef enum
{
/******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                        */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

/******  STM32F0 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                               */
  PVD_VDDIO2_IRQn             = 1,      /*!< PVD & VDDIO2 Interrupt through EXTI Lines 16 and 31             */
  RTC_IRQn                    = 2,      /*!< RTC Interrupt through EXTI Lines 17, 19 and 20                  */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                          */
  RCC_CRS_IRQn                = 4,      /*!< RCC & CRS global Interrupt                                      */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt                                     */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupt                                     */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupt                                     */
  TSC_IRQn                    = 8,      /*!< Touch Sensing Controller Interrupts                             */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                        */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupt                          */
  DMA1_Channel4_5_6_7_IRQn    = 11,     /*!< DMA1 Channel 4 to Channel 7 Interrupt                           */
  ADC1_COMP_IRQn              = 12,     /*!< ADC1 and COMP interrupts (ADC interrupt combined with EXTI Lines 21 and 22 */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupt           */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                           */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                           */
  TIM6_DAC_IRQn               = 17,     /*!< TIM6 global and DAC channel underrun error Interrupt            */
  TIM7_IRQn                   = 18,     /*!< TIM7 global Interrupt                                           */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                          */
  TIM15_IRQn                  = 20,     /*!< TIM15 global Interrupt                                          */
  TIM16_IRQn                  = 21,     /*!< TIM16 global Interrupt                                          */
  TIM17_IRQn                  = 22,     /*!< TIM17 global Interrupt                                          */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)      */
  I2C2_IRQn                   = 24,     /*!< I2C2 Event Interrupt                                            */
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                           */
  SPI2_IRQn                   = 26,     /*!< SPI2 global Interrupt                                           */
  USART1_IRQn                 = 27,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup) */
  USART2_IRQn                 = 28,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup) */
  USART3_4_IRQn               = 29,     /*!< USART3 and USART4 global Interrupt                              */
  CEC_CAN_IRQn                = 30,     /*!< CEC and CAN global Interrupts & EXTI Line27 Interrupt           */
  USB_IRQn                    = 31      /*!< USB global Interrupt  & EXTI Line18 Interrupt                   */
} IRQn_Type;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define __NVIC_PRIO_BITS          2 /*!< STM32F0xx uses 2 Bits for the Priority Levels */
#define _BIT_SHIFT(IRQn)         (  ((((uint32_t)(int32_t)(IRQn))         )      &  0x03UL) * 8UL)
#define _SHP_IDX(IRQn)           ( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >>    2UL)      )
#define _IP_IDX(IRQn)            (   (((uint32_t)(int32_t)(IRQn))                >>    2UL)      )

__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IP[_IP_IDX(IRQn)]  = ((uint32_t)(NVIC->IP[_IP_IDX(IRQn)]  & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
       (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
  }
  //else
  {
  //  SCB->SHP[_SHP_IDX(IRQn)] = ((uint32_t)(SCB->SHP[_SHP_IDX(IRQn)] & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
   //    (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
  }
}

#define NVIC_SetPriority            __NVIC_SetPriority

__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

#define NVIC_EnableIRQ              __NVIC_EnableIRQ
  
#endif