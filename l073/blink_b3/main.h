#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#define __IO volatile /*!< Defines 'read / write' permissions */
#define __STATIC_INLINE static inline

#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define UNUSED(X) (void)X /* To avoid gcc/g++ warnings */

typedef struct
{
  __IO uint32_t MODER;   /*!< GPIO port mode register,                     Address offset: 0x00 */
  __IO uint32_t OTYPER;  /*!< GPIO port output type register,              Address offset: 0x04 */
  __IO uint32_t OSPEEDR; /*!< GPIO port output speed register,             Address offset: 0x08 */
  __IO uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C */
  __IO uint32_t IDR;     /*!< GPIO port input data register,               Address offset: 0x10 */
  __IO uint32_t ODR;     /*!< GPIO port output data register,              Address offset: 0x14 */
  __IO uint32_t BSRR;    /*!< GPIO port bit set/reset registerBSRR,        Address offset: 0x18 */
  __IO uint32_t LCKR;    /*!< GPIO port configuration lock register,       Address offset: 0x1C */
  __IO uint32_t AFR[2];  /*!< GPIO alternate function register,            Address offset: 0x20-0x24 */
  __IO uint32_t BRR;     /*!< GPIO bit reset register,                     Address offset: 0x28 */
} GPIO_TypeDef;
/**
 * @brief Reset and Clock Control
 */

typedef struct
{
  __IO uint32_t CR;        /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t ICSCR;     /*!< RCC Internal clock sources calibration register,              Address offset: 0x04 */
  __IO uint32_t CRRCR;     /*!< RCC Clock recovery RC register,                               Address offset: 0x08 */
  __IO uint32_t CFGR;      /*!< RCC Clock configuration register,                             Address offset: 0x0C */
  __IO uint32_t CIER;      /*!< RCC Clock interrupt enable register,                          Address offset: 0x10 */
  __IO uint32_t CIFR;      /*!< RCC Clock interrupt flag register,                            Address offset: 0x14 */
  __IO uint32_t CICR;      /*!< RCC Clock interrupt clear register,                           Address offset: 0x18 */
  __IO uint32_t IOPRSTR;   /*!< RCC IO port reset register,                                   Address offset: 0x1C */
  __IO uint32_t AHBRSTR;   /*!< RCC AHB peripheral reset register,                            Address offset: 0x20 */
  __IO uint32_t APB2RSTR;  /*!< RCC APB2 peripheral reset register,                           Address offset: 0x24 */
  __IO uint32_t APB1RSTR;  /*!< RCC APB1 peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t IOPENR;    /*!< RCC Clock IO port enable register,                            Address offset: 0x2C */
  __IO uint32_t AHBENR;    /*!< RCC AHB peripheral clock enable register,                     Address offset: 0x30 */
  __IO uint32_t APB2ENR;   /*!< RCC APB2 peripheral enable register,                          Address offset: 0x34 */
  __IO uint32_t APB1ENR;   /*!< RCC APB1 peripheral enable register,                          Address offset: 0x38 */
  __IO uint32_t IOPSMENR;  /*!< RCC IO port clock enable in sleep mode register,              Address offset: 0x3C */
  __IO uint32_t AHBSMENR;  /*!< RCC AHB peripheral clock enable in sleep mode register,       Address offset: 0x40 */
  __IO uint32_t APB2SMENR; /*!< RCC APB2 peripheral clock enable in sleep mode register,      Address offset: 0x44 */
  __IO uint32_t APB1SMENR; /*!< RCC APB1 peripheral clock enable in sleep mode register,      Address offset: 0x48 */
  __IO uint32_t CCIPR;     /*!< RCC clock configuration register,                             Address offset: 0x4C */
  __IO uint32_t CSR;       /*!< RCC Control/status register,                                  Address offset: 0x50 */
} RCC_TypeDef;

#define PERIPH_BASE (0x40000000UL) /*!< Peripheral base address in the alias region */
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000UL)
#define IOPPERIPH_BASE (PERIPH_BASE + 0x10000000UL)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000UL)
// 0x40021000

#define RCC ((RCC_TypeDef *)RCC_BASE)

#define GPIOA_BASE (IOPPERIPH_BASE + 0x00000000UL)
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)

#define GPIOB_BASE (IOPPERIPH_BASE + 0x00000400UL)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)

#define GPIO_PIN_13 (1 << 13)
#define GPIOC_BASE (IOPPERIPH_BASE + 0x00000800UL)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define BUTTON_PIN GPIO_PIN_13
#define BUTTON_PORT GPIOC

#define RCC_IOPENR_IOPAEN_Pos (0U)
#define RCC_IOPENR_IOPAEN_Msk (0x1UL << RCC_IOPENR_IOPAEN_Pos) /*!< 0x00000001 */
#define RCC_IOPENR_IOPAEN RCC_IOPENR_IOPAEN_Msk                /*!< GPIO port A clock enable */
#define RCC_IOPENR_GPIOAEN RCC_IOPENR_IOPAEN                   /*!< GPIO port A clock enable */

#define RCC_IOPENR_IOPBEN_Pos (1U)
#define RCC_IOPENR_IOPBEN_Msk (0x1UL << RCC_IOPENR_IOPBEN_Pos) /*!< 0x00000002 */
#define RCC_IOPENR_IOPBEN RCC_IOPENR_IOPBEN_Msk                /*!< GPIO port B clock enable */
#define RCC_IOPENR_GPIOBEN RCC_IOPENR_IOPBEN                   /*!< GPIO port B clock enable */

#define RCC_IOPENR_IOPCEN_Pos (2U)
#define RCC_IOPENR_IOPCEN_Msk (0x1UL << RCC_IOPENR_IOPCEN_Pos) /*!< 0x00000004 */
#define RCC_IOPENR_IOPCEN RCC_IOPENR_IOPCEN_Msk                /*!< GPIO port C clock enable */
#define RCC_IOPENR_GPIOCEN RCC_IOPENR_IOPCEN                   /*!< GPIO port C clock enable */

#define __HAL_RCC_GPIOA_CLK_ENABLE()                    \
  do                                                    \
  {                                                     \
    __IO uint32_t tmpreg;                               \
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);           \
    /* Delay after an RCC peripheral clock enabling */  \
    tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN); \
    UNUSED(tmpreg);                                     \
  } while (0)

#define __HAL_RCC_GPIOB_CLK_ENABLE()                    \
  do                                                    \
  {                                                     \
    __IO uint32_t tmpreg;                               \
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);           \
    /* Delay after an RCC peripheral clock enabling */  \
    tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN); \
    UNUSED(tmpreg);                                     \
  } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()                    \
  do                                                    \
  {                                                     \
    __IO uint32_t tmpreg;                               \
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN);           \
    /* Delay after an RCC peripheral clock enabling */  \
    tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN); \
    UNUSED(tmpreg);                                     \
  } while (0)

typedef struct
{
  __IO uint32_t CFGR1;     /*!< SYSCFG configuration register 1,                    Address offset: 0x00 */
  __IO uint32_t CFGR2;     /*!< SYSCFG configuration register 2,                    Address offset: 0x04 */
  __IO uint32_t EXTICR[4]; /*!< SYSCFG external interrupt configuration register,   Address offset: 0x14-0x08 */
  uint32_t RESERVED[2];    /*!< Reserved,                                           0x18-0x1C */
  __IO uint32_t CFGR3;     /*!< SYSCFG configuration register 3,                    Address offset: 0x20 */
} SYSCFG_TypeDef;

#define APBPERIPH_BASE PERIPH_BASE
#define SYSCFG_BASE (APBPERIPH_BASE + 0x00010000UL)
#define SYSCFG ((SYSCFG_TypeDef *)SYSCFG_BASE)
#define RCC_APB2ENR_SYSCFGEN_Pos (0U)
#define RCC_APB2ENR_SYSCFGEN_Msk (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_SYSCFGEN RCC_APB2ENR_SYSCFGEN_Msk                /*!< SYSCFG clock enable */

#define __HAL_RCC_SYSCFG_CLK_ENABLE() SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_SYSCFGEN))

typedef struct
{
  __IO uint32_t IMR;   /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;   /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;  /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;  /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER; /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;    /*!<EXTI Pending register,                        Address offset: 0x14 */
} EXTI_TypeDef;

#define EXTI_BASE (APBPERIPH_BASE + 0x00010400UL)
#define EXTI ((EXTI_TypeDef *)EXTI_BASE)

#define BUTTON_PIN_INDEX 13

/**
 * @brief stm32l073xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */

/*!< Interrupt Number Definition */
typedef enum
{
  /******  Cortex-M0 Processor Exceptions Numbers ******************************************************/
  NonMaskableInt_IRQn = -14, /*!< 2 Non Maskable Interrupt                                */
  HardFault_IRQn = -13,      /*!< 3 Cortex-M0+ Hard Fault Interrupt                       */
  SVCall_IRQn = -5,          /*!< 11 Cortex-M0+ SV Call Interrupt                         */
  PendSV_IRQn = -2,          /*!< 14 Cortex-M0+ Pend SV Interrupt                         */
  SysTick_IRQn = -1,         /*!< 15 Cortex-M0+ System Tick Interrupt                     */

  /******  STM32L-0 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn = 0,                 /*!< Window WatchDog Interrupt                               */
  PVD_IRQn = 1,                  /*!< PVD through EXTI Line detect Interrupt                  */
  RTC_IRQn = 2,                  /*!< RTC through EXTI Line Interrupt                         */
  FLASH_IRQn = 3,                /*!< FLASH Interrupt                                         */
  RCC_CRS_IRQn = 4,              /*!< RCC and CRS Interrupts                                  */
  EXTI0_1_IRQn = 5,              /*!< EXTI Line 0 and 1 Interrupts                            */
  EXTI2_3_IRQn = 6,              /*!< EXTI Line 2 and 3 Interrupts                            */
  EXTI4_15_IRQn = 7,             /*!< EXTI Line 4 to 15 Interrupts                            */
  TSC_IRQn = 8,                  /*!< TSC Interrupt                                           */
  DMA1_Channel1_IRQn = 9,        /*!< DMA1 Channel 1 Interrupt                                */
  DMA1_Channel2_3_IRQn = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupts                 */
  DMA1_Channel4_5_6_7_IRQn = 11, /*!< DMA1 Channel 4, Channel 5, Channel 6 and Channel 7 Interrupts */
  ADC1_COMP_IRQn = 12,           /*!< ADC1, COMP1 and COMP2 Interrupts                        */
  LPTIM1_IRQn = 13,              /*!< LPTIM1 Interrupt                                        */
  USART4_5_IRQn = 14,            /*!< USART4 and USART5 Interrupt                             */
  TIM2_IRQn = 15,                /*!< TIM2 Interrupt                                          */
  TIM3_IRQn = 16,                /*!< TIM3 Interrupt                                          */
  TIM6_DAC_IRQn = 17,            /*!< TIM6 and DAC Interrupts                                 */
  TIM7_IRQn = 18,                /*!< TIM7 Interrupt                                          */
  TIM21_IRQn = 20,               /*!< TIM21 Interrupt                                         */
  I2C3_IRQn = 21,                /*!< I2C3 Interrupt                                          */
  TIM22_IRQn = 22,               /*!< TIM22 Interrupt                                         */
  I2C1_IRQn = 23,                /*!< I2C1 Interrupt                                          */
  I2C2_IRQn = 24,                /*!< I2C2 Interrupt                                          */
  SPI1_IRQn = 25,                /*!< SPI1 Interrupt                                          */
  SPI2_IRQn = 26,                /*!< SPI2 Interrupt                                          */
  USART1_IRQn = 27,              /*!< USART1 Interrupt                                        */
  USART2_IRQn = 28,              /*!< USART2 Interrupt                                        */
  RNG_LPUART1_IRQn = 29,         /*!< RNG and LPUART1 Interrupts                              */
  LCD_IRQn = 30,                 /*!< LCD Interrupt                                           */
  USB_IRQn = 31,                 /*!< USB global Interrupt                                    */
} IRQn_Type;

#define __IM volatile const /*! Defines 'read only' structure member permissions */
#define __IOM volatile      /*! Defines 'read / write' structure member permissions */
/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM uint32_t ISER[1U]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[31U];
  __IOM uint32_t ICER[1U]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RSERVED1[31U];
  __IOM uint32_t ISPR[1U]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[31U];
  __IOM uint32_t ICPR[1U]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[31U];
  uint32_t RESERVED4[64U];
  __IOM uint32_t IP[8U]; /*!< Offset: 0x300 (R/W)  Interrupt Priority Register */
} NVIC_Type;
/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM uint32_t CPUID; /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR; /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
  __IOM uint32_t VTOR; /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
#else
  uint32_t RESERVED0;
#endif
  __IOM uint32_t AIRCR; /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;   /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;   /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  uint32_t RESERVED1;
  __IOM uint32_t SHP[2U]; /*!< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED */
  __IOM uint32_t SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
} SCB_Type;

#define SCS_BASE (0xE000E000UL)         /*!< System Control Space Base Address */
#define NVIC_BASE (SCS_BASE + 0x0100UL) /*!< NVIC Base Address */
#define SCB_BASE (SCS_BASE + 0x0D00UL)  /*!< System Control Block Base Address */

#define SCB ((SCB_Type *)SCB_BASE)    /*!< SCB configuration struct */
#define NVIC ((NVIC_Type *)NVIC_BASE) /*!< NVIC configuration struct */

/* Interrupt Priorities are WORD accessible only under Armv6-M                  */
/* The following MACROS handle generation of the register offset and byte masks */
#define _BIT_SHIFT(IRQn) (((((uint32_t)(int32_t)(IRQn))) & 0x03UL) * 8UL)
#define _SHP_IDX(IRQn) ((((((uint32_t)(int32_t)(IRQn)) & 0x0FUL) - 8UL) >> 2UL))
#define _IP_IDX(IRQn) ((((uint32_t)(int32_t)(IRQn)) >> 2UL))

#define __NVIC_PRIO_BITS 2U /*!< STM32L0xx uses 2 Bits for the Priority Levels */

__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IP[_IP_IDX(IRQn)] = ((uint32_t)(NVIC->IP[_IP_IDX(IRQn)] & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
                               (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
  }
  else
  {
    SCB->SHP[_SHP_IDX(IRQn)] = ((uint32_t)(SCB->SHP[_SHP_IDX(IRQn)] & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
                                (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
  }
}

#define NVIC_SetPriority __NVIC_SetPriority


__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

#define NVIC_EnableIRQ __NVIC_EnableIRQ

#endif