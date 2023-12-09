/* Main program. */
#include <stdint.h>
#define __IO volatile /*!< Defines 'read / write' permissions */
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

#define GPIOB_BASE            (IOPPERIPH_BASE + 0x00000400UL)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)

#define RCC_IOPENR_IOPAEN_Pos (0U)
#define RCC_IOPENR_IOPAEN_Msk (0x1UL << RCC_IOPENR_IOPAEN_Pos) /*!< 0x00000001 */
#define RCC_IOPENR_IOPAEN RCC_IOPENR_IOPAEN_Msk                /*!< GPIO port A clock enable */
#define RCC_IOPENR_GPIOAEN RCC_IOPENR_IOPAEN                   /*!< GPIO port A clock enable */

#define RCC_IOPENR_IOPBEN_Pos            (1U) 
#define RCC_IOPENR_IOPBEN_Msk            (0x1UL << RCC_IOPENR_IOPBEN_Pos)       /*!< 0x00000002 */
#define RCC_IOPENR_IOPBEN                RCC_IOPENR_IOPBEN_Msk                 /*!< GPIO port B clock enable */
#define RCC_IOPENR_GPIOBEN                  RCC_IOPENR_IOPBEN        /*!< GPIO port B clock enable */

#define __HAL_RCC_GPIOA_CLK_ENABLE()                        \
    do                                                      \
    {                                                       \
        __IO uint32_t tmpreg;                               \
        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);           \
        /* Delay after an RCC peripheral clock enabling */  \
        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN); \
        UNUSED(tmpreg);                                     \
    } while (0)


#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);\
                                        UNUSED(tmpreg); \
                                      } while(0)

void init_gpio()
{
    // section 3.3 or table 3 register boundary addresses
    // RCC 0x40021000
    // The register that enables the PORTC is the APB2 peripheral clock enable register (or RCC_APB2ENR)
    // 8.3.7 APB2 peripheral clock enable register

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO A pin 5 as output.
    GPIOA->MODER = 1 << (5 * 2);   // OUTPUT
    GPIOA->OTYPER = 0 << (5);      // PUSH PULL
    GPIOA->OSPEEDR = 0 << (5 * 2); // LOW SPEED
    GPIOA->ODR = 1 << (5);         // OUTPUT HIGH

    // Configure GPIO B pin 3 as output.
    GPIOB->MODER = 1 << (3 * 2);   // OUTPUT
    GPIOB->OTYPER = 0 << (3);      // PUSH PULL
    GPIOB->OSPEEDR = 0 << (3 * 2); // LOW SPEED
    GPIOB->ODR = 1 << (3);         // OUTPUT HIGH
}

void SystemInit(void)
{
}

int main(void)
{
    init_gpio();
    for (;;)
    {

#if 1
        // Set the output bit.
        GPIOA->BSRR = 1 << (5);
        GPIOB->BSRR = 1 << (3);
        for (uint32_t i = 0; i < 400000; ++i)
        {
            __asm__ volatile("nop");
        }
        // Reset it again.
        GPIOA->BRR = 1 << (5);
        GPIOB->BRR = 1 << (3);
        for (uint32_t i = 0; i < 80000; ++i)
        {
            __asm__ volatile("nop");
        }

#endif
    }
}