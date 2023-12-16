/* Main program. */
#include <stdint.h>

#include "main.h"

#define USER_LED_INDEX 3
#define BUTTON_PIN_INDEX 13

void init_gpio()
{
    // section 3.3 or table 3 register boundary addresses
    // RCC 0x40021000
    // The register that enables the PORTC is the APB2 peripheral clock enable register (or RCC_APB2ENR)
    // 8.3.7 APB2 peripheral clock enable register

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // on board led
    //  Configure GPIO A pin 5 as output.//PA5
    GPIOA->MODER = 1 << (5 * 2);   // OUTPUT
    GPIOA->OTYPER = 0 << (5);      // PUSH PULL, only used in OUTPUT mode
    GPIOA->OSPEEDR = 0 << (5 * 2); // LOW SPEED, only used in OUTPUT mode
    GPIOA->ODR = 1 << (5);         // OUTPUT HIGH

    // external my led
    //  Configure GPIO B pin 3 as output.
    GPIOB->MODER = 1 << (USER_LED_INDEX * 2);   // OUTPUT
    GPIOB->OTYPER = 0 << (USER_LED_INDEX);      // PUSH PULL
    GPIOB->OSPEEDR = 0 << (USER_LED_INDEX * 2); // LOW SPEED
    GPIOB->ODR = 1 << (USER_LED_INDEX);         // OUTPUT HIGH

    // B1 should be set to 'input' mode with pull-up. //PC13
    GPIOC->MODER &= ~(0x3 << (BUTTON_PIN_INDEX * 2));
    GPIOC->PUPDR &= ~(0x3 << (BUTTON_PIN_INDEX * 2));
    GPIOC->PUPDR |= (0x1 << (BUTTON_PIN_INDEX * 2));
    GPIOC->MODER = 0 << (BUTTON_PIN_INDEX * 2);
}

void init_exti()
{
    // enable SYSCFG
    // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    SYSCFG->EXTICR[BUTTON_PIN_INDEX / 4] &= ~(0XF << ((BUTTON_PIN_INDEX % 4) * 4));
    SYSCFG->EXTICR[BUTTON_PIN_INDEX / 4] |= (0X1 << ((BUTTON_PIN_INDEX % 4) * 4));

    // Setup the button's EXTI line as an interrupt.
    EXTI->IMR |= (1 << BUTTON_PIN_INDEX);
    // Disable the 'rising edge' trigger (button release).
    EXTI->RTSR &= ~(1 << BUTTON_PIN_INDEX);
    // Enable the 'falling edge' trigger (button press).
    EXTI->FTSR |= (1 << BUTTON_PIN_INDEX);

    /* EXTI interrupt init*/
    NVIC_SetPriority(EXTI4_15_IRQn, 0x03);
    //NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void SystemInit(void)
{
}

void EXTI4_15_IRQHandler(void)
{
    if (EXTI->PR & (1 << BUTTON_PIN_INDEX) == (1 << BUTTON_PIN_INDEX))
    {
        // Clear the EXTI status flag.
        EXTI->PR |= (1 << BUTTON_PIN_INDEX);
        toggle_led();
    }
}

void toggle_led()
{
    static int x = 1;
    if (x)
    {
        GPIOA->BSRR = 1 << (5);
        // Set the output bit.
        GPIOB->BSRR = 1 << (USER_LED_INDEX);
        x = 0;
    }
    else
    {
        GPIOA->BRR = 1 << (5);
        x = 1;
        // Reset it again.
        GPIOB->BRR = 1 << (USER_LED_INDEX);
    }
}

int main(void)
{
    int toggle = 40000;

    init_gpio();
    init_exti();

    // Keep track of whether the button is pressed.
    uint8_t button_down = 0;
    // GPIOA->BRR = 1 << (5);
    // GPIOB->BRR = 1 << (USER_LED_INDEX);

    for (;;)
    {
        for (uint32_t i = 0; i < toggle; ++i)
        {
            __asm__ volatile("nop");
        }
        toggle_led();
#if 0
        uint32_t idr_val = (GPIOC->IDR);
        if (idr_val & (1 << BUTTON_PIN_INDEX))
        {
            // The button is pressed; if it was not already
            // pressed, fast blink LED.
            if (!button_down)
            {
                toggle = toggle / 2;
            }
            button_down = 1;
        }
        else
        {
            button_down = 0;
        }

        // Set the output bit.
        GPIOA->BSRR = 1 << (5);
        GPIOB->BSRR = 1 << (USER_LED_INDEX);
        for (uint32_t i = 0; i < toggle; ++i)
        {
            __asm__ volatile("nop");
        }
        // Reset it again.
        GPIOA->BRR = 1 << (5);
        GPIOB->BRR = 1 << (USER_LED_INDEX);
        for (uint32_t i = 0; i < toggle / 2; ++i)
        {
            __asm__ volatile("nop");
        }
#endif
    }
}