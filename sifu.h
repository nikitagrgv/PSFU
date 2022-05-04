#pragma once
#include <math.h>
#include "stm32f10x.h"

uint16_t UART_VALUE = 0;
uint16_t UART_VALUE_TMP = 0;
uint8_t UART_CURRENT_DIGIT = 0;

uint16_t SIFU_COUNTER = 0;
uint16_t COMPARE1 = 301 % 1000;
uint16_t COMPARE2 = 634 % 1000;
uint16_t COMPARE3 = 967 % 1000;
// -----------------------------------------------------------------------------
// interrupts
// -----------------------------------------------------------------------------

// EXTI1 interrupt. Triggered by falling edge on PB1
void EXTI1_IRQHandler(void)
{
    // clear interrupt flag
    EXTI->PR = EXTI_PR_PR1;

    // start timer
    TIM1->CR1 = TIM_CR1_CEN;

    // reload sifu counter
    SIFU_COUNTER = 0;
}

// TIM1 update interrupt. For SIFU
void TIM1_UP_IRQHandler(void)
{
    // clear update interrupt flag
    TIM1->SR &= ~TIM_SR_UIF;

    // increment counter for sifu
    SIFU_COUNTER++;

    // if uart value = 0, do not open thyristors
    if (UART_VALUE == 0)
    {
        GPIOA->BSRR = GPIO_BSRR_BR0 |
                      GPIO_BSRR_BR1 |
                      GPIO_BSRR_BR2;
    }
    else if (SIFU_COUNTER == COMPARE1)
    {
        // set A0, reset others
        GPIOA->BSRR = GPIO_BSRR_BS0 |
                      GPIO_BSRR_BR1 |
                      GPIO_BSRR_BR2;
    }
    else if (SIFU_COUNTER == COMPARE2)
    {
        // set A1, reset others
        GPIOA->BSRR = GPIO_BSRR_BR0 |
                      GPIO_BSRR_BS1 |
                      GPIO_BSRR_BR2;
    }
    else if (SIFU_COUNTER == COMPARE3)
    {
        // set A2, reset others
        GPIOA->BSRR = GPIO_BSRR_BR0 |
                      GPIO_BSRR_BR1 |
                      GPIO_BSRR_BS2;
    }
}

// USART3 interrupt. For receiving and forming value
void USART3_IRQHandler(void)
{
    // read UART data
    char data = USART3->DR;

    // if not digit, ignore
    if (data < '0' || data > '9')
    {
        return;
    }

    // if new value, clear current temporary value
    if (UART_CURRENT_DIGIT == 0)
    {
        UART_VALUE_TMP = 0;
    }

    // add new digit to value
    UART_VALUE_TMP = UART_VALUE_TMP * 10 + (data - '0');
    UART_CURRENT_DIGIT++;

    // if last digit, set value
    if (UART_CURRENT_DIGIT == 3)
    {
        UART_CURRENT_DIGIT = 0;
        // if value greater than 300, set value to 300
        if (UART_VALUE_TMP > 300)
        {
            UART_VALUE_TMP = 300;
        }

        // if value is the same, ignore
        if (UART_VALUE == UART_VALUE_TMP)
        {
            return;
        }

        // set uart value
        UART_VALUE = UART_VALUE_TMP;

        // calculate compare values for sifu
        COMPARE1 = (uint16_t)(159.155 * acosf(0.0040145f * UART_VALUE - 1.f) + 83.3333f) % 1000;
        COMPARE2 = (uint16_t)(159.155 * acosf(0.0040145f * UART_VALUE - 1.f) + 416.667f) % 1000;
        COMPARE3 = (uint16_t)(159.155 * acosf(0.0040145f * UART_VALUE - 1.f) + 750.f) % 1000;

        GPIOA->BSRR = GPIO_BSRR_BR0 |
                      GPIO_BSRR_BR1 |
                      GPIO_BSRR_BR2;
        SIFU_COUNTER = 0;
        TIM1->CR1 &= ~TIM_CR1_CEN; // stop timer
    }
}
