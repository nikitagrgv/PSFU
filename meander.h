#pragma once
#include "stm32f10x.h"

const uint16_t TIM4_AUTORELOAD_VALUE;
const uint16_t MEANDER_MIN = 2e3;  // min value of meander
const uint16_t MEANDER_MAX = 17e3; // max value of meander

uint32_t MEANDER_EDGES = 0; // current count of falling edges
uint16_t MEANDER_FREQ = 0;  // current meander frequency

// -----------------------------------------------------------------------------
// interrupts
// -----------------------------------------------------------------------------

// TIM2 interrupt. TIM2 has a period of 10ms (100 Hz)
void TIM2_IRQHandler(void)
{
    // clear interrupt flag
    TIM2->SR = 0;
    // frequency = TIM2 frequency * edges count
    MEANDER_FREQ = MEANDER_EDGES * 100;
    // clear count of edges
    MEANDER_EDGES = 0;

    // -------------------------------------------------------------------------
    // calculate corresponding compare value for PWM
    // y = k*x - b
    //
    // k = TIM4_AUTORELOAD_VALUE / (MEANDER_MAX - MEANDER_MIN)
    // b = (MEANDER_MIN * TIM4_AUTORELOAD_VALUE) / (MEANDER_MAX - MEANDER_MIN)
    //
    // y = (TIM4_AUTORELOAD_VALUE * (x - MEANDER_MIN)) / (MEANDER_MAX - MEANDER_MIN)
    // -------------------------------------------------------------------------
    int32_t compare_value = (int32_t)TIM4_AUTORELOAD_VALUE *
                            ((int32_t)MEANDER_FREQ - MEANDER_MIN) /
                            ((int32_t)MEANDER_MAX - MEANDER_MIN);
    // if > max
    if (compare_value > TIM4_AUTORELOAD_VALUE)
    {
        compare_value = TIM4_AUTORELOAD_VALUE;
    }
    // if < min
    else if (compare_value < 0)
    {
        compare_value = 0;
    }
    TIM4->CCR1 = compare_value;
}

// EXTI0 interrupt. Triggered by falling edge on PB0
void EXTI0_IRQHandler(void)
{
    // clear interrupt flag
    EXTI->PR = EXTI_PR_PR0;
    // add new edge
    MEANDER_EDGES++;
}
