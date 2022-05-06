#include "stm32f10x.h"
#include "modbus.h"
#include "meander.h"

const uint16_t TIM4_AUTORELOAD_VALUE = 72e6 / 11e3; // 11 kHz PWM

void InitPeriph()
{
    // -------------------------------------------------------------------------
    // clocks
    // -------------------------------------------------------------------------
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |   // GPIOA
                    RCC_APB2ENR_IOPBEN |   // GPOIB
                    RCC_APB2ENR_AFIOEN |   // AFIO
                    RCC_APB2ENR_USART1EN | // USART1
                    RCC_APB2ENR_TIM1EN;    // TIM1

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN |  // TIM2
                    RCC_APB1ENR_TIM3EN |  // TIM3
                    RCC_APB1ENR_TIM4EN |  // TIM4
                    RCC_APB1ENR_USART3EN; // USART3

    // -------------------------------------------------------------------------
    // GPIO
    // -------------------------------------------------------------------------
    // A0, A1, A2 - push pull - for SIFU
    GPIOA->CRL = 0x00000222;
    // A9 - alternate push pull output - USART1 TX
    // A10 - floating input - USART1 RX
    GPIOA->CRH = 0x000004B0;
    // B0 - floating input - EXTI for meander measurement
    // B1 - floating input - EXTI for SIFU
    // B6 - alternate push pull output - TIM4 ouput, for PWM
    GPIOB->CRL = 0x0B000044;
    // B11 - floating input - USART3 RX
    GPIOB->CRH = 0x00004000;

    // -------------------------------------------------------------------------
    // EXTI - for SIFU. Falling edge detection on B1
    // -------------------------------------------------------------------------
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB; // choose B1 pin
    EXTI->FTSR |= EXTI_FTSR_TR1;              // falling edge trigger
    EXTI->IMR |= EXTI_IMR_MR1;                // enable interrupt for EXTI0

    // -------------------------------------------------------------------------
    // TIM1 - for SIFU. 50*1000 Hz update rate
    // -------------------------------------------------------------------------
    TIM1->ARR = 1440 - 1;      // auto reload value for 50*1000 Hz
    TIM1->DIER = TIM_DIER_UIE; // enable update interrupt
    TIM1->CR1 = TIM_CR1_CEN;   // start timer

    // -------------------------------------------------------------------------
    // TIM4 - for PWM generation
    // -------------------------------------------------------------------------
    TIM4->CR1 |= TIM_CR1_DIR;              // downcounting direction
    TIM4->CCMR1 |= 0x60;                   // PWM compare mode
    TIM4->ARR = TIM4_AUTORELOAD_VALUE - 1; // auto reload value for 11 kHz
    TIM4->CCR1 = 0;                        // set compare value
    TIM4->CCER = TIM_CCER_CC1E;            // enable timer signal on T4C1 pin (B6)
    TIM4->CR1 |= TIM_CR1_CEN;              // start timer

    // -------------------------------------------------------------------------
    // EXTI - for meander measurement. Falling edge detection on B0
    // -------------------------------------------------------------------------
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB; // choose B0 pin
    EXTI->FTSR |= EXTI_FTSR_TR0;              // falling edge trigger
    EXTI->IMR |= EXTI_IMR_MR0;                // enable interrupt for EXTI0

    // -------------------------------------------------------------------------
    // TIM2 - for meander measurement. 100 Hz update rate
    // -------------------------------------------------------------------------
    TIM2->PSC = 7200 - 1;      // reduce clock frequency to 10kHz
    TIM2->ARR = 100 - 1;       // auto reload value for 100 Hz
    TIM2->DIER = TIM_DIER_UIE; // enable update interrupt
    TIM2->CR1 = TIM_CR1_CEN;   // start timer

    // -------------------------------------------------------------------------
    // TIM3 - for modbus packets separaion. 1.5 symbols - separate time
    // -------------------------------------------------------------------------
    TIM3->PSC = 720 - 1;       // reduce clock frequency to 100kHz
    TIM3->ARR = 156 - 1;       // auto reload value for period of 1.5 symbols
    TIM3->DIER = TIM_DIER_UIE; // enable update interrupt
    TIM3->CR1 = TIM_CR1_CEN;   // start timer

    // -------------------------------------------------------------------------
    // USART1 - for Modbus. 9600 baudrate
    // -------------------------------------------------------------------------
    USART1->BRR = 7500; // USARTDIV = 72e6 / 9600 = 7500

    USART1->CR1 = USART_CR1_UE |    // enable USART
                  USART_CR1_TE |    // enable transmitter
                  USART_CR1_RE |    // enable receiver
                  USART_CR1_RXNEIE; // enable receive interrupt

    // -------------------------------------------------------------------------
    // USART3 - for receiving value. 9600 baudrate
    // -------------------------------------------------------------------------
    USART3->BRR = 3750; // USARTDIV = 36e6 / 9600 = 7500

    USART3->CR1 = USART_CR1_UE |    // enable USART
                  USART_CR1_RE |    // enable receiver
                  USART_CR1_RXNEIE; // enable receive interrupt

    // -------------------------------------------------------------------------
    // enable interrupts in NVIC
    // -------------------------------------------------------------------------
    NVIC_EnableIRQ(EXTI0_IRQn);   // EXTI0
    NVIC_EnableIRQ(EXTI1_IRQn);   // EXTI1
    NVIC_EnableIRQ(TIM1_UP_IRQn); // TIM1
    NVIC_EnableIRQ(TIM2_IRQn);    // TIM2
    NVIC_EnableIRQ(TIM3_IRQn);    // TIM3
    NVIC_EnableIRQ(USART1_IRQn);  // USART1
    NVIC_EnableIRQ(USART3_IRQn);  // USART3
    __enable_irq();               // enable interrupts
}

int main()
{
    // initialize peripherals
    InitPeriph();

    // modbus initialization
    Modbus.in.size = 0;
    Modbus.out.size = 0;
    Modbus.state = READY;

    while (1)
    {
        // if modbus has received packet, go to process it
        if (Modbus.state == PROCESSING)
        {
            ProcessPacket();
            Modbus.state = READY;
        }
    }
}
