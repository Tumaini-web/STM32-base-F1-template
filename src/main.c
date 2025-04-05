// #include "stm32f3xx.h"

// // Quick and dirty delay
// static void delay (unsigned int time) {
//     for (unsigned int i = 0; i < time; i++)
//         for (volatile unsigned int j = 0; j < 2000; j++);
// }


#include "stm32f3xx.h"

volatile uint32_t last_capture = 0;
volatile uint32_t frequency = 0;
volatile uint32_t capture;

void init_gpio(void) {
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA1 as analog for COMP1 input
    GPIOA->MODER |= (3 << (1 * 2));  // Analog mode
}

void init_comp1(void) {
    // Enable SYSCFG clock (for comparator)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Enable comparator
    COMP1->CSR |= COMP1_CSR_COMP1EN; // Enable COMP1, default INP is PA1, INM is VREFINT
    // Output is high when INP > INM (default 1.22V)


    // default INP is PA1, INM is VREFINT
    COMP1->CSR |= COMP1_CSR_COMP1INSEL_0;
    COMP1->CSR |= COMP1_CSR_COMP1INSEL_1;
    COMP1->CSR &= ~(COMP1_CSR_COMP1INSEL_2);
    

    // TIM2 CH4 input capture mapped to COMP1 output (internal routing)
    COMP1->CSR |= COMP1_CSR_COMP1OUTSEL_3;
    COMP1->CSR &= ~(COMP1_CSR_COMP1OUTSEL_2);
    COMP1->CSR &= ~(COMP1_CSR_COMP1OUTSEL_1);
    COMP1->CSR &= ~(COMP1_CSR_COMP1OUTSEL_0);
}

void init_timer2_input_capture(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    

    // Route COMP1 output to TIM2 CH4
    TIM2->PSC = 7999;         // Prescaler for 1 KHz timer clock (8 MHz sysclk)
    TIM2->ARR = 10000;     // Max ARR

    TIM2 -> SMCR |= TIM_SMCR_SMS_1;
    TIM2 -> SMCR |= TIM_SMCR_SMS_2;
    TIM2 -> SMCR &= ~(TIM_SMCR_SMS_0);
    TIM2 -> SMCR &= ~(TIM_SMCR_SMS_3);

    TIM2->CCMR2 |= TIM_CCMR2_CC4S_0; // CC4S = 01: CC4 channel is input, IC4 is mapped to TI4
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC4S_1); // CC4S = 01: CC4 channel is input, IC4 is mapped to TI4

    TIM2->CCER |= TIM_DIER_UIE;
    TIM2->CCER |= TIM_CCER_CC4E;  // Enable capture
    TIM2->DIER |= TIM_DIER_CC4IE; // Enable interrupt on capture
    TIM2->CR1 |= TIM_CR1_CEN;     // Enable timer

    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC4IF) {
        capture = TIM2->CCR4;
        uint32_t diff = (capture - last_capture) & 10000;
        last_capture = capture;

        // Frequency = timer_clock / period_ticks
        frequency = 1000000 / diff;  // Since timer clock = 1 MHz

        TIM2->SR &= ~TIM_SR_CC4IF;
    }
}

int main(void) {
    init_gpio();
    init_comp1();
    init_timer2_input_capture();

    while (1) {
        // frequency variable gets updated in interrupt
        // use it as needed
    }
}