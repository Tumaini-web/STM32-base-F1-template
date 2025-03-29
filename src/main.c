#include "stm32f3xx.h"

// Quick and dirty delay
static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}


int main (void) {


    // Turn on the GPIOC and set it as alternate function
    RCC-> AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER &= ~(GPIO_MODER_MODER8_0);
    GPIOA -> MODER |= GPIO_MODER_MODER8_1;

    GPIOA -> OTYPER |= GPIO_OTYPER_OT_8;

    GPIOA -> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);

    GPIOA -> AFR[7] |= 0X6;


    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC=7999; //set prescaller to 0 (no divider)
    TIM1->ARR=255; //set the maximum count value
     //seset the current count

    TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_1;
    TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_2;
    TIM1 -> CCMR1 &= ~(TIM_CCMR1_OC1M_0);
    TIM1 -> CCMR1 &= ~(TIM_CCMR1_OC1M_3);

    TIM1 -> CCMR1 |= TIM_CCMR1_OC1PE;

    TIM1 -> CCER |= TIM_CCER_CC1P;
    TIM1 -> CCER |= TIM_CCER_CC1E;


    TIM1 -> CR1 |= TIM_CR1_CEN;





    while (1) {


		TIM1 -> CCR1 = 20;
    }

    // Return 0 to satisfy compiler
    return 0;
}


