#include "stm32f3xx.h"

// Quick and dirty delay
static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}


int main (void) {


    // Turn on the GPIOC and B peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER |= GPIO_MODER_MODER15_0;
    GPIOA -> MODER &= ~(GPIO_MODER_MODER15_1);

    GPIOA -> OTYPER |= GPIO_OTYPER_OT_15;

    GPIOA -> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR15);



    while (1) {


		GPIOA -> BSRR |= GPIO_BSRR_BS_15;

        delay(200);
        
		GPIOA -> BSRR |= GPIO_BSRR_BR_15;

        delay(200);
    }

    // Return 0 to satisfy compiler
    return 0;
}


