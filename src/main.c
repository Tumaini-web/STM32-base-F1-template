#include "stm32f1xx.h"

// Quick and dirty delay
static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}

int main (void) {
    // Turn on the GPIOC and B peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // Put pin 13 in general purpose push-pull mode
    GPIOC->CRH &= ~(GPIO_CRH_CNF13);
    // Set the output mode to max. 2MHz
    GPIOC->CRH |= GPIO_CRH_MODE13_1;

    // Put pin 12 in general purpose push-pull mode
    GPIOB->CRH &= ~(GPIO_CRH_CNF12);
    // Set the output mode to max. 2MHz
    GPIOB->CRH |= GPIO_CRH_MODE12_1;

    while (1) {
        // Reset the state of pin 13 to output low
        GPIOC->BSRR = GPIO_BSRR_BR13;

        GPIOB->BSRR = GPIO_BSRR_BR12;

        delay(200);

        // Set the state of pin 13 to output high
        GPIOC->BSRR = GPIO_BSRR_BS13;

        GPIOB->BSRR = GPIO_BSRR_BS12;

        delay(200);
    }

    // Return 0 to satisfy compiler
    return 0;
}
