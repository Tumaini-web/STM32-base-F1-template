#include "stm32f1xx.h"

#include "encoder.h"

// Quick and dirty delay
static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}

uint16_t encoder_read_previous;

void Numb(uint16_t digit);

int main (void) {


	encoder_init(400);
    // Turn on the GPIOC and B peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    

    // Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF0);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE0_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE1_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF2);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE2_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF3);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE3_1;

	GPIOA->CRL &= ~(GPIO_CRL_CNF4);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE4_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF5);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE5_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF6);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE6_1;

	// Put pin 13 in general purpose push-pull mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF7);
    // Set the output mode to max. 2MHz
    GPIOA->CRL |= GPIO_CRL_MODE7_1;

   

    while (1) {


		if(encoder_read()!=encoder_read_previous)
		{
			encoder_read_previous=encoder_read()/4;

			Numb(encoder_read_previous);
			//printf("Encoder counts = %d\r\n",encoder_read());

		}
        // Reset the state of pin 13 to output low
        // GPIOA->BSRR = GPIO_BSRR_BR0;

        // GPIOA->BSRR = GPIO_BSRR_BR1;

		// GPIOA->BSRR = GPIO_BSRR_BR2;

        // GPIOA->BSRR = GPIO_BSRR_BR3;

        // delay(200);

        // Set the state of pin 13 to output high
        
		
    }

    // Return 0 to satisfy compiler
    return 0;
}


void Numb(uint16_t digit){
		
		// GPIOA->BSRR = GPIO_BSRR_BR0;    // G
        // GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		// GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        // GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		// GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        // GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		// GPIOA->BSRR = GPIO_BSRR_BS6;	// E
        // GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        // delay(200);

		if (digit == 1){
			GPIOA->BSRR = GPIO_BSRR_BR0;    // G
        GPIOA->BSRR = GPIO_BSRR_BR1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BR5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BR7;	// A
        //delay(200);
		}
		
		if (digit == 2){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BR3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BR5;	// F
		GPIOA->BSRR = GPIO_BSRR_BS6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 3){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BR5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 4){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BR1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BR7;	// A
        //delay(200);
		}
		
		if (digit == 5){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BR4;	// B
        GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 6){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BR4;	// B
        GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		GPIOA->BSRR = GPIO_BSRR_BS6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 7){
			GPIOA->BSRR = GPIO_BSRR_BR0;    // G
        GPIOA->BSRR = GPIO_BSRR_BR1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BR5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 8){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BS1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		GPIOA->BSRR = GPIO_BSRR_BS6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);
		}
		
		if (digit == 9){
			GPIOA->BSRR = GPIO_BSRR_BS0;    // G
        GPIOA->BSRR = GPIO_BSRR_BR1;	// D
		GPIOA->BSRR = GPIO_BSRR_BR2;	// H
        GPIOA->BSRR = GPIO_BSRR_BS3;	// C
		GPIOA->BSRR = GPIO_BSRR_BS4;	// B
        GPIOA->BSRR = GPIO_BSRR_BS5;	// F
		GPIOA->BSRR = GPIO_BSRR_BR6;	// E
        GPIOA->BSRR = GPIO_BSRR_BS7;	// A
        //delay(200);

		}
		
}