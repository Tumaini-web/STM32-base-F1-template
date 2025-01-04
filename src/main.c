#include "stm32f1xx.h"

uint8_t led_state = 0;
uint32_t timee;
uint32_t start_timee;
uint32_t final_time;
uint32_t flag = 1;

uint8_t buttonState = 0;
uint8_t lastButtonState = 1;

static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}

// void delay2(uint16_t millis){

// 	TIM1->CR2 |= TIM_CR2_MMS_1;
// 	TIM1->ARR = millis - 1;

// 	TIM2->SMCR &= ~(TIM_SMCR_TS);
// 	TIM2->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;

// 	TIM1->CR1 |= TIM_CR1_CEN;
// }

// void delay(uint16_t ms)
// {
	

// 	TIM1->PSC=8000-1; //8 000 000 Hz / 8 000 = 1 000 Hz (1 ms)
	
// 	TIM1->ARR = ms - 1;


// 	// TIM1->CR1 |= TIM_CR1_CEN;
	
// 	while(!(TIM1->SR&TIM_SR_UIF)){} //wait UIF to be set
// 	TIM1->SR&=~TIM_SR_UIF; //reset UIF
// 	TIM1->CR1&=~TIM_CR1_CEN; // Disable the timer
// }



int main(void)
{
	/*Enable clock access to GPIOA*/
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

	/*Configure PA0 as output*/
	GPIOA->CRL|=GPIO_CRL_MODE0;
	GPIOA->CRL&=~(GPIO_CRL_CNF0);

	GPIOA->CRH &= ~(GPIO_CRH_MODE9);
	GPIOA->CRH |= GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);


  
	/*Enable clock access to timer2*/
	
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;


	TIM1->CR2 |= TIM_CR2_MMS_1;
	TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM1->CCER &= ~(TIM_CCER_CC1P);
	TIM1->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2;
	TIM1->SMCR |= TIM_SMCR_TS_0 | TIM_SMCR_TS_2;

	TIM1->PSC=8000-1; //8 000 000 Hz / 8 000 = 1 000 Hz (1 ms)
	
	TIM1->ARR = 10000 - 1;

	TIM1->CR1 |= TIM_CR1_CEN;



	


	while(1)
	{

		// buttonState = (READ_BIT(GPIOA->IDR, GPIO_IDR_IDR8)) ? 1 : 0;
            
        //     if(lastButtonState == 1 && buttonState == 0){
		// 		start_timee = TIM1->CNT;
        //     }

        //     lastButtonState = buttonState;

			
			
		
			final_time = TIM1->CNT;
			

			// timee = final_time - start_timee;

			
			// if(timee >= 500){

			// 	GPIOA->BSRR=GPIO_BSRR_BS0;//Set PA0 to high
			// }
			// if(timee >1000){
			// 	GPIOA->BSRR=GPIO_BSRR_BR0; // Set PA0 to low
			// }
		
	
		
		
		// GPIOA->BSRR=GPIO_BSRR_BS0;//Set PA0 to high
		// led_state = 1;

		// delay(1000); //Delay for 1 seconds
		// GPIOA->BSRR=GPIO_BSRR_BR0; // Set PA0 to low
		// led_state = 0;

		

		// delay(1000); // Delay for 1 seconds

	}
}

