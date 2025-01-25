/*
 * encoder.c
 *
 *  Created on: Jan 19, 2023
 *      Author: hussamaldean
 */


// #include "encoder.h"
#include "stm32f1xx.h"

void encoder_init(uint16_t max_value)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;

	/*Configure PA0 as Output Alternate Push/Pull */
	GPIOB->CRL&=~GPIO_CRL_MODE6;
	GPIOB->CRL|=(GPIO_CRL_CNF6_0);
	GPIOB->CRL&=~(GPIO_CRL_CNF6_1);

	/*Configure PA1 as Output Alternate Push/Pull*/

	GPIOB->CRL&=~GPIO_CRL_MODE7;
	GPIOB->CRL|=(GPIO_CRL_CNF7_0);
	GPIOB->CRL&=~(GPIO_CRL_CNF1_1);


	/*Enable clock access to timer2*/
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;

	/*Configure timer2*/
	TIM4->ARR=max_value-1;

	TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );
	TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM4->CR1 |= TIM_CR1_CEN;


}

uint16_t encoder_read(void){

	return  TIM4->CNT;
}