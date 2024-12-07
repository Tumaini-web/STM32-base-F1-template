#include "stm32f1xx.h"


int main(void)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

	/*Configure PA0 as Output Alternate Push/Pull */
	GPIOA->CRL|=GPIO_CRL_MODE0;
	GPIOA->CRL|=(GPIO_CRL_CNF0_1);
	GPIOA->CRL&=~(GPIO_CRL_CNF0_0);


   /*Set PA1 to analog Mode*/
	GPIOA->CRL&=~GPIO_CRL_CNF1;
	GPIOA->CRL&=~GPIO_CRL_MODE1;


    /*Enable clock access to ADC1*/
	RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;

    /*Set trigger mode to be external*/
	ADC1->CR2|=ADC_CR2_EXTTRIG;

	/*Set external trigger to be TIM3_TRGO event*/

	ADC1->CR2|=ADC_CR2_EXTSEL_2;

    

	/*Don't remap the pin*/
	AFIO->MAPR&=~AFIO_MAPR_TIM2_REMAP;


	/*Enable clock access to timer2 and 3*/
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;

    RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;

	/*Configure timer2*/
	TIM2->PSC=18;                         // DIVIDE 8MHz by 57 == 140000
	TIM2->ARR=6;                         // 14000 divided by 2 == 70KHz
	TIM2->CCMR1|=TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1;
	TIM2->CCER|=TIM_CCER_CC1E;
	TIM2->CR1|=TIM_CR1_CEN;


    /*Configure timer3*/

    TIM3->CR2|=TIM_CR2_MMS_1;

    TIM3->PSC=8000-1;
	TIM3->ARR=1000-1;

    TIM3->CR1|=TIM_CR1_CEN;

    
    	/*Power up the adc*/
	ADC1->CR2|=ADC_CR2_ADON;


	/*Launch the ADC*/
	ADC1->CR2|=ADC_CR2_ADON;



	while(1)
	{
		

            TIM2->CCR1=3;

            
		/*Relaunch the ADC*/
		ADC1->CR2|=ADC_CR2_ADON;
		/*Launch the ADC conversion*/
		ADC1->CR2|=ADC_CR2_SWSTART;

		/*wait for EOC*/

		while(!(ADC1->SR &ADC_SR_EOC));

		adc_data=ADC1->DR;


	}
}
