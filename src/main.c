#include "stm32f1xx.h"


int main(void)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

	/*Configure PA0 as Output Alternate Push/Pull */
	GPIOA->CRL|=GPIO_CRL_MODE0;
	GPIOA->CRL|=(GPIO_CRL_CNF0_1);
	GPIOA->CRL&=~(GPIO_CRL_CNF0_0);


    /*Configure PA1 as input with Push/Pull*/
    GPIOA->CRL &= ~(GPIO_CRL_MODE4);
    GPIOA->CRL |= GPIO_CRL_CNF4_1;
    GPIOA->CRL &= ~(GPIO_CRL_CNF4_0);

    /*Configure with puLL UP resistor*/

    GPIOA->ODR &= ~(GPIO_ODR_ODR4);

    

	/*Don't remap the pin*/
	AFIO->MAPR&=~AFIO_MAPR_TIM2_REMAP;


	/*Enable clock access to timer2*/
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;

	/*Configure timer2*/
	TIM2->PSC=18;                         // DIVIDE 8MHz by 57 == 140000
	TIM2->ARR=6;                         // 14000 divided by 2 == 70KHz
	TIM2->CCMR1|=TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1;
	TIM2->CCER|=TIM_CCER_CC1E;
	TIM2->CR1|=TIM_CR1_CEN;

    
    


	while(1)
	{
		// for (volatile int i=0;i<100;i++)
		// 	{
		// 		TIM2->CCR1=i;
		// 		for (int j=0;j<10000;j++);
		// 	}

		// for (volatile int i=100;i>0;i--)
		// 	{
		// 		TIM2->CCR1=i;
		// 		for (int j=0;j<1000;j++);
		// 	}
        

            TIM2->CCR1=3;


	}
}
