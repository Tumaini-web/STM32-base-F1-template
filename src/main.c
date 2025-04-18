#include "stm32f3xx.h"

uint32_t charge_time = 0;
uint32_t discharge_time = 0;

void init_gpio_pa1_pa5(void) {
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA1: Analog input (capacitor voltage)
    GPIOA->MODER |= GPIO_MODER_MODER1;

    // PA5: Output push-pull (charge control)
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
}

void init_dac1_for_comp_ref(uint16_t val12bit) {
    RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;

    // Disable DAC before configuration
    DAC1->CR &= ~DAC_CR_EN1;

    // Optional: disable trigger (default), enable buffer (optional)
    DAC1->CR &= ~DAC_CR_TEN1;         // Disable trigger
    DAC1->CR &= ~DAC_CR_BOFF1;        // Enable output buffer (good for most loads)

    // Set DAC value BEFORE enabling
    DAC1->DHR12R1 = val12bit;         // 12-bit right-aligned value (0–4095)

    // Enable DAC
    DAC1->CR |= DAC_CR_EN1;
}


void init_comp1_for_dac_ref(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Disable COMP1
    COMP1->CSR &= ~COMP1_CSR_COMP1EN;

    // Use PA1 as non-inverting input (default)
   // COMP1->CSR &= ~COMP1_CSR_COMP1INSEL;

    // Use DAC1_CH1 output as inverting input
    COMP1->CSR &= ~COMP1_CSR_COMP1INSEL;
    COMP1->CSR |= COMP1_CSR_COMP1INSEL_2;

    // Enable COMP1
    COMP1->CSR |= COMP1_CSR_COMP1EN;
}

void init_tim2_us(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 8 - 1;         // 8 MHz / 8 = 1 MHz -> 1 µs ticks
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t time_until_comp_state(uint8_t target_state) {
    uint32_t t_start = TIM2->CNT;
    while (((COMP1->CSR & COMP1_CSR_COMP1OUT) ? 1 : 0) != target_state);
    uint32_t t_end = TIM2->CNT;
    return (t_end >= t_start) ? (t_end - t_start) : (0xFFFFFFFF - t_start + t_end + 1);
}

int main(void) {
    init_gpio_pa1_pa5();
    init_dac1_for_comp_ref(2483); // 2483/4095 * 3.3V ≈ 2.0V
    init_comp1_for_dac_ref();
    init_tim2_us();

    // uint32_t charge_time = 0;
    // uint32_t discharge_time = 0;

    while (1) {
        // === CHARGE ===
        init_dac1_for_comp_ref(2483); // 2483/4095 * 3.3V ≈ 2.0V
        GPIOA->ODR |= GPIO_ODR_5; // Set PA5 high
        charge_time = time_until_comp_state(1); // Wait until Vcap > 2V

        // === DISCHARGE ===
        init_dac1_for_comp_ref(1241); // 1V threshold
        GPIOA->ODR &= ~GPIO_ODR_5; // Set PA5 low
        discharge_time = time_until_comp_state(0); // Wait until Vcap < 2V

        // Now you have charge_time and discharge_time in µs
        // You could log, toggle an LED, etc.
    }
}
