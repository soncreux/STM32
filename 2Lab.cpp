#include "stm32f4xx.h"

int data;
int data1;
enum PORT {A, B, C, D, E, F, G, H, I}; // Перечисление доступных портов


void SetAltFunc(GPIO_TypeDef* Port, int Channel, int AF)
{
 Port->MODER &= ~(3<<(2*Channel)); // Сброс режима
 Port->MODER |= 2<<(2*Channel); // Установка альт. Режима
 if(Channel<8) // Выбор регистра зависит от номера контакта
 {
 Port->AFR[0] &= ~(15<<4*Channel); // Сброс альт. функции
 Port->AFR[0] |= AF<<(4*Channel); // Установка альт. функции
 }
 else
 {
 Port->AFR[1] &= ~(15<<4*(Channel-8)); // Сброс альт. функции
 Port->AFR[1] |= AF<<(4*(Channel-8)); // Установка альт. функции
 }
}

extern "C" void EXTI4_IRQHandler() // Название функции для EXTI4
{
 EXTI->PR = 1<<4; // Снять бит активности прерывания (прерывание 4 обработано)
 if (GPIOE->IDR & (1<<4)) // Прочесть значение входящего сигнала в PE4
 {
 
 }
 else
 {
 
 }
}


void SetEXTI(PORT Port, int Channel, bool Rise, bool Fall)
{
 SYSCFG->EXTICR[Channel/4] &= ~(15<<(4*(Channel%4))); // Сбросить порт
 SYSCFG->EXTICR[Channel/4] |= Port<<(4*(Channel%4)); // Выбрать порт
 
 EXTI->IMR |= 1<<Channel; // Прерывание выбрано
 
 if(Rise) EXTI->RTSR |= 1<<Channel; // Ловить повышение напряжения
 else EXTI->RTSR &= ~(1<<Channel); // Не ловить повышение напряжения
 
 if(Fall) EXTI->FTSR |= 1<<Channel; // Ловить падение напряжения
 else EXTI->FTSR &= ~(1<<Channel); // Не ловить падение напряжения
}

int main()
{

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER &= ~GPIO_MODER_MODER0;
  GPIOC->MODER |= GPIO_MODER_MODER0;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SQR3 = 10;
  GPIOC->MODER &= ~GPIO_MODER_MODER1;
  GPIOC->MODER |= GPIO_MODER_MODER1;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SQR3 = 11;

  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Порт D задействован
SetAltFunc(GPIOD, 12, 2); // Установка альт. режима AF2 для TIM4 CH1 (PD12)
SetAltFunc(GPIOD, 13, 2); // Установка альт. режима AF2 для TIM4 CH2 (PD13)
SetAltFunc(GPIOD, 14, 2); // Установка альт. режима AF2 для TIM4 CH3 (PD14)
TIM4->ARR = 1000 - 1; // Диапазон сравнения 1000
TIM4->PSC = (84000000/1000/2000)-1; // Задан делитель
TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // Режим CCR1
TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE; // Режим CCR2
TIM4->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; // Режим CCR3
TIM4->CCER = TIM_CCER_CC1E; // Выход PD12 активен
TIM4->CCER |= TIM_CCER_CC2E; // Выход PD13 активен
TIM4->CCER |= TIM_CCER_CC3E; // Выход PD14 активен
TIM4->CR1 = TIM_CR1_CEN; // Таймер запущен


while(1) { 
  ADC1->SQR3 = 10;
  for(int a=0;a<20;a++) {}
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while(!(ADC1->SR & ADC_SR_EOC)) { }
  data = ADC1->DR;
  ADC1->SQR3 = 11;
  for(int a=0;a<20;a++) {}
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while(!(ADC1->SR & ADC_SR_EOC)) { }
  data1 = ADC1->DR;
TIM4->CCR1 = data/4; // Скважность PD12 = 100*500/1000 = 50% (RED)
TIM4->CCR2 = 500; // Скважность PD13 = 100*500/1000 = 25% (GREEN)
TIM4->CCR3 = data1/4; // Скважность PD14 = 100*500/1000 = 75% (BLUE)

}
}

/* Задание 2
 RCC->APB1ENR |= RCC_APB1ENR_DACEN;
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
 GPIOA->MODER &= ~GPIO_MODER_MODER4;
 DAC->CR |= DAC_CR_EN1;
 
 while(1) {
   
   for(int a=0;a<4096;a++) {
 DAC->DHR12R1 = a;
   }
    for(int i=4096;i!=0;i--) {
 DAC->DHR12R1 = i;
 }
 
 }
*/

 /*1 task
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER &= ~GPIO_MODER_MODER3;
  GPIOC->MODER |= GPIO_MODER_MODER3;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SQR3 = 13;
  while(1)
  {
 for(int a=0; a<20; a++) { }
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while(!(ADC1->SR & ADC_SR_EOC)) { }
   data = ADC1->DR;
  }*/