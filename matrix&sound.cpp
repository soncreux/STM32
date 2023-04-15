#include "stm32f4xx.h"
char img1[8] = {0b00000000,
0b00011000,
0b00001100,
0b11111110,
0b00001100,
0b00011000,
0b00000000,
0b00000000}; 
char img2[8] = {

0b11111110,
}; 
int key;
static int last;
char* MatrixData = 0;
int N,X,Y;
int value;
enum PORT {A, B, C, D, E, F, G, H, I}; 
int Encoder_Count=0; 
int AnalogRead(int N) 
{
 ADC1->SQR3 = N; 
 for(int a=0; a<100; a++) { asm("NOP"); } 
 ADC1->CR2 |= ADC_CR2_SWSTART; 
 while(!(ADC1->SR & ADC_SR_EOC)) { asm("NOP"); } 
 return ADC1->DR; 
}
void SetAltFunc(GPIO_TypeDef* Port, int Channel, int AF)
{
 Port->MODER &= ~(3<<(2*Channel)); 
 Port->MODER |= 2<<(2*Channel); 
 if(Channel<8) 
 {
 Port->AFR[0] &= ~(15<<4*Channel);
 Port->AFR[0] |= AF<<(4*Channel);
 }
 else
 {
 Port->AFR[1] &= ~(15<<4*(Channel-8)); 
 Port->AFR[1] |= AF<<(4*(Channel-8));
 }
}
extern "C" void EXTI15_10_IRQHandler() 
{
 EXTI->PR = 1<<12 | 1<<13; 
 last=0; 
 value=(GPIOB->IDR>>12) & 3; 
 switch(last) 
 {
 case 0: if(value==1) Encoder_Count++; else Encoder_Count--; break;
 case 1: if(value==3) Encoder_Count++; else Encoder_Count--; break;
 case 3: if(value==2) Encoder_Count++; else Encoder_Count--; break;
 case 2: if(value==0) Encoder_Count++; else Encoder_Count--; break;
 }
 last=value; 
}
void SetEXTI(PORT Port, int Channel, bool Rise, bool Fall)
{
 SYSCFG->EXTICR[Channel/4] &= ~(15<<(4*(Channel%4))); 
 SYSCFG->EXTICR[Channel/4] |= Port<<(4*(Channel%4)); 
 
 EXTI->IMR |= 1<<Channel; 
 
 if(Rise) EXTI->RTSR |= 1<<Channel; 
 else EXTI->RTSR &= ~(1<<Channel); 
 
 if(Fall) EXTI->FTSR |= 1<<Channel; 
 else EXTI->FTSR &= ~(1<<Channel); 
}
extern "C" void TIM2_IRQHandler() 
{
 TIM2->SR &= ~TIM_SR_UIF; 
 static int next = 0; 
 if(MatrixData) GPIOG->ODR = (1<<(next+8)) | MatrixData[next]; 
 else GPIOG->ODR = 0; 
 if(next<7) next++; 
 else next = 0; 
}
int main()
{
 RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; 
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; 
SetAltFunc(GPIOI, 0, 2); 
TIM5->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1; 
TIM5->ARR = 100 - 1; 
TIM5->PSC = (84000000/100/2000)-1; 
TIM5->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 |
TIM_CCMR2_OC4PE; 
TIM5->CCER = TIM_CCER_CC4E; 
TIM5->CR1 = TIM_CR1_CEN; 
TIM5->CCR4 = 1; 
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
GPIOG->MODER = 0x55555555; 
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
TIM2->ARR = 1000-1; 
TIM2->PSC = (84000000/1000/2000)-1; 
TIM2->DIER = TIM_DIER_UIE; 
NVIC_SetPriority(TIM2_IRQn, 0); 
NVIC_EnableIRQ(TIM2_IRQn); 
TIM2->CR1 = TIM_CR1_CEN; 
 RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 
GPIOC->MODER &= ~(GPIO_MODER_MODER0 | 
GPIO_MODER_MODER1); 
GPIOC->MODER |= GPIO_MODER_MODER0_0 | 
GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_0 | 
GPIO_MODER_MODER1_1; 
ADC1->CR2 = ADC_CR2_ADON; 
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
 GPIOB->MODER &= ~(GPIO_MODER_MODER12 | 
GPIO_MODER_MODER13); 
 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; 
 SetEXTI(PORT::B, 12, true, true); 
 SetEXTI(PORT::B, 13, true, true); 
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; 
GPIOE->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 
| GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
NVIC_SetPriority(EXTI15_10_IRQn, 0); 
NVIC_EnableIRQ(EXTI15_10_IRQn); 
while(1)
{
 key = GPIOE->IDR & 15;
 key = key * 100;
TIM5->PSC = (84000000/100/key)-1; 
X = AnalogRead(10); 
Y = AnalogRead(11); 
if (X > 3000)
 MatrixData=img1; 
else if (X < 1000)
 MatrixData=img2; 
}
}
