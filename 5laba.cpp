#include "stm32f4xx.h"
const int Address = 0x68; // Адрес устройства датчик ориентации ICM-20948 для получения показаний
const int Addr = 0x39; // Адрес устройства APDS-9960 для определения отраженного цвета

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




int AnalogRead(int N) // Функция принимает номер канала для преобразования
{
 ADC1->SQR3 = N; // Выбран полученный из аргумента канал
 for(int a=0; a<100; a++) { asm("NOP"); } // Ожидать больше 100 тактов
 ADC1->CR2 |= ADC_CR2_SWSTART; // Начать преобразование
 while(!(ADC1->SR & ADC_SR_EOC)) { asm("NOP"); } // Ждать установки бита конца операции
 return ADC1->DR; // Вернуть результат преобразования
}




void MOTOR_INIT() {
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; // Порт H задействован
  GPIOH->MODER &= ~GPIO_MODER_MODER11; // Сброс режима для PH11
  GPIOH->MODER |= GPIO_MODER_MODER11_0; // Установка режима на выход PH11
  GPIOH->BSRRL = 1<<11; // Установить значение HIGH (3.3V) для PH11
  
}




void PI5_6_INIT() {
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; // Порт I задействован
  SetAltFunc(GPIOI, 5, 3); // Установка альт. режима AF3 для TIM8_CH1(PI5)
  SetAltFunc(GPIOI, 6, 3); // Установка альт. режима AF3 для TIM8_CH2(PI6)

}



void TIM8_INIT()  {
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Таймер 8 задействован (APB2 x2 = 168МГц)
TIM8->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1; // Генерация пульса сравнения
TIM8->ARR = 100 - 1; // Диапазон сравнения 100
TIM8->PSC = (168000000/100/20000)-1; // Задан делитель на 20кГц
TIM8->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // Режим PWM 1 для CH1
TIM8->CCMR1|= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE; // Режим PWM 1 для CH2
TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // Выходы CH1 и CH2 активны
TIM8->BDTR = TIM_BDTR_MOE; // Разрешить работу активных выходов
TIM8->CR1 = TIM_CR1_CEN; // Таймер запущен

}



void DRIVER_POWER() {
  
 while(1)
{
 TIM8->CCR2 = 0; TIM8->CCR1 = 10; // Установить 10% мощности
 for(int a=0; a<20000000; a++) { } // Ждем примерно 1 секунду
 TIM8->CCR1 = 0; TIM8->CCR2 = 50; // Установить 50% мощности и поменять полярность
 for(int a=0; a<20000000; a++) { } // Ждем примерно 1 секунду
} 
  
}




void VOLTAGE_INIT() {
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; // Порт H задействован
GPIOH->MODER &= ~GPIO_MODER_MODER12; // Сброс режима для PH12
GPIOH->MODER |= GPIO_MODER_MODER12_0; // Установка режима на выход PH12
GPIOH->BSRRL = 1<<12; // Установить значение HIGH (3.3V) для PH12
  
}



void PH6_INIT() {
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; // Порт H задействован
SetAltFunc(GPIOH, 6, 9); // Установка альт. режима AF9 для TIM12_CH1(PH6)

}




void TIM2_INIT() {
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN; // Таймер 12 задействован (APB1 x2 = 84МГц)
  TIM12->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1; // Генерация пульса сравнения
  TIM12->ARR = 100 - 1; // Диапазон сравнения 100
  TIM12->PSC = (84000000/100/40000)-1; // Задан делитель на 40кГц
  TIM12->CCMR1= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // Режим PWM 1 для CH1
  TIM12->CCER = TIM_CCER_CC1E; // Выход CH1 активен
  TIM12->CR1 = TIM_CR1_CEN; // Таймер запущен
  TIM12->CCR1 = 35; // Установить 35% мощности
}



void ADC_INIT() {
  
  
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // АЦП задействован
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Порт A задействован
GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER6); // Сброс режима для PA1 и PA6
GPIOA->MODER |= GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1; // Аналоговый вход PA1
GPIOA->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER6_1; // Аналоговый вход PA6
ADC1->CR2 = ADC_CR2_ADON; // АЦП активен

  
}


void Napr_Read(){

int mV = AnalogRead(3)*11966/4096; // Значение напряжения на выходе в милливольтах
int mA = AnalogRead(6)*3300/4096; // Значение тока на выходе в миллиамперах

}


void PH13_INIT() {

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; // Порт H задействован
GPIOH->MODER &= ~GPIO_MODER_MODER13; // Сброс режима для PH13
GPIOH->MODER |= GPIO_MODER_MODER13_0; // Установка режима на выход PH13
GPIOH->BSRRL = 1<<13; // Установить значение HIGH (3.3V) для PH13
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // АЦП задействован
ADC1->CR2 = ADC_CR2_ADON; // АЦП активен
}



void CURRENT_SENSOR() {

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Порт A задействован
GPIOA->MODER &= ~GPIO_MODER_MODER7; // Сброс режима для PA7
GPIOA->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER7_1; // Аналоговый вход PA7
int mA = (AnalogRead(7)-2048)*6250/2048; // 6250=(3300/2)*(1000/264) // 264mV=1A
}



void LIGHT_SENSOR() {

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Порт B задействован
GPIOB->MODER &= ~GPIO_MODER_MODER1; // Сброс режима для PB1
GPIOB->MODER |= GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1; // Аналоговый вход PB1
int light = AnalogRead(9); // Интенсивность светового потока в условных единицах
}



void OBSTACLE_SENSOR() {

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Порт B задействован
GPIOB->MODER &= ~GPIO_MODER_MODER0; // Сброс режима для PB0
GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1; // Аналоговый вход PB0
int light = AnalogRead(8); // Интенсивность поглощения ИК света объектом
}


void I2C3_INIT() {

int timer=1000000; while(timer--) {} // Подождем готовности устройства к работе
char reg[2];
reg[0]=0x06; reg[1]=0x80; // Сброс устройства
I2C_Write(AddressIMU, reg, 2); 
timer=1000000; while(timer--) {} // Подождем готовности устройства к работе
reg[0]=0x06; reg[1]=0x01; // Включить устройство и выбрать тактирование
I2C_Write(AddressIMU, reg, 2);
timer=1000000; while(timer--) {} // Подождем готовности устройства к работе

}




void accelerometer_init() {

char data[6];
data[0]=0x2D; // Выбрать акселерометр
I2C_Write(Address, data, 1);
I2C_Read(Address, data, 6);
short AccX=(((short)data[0])<<8) + data[1]; // Показания акселерометра ось X
short AccY=(((short)data[2])<<8) + data[3]; // Показания акселерометра ось Y
short AccZ=(((short)data[4])<<8) + data[5]; // Показания акселерометра ось Z

}




void GYRO_INIT() {

data[0]=0x33; // Выбрать гироскоп
I2C_Write(Address, data, 1);
I2C_Read(Address, data, 6);
short GyrX=(((short)data[0])<<8) + data[1]; // Показания гироскопа ось X
short GyrY=(((short)data[2])<<8) + data[3]; // Показания гироскопа ось Y
short GyrZ=(((short)data[4])<<8) + data[5]; // Показания гироскопа ось Z

}