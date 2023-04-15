#include "stm32f4xx.h"
#include <stdio.h>
#include <string>
#include <iostream>
const int DATA_Size=128; // Размер принимающего буфера
char DATA_Buffer[DATA_Size]; // Принимающий буфер
int DATA_Head=0; // Позиция для следующий записи
int DATA_Tail=0; // Позиция для следующего чтения
const int I2C_Address = 0x20;
char reg[2]={0x06, 0b11111010}; // Зададим регистр IODIR0 для исходящего P00 и P02








void I2C_Read(int Address, char* Data, int Size) // Функция приёма
{
 if(Size>1) I2C3->CR1 |= I2C_CR1_ACK; // Прём больше 1 байт то нужен ACK
 else I2C3->CR1 &= ~I2C_CR1_ACK; // Прём меньше 2 байт то ACK не нужен
 I2C3->CR1 |= I2C_CR1_START; // Занимаем линию связи для данных
 while (!(I2C3->SR1 & I2C_SR1_SB)) { } // Ждём занятия линии
 I2C3->DR = (Address<<1) | I2C_OAR1_ADD0; // Шлём адрес для приёма данных
 while (!(I2C3->SR1 & I2C_SR1_ADDR)) { } // Ждём успешной связи с устройством
 I2C3->SR2; // Читаем SR2 для его отчистки
 while (Size--) // Цикл приёма байт
 {
 while (!(I2C3->SR1 & I2C_SR1_RXNE)) { } // Ждем наличия байта для чтения
 if(Size == 1) I2C3->CR1 &= ~I2C_CR1_ACK; // Если остался 1 байт, то ACK убрать
 *Data++ = I2C3->DR; // Читаем пришедший байт
 }
 I2C3->CR1 |= I2C_CR1_STOP; // Освобождаем линию связи 
 while (I2C3->CR1 & I2C_CR1_STOP) { } // Ждём освобождения линии
}





void I2C_Write(int Address, char* Data, int Size) // Функция передачи
{
 I2C3->CR1 |= I2C_CR1_START; // Занимаем линию связи для данных
 while (!(I2C3->SR1 & I2C_SR1_SB)) { } // Ждём занятия линии
 I2C3->DR = (Address<<1) & ~I2C_OAR1_ADD0; // Шлём адрес для передачи данных
 while (!(I2C3->SR1 & I2C_SR1_ADDR)) { } // Ждём успешной связи с устройством
 I2C3->SR2; // Читаем SR2 для его отчистки
 while (Size--) // Цикл передачи байт
 {
 while (!(I2C3->SR1 & I2C_SR1_TXE)) { } // Ждем готовности передать байт
 I2C3->DR = *Data++; // Передаём байт
 }
 while (!(I2C3->SR1 & I2C_SR1_BTF)) { } // Ждём окончания передачи данных
 I2C3->CR1 |= I2C_CR1_STOP; // Освобождаем линию связи
 while (I2C3->CR1 & I2C_CR1_STOP) { } // Ждём освобождения линии
}




//Функции приема и передачи данных

int UART2_Recv(char* Data, int Size, bool WaitAll=false) // Функция приёма байт
{
 int size; // Размер принятых данных
 for(size=0; size<Size; size++) // Цикл приёма данных с учётом допустимого размера
 {
 if(WaitAll) while(DATA_Tail==DATA_Head) { } // Ждать прихода данных
 else if(DATA_Tail==DATA_Head) break; // Данных больше нет, выходим из цикла
 Data[size]=DATA_Buffer[DATA_Tail++]; // Читаем байт и задаём след. позицию
 if(DATA_Tail>=DATA_Size) DATA_Tail=0; // Превышение размера, идём сначала
 }
 return size; // Вернуть размер полученных данных
}


int UART2_GetString(char* Data, int Size) // Функция приёма строки
{
 int size; // Размер принятых данных
 for(size=0; size<(Size-1); size++) // Цикл приёма данных с учётом допустимого размера
 {
 while(DATA_Tail==DATA_Head); // Ждать прихода данных
 Data[size]=DATA_Buffer[DATA_Tail++]; // Читаем байт и задаём след. позицию
 if(DATA_Tail>=DATA_Size) DATA_Tail=0; // Превышение размера, идём сначала
 if(Data[size]=='\n') { size++; break; } // Обнаружить новую строку
 }
 Data[size]='\0'; // Установить конец строки
 return size; // Вернуть размер полученных данных
}


void UART2_Send(char* Data, int Size) // Функция передачи байт
{
 while(Size--) // Цикл передачи данных
 {
 while(!(USART2->SR & USART_SR_TXE)) { } // Ждать возможности передавать
 USART2->DR=*Data++; // Передать данные и задать след. позицию
 }
 while(!(USART2->SR & USART_SR_TC)) { } // Ждать завершения передачи
}





extern "C" void USART2_IRQHandler() // Функция обработки прерывания
{
 int test=USART2->SR; // Читаем SR чтобы обработать прерывание
 DATA_Buffer[DATA_Head++]=USART2->DR; // Читаем данные и задаём след. позицию
 if(DATA_Head>=DATA_Size) DATA_Head=0; // Начинаем с начала если превысили размер
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


void UART_SET() {
  
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // UART 2 задействован (APB1=42МГц)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Порт A задействован
  SetAltFunc(GPIOA, 2, 7); // Установка альт. режима AF7 для TX(PA2) (см. лаб. 2)
  SetAltFunc(GPIOA, 3, 7); // Установка альт. режима AF7 для RX(PA3) (см. лаб. 2)
 
}


void FREQ_SET() {
  
 USART2->BRR = (42000000)/115200; // Делитель установлен на 115200
 
}


void Priemoperedatchik_Activate() {
  
  USART2->CR1 = USART_CR1_RE | USART_CR1_TE; // Приём и передача активны
  USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_UE; // Прерывание на приём и запуск устройства
  
}


void Interrupt_Set() {
  
  NVIC_SetPriority(USART2_IRQn, 0); // Высший приоритет прерывания
  NVIC_EnableIRQ(USART2_IRQn); // Прерывание активно

}


void ECHO_UART() {
  
    while(1)
  {
   char test[32];
   //char pidor[32]="pidor";
   //std::string nazimov = "nazimov";// Буфер временных данных
   int size=UART2_Recv(test, sizeof(test)); // Получаем входящие данные
   if(size) UART2_Send(test, size); // Если что то пришло, то отправим их обратно
   /*if (size) {
      UART2_Send(test, size);
     if (test==nazimov){
       //UART2_Send(test, size);*/
     }
  }

   
    
 
 



void I2C_SET() {
  

    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN; // I2C 3 задействован
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; // Порт H задействован
  GPIOH->OTYPER |= GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8; // Открытый сток для PH7 и PH8
  GPIOH->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR7_1; // Мах. скорость PH7 
  GPIOH->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1; // Мах. скорость PH8
  GPIOH->PUPDR |= GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR8_0; // Подтяжка 3.3V для PH7 и PH8
  SetAltFunc(GPIOH, 7, 4); // Установка альт. режима AF4 для SCL(PH7) (см. лаб. 2)
  SetAltFunc(GPIOH, 8, 4); // Установка альт. режима AF4 для SDA(PH8) (см. лаб. 2)

}


void I2C_FREQ() {
  
  I2C3->CR2 = (I2C_CR2_FREQ & 0x2A);
  I2C3->CCR = I2C_CCR_FS | (I2C_CCR_CCR & 0x006C);
  I2C3->TRISE = (I2C_TRISE_TRISE & 0x14);
  
}



void I2C_START() {
  
  I2C3->CR1 = I2C_CR1_PE;
  while(I2C3->SR2 & I2C_SR2_BUSY) { }
}


void P0_SET() {

I2C_Write(I2C_Address, reg, 2); // Отправим данные на устройство

}



void P0_EXIT() {
  
  reg[0]=0x00; // Выберем регистр GP0 для установки значений контактов
while(1)
{
 reg[1]=0b00000001; // Высокий сигнал (3.3V) на контакте P00, низкий сигнал (0V) на P02
 I2C_Write(I2C_Address, reg, 2); // Отправим данные на устройство
 for(int a=0; a<20000000; a++); // Ждем примерно 1 секунду
 reg[1]=0b00000100; // Высокий сигнал (3.3V) на контакте P02, низкий сигнал (0V) на P00
 I2C_Write(I2C_Address, reg, 2); // Отправим данные на устройство
 for(int a=0; a<20000000; a++); // Ждем примерно 1 секунду
}
  
}


void P0_READ() {
  
  char get=0x00; // Выберем регистр GP0 для получения значений контактов
I2C_Write(I2C_Address, &get, 1); // Отправим данные на устройство
while(1)
{
 char data; // Текущие значения контактов
 I2C_Read(I2C_Address, &data, 1); // Получим данные из устройства
}
  
}


void MAKE_ALL() {
  
  UART_SET();
  FREQ_SET();
  Priemoperedatchik_Activate();
  Interrupt_Set();
  ECHO_UART();
  I2C_SET();
  I2C_FREQ();
  I2C_START();
  P0_SET();
  P0_EXIT();
  P0_READ();
  
}


int main() {
  
  
  
  MAKE_ALL();
 for(int i=0; i<10; i++)
 {
 printf("Value: %d\n", i); // Вывод на экран значения переменной «i»
 for(int a=0; a<10000000; a++); // Ждем примерно 1/2 секунды
 }
 
}
