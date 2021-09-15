#ifndef  __YF_STM32F10X_H
 #define  __YF_STM32F10X_H
 
 #include "stm32f10x.h"
 
 typedef enum {
	 PWM_CLK_N
 }NOTHING;
 
 typedef enum {
	 CH1,
	 CH2,
	 CH3,
	 CH4
 }CHannel;
 
 typedef struct {
	 GPIO_TypeDef* GPIOx;
	 uint16_t GPIO_Pin;
	 uint32_t RCC_APB2Periph_CLK;
	 uint32_t ADC1_value;
	 uint32_t PWM_CLK;
	 TIM_TypeDef* timx;
	 CHannel cha;
 }GPIO_struct;
 
 typedef enum {
	 LOW,
	 HIGH,
	 OUTPUT_OD,
	 OUTPUT_PP,
	 OUTPUT_AF_OD,
	 OUTPUT_AF_PP,
	 INPUT,
	 INPUT_ANALOG
 }STATE;
 
 typedef enum
{
  RISING = 0x08,
  FALLING = 0x0C,  
  CHANGING = 0x10
}Trigger_state;

typedef enum
{
	Pre_emption_0 = (uint32_t)0x700,
	Pre_emption_1 = (uint32_t)0x600,
	Pre_emption_2 = (uint32_t)0x500,
	Pre_emption_3 = (uint32_t)0x400,
	Pre_emption_4 = (uint32_t)0x300
}PRE_EMPTION;

typedef union {
	int i;
	unsigned int i1;
	float j;
	char k;
	unsigned char k1;
	short l;
	unsigned short l1;
}DATA;

 #define DMA_Peripheralbaseaddr (ADC1_BASE+0x4C)
 
 extern USART_TypeDef* usartx;
 
// extern GPIO_struct PA0 ;
// extern GPIO_struct PA1 ;
// extern GPIO_struct PA2 ;
// extern GPIO_struct PA3 ;
// extern GPIO_struct PA4 ;
// extern GPIO_struct PA5 ;
// extern GPIO_struct PA6 ;
// extern GPIO_struct PA7 ;
// extern GPIO_struct PA8 ;
// extern GPIO_struct PA9 ;
// extern GPIO_struct PA10 ;
// extern GPIO_struct PA11 ;
// extern GPIO_struct PA12 ;
// extern GPIO_struct PA13 ;
// extern GPIO_struct PA14 ;
// extern GPIO_struct PA15 ;
// 
// extern GPIO_struct PB0 ;
// extern GPIO_struct PB1 ;
// extern GPIO_struct PB2 ;
// extern GPIO_struct PB3 ;
// extern GPIO_struct PB4 ;
// extern GPIO_struct PB5 ;
// extern GPIO_struct PB6 ;
// extern GPIO_struct PB7 ;
// extern GPIO_struct PB8 ;
// extern GPIO_struct PB9 ;
// extern GPIO_struct PB10 ;
// extern GPIO_struct PB11 ;
// extern GPIO_struct PB12 ;
// extern GPIO_struct PB13 ;
// extern GPIO_struct PB14 ;
// extern GPIO_struct PB15 ;
// 
// extern GPIO_struct PC13 ;

extern GPIO_struct *PA0;
extern GPIO_struct *PA1;
extern GPIO_struct *PA2;
extern GPIO_struct *PA3;
extern GPIO_struct *PA4;
extern GPIO_struct *PA5;
extern GPIO_struct *PA6;
extern GPIO_struct *PA7;
extern GPIO_struct *PA8;
extern GPIO_struct *PA9;
extern GPIO_struct *PA10;
extern GPIO_struct *PA11;
extern GPIO_struct *PA12;
extern GPIO_struct *PA13;
extern GPIO_struct *PA14;
extern GPIO_struct *PA15;

extern GPIO_struct *PB0;
extern GPIO_struct *PB1;
extern GPIO_struct *PB3;
extern GPIO_struct *PB4;
extern GPIO_struct *PB5;
extern GPIO_struct *PB6;
extern GPIO_struct *PB7;
extern GPIO_struct *PB8;
extern GPIO_struct *PB9;
extern GPIO_struct *PB10;
extern GPIO_struct *PB11;
extern GPIO_struct *PB12;
extern GPIO_struct *PB13;
extern GPIO_struct *PB14;
extern GPIO_struct *PB15;
	
extern GPIO_struct *PC13;
 
extern uint8_t Num ;
 
extern uint16_t value_adc[10];

extern uint8_t f;

#define toggle(a,b) {a->ODR ^= b;}
#define TOG          toggle(GPIOA,GPIO_Pin_1)
#define TOGG         toggle(GPIOA,GPIO_Pin_2)

void pinMode(GPIO_struct *Pin,STATE state);
void digitalWrite(GPIO_struct *Pin,STATE state);
uint8_t digitalRead(GPIO_struct *Pin);
float analogRead(GPIO_struct *Pin);
void analogInit(ADC_TypeDef* ADCx,GPIO_struct* Pin);
void Usart_begin(USART_TypeDef* USARTx,uint32_t Rate);
void USART1_print(unsigned char *ch);
void USART2_print(char *ch);
void USART3_print(char *ch);
void analogWrite_Init(GPIO_struct *Pin,uint32_t frequency,float duty_cycle);
void analogWrite(GPIO_struct *Pin,float duty_cycle);
uint8_t find_pin(uint16_t pin);
uint8_t isDigital(char letter);
void attachInterrupt(GPIO_struct *Pin,Trigger_state STATE,PRE_EMPTION pre,uint8_t num);
#endif
