#ifndef __SERIAL_H
  #define __SERIAL_H
	
	#include "stm32f10x.h"
	#include "YF_STM32F10X.h"
	#include "stdio.h"
	
	void Usart_begin(USART_TypeDef* USARTx,uint32_t Rate);
	int fputc(int ch, FILE *f);
	void USART1_print(char *ch );
	void USART2_print(char *ch );
	void USART3_print(char *ch );
	uint8_t isDigital(char letter);
	void Clear_String(char *Str);
	int pulse(int pl);
	int toInit(char *Str);
#endif

