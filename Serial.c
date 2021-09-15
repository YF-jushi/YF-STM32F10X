#include "Serial.h"

void Usart_begin(USART_TypeDef* USARTx,uint32_t Rate)
{
	USART_InitTypeDef USART_InitStruct;
	IRQn_Type IRQN_TYPE;
	
	if(USARTx == USART1){
		IRQN_TYPE = USART1_IRQn;
		pinMode(PA9,OUTPUT_AF_PP);
		pinMode(PA10,INPUT);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	if(USARTx == USART2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		IRQN_TYPE = USART2_IRQn;
		pinMode(PA2,OUTPUT_AF_PP);
		pinMode(PA3,INPUT);
	}
	if(USARTx == USART3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		IRQN_TYPE = USART3_IRQn;
		pinMode(PB10,OUTPUT_AF_PP);
		pinMode(PB11,INPUT);
	}
	
	USART_InitStruct.USART_BaudRate = Rate; 
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No; 
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USARTx, &USART_InitStruct);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStruct.NVIC_IRQChannel = IRQN_TYPE;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);	
	
	USART_Cmd(USARTx, ENABLE);
	
}

int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART3, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
		
		return (ch);
}

void USART1_print(char *ch )
{
	uint8_t i;
	for(i = 0;ch[i] != '\0';i++){
	//发送一个字节数据到USART 
	USART_SendData(USART1,ch[i]);
		
	//等待发送数据寄存器为空 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}		
}

void USART2_print(char *ch)
{
	uint8_t i;
	for(i = 0;ch[i] != '\0';i++){
	//发送一个字节数据到USART 
	USART_SendData(USART2,ch[i]);
		
	//等待发送数据寄存器为空 
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}		
}

void USART3_print(char *ch)
{
	uint8_t i;
	for(i = 0;ch[i] != '\0';i++){
	//发送一个字节数据到USART 
	USART_SendData(USART3,ch[i]);
		
	//等待发送数据寄存器为空 
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}		
}

uint8_t isDigital(char letter)
{
	if(letter>='0' && letter<= '9') return 1;
	else return 0;
}

void Clear_String(char *Str)
{
	uint32_t n;
	for(n = 0;*(Str+n) != '\0';n++)
	*(Str+n) = '\0';
}

int pulse(int pl)
{
	int pl_;
	
	uint32_t all = 1;
	
	for(pl_ = pl;pl_>0;pl_--) 
	all *= 10;
	
	return all;
	
}

int toInit(char *Str)
{
	int ti_num,ti_len,result = 0;
	for(ti_num = 0;*(Str+ti_num) != '\0';ti_num++){}

	for(ti_len = ti_num;ti_len > 0;ti_len--){
	 int a = pulse(ti_len-1);
	 result += (*(Str+ti_num-ti_len)-48)*a;
	}
	return result;
}

