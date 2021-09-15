#include "YF_STM32F10X.h"
#include "stdio.h"
#include "stm32f10x_it.h"

GPIO_struct pa0 = {GPIOA,GPIO_Pin_0,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM2,TIM2,CH1};
GPIO_struct pa1 = {GPIOA,GPIO_Pin_1,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM2,TIM2,CH2};
GPIO_struct pa2 = {GPIOA,GPIO_Pin_2,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM2,TIM2,CH3};
GPIO_struct pa3 = {GPIOA,GPIO_Pin_3,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM2,TIM2,CH4};
GPIO_struct pa4 = {GPIOA,GPIO_Pin_4,RCC_APB2Periph_GPIOA,5,PWM_CLK_N,NULL};
GPIO_struct pa5 = {GPIOA,GPIO_Pin_5,RCC_APB2Periph_GPIOA,0,PWM_CLK_N,NULL};
GPIO_struct pa6 = {GPIOA,GPIO_Pin_6,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM3,TIM3,CH1};
GPIO_struct pa7 = {GPIOA,GPIO_Pin_7,RCC_APB2Periph_GPIOA,0,RCC_APB1Periph_TIM3,TIM3,CH2};
GPIO_struct pa8 = {GPIOA,GPIO_Pin_8,RCC_APB2Periph_GPIOA,0,RCC_APB2Periph_TIM1,TIM1,CH1};
GPIO_struct pa9 = {GPIOA,GPIO_Pin_9,RCC_APB2Periph_GPIOA,0,RCC_APB2Periph_TIM1,TIM1,CH2};
GPIO_struct pa10 = {GPIOA,GPIO_Pin_10,RCC_APB2Periph_GPIOA,0,RCC_APB2Periph_TIM1,TIM1,CH3};
GPIO_struct pa11 = {GPIOA,GPIO_Pin_11,RCC_APB2Periph_GPIOA,0,RCC_APB2Periph_TIM1,TIM1,CH4};
GPIO_struct pa12 = {GPIOA,GPIO_Pin_12,RCC_APB2Periph_GPIOA,0,PWM_CLK_N,NULL};
GPIO_struct pa13 = {GPIOA,GPIO_Pin_13,RCC_APB2Periph_GPIOA,0,PWM_CLK_N,NULL};
GPIO_struct pa14 = {GPIOA,GPIO_Pin_14,RCC_APB2Periph_GPIOA,0,PWM_CLK_N,NULL};
GPIO_struct pa15 = {GPIOA,GPIO_Pin_15,RCC_APB2Periph_GPIOA,0,PWM_CLK_N,NULL};

GPIO_struct pb0 = {GPIOB,GPIO_Pin_0,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM3,TIM3,CH3};
GPIO_struct pb1 = {GPIOB,GPIO_Pin_1,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM3,TIM3,CH4};
GPIO_struct pb3 = {GPIOB,GPIO_Pin_3,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb4 = {GPIOB,GPIO_Pin_4,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb5 = {GPIOB,GPIO_Pin_5,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb6 = {GPIOB,GPIO_Pin_6,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM4,TIM4,CH1};
GPIO_struct pb7 = {GPIOB,GPIO_Pin_7,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM4,TIM4,CH2};
GPIO_struct pb8 = {GPIOB,GPIO_Pin_8,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM4,TIM4,CH3};
GPIO_struct pb9 = {GPIOB,GPIO_Pin_9,RCC_APB2Periph_GPIOB,0,RCC_APB1Periph_TIM4,TIM4,CH4};
GPIO_struct pb10 = {GPIOB,GPIO_Pin_10,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb11 = {GPIOB,GPIO_Pin_11,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb12 = {GPIOB,GPIO_Pin_12,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb13 = {GPIOB,GPIO_Pin_13,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb14 = {GPIOB,GPIO_Pin_14,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
GPIO_struct pb15 = {GPIOB,GPIO_Pin_15,RCC_APB2Periph_GPIOB,0,PWM_CLK_N,NULL};
	
GPIO_struct pc13 = {GPIOC,GPIO_Pin_13,RCC_APB2Periph_GPIOC,0,PWM_CLK_N};

GPIO_struct *PA0 = &pa0;
GPIO_struct *PA1 = &pa1;
GPIO_struct *PA2 = &pa2;
GPIO_struct *PA3 = &pa3;
GPIO_struct *PA4 = &pa4;
GPIO_struct *PA5 = &pa5;
GPIO_struct *PA6 = &pa6;
GPIO_struct *PA7 = &pa7;
GPIO_struct *PA8 = &pa8;
GPIO_struct *PA9 = &pa9;
GPIO_struct *PA10 = &pa10;
GPIO_struct *PA11 = &pa11;
GPIO_struct *PA12 = &pa12;
GPIO_struct *PA13 = &pa13;
GPIO_struct *PA14 = &pa14;
GPIO_struct *PA15 = &pa15;

GPIO_struct *PB0 = &pb0;
GPIO_struct *PB1 = &pb1;
GPIO_struct *PB3 = &pb3;
GPIO_struct *PB4 = &pb4;
GPIO_struct *PB5 = &pb5;
GPIO_struct *PB6 = &pb6;
GPIO_struct *PB7 = &pb7;
GPIO_struct *PB8 = &pb8;
GPIO_struct *PB9 = &pb9;
GPIO_struct *PB10 = &pb10;
GPIO_struct *PB11 = &pb11;
GPIO_struct *PB12 = &pb12;
GPIO_struct *PB13 = &pb13;
GPIO_struct *PB14 = &pb14;
GPIO_struct *PB15 = &pb15;
	
GPIO_struct *PC13 = &pc13;

uint16_t Value = 0;

uint8_t Num = 1;

uint16_t value_adc[10]={0};

uint8_t f = 0;

void pinMode(GPIO_struct *Pin,STATE state)
{
	RCC_APB2PeriphClockCmd(Pin->RCC_APB2Periph_CLK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = Pin->GPIO_Pin;
	
	switch(state){
		case LOW:{}
		case HIGH:{}
			
		case OUTPUT_OD:{
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
			break;
		}
	  case OUTPUT_PP:{
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;

			break;
		}
		case OUTPUT_AF_OD:{
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			break;
		}
	  case OUTPUT_AF_PP:{
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			break;
		}
	  case INPUT:{
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			break;
		}
	  case INPUT_ANALOG:{
			GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_AIN;
			analogInit(ADC1,Pin);
			break;
		}
}
GPIO_Init(Pin->GPIOx, &GPIO_InitStruct);
}

void digitalWrite(GPIO_struct *Pin,STATE state)
{
	if(state) GPIO_SetBits(Pin->GPIOx, Pin->GPIO_Pin);
	else GPIO_ResetBits(Pin->GPIOx, Pin->GPIO_Pin);
}

uint8_t digitalRead(GPIO_struct *Pin)
{
	uint8_t state;
	state = GPIO_ReadInputDataBit(Pin->GPIOx, Pin->GPIO_Pin);
	return state;
}

float analogRead(GPIO_struct *Pin)
{
	float value = value_adc[Pin->ADC1_value-1]/4096.0*3.3 ;
//	float value = value_adc[Pin.ADC1_value-1];
	return value;
}

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

void USART1_print(unsigned char *ch )
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

TIM_OCInitTypeDef TIM_OCInitStruct;

TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

uint32_t period;

void analogWrite_Init(GPIO_struct *Pin,uint32_t frequency,float duty_cycle)
{
	if((Pin->PWM_CLK != PWM_CLK_N)&&(Pin->timx != NULL)){
	RCC_APB1PeriphClockCmd(Pin->PWM_CLK, ENABLE);
	
	//设置定时器时钟与数字滤波器之间的分频比例（与输入捕获相关）
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	period = 1000000/frequency;
	
	TIM_TimeBaseInitStruct.TIM_Period = (period-1);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(Pin->timx, &TIM_TimeBaseInitStruct);
		
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;

	uint32_t pulse = period * (duty_cycle/100.0);
	
	TIM_OCInitStruct.TIM_Pulse = pulse ;
	
	if(Pin->cha == CH1){
	TIM_OC1Init(Pin->timx, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(Pin->timx, TIM_OCPreload_Enable);
	}
	
	if(Pin->cha == CH2){
	TIM_OC2Init(Pin->timx, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(Pin->timx, TIM_OCPreload_Enable);	
	}
	
	if(Pin->cha == CH3){
	TIM_OC3Init(Pin->timx, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(Pin->timx, TIM_OCPreload_Enable);		
	}
	
	if(Pin->cha == CH4){
	TIM_OC4Init(Pin->timx, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(Pin->timx, TIM_OCPreload_Enable);	
	}
	
	TIM_Cmd(Pin->timx, ENABLE);
 }
}

void analogWrite(GPIO_struct *Pin,float duty_cycle)
{
			
			uint32_t pulse = period * (duty_cycle/100.0);
			
	    switch(Pin->cha){
				case CH1:TIM_SetCompare1(Pin->timx, pulse); break;  //如果使用这个就不用声明这个全局变量
	      case CH2:TIM_SetCompare2(Pin->timx, pulse); break;                                             //TIM_OCInitTypeDef TIM_OCInitStruct
			  case CH3:TIM_SetCompare3(Pin->timx, pulse); break;
	      case CH4:TIM_SetCompare4(Pin->timx, pulse); break;

		}
	}

uint8_t find_pin(uint16_t pin)
{
	uint8_t i;
	
	for(i = 0;i<16;i++)
	if((pin & (uint16_t)(1<<i)) == pin) return (i);
  return (100);
}

void attachInterrupt(GPIO_struct *Pin,Trigger_state STATE,PRE_EMPTION pre,uint8_t num)
{
	uint8_t jinzhi = 0;
	
	switch(pre){
		case(Pre_emption_0) :jinzhi = 16;break;
		case(Pre_emption_1) :jinzhi = 8;break;
		case(Pre_emption_2) :jinzhi = 4;break;
		case(Pre_emption_3 ):jinzhi = 2;break;
		case(Pre_emption_4) :jinzhi = 1;break;
		}
	
	if(jinzhi != 0){
		uint8_t pin = find_pin(Pin->GPIO_Pin);
		
		uint8_t IRQN;
		
		if(pin != 100){
			switch(pin){
				case 0:IRQN = EXTI0_IRQn; break;
				case 1:IRQN = EXTI1_IRQn; break;
				case 2:IRQN = EXTI2_IRQn; break;
				case 3:IRQN = EXTI3_IRQn; break;
				case 4:IRQN = EXTI4_IRQn; break;
				case 5:IRQN = EXTI9_5_IRQn; break;
				case 6:IRQN = EXTI9_5_IRQn; break;
				case 7:IRQN = EXTI9_5_IRQn; break;
				case 8:IRQN = EXTI9_5_IRQn; break;
				case 9:IRQN = EXTI9_5_IRQn; break;
				case 10:IRQN = EXTI15_10_IRQn;break;
				case 11:IRQN = EXTI15_10_IRQn;break;
				case 12:IRQN = EXTI15_10_IRQn;break;
				case 13:IRQN = EXTI15_10_IRQn;break;
				case 14:IRQN = EXTI15_10_IRQn;break;
				case 15:IRQN = EXTI15_10_IRQn;break;
			}
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
			
			uint8_t gpio_portsource;
			
			if(Pin->GPIOx == GPIOA) gpio_portsource = 0X00;
			if(Pin->GPIOx == GPIOB) gpio_portsource = 0X01;
			if(Pin->GPIOx == GPIOC) gpio_portsource = 0X02;
			if(Pin->GPIOx == GPIOD) gpio_portsource = 0X03;
			
			uint8_t gpio_pinsource = pin;
			
			GPIO_EXTILineConfig(gpio_portsource, gpio_pinsource);
			
			EXTI_InitTypeDef EXTI_InitStruct;
			
			uint32_t line = (uint32_t)Pin->GPIO_Pin;
			
			EXTI_InitStruct.EXTI_Line = line;
			
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;
			
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			
			EXTI_InitStruct.EXTI_Trigger = (EXTITrigger_TypeDef)STATE;
			
			EXTI_Init(&EXTI_InitStruct);
			
			NVIC_PriorityGroupConfig(pre);
			
			NVIC_InitTypeDef NVIC_InitStruct;
			
			NVIC_InitStruct.NVIC_IRQChannel = IRQN;
			
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = num/jinzhi;
			
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = num%jinzhi;
			
			NVIC_Init(&NVIC_InitStruct);
	}
 }
}


void analogInit(ADC_TypeDef* ADCx,GPIO_struct* Pin)
{
 DMA_InitTypeDef DMA_InitStruct;
 ADC_InitTypeDef ADC_InitStruct;
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
 uint8_t i;
	if((Pin->GPIOx == GPIOA)&&((Pin->GPIO_Pin&(uint32_t)0xFFFF) == Pin->GPIO_Pin)){
		for(i = 0;i<15;i++)
		if((Pin->GPIO_Pin & (uint16_t)(1<<i))== Pin->GPIO_Pin){
			break;
		}
	else if((Pin->GPIOx == GPIOB)&&((Pin->GPIO_Pin&(uint32_t)0xFFFF) == Pin->GPIO_Pin)) {
		for(i = 0;i<2;i++)
		if((Pin->GPIO_Pin & (uint16_t)(1<<i))== Pin->GPIO_Pin){
			if(i==0) i = (uint8_t)0x08;
			if(i==1) i = (uint8_t)0x09;
			break;
		}
	}
 }
	
 DMA_DeInit(DMA1_Channel1);
 DMA_InitStruct.DMA_BufferSize = Num;
 DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
 DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
 DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(value_adc);
 DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
 DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
 DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
 DMA_InitStruct.DMA_PeripheralBaseAddr = ( uint32_t ) ( & ( ADCx->DR ) );
 DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
 DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
 DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	
 Pin->ADC1_value = Num;
 
 DMA_Init(DMA1_Channel1, &DMA_InitStruct);
 DMA_Cmd(DMA1_Channel1, ENABLE);
	
 ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
 ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
 ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
 ADC_InitStruct.ADC_ScanConvMode = ENABLE;
 
 ADC_InitStruct.ADC_NbrOfChannel = Num;
 ADC_Init(ADCx, &ADC_InitStruct);
	
 printf(" Num = %d",Num);
 
 RCC_ADCCLKConfig(RCC_PCLK2_Div8);
 ADC_RegularChannelConfig(ADCx, i , Num , ADC_SampleTime_55Cycles5);
  
 
 
 if(Num == 1){
 ADC_DMACmd(ADCx, ENABLE);
 ADC_Cmd(ADCx, ENABLE);
 ADC_ResetCalibration(ADCx);
 while(ADC_GetResetCalibrationStatus(ADCx));
 ADC_StartCalibration(ADCx);
 while(ADC_GetCalibrationStatus(ADCx));
 ADC_SoftwareStartConvCmd(ADCx, ENABLE);
 }
 Num++;
}

uint8_t isDigital(char letter)
{
	if(letter>='0' && letter<= '9') return 1;
	else return 0;
}
