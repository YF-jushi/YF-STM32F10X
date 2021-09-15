#include "Servo.h"

void Servo_attach(GPIO_struct *Pin)
{
	pinMode(Pin,OUTPUT_AF_PP);
	analogWrite_Init(Pin,50,7.5);
}

float map(float value,float a,float b,float c,float d)
{
	float K = (float)(b-a)/(d-c);
	
	float result = (value*K) + a;
	
	return result;
}

float Servo_write(GPIO_struct *Pin,uint8_t Pos)
{
	float Duty_cycle =  map(Pos,2.5,12.5,0,180);
	
	analogWrite(Pin,Duty_cycle);
	
	return Duty_cycle;
}
