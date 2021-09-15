#ifndef __SERVO_H
 #define __SERVO_H
 
 #include "GPIO_footprint.h"
 
 void Servo_attach(GPIO_struct *Pin);
 float map(float value,float a,float b,float c,float d);
 float Servo_write(GPIO_struct *Pin,uint8_t Pos);
 
#endif
