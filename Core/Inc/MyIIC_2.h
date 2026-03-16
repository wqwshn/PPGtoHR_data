#ifndef _MYIIC_2_H
#define _MYIIC_2_H
#include "main.h"

//#include "myiic.h"

//IIC所有操作函数		 
void Delay2_us(uint8_t nus);     //延时
void IIC2_Start(void);				//发送IIC开始信号
void IIC2_Stop(void);	  			//发送IIC停止信号
void IIC2_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC2_Read_Byte(uint8_t ack);//IIC读取一个字节
uint8_t IIC2_Wait_Ack(void); 				//IIC等待ACK信号
void IIC2_Ack(void);					//IIC发送ACK信号
void IIC2_NAck(void);				//IIC不发送ACK信号


#endif
