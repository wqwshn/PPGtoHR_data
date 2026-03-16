#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"

//IIC所有操作函数
void delay_us(uint8_t nus);
void IIC_Start(void);				//发送IIC起始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC发送不ACK信号
void IIC_ReadBytes(uint8_t *buf, uint8_t len);  //连续读取len个字节


#endif
