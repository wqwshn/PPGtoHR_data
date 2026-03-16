#ifndef _MAX30101_2_H
#define _MAX30101_2_H
#include "main.h"
#include "myiic_2.h"
#include "MAX30101.h"

void MAX30101_2_Init(void) ; //初始化mAX
uint8_t MAX2_ReadOneByte(uint16_t ReadAddr);							//指定地址读取一个字节
void MAX2_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//指定地址写入一个字节
void MAX2_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//指定地址开始写入指定长度的数据
void MAX2_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len);		//指定地址开始读取指定长度数据
void MAX2_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//从指定地址开始写入指定长度的数据
void MAX2_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//从指定地址开始读出指定长度的数据
uint8_t MAX2_Check(void);  //检查器件
void MAX2_ReadFIFOByte(uint32_t *pBuffer);
void MAX2_SpO2_ReadFIFOByte(uint32_t *pBuffer);    //血氧模式专用


#endif
