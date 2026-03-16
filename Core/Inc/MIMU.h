#ifndef _MIMU_H
#define _MIMU_H
#include "main.h"
#include "spi.h"
#include "usart.h"

#define CTRL_REG6_XL                   0x20   //加速度寄存器写CTRL_REG6_XL (20h)
#define CTRL_REG8                      0x22   //加速度陀螺寄存器写CTRL_REG8 (22h)，读完后再更新数据
#define CTRL_REG9                      0x23   //加速度陀螺寄存器写CTRL_REG9 (23h)，禁用I2C
#define CTRL_REG1_G                    0x10   //陀螺寄存器写CTRL_REG1_G (10h),数据更新率952Hz，LPF1带宽100Hz，量程2000 dps
#define CTRL_REG1_M                    0x20   //磁寄存器写CTRL_REG1_M (20h)，磁温补，XY轴磁UHP模式，ODR=80Hz
#define CTRL_REG2_M                    0x21   //磁寄存器写CTRL_REG2_M (21h)，量程+-4gauss
#define CTRL_REG3_M                    0x22   //磁寄存器写CTRL_REG3_M (22h)，禁用I2C、SPI读写功能，连续转换模式
#define CTRL_REG4_M                    0x23   //磁寄存器写CTRL_REG4_M (23h)，Z轴磁UHP模式
#define CTRL_REG5_M                    0x24   //磁寄存器写CTRL_REG5_M (24h)，数据读出后再更新
#define WHO_AM_I                       0x0F  
#define WHO_AM_I_M                     0x0F  


void ACC_GYRO_Write(uint8_t RegAdress, uint8_t txData);  //MIMU ACC GYRO的写入
void MAG_Write(uint8_t RegAdress, uint8_t txData);    //MIMU MAG的写入

uint8_t ACC_GYRO_Read(uint8_t RegAdress); //MIMU ACC GYRO的寄存器读取
uint8_t MAG_Read(uint8_t RegAdress);      //MIMU mag的寄存器读取

void ACC_6BytesRead(void); //加速度和角速度多字节读取
void GYRO_6BytesRead(void); //加速度和角速度多字节读取
void MAG_6BytesRead(void); //磁多字节读取

void MIMU_Init(void);                                      //MIMU初始化
uint8_t MIMU_check(void);                                //MIMU连接是否正常检查

#endif
