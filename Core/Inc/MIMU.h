#ifndef _MIMU_H
#define _MIMU_H
#include "main.h"
#include "spi.h"
#include "usart.h"

/* ── LSM9DS1 A/G 寄存器地址定义 ── */
#define CTRL_REG1_G                    0x10   // 陀螺仪基础配置: ODR, 量程, BW
#define CTRL_REG3_G                    0x12   // 陀螺仪高级配置: 高通滤波, 低功耗
#define CTRL_REG6_XL                   0x20   // 加速度计基础配置: ODR, 量程, BW
#define CTRL_REG7_XL                   0x21   // 加速度计高级配置: 高分辨率, 数字滤波
#define CTRL_REG8                      0x22   // 系统控制: BDU, IF_ADD_INC, SW_RESET
#define CTRL_REG9                      0x23   // FIFO/I2C控制

/* ── LSM9DS1 Magnetometer 寄存器地址 ── */
#define CTRL_REG1_M                    0x20   // 磁力计配置
#define CTRL_REG2_M                    0x21   // 磁力计量程
#define CTRL_REG3_M                    0x22   // 磁力计模式
#define CTRL_REG4_M                    0x23   // Z轴模式
#define CTRL_REG5_M                    0x24   // 磁力计更新模式
#define WHO_AM_I                       0x0F
#define WHO_AM_I_M                     0x0F


void ACC_GYRO_Write(uint8_t RegAdress, uint8_t txData);  // MIMU ACC GYRO 写入
void MAG_Write(uint8_t RegAdress, uint8_t txData);    // MIMU MAG 写入

uint8_t ACC_GYRO_Read(uint8_t RegAdress); // MIMU ACC GYRO 寄存器读取
uint8_t MAG_Read(uint8_t RegAdress);      // MIMU mag 寄存器读取

void ACC_6BytesRead(void);  // 加速度计多字节读取
void GYRO_6BytesRead(void); // 陀螺仪多字节读取
void MAG_6BytesRead(void);  // 磁力计多字节读取

void MIMU_Init(void);       // MIMU 初始化
uint8_t MIMU_check(void);   // MIMU 连接检查

#endif
