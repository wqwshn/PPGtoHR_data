#ifndef _ADS_ADS124_H
#define _ADS_ADS124_H

#include "gpio.h"
#include "spi.h"

#define RXBUFFERSIZE  256 // max receive buff size
#define ADC_ID_REG 0x00       // 用于检查adc ID
#define ADC_STATUS_REG 0X01   // 检查adc状态
#define ADC_MUX_REG 0x02      // 控制差分电压正负极的连接
#define ADC_GSET_REG 0x03   	// 设置PGA, 不用修改
#define ADC_RATE_REG 0x04   	// setup data rate
#define ADC_RCON_REG 0x05     // reference voltage control
#define ADC_CUR1_REG 0x06  		// IDAC设置，用于输出电流的，默认设置不用改
#define ADC_CUR2_REG 0x07   	// IDAC引脚设置，不用改
#define ADC_BIAS_REG 0x08   	// BIAS voltage设置，把约2.5V电压接到AIN4上面
#define ADC_SCON_REG 0x09   	// 正常使用情况下什么都不用改，0x0A ~ 0x11都不用设置


void ADC_Init(void);     // ADC ADS124S06初始化
void ADC_RREG(uint8_t Address_Reg, uint8_t *rxData);     // 读取adc寄存器的值
void ADC_WREG(uint8_t Address_Reg, uint8_t txData);     // 写入adc寄存器的值    
void ADC_RDATA(uint8_t *pRxData);    // RDATA  读取电压转换值
void ADC_1to4Voltage(void);    //读取AIN0~AIN3的电压
uint8_t ADC_check(void);        //检查adc是否正常连接

#endif
