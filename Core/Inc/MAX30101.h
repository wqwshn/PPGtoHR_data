#ifndef _MAX30101_H
#define _MAX30101_H
#include "main.h"
#include "myiic.h"



#define INTERRUPT_REG                    0X00
#define INTERRUPT_REG_A_FULL            (0X01<<7)
#define INTERRUPT_REG_PPG_RDY           (0X01<<6)
#define INTERRUPT_REG_ALC_OVF           (0X01<<5)
#define INTERRUPT_REG_PWR_RDY           (0X01<<0)

#define INTERRUPT_ENABLE_REG                     0X02
#define INTERRUPT_ENABLE_REG_A_FULL_EN           (0X01<<7)
#define INTERRUPT_ENABLE_REG_PPG_RDY_EN          (0X01<<6)
#define INTERRUPT_ENABLE_REG_ALC_OVF_EN          (0X01<<5)

#define INTERRUPT_DIE_TEMP_REG                   0X03
#define INTERRUPT_DIE_TEMP_REG_DIE_TEMP_EN       (0X01<<1)

#define FIFO_WR_PTR_REG                0X04
#define OVF_COUNTER_REG                0X05
#define RD_PTR_REG                     0X06
#define FIF0_DATA_REG                  0X07

#define FIFO_CONFIG_REG                0X08
#define FIFO_CONFIG_REG_SMP_AVE0        (0X02<<5)    //SPM_AVE[2:0] = 0x02 4 samples average     00 no average
#define FIFO_CONFIG_REG_FIFO_ROLLOVER_EN (0X01<<4)  //  Enable FIFO rollover, new data will overwrite old data
#define FIFO_CONFIG_REG_FIFO_ROLLOVER_DIS (0X00<<4)  //  Disable FIFO rollover
#define FIFO_CONFIG_REG_FIFO_ALL_FULL    (0X00<<0)  // READ 17 data  for one  interrupt

#define MODE_CONFIG_REG                0X09
#define MODE_CONFIG_REG_SHDN           (0X00<<7)  // shutdown control
#define MODE_CONFIG_REG_SHDN_REAL           (0X01<<7)  // power-save mode
#define MODE_CONFIG_REG_RESET          (0X00<<6)  // reset  control
#define MODE_CONFIG_REG_MODE_Spo2           (0X03<<0)  // Spo2  mode
#define MODE_CONFIG_REG_MODE_HR           (0X02<<0)  // Heart rate mode red only
#define MODE_CONFIG_REG_MODE_Multi           (0X07<<0)  // Heart rate mode multi-led red ir green

#define SPO2_CONFIG_REG                0X0A
#define SPO2_CONFIG_REG_ADC_RGE        (0X01<<5)  // SP02_ADC_RGE[1:0]=11  largest range
#define ADC_RGE_00        (0X00<<5)  // SP02_ADC_RGE[1:0]=00  smallest range

#define SAMPLE_1600 (0X06<<2)  // SP02_SR[2:0]=110     Sample Rate = 1600
#define SAMPLE_3200 (0X07<<2)  // SP02_SR[2:0]=111     Sample Rate = 3200
//#define SPO2_CONFIG_REG_SR         (0X07<<2)  // SP02_SR[2:0]=111     Sample Rate = 3200
//#endif

//#010 200   011 400  100 800     000 50    001 100
#define SPO2_CONFIG_REG_LED_PW      (0X00<<0)  // SP02_LED_PW[1:0]=00  shortest pulse width
#define LED1_PA_REG                 0X0C
#define LED2_PA_REG                 0X0D
#define LED3_PA_REG                 0X0E
#define LED4_PA_REG                 0X0F
#define ONES_READ_DATA_BY_FIFO      (32-INTERRUPT_FIFO_CONFIG_REG_FIFO_ALL_FULL)  // READ NUM  data  for one  interrupt

#define LED_CONTROL1 0x11   //Multi-LED Mode control each slot led color
#define LED_CONTROL2 0x12


uint8_t MAX_ReadOneByte(uint16_t ReadAddr);							//Read one byte from register address
void MAX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//Write one byte to register address
void MAX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//Write specified length data starting from register address
void MAX_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len);		//Read specified length data starting from register address
void MAX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//Write specified number of data starting from register address
void MAX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//Read specified number of data starting from register address
uint8_t MAX_Check(void);  //Check for presence
void MAX30101_Init(void) ; //Initialize MAX
void MAX_ReadFIFOByte(uint32_t *pBuffer);
void MAX_SpO2_ReadFIFOByte(uint32_t *pBuffer);
void MAX_ReadFIFO_Burst(uint8_t *buf, uint8_t len);  //连续读取FIFO数据


#endif
