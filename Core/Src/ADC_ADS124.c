#include "ADC_ADS124.h"

extern uint8_t DIN[4];
extern uint8_t DOUT[4];
extern uint8_t ADC_1to4Voltage_flag;

// 读取adc寄存器的值
void ADC_RREG(uint8_t Address_Reg, uint8_t *rxData){          //  读取ADC寄存器值 1byte 地址|0x20 2byte 0（字节数-1） 3byte 0
	uint8_t spiTxData[3] = {0};
	spiTxData[0] = 0x20 | Address_Reg; 
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxData, rxData, 3); 
}


// 写入adc寄存器的值
void ADC_WREG(uint8_t Address_Reg, uint8_t txData){          //  写入ADC寄存器值 1byte 地址|0x40 2byte 0（字节数-1） 3byte 要写入的数据
	uint8_t spiTxData[3] = {0};
	spiTxData[0] = 0x40 | Address_Reg; 
	spiTxData[2] = txData; 
	uint8_t spiRxData[3] = {0};
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxData, spiRxData, 3); 
}


// RDATA  读取电压转换值
void ADC_RDATA(uint8_t *pRxData){
	uint8_t pRxBuf[4] = {0};  //用于接收4个字节的数据，其中第1个字节没用，后3个字节去前2个字节用于传输
	HAL_SPI_TransmitReceive(&hspi1, DIN, pRxBuf, 4, 0xffff);
	*pRxData = pRxBuf[1];
	pRxData++;
	*pRxData = pRxBuf[2];
}


//读取AIN0~AIN3的电压 正端用AIN0~3，负端用AINCOM，这样可以保证电压在-2.5V~+2.5V
void ADC_1to4Voltage(void){
	switch(ADC_1to4Voltage_flag){
			case 0:
				ADC_1to4Voltage_flag = 1;
				HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
				ADC_WREG(ADC_MUX_REG, 0X0C);
				HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
				break;
			case 1:
				ADC_1to4Voltage_flag = 2;
				HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
				ADC_WREG(ADC_MUX_REG, 0X1C);
				HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
				break; 
			case 2:
				ADC_1to4Voltage_flag = 3;
				HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
				ADC_WREG(ADC_MUX_REG, 0X2C);
				HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
				break; 
			case 3:
				ADC_1to4Voltage_flag = 0;
				HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
				ADC_WREG(ADC_MUX_REG, 0X3C);
				HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
				break; 
			default: 
				 ADC_1to4Voltage_flag = 4;
	}
}


// ADC ADS124S06初始化
void ADC_Init(void){  
	//将DRDY设置为下降沿触发中断，在中断内用direct read法读取电压值
	//下降沿触发中断已经在MX_GPIO_Init()设置好了，中断回调函数为HAL_GPIO_EXTI_Callback()
	
	//SPI1_NSS_Pin设为0，选通ADC
	HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	
	//上电之后reset一下
	HAL_GPIO_WritePin(GPIOA, AD_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, AD_RESET_Pin, GPIO_PIN_SET);
	
	//检查STATUS RDY是否为0，为0表示adc ready
	uint8_t RxData_ADC[3] = {0};
	ADC_RREG(ADC_STATUS_REG, RxData_ADC);
	
	while((RxData_ADC[2] & 0x40) != 0){     //如果RDY位不为0，则说明没有ready，再reset
		HAL_GPIO_WritePin(GPIOA, AD_RESET_Pin, GPIO_PIN_RESET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, AD_RESET_Pin, GPIO_PIN_SET);
		ADC_RREG(ADC_STATUS_REG, RxData_ADC);
	}
	
	//给STATUS FL_POR置0，Indicates Register  has  been  cleared  and  no  POR  event  has  occurred
	HAL_Delay(1);
	ADC_WREG(ADC_STATUS_REG, 0X00);
	
	//MUXP设为AIN5(5号连地), MUXN设为AINCOM（剪断了，现在飞线接2.5v）
	HAL_Delay(1);     
	ADC_WREG(ADC_MUX_REG, 0X3C);
	
	//设置PGA 关闭掉
	HAL_Delay(1);
	ADC_WREG(ADC_GSET_REG, 0x00);  
	
	//使用single-shot模式，low-latency  filter滤波，data rate 2000sps
	HAL_Delay(1);
	ADC_WREG(ADC_RATE_REG, 0X2C);
	
	//设置参考电压，内部2.5V，常开
	HAL_Delay(1);
	ADC_WREG(ADC_RCON_REG, 0x3A);
	
	//设置偏置电压，不连
	HAL_Delay(1);
	ADC_WREG(ADC_BIAS_REG, 0x00);
	
	//设置system control ，关掉测试项
	HAL_Delay(1);
	ADC_WREG(ADC_SCON_REG, 0x10);
	HAL_Delay(1);

	//CS拉高
	HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

}



//检查adc是否正常连接
uint8_t ADC_check(void){
	uint8_t rxDataaa[3] = {0};
	HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);   // ADC CS选中
	
	ADC_RREG(ADC_ID_REG, rxDataaa);    //读取ADC_ID_REG寄存器的值，ads124s06的值是0x01 
	HAL_Delay(1);
	
	if((rxDataaa[2] & 0x01) == 1)
		return 1;
	else
		return 0;
}