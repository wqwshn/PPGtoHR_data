#include "MIMU.h"

extern uint8_t ACC_XYZ[];
extern uint8_t GYRO_XYZ[];
extern uint8_t MAG_XYZ[];

//MIMU ACC GYRO的写入
void ACC_GYRO_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);	//选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);      //不选磁
	HAL_Delay(5);
	
	spiTxData[0] = RegAddress; 
	spiTxData[1] = txData; 
	
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);	
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  

}


//MIMU MAG的写入
void MAG_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);      //选磁
	
	spiTxData[0] = RegAddress; 
	spiTxData[1] = txData; 
	
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);	
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

}


//MIMU ACC GYRO的寄存器读取
uint8_t ACC_GYRO_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	//uint8_t ReadData = 0;
	spiTxData[0] =  0x80|RegAddress;
	
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);	//选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);      //不选磁
	HAL_Delay(5);
	
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  
	
	
	return spiRxData[1];
}


//MIMU mag的寄存器读取
uint8_t MAG_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	spiTxData[0] =  0x80|RegAddress;
	
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);      //选磁
	HAL_Delay(5);
	
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);   	
	
	return spiRxData[1];
}


//加速度多字节读取  3轴，每个轴2bytes数据
void ACC_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);	//选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);      //不选磁
	
	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};    //
	spiTxData[0] = 0x80|0x28;   //表示读取RegAddress
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);   
	
	for(uint8_t i = 0; i < 6; i++){
		ACC_XYZ[i] = spiRxData[i+1];
	}			

	//HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	
	//不选加速度和角速度 这里可能和spi语句间隔太短，所以写在这里不好，放在了adc开始转换的语句那里	
}


//gyro
void GYRO_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);	//选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);      //不选磁
	
	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};    //
	spiTxData[0] = 0x80|0x18;   //表示读取RegAddress
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);   
	
	for(uint8_t i = 0; i < 6; i++){
		GYRO_XYZ[i] = spiRxData[i+1];
	}		
	
	//HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  	
}


//磁多字节读取  3轴，每个轴2bytes数据
void MAG_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度  
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);      //选磁
	
	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};    //
	spiTxData[0] = 0xC0|0x28;   //表示读取RegAddress
		
	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff); 
	
	for(uint8_t i = 0; i < 6; i++){     //直接赋值太快了，会显示都是0，不带DMA就行了   这里是6还是7啊
		MAG_XYZ[i] = spiRxData[i+1];
	}
	
	//HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);	
}


//MIMU initialize
void MIMU_Init(){   // 全部没写进去
	ACC_GYRO_Write(CTRL_REG6_XL,0xD6); //加速度寄存器写CTRL_REG6_XL (20h),数据更新率952Hz，带宽100Hz，加速度范围+-4g
	//ACC_GYRO_Write(CTRL_REG6_XL,0xCE); //加速度寄存器写CTRL_REG6_XL (20h)，数据更新率952Hz，带宽100Hz，加速度范围+-16g  （DATASHEET P52）	
	ACC_GYRO_Write(CTRL_REG8,0x04); //加速度陀螺寄存器写CTRL_REG8 (22h)，持续更新..读完后再更新数据 0x44持续更新
	ACC_GYRO_Write(CTRL_REG9,0x04); //加速度陀螺寄存器写CTRL_REG9 (23h)，禁用I2C 0x04    
	
	ACC_GYRO_Write(CTRL_REG1_G,0xC8); //陀螺寄存器写CTRL_REG1_G (10h),数据更新率952Hz，量程500 dps，LPF1带宽100Hz
	//ACC_GYRO_Write(CTRL_REG1_G,0xD8); //陀螺寄存器写CTRL_REG1_G (10h),数据更新率952Hz，LPF1带宽100Hz，量程2000 dps
	
	MAG_Write(CTRL_REG1_M,0xFC); //磁寄存器写CTRL_REG1_M (20h)，磁温补，XY轴磁UHP模式，ODR=80Hz
	MAG_Write(CTRL_REG2_M,0x00); //磁寄存器写CTRL_REG2_M (21h)，量程+-4gauss
	MAG_Write(CTRL_REG3_M,0x80); //磁寄存器写CTRL_REG3_M (22h)，禁用I2C、SPI读写功能，连续转换模式
	MAG_Write(CTRL_REG4_M,0x0C); //磁寄存器写CTRL_REG4_M (23h)，Z轴磁UHP模式
	MAG_Write(CTRL_REG5_M,0x00); //磁寄存器写CTRL_REG5_M (24h)，持续更新模式。数据读出后再更新
}


//MIMU连接是否正常检查
uint8_t MIMU_check(void){
	uint8_t mimu_id[2] = {0};
	mimu_id[0] = ACC_GYRO_Read(WHO_AM_I);
	mimu_id[1] = MAG_Read(WHO_AM_I_M);
	
//	
//	mimu_id[0] = ACC_GYRO_Read(0x1E);
//	mimu_id[1] = ACC_GYRO_Read(0x1F);
//	HAL_UART_Transmit_DMA(&huart2, mimu_id, 2);   
	
	
	
	if((mimu_id[0] == 0x68) & (mimu_id[1] == 0x3D))
		return 1;
	else
		return 0;
}

