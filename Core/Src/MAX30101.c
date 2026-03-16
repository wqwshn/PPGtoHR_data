#include "MAX30101.h"


void MAX30101_Init(void)      //max30101 registers init
{
	/* Enable interrupts */
	MAX_WriteOneByte(0x00,0x00); //all interrupt clear
	MAX_WriteOneByte(0x01,0x00); //all interrupt clear
	MAX_WriteOneByte(0x02,0x00); //close all of interrupt
	MAX_WriteOneByte(0x03,0x00); //DIE_TEMP_RDY_EN

	/* Clear FIFO pointers */
	MAX_WriteOneByte(FIFO_WR_PTR_REG, 0x00);   //set FIFO write Pointer reg = 0x00 for clear it
  MAX_WriteOneByte(OVF_COUNTER_REG, 0x00);        //set Over Flow Counter  reg = 0x00 for clear it
  MAX_WriteOneByte(RD_PTR_REG, 0x00);        //set FIFO Read Pointer  reg = 0x00 for clear it

	/* Configure mode */
	MAX_WriteOneByte(FIFO_CONFIG_REG,  FIFO_CONFIG_REG_SMP_AVE0|
                                       FIFO_CONFIG_REG_FIFO_ROLLOVER_EN  |
                                       FIFO_CONFIG_REG_FIFO_ALL_FULL);

	MAX_WriteOneByte(MODE_CONFIG_REG, MODE_CONFIG_REG_SHDN |MODE_CONFIG_REG_RESET |MODE_CONFIG_REG_MODE_Multi);// ;

	MAX_WriteOneByte(SPO2_CONFIG_REG,  ADC_RGE_00 |
                                       SAMPLE_3200  |
                                       SPO2_CONFIG_REG_LED_PW);

	MAX_WriteOneByte(LED1_PA_REG, 0xB1);      //LED pulse amplitude red
	MAX_WriteOneByte(LED2_PA_REG, 0xB1);      //LED pulse amplitude  ir
	MAX_WriteOneByte(LED3_PA_REG, 0xB1);      //LED pulse amplitude  green-start
	MAX_WriteOneByte(LED4_PA_REG, 0xB1);      //LED pulse amplitude   green-end

	MAX_WriteOneByte(LED_CONTROL1, 0x01);   // LED control
	MAX_WriteOneByte(LED_CONTROL2, 0x00);


}

//Read MAX register data
//return value:register data
uint8_t MAX_ReadOneByte(uint16_t ReadAddr)
{
	uint8_t temp=0;
    IIC_Start();
	IIC_Send_Byte(0XAE);   //Send device write address
	IIC_Wait_Ack();
    IIC_Send_Byte(ReadAddr);   //Send reg address
	IIC_Wait_Ack();
    IIC_Start();
	IIC_Send_Byte(0XAF);           //Send device read address
	IIC_Wait_Ack();
    temp=IIC_Read_Byte(0);
    IIC_Stop();//Generate a stop signal
	return temp;
}



//Write one byte to MAX register
//WriteAddr  :Destination address for data to be written
//DataToWrite:Data to be written
void MAX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{
    IIC_Start();
	IIC_Send_Byte(0XAE);   //Send device write address
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);   //Send REG address
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite);     //Send byte
	IIC_Wait_Ack();
    IIC_Stop();//Generate a stop signal
	//delay_ms(10);
}



//Write length of Len data continuously starting from the specified address of MAX
//This function can be used to write 16bit or 32bit data.
//WriteAddr  :Starting address to write
//DataToWrite:First address of data array
//Len        :Length of data to be written 2,4
void MAX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		MAX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}
}



//Read length of Len data continuously starting from the specified address of MAX
//This function can be used to read 16bit or 32bit data.
//ReadAddr   :Starting address to read
//return value     :data
//Len        :Length of data to be read 2,4
void MAX_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len)
{

	uint8_t i = 0;
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);           //Send device read address
	IIC_Wait_Ack();

	for(i = 0;i < Len; i++ )
	{

		if(i == Len -1)
			*pBuffer = IIC_Read_Byte(0);
		else
		{
			*pBuffer = IIC_Read_Byte(1);
			pBuffer++;
		}
	}

	IIC_Stop();//Generate a stop signal

}



//Check if MAX30101 is connected
//return 1:connection failed
//        0:success
uint8_t MAX_Check(void)  //Check for presence
{
	uint8_t temp = 0;
	temp=MAX_ReadOneByte(0xff);//Read revision ID every time before writing to MAX
	if(temp==0X15) return 1;
	else//Exclude the possibility of initialization failure
//	{
//		MAX_WriteOneByte(255,0X55);
//	    temp=MAX_ReadOneByte(255);
//		if(temp==0X55)return 0;
//	}
		return 0;
}


//Read specified number of data continuously starting from the specified address of MAX
//pBuffer  :First address of data array
//NumToRead:Number of data to read
void MAX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=MAX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}


//Write specified number of data continuously starting from the specified address of MAX
//pBuffer   :First address of data array
//NumToWrite:Number of data to write
void MAX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		MAX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}



//Read Len byte data from FIFO interrupt
void MAX_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[3] = {0};
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(FIF0_DATA_REG);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);           //Send device read address
	IIC_Wait_Ack();

	Buf_temp[0] = IIC_Read_Byte(1) & 0x03;
	Buf_temp[1] = IIC_Read_Byte(1);
	Buf_temp[2] = IIC_Read_Byte(0);
	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];

	IIC_Stop();//Generate a stop signal
}

//SpO2 mode read 2 channel data each time, occupying 6 bytes
void MAX_SpO2_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[6] = {0};
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(FIF0_DATA_REG);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);           //Send device read address
	IIC_Wait_Ack();

	Buf_temp[0] = IIC_Read_Byte(1) & 0x03;
	Buf_temp[1] = IIC_Read_Byte(1);
	Buf_temp[2] = IIC_Read_Byte(1);
	Buf_temp[3] = IIC_Read_Byte(1) & 0x03;
	Buf_temp[4] = IIC_Read_Byte(1);
	Buf_temp[5] = IIC_Read_Byte(0);
	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];
	pBuffer++;
	*pBuffer = ((Buf_temp[3] << 16) | (Buf_temp[4] << 8)) | Buf_temp[5];

	IIC_Stop();//Generate a stop signal
}

// 连续读取 FIFO 数据 (用于主循环 FIFO 排空)
// 单绿光模式下每个样本占用 3 个字节
void MAX_ReadFIFO_Burst(uint8_t *buf, uint8_t len)
{
	IIC_Start();
	IIC_Send_Byte(0XAE);           // 发送设备写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(FIF0_DATA_REG);  // 发送 FIFO 数据寄存器地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);           // 发送设备读地址
	IIC_Wait_Ack();

	// 连续读取 len 个字节
	IIC_ReadBytes(buf, len);

	IIC_Stop();
}
