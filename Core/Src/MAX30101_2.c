#include "MAX30101_2.h"


void MAX30101_2_Init(void)      //max30101_2 registers init
{
	/*设置中断*/
	MAX2_WriteOneByte(0x00,0x00); //all interrupt clear
	MAX2_WriteOneByte(0x01,0x00); //all interrupt clear
	MAX2_WriteOneByte(0x02,0x00); //close all of interrupt
	MAX2_WriteOneByte(0x03,0x00); //DIE_TEMP_RDY_EN
	
	//清除fifo前面三个reg
	MAX2_WriteOneByte(FIFO_WR_PTR_REG, 0x00);   //set FIFO write Pointer reg = 0x00 for clear it
  MAX2_WriteOneByte(OVF_COUNTER_REG, 0x00);        //set Over Flow Counter  reg = 0x00 for clear it
  MAX2_WriteOneByte(RD_PTR_REG, 0x00);        //set FIFO Read Pointer  reg = 0x00 for clear it
	
	/*设置模式*/
	MAX2_WriteOneByte(FIFO_CONFIG_REG,  FIFO_CONFIG_REG_SMP_AVE0|
                                       FIFO_CONFIG_REG_FIFO_ROLLOVER_EN  |
                                       FIFO_CONFIG_REG_FIFO_ALL_FULL);
	
	MAX2_WriteOneByte(MODE_CONFIG_REG, MODE_CONFIG_REG_SHDN |MODE_CONFIG_REG_RESET |MODE_CONFIG_REG_MODE_Multi);// ; 
	
	MAX2_WriteOneByte(SPO2_CONFIG_REG,  ADC_RGE_00 |
                                       SAMPLE_3200  |
                                       SPO2_CONFIG_REG_LED_PW);        

	MAX2_WriteOneByte(LED1_PA_REG, 0xB1);      //发光强度 red
	MAX2_WriteOneByte(LED2_PA_REG, 0xB1);      //发光强度  ir
	MAX2_WriteOneByte(LED3_PA_REG, 0xB1);      //发光强度  green-start
	MAX2_WriteOneByte(LED4_PA_REG, 0xB1);      //发光强度   green-end

	MAX2_WriteOneByte(LED_CONTROL1, 0x01); // 先发红光  
	MAX2_WriteOneByte(LED_CONTROL2, 0x00);
	     
	//MAX2_WriteOneByte(0x12, 0x03);      // 
	//MAX2_WriteOneByte(0x21, 0x01); //SET   TEMP_EN
									
}

//在MAX2指定地址读出一个数据 1212
//返回值  :读到的数据
uint8_t MAX2_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
    IIC2_Start();  
	IIC2_Send_Byte(0XAE);   //发送器件地址0XAE,写数据 	   
	IIC2_Wait_Ack(); 
    IIC2_Send_Byte(ReadAddr);   //发送reg地址
	IIC2_Wait_Ack();	    
	IIC2_Start();  	 	   
	IIC2_Send_Byte(0XAF);           //进入接收模式			   
	IIC2_Wait_Ack();	 
    temp=IIC2_Read_Byte(0);		   
    IIC2_Stop();//产生一个停止条件	    
	return temp;
}



//在MAX2指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void MAX2_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
    IIC2_Start();  
	IIC2_Send_Byte(0XAE);   //发送器件地址0XAE,写数据 	 
	IIC2_Wait_Ack();	   
    IIC2_Send_Byte(WriteAddr);   //发送REG地址
	IIC2_Wait_Ack(); 	 										  		   
	IIC2_Send_Byte(DataToWrite);     //发送字节							   
	IIC2_Wait_Ack();  		    	   
    IIC2_Stop();//产生一个停止条件 
	//delay_ms(10);	 
}




//在MAX2里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void MAX2_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		MAX2_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}



//在MAX2里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
void MAX2_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len)
{

	uint8_t i = 0;
	IIC2_Start(); 
	IIC2_Send_Byte(0XAE);
	IIC2_Wait_Ack(); 
	IIC2_Send_Byte(ReadAddr);
	IIC2_Wait_Ack();
	IIC2_Start(); 
	IIC2_Send_Byte(0XAF);           //进入接收模式
	IIC2_Wait_Ack();
	
	for(i = 0;i < Len; i++ )
	{
		
		if(i == Len -1)
			*pBuffer = IIC2_Read_Byte(0);
		else
		{
			*pBuffer = IIC2_Read_Byte(1);
			pBuffer++;
		}		
	}

	IIC2_Stop();//产生一个停止条件

}



//检查MAX230101是否能连上
//返回1:检测失败
//返回0:检测成功
uint8_t MAX2_Check(void)  //这里没改
{
	uint8_t temp = 0;
	temp=MAX2_ReadOneByte(0xff);//避免每次开机都写MAX2			   
	if(temp==0X15) return 1;		   
	else//排除第一次初始化的情况
//	{
//		MAX2_WriteOneByte(255,0X55);
//	    temp=MAX2_ReadOneByte(255);	  
//		if(temp==0X55)return 0;
//	}
		return 0;											  
}


//在MAX2里面的指定地址开始读出指定个数的数据
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void MAX2_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++ = MAX2_ReadOneByte(ReadAddr++);	
		//*pBuffer++ = MAX2_ReadOneByte(ReadAddr);	
		NumToRead--;
	}
}  


//在MAX2里面的指定地址开始写入指定个数的数据
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void MAX2_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		MAX2_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}



//从FIFO中读取Len个byte的数据
void MAX2_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[3] = {0};
	IIC2_Start(); 
	IIC2_Send_Byte(0XAE);
	IIC2_Wait_Ack(); 
	IIC2_Send_Byte(FIF0_DATA_REG);
	IIC2_Wait_Ack();
	IIC2_Start(); 
	IIC2_Send_Byte(0XAF);           //进入接收模式
	IIC2_Wait_Ack();
	
	Buf_temp[0] = IIC2_Read_Byte(1) & 0x03;
	Buf_temp[1] = IIC2_Read_Byte(1);
	Buf_temp[2] = IIC2_Read_Byte(0);
	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];
	
	IIC2_Stop();//产生一个停止条件
}

//血氧模式，每次读取2个channel的数据，共计6字节
void MAX2_SpO2_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[6] = {0};
	IIC2_Start(); 
	IIC2_Send_Byte(0XAE);
	IIC2_Wait_Ack(); 
	IIC2_Send_Byte(FIF0_DATA_REG);
	IIC2_Wait_Ack();
	IIC2_Start(); 
	IIC2_Send_Byte(0XAF);           //进入接收模式
	IIC2_Wait_Ack();
	
	Buf_temp[0] = IIC2_Read_Byte(1) & 0x03;
	Buf_temp[1] = IIC2_Read_Byte(1);
	Buf_temp[2] = IIC2_Read_Byte(1);
	Buf_temp[3] = IIC2_Read_Byte(1) & 0x03;
	Buf_temp[4] = IIC2_Read_Byte(1);
	Buf_temp[5] = IIC2_Read_Byte(0);
	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];
	pBuffer++;
	*pBuffer = ((Buf_temp[3] << 16) | (Buf_temp[4] << 8)) | Buf_temp[5];
	
	IIC2_Stop();//产生一个停止条件
}
