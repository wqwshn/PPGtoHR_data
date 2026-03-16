#include "MYIIC.h"


#define SCL1_H 	HAL_GPIO_WritePin(I2C_SCL1_GPIO_Port, I2C_SCL1_Pin, GPIO_PIN_SET);
#define SCL1_L 	HAL_GPIO_WritePin(I2C_SCL1_GPIO_Port, I2C_SCL1_Pin, GPIO_PIN_RESET);

#define SDA1_H 	HAL_GPIO_WritePin(I2C_SDA1_GPIO_Port, I2C_SDA1_Pin, GPIO_PIN_SET);
#define SDA1_L 	HAL_GPIO_WritePin(I2C_SDA1_GPIO_Port, I2C_SDA1_Pin, GPIO_PIN_RESET);

#define READ_SDA1 HAL_GPIO_ReadPin(I2C_SDA1_GPIO_Port, I2C_SDA1_Pin)     // Read SDA signal



// Small delay function
void delay_us(uint8_t nus)
{
	uint8_t i = 0;
	while(nus--)
	{
		i = 1;                       //Adjust i value to get accurate delay
		while(i--);
	}
}


//IIC bus initialization - configured in gpio.c


// SDA in open-drain mode, so no need to switch to input mode, can also read level
// When writing 1(PCout()=1), the bus can be released to open-drain state(PCin()), no need to switch direction in the program, open-drain mode can realize I2C bidirectional read and write

//Generate IIC start signal
void IIC_Start(void){
	SDA1_H;
	SCL1_H;
	delay_us(4);
	SDA1_L;         //START:when CLK is high,DATA change form high to low
	delay_us(4);
	SCL1_L;          //Hold I2C bus, ready to send or receive data
}


//Generate IIC stop signal
void IIC_Stop(void)
{
	SCL1_L;
	SDA1_L;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	SCL1_H;
	SDA1_H;//Send I2C bus end signal
	delay_us(4);
}


//Wait for arrival of acknowledge signal
//return value:1:failed to receive acknowledge
//        0:acknowledge received successfully
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA1_H;
	delay_us(1);
	SCL1_H;
	delay_us(1);
	while(READ_SDA1)  // ************************************************No response here yet
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL1_L;//Clock set to 0
	return 0;
}


//Generate ACK response
void IIC_Ack(void)
{
	SCL1_L;
	SDA1_L;
	delay_us(2);
	SCL1_H;
	delay_us(2);
	SCL1_L;
	SDA1_H;   //2022.02.22   Every time data is read and sent, need to release the bus, keep the bus in open-drain state

}


//Generate no ACK response
void IIC_NAck(void)
{
	SCL1_L;
	SDA1_H;
	delay_us(2);
	SCL1_H;
	delay_us(2);
	SCL1_L;
	SDA1_H;   //2022.02.22   Every time data is read and sent, need to release the bus, keep the bus in open-drain state

}

//IIC send one byte
//Return value from slave:whether there is acknowledge
//1:acknowledge
//0:no acknowledge
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SCL1_L;//Pull down clock to start data transmission
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7){
			SDA1_H;
		}
		else{
			SDA1_L;
		}
        txd<<=1;
		delay_us(2);
		SCL1_H;
		delay_us(2);
		SCL1_L;
		delay_us(2);
    }
}


//Read one byte, when ack=1 send ACK, when ack=0 send nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
    for(i=0;i<8;i++ )
	{
        SCL1_L;
        delay_us(2);
		SCL1_H;
        receive<<=1;
        if(READ_SDA1)
			receive++;
		delay_us(1);
    }
    if (!ack)
        IIC_NAck();//send nACK
    else
        IIC_Ack(); //send ACK
    return receive;
}

// 连续读取 len 个字节 (用于 MAX30101 FIFO 批量读取)
void IIC_ReadBytes(uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        // 最后一个字节发送 NACK，其余发送 ACK
        buf[i] = IIC_Read_Byte(i < len - 1);
    }
}
