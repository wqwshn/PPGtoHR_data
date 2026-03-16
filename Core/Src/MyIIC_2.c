#include "MYIIC_2.h"


#define SCL2_H 	HAL_GPIO_WritePin(I2C_SCL2_GPIO_Port, I2C_SCL2_Pin, GPIO_PIN_SET);
#define SCL2_L 	HAL_GPIO_WritePin(I2C_SCL2_GPIO_Port, I2C_SCL2_Pin, GPIO_PIN_RESET);

#define SDA2_H 	HAL_GPIO_WritePin(I2C_SDA2_GPIO_Port, I2C_SDA2_Pin, GPIO_PIN_SET);
#define SDA2_L 	HAL_GPIO_WritePin(I2C_SDA2_GPIO_Port, I2C_SDA2_Pin, GPIO_PIN_RESET);

#define READ_SDA2 HAL_GPIO_ReadPin(I2C_SDA2_GPIO_Port, I2C_SDA2_Pin)     // 读取SDA信号

//延迟函数就用myiic.c里面的Delay2_us()

//IIC引脚初始化 在gpio.c里面已完成

// SDA为开漏模式，可以不用切换为输入模式，也可以读取输入电平。
// 如输出1(PCout()=1),从它的输入引脚可以读出状态(PCin()),这样在不需要配置输入,输出模式,就可以实现iic数据线的写和读。

//延时
void Delay2_us(uint8_t nus){
	uint8_t i = 0;
	while(nus--)         
	{
		i = 1;                       //需要根据实际情况设定i值大小，以得到正确的时钟
		while(i--);
	}
}

//发送IIC开始信号
void IIC2_Start(void){
	SDA2_H;
	SCL2_H;
	Delay2_us(4);
	SDA2_L;         //START:when CLK is high,DATA change form high to low 
	Delay2_us(4);
	SCL2_L;          //钳住I2C总线，准备发送或接收数据 
}	


//产生IIC停止信号
void IIC2_Stop(void)
{
	SCL2_L;
	SDA2_L;//STOP:when CLK is high DATA change form low to high
 	Delay2_us(4);
	SCL2_H; 
	SDA2_H;//发送I2C总线结束信号
	Delay2_us(4);							   	
}


//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC2_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA2_H; 
	Delay2_us(1);	   
	SCL2_H; 
	Delay2_us(1);	 
	while(READ_SDA2)  // ************************************************这里还没改
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	SCL2_L;//时钟输出0 	   
	return 0;  
} 


//产生ACK应答
void IIC2_Ack(void)
{
	SCL2_L;
	SDA2_L;
	Delay2_us(2);
	SCL2_H;
	Delay2_us(2);
	SCL2_L;
	SDA2_H;   //2022.02.22加   每回进行数据读取和发送的时候需要最后把总线释放，让总线处于空闲状态，才能有其他的操作
}

//不产生ACK应答		    
void IIC2_NAck(void)
{
	SCL2_L;
	SDA2_H;
	Delay2_us(2);
	SCL2_H;	
	Delay2_us(2);
	SCL2_L;
	SDA2_H;   //2022.02.22加
}	

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC2_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
    SCL2_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
			if((txd&0x80)>>7){
				SDA2_H;
			}
			else{
				SDA2_L;
			}
			txd<<=1; 	  
			Delay2_us(2);   
			SCL2_H;
			Delay2_us(2); 
			SCL2_L;	
			Delay2_us(2);
    }	 
}


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC2_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
    for(i=0;i<8;i++ ){
			SCL2_L; 
			Delay2_us(2);
			SCL2_H;
			receive<<=1;
			if(READ_SDA2) 
				receive++;   
			Delay2_us(1); 
    }					 
    if (!ack)
        IIC2_NAck();//发送nACK
    else
        IIC2_Ack(); //发送ACK   
    return receive;
}
