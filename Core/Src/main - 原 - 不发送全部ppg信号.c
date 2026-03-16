/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "ADC_ADS124.h"
#include "MAX30101.h"
#include "MAX30101_2.h"
#include "MIMU.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aTxBuffer[] = "USART DMA OK!"; 

uint8_t it_times = 0;    //进入中断的次数
uint8_t rx_len2[1] = {0};
uint8_t flag_0 = 1;
uint8_t ReceiveState = 0;
uint8_t aRxBuffer2[3];
uint8_t ADC_1to4Voltage_flag = 4;
uint8_t ADC_1to4Voltages[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};   
uint8_t ACC_XYZ[6]  = {0};  
uint8_t GYRO_XYZ[6] = {0};  
uint8_t MAG_XYZ[6]  = {0}; 
uint8_t *p;
uint8_t allData[40] = {0};
uint8_t ColorToggle = 1;
uint8_t test[1]={0xdd};

uint8_t Pulse_IIC[3] = {0};
uint8_t Pulse_IIC_all[1200] = {0};
uint16_t Pulse_IIC_times = 0;

uint8_t Pulse_IIC2[3] = {0};
uint8_t Pulse_IIC2_all[600] = {0};
uint16_t Pulse_IIC2_times = 0;

uint16_t adc_times = 0;
uint8_t adcMimu[600] = {0};
uint16_t times_2 = 0;
uint8_t times_2_arr[2] = {0};


/* MIMU量程档位初始值 */
uint8_t DR_ACC =2;
//	uint8_t DR_GYRO =2;
//	uint8_t DR_MAG =1; 
/* PPG颜色、亮度、adc量程初始值 */
uint8_t DR_PPG_COLOR =1;    //1绿色 2红色 3红外
uint8_t DR_PPG_INTENSITY =1;    //1~9挡位
uint8_t DR_PPG_ADCRANGE =1;     //0~3
uint8_t DR_PPG_SMPAVE = 1;
uint8_t DR_PPG_SR = 1;
uint8_t DR_PPG_PW = 1;
uint8_t PPG_uart_ADCRANGE = 0;
uint8_t PPG_uart_SR = 0;
uint8_t PPG_uart_PW = 0;
uint8_t DR_If_G5V = 0;    //是否5V供电
uint8_t DR_If_G18V = 0;    //是否1.8V供电

uint8_t DIN[4] = {0X12, 0, 0, 0};   //0X12表示读取adc转换信息
uint8_t DOUT[4] =   {0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Sensor_config(void);   //  蓝牙初始化
static void BLE_Init(void);   //  蓝牙初始化

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	uint8_t PPG1buff[] = "PPG1 ERROR!"; 
	uint8_t PPG2buff[] = "PPG2 ERROR!"; 
	uint8_t ADCbuff[] = "ADC ERROR!"; 
	uint8_t MIMUbuff[] = "MIMU ERROR!"; 
	uint8_t times_30 = 0;	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_MspInit(&huart2);
	HAL_SPI_MspInit(&hspi1);
	HAL_SPI_MspInit(&hspi2);
	
	BLE_Init();    //初始化蓝牙
	
	while(!MAX_Check()){  // 检查PPG是否连接好
		HAL_UART_Transmit_DMA(&huart2, PPG1buff, sizeof(PPG1buff));   
		HAL_Delay(500);
	}
	
	while(!MAX2_Check()){  // 检查PPG是否连接好
		HAL_UART_Transmit_DMA(&huart2, PPG2buff, sizeof(PPG2buff));   
		HAL_Delay(500);
	}

	while(!ADC_check()){   //检查adc是否连接好
		HAL_UART_Transmit_DMA(&huart2, ADCbuff, sizeof(ADCbuff));   
		HAL_Delay(500);
	}
		
	while(!MIMU_check()){   //检查mimu是否连接好
		HAL_UART_Transmit_DMA(&huart2, MIMUbuff, sizeof(MIMUbuff));   
		HAL_Delay(500);
	}
	
	

	MIMU_Init();
	ADC_Init();    // 初始化ADC
	
	MAX30101_Init(); // PPG initialize
	MAX30101_2_Init(); // PPG initialize
	
	
	MAX2_WriteOneByte(0x12, 0x01);       //绿色变红色
	MAX_WriteOneByte(0x12, 0x01);       //绿色变红色
	
	HAL_TIM_Base_Start_IT(&htim16);   // enable TIM16 in interrupt mode 	必需的
	HAL_TIM_Base_Start_IT(&htim3);   // enable TIM3 in interrupt mode 	必需的
	
	ADC_1to4Voltage_flag = 0;
	
	//帧头  帧尾
	allData[0] = 0xAA;
	allData[1] = 0xBB;
	allData[26] = 0XCC;
	allData[27] = 0XDD;
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //使能IDLE中断
	//DMA接收函数，此句一定要加，不加接收不到第一次传进来的实数据，是空的，且此时接收到的数据长度为缓存器的数据长度
	HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);
	// 对于串口dma中断来说：程序运行一开始，必须先发buffer_size-1个字节的数，把它填满，之后再发多少都好使。有线串口不好使，得用蓝牙，蓝牙好使。
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{	
		if(recv_end_flag == 1) { //接收完成标志
			if( (rx_buffer[0] == 0xAB && rx_buffer[1] == 0xCD)  && 	(rx_buffer[rx_len-2] == 0xEF && rx_buffer[rx_len-1] == 0xFA)){
				Sensor_config();
			}
	
			
			HAL_UART_Transmit(&huart2, rx_buffer,rx_len,0xffff);
			memset(rx_buffer,0,rx_len);
			rx_len = 0;//清除计数
			recv_end_flag = 0;//清除接收结束标志位	
			HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
		}
		
		
		if(ADC_1to4Voltage_flag == 4){
			//这里已经转换完了4个电压
			
			allData[2] = Pulse_IIC[0] & 0x03;
			allData[3] = Pulse_IIC[1];
			allData[4] = Pulse_IIC[2];
			
			allData[5] = Pulse_IIC2[0] & 0x03;
			allData[6] = Pulse_IIC2[1];
			allData[7] = Pulse_IIC2[2];

			allData[8] = ADC_1to4Voltages[0][0];
			allData[9] = ADC_1to4Voltages[0][1];
			allData[10] = ADC_1to4Voltages[0][2];
			
			allData[11] = ADC_1to4Voltages[1][0];
			allData[12] = ADC_1to4Voltages[1][1];
			allData[13] = ADC_1to4Voltages[1][2];
			
			allData[14] = ADC_1to4Voltages[2][0];
			allData[15] = ADC_1to4Voltages[2][1];
			allData[16] = ADC_1to4Voltages[2][2];
			
			allData[17] = ADC_1to4Voltages[2][0];
			allData[18] = ADC_1to4Voltages[2][1];
			allData[19] = ADC_1to4Voltages[2][2];

			allData[20] = ACC_XYZ[1];
			allData[21] = ACC_XYZ[0];
			allData[22] = ACC_XYZ[3];
			allData[23] = ACC_XYZ[2];
			allData[24] = ACC_XYZ[5];
			allData[25] = ACC_XYZ[4];
			
//			allData[24] = GYRO_XYZ[1];
//			allData[25] = GYRO_XYZ[0];
//			allData[26] = GYRO_XYZ[3];
//			allData[27] = GYRO_XYZ[2];
//			allData[28] = GYRO_XYZ[5];
//			allData[29] = GYRO_XYZ[4];
//			allData[30] = MAG_XYZ[1];
//			allData[31] = MAG_XYZ[0];
//			allData[32] = MAG_XYZ[3];
//			allData[33] = MAG_XYZ[2];
//			allData[34] = MAG_XYZ[5];
//			allData[35] = MAG_XYZ[4];
			
			if(adc_times >= 280){

				HAL_UART_Transmit_DMA(&huart2, adcMimu, 280);     //2000Hz的每采集16个和其他数据一起发一次。
				adc_times = 0;
		//		times_2++;
			}
		
			times_30 = 0;	
			while(times_30 < 28){
				adcMimu[adc_times] = allData[times_30];
				adc_times++;
				times_30++;
			}
			
			ADC_1to4Voltage_flag = 0;

		}

          
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 蓝牙初始化
static void BLE_Init(void){	
//	uint8_t BAUD_setup[17]= "<ST_BAUD=115200>";
//	
	for(int i = 0; i < 2 ; i ++){
			HAL_UART_Transmit(&huart2, "<ST_TX_POWER=+2.5>", strlen("<ST_TX_POWER=+2.5>"), 0xffff);
			HAL_Delay(100);
	}
	
	for(int i = 0; i < 2 ; i ++){
			HAL_UART_Transmit(&huart2, "<ST_WAKE=FOREVER>", strlen("<ST_WAKE=FOREVER>"), 0xffff);
			HAL_Delay(100);
	}

	
	for(int i = 0; i < 2 ; i ++){
			HAL_UART_Transmit(&huart2, "<ST_NAME=HJ-131-WLQ>", strlen("<ST_NAME=HJ-131-WLQ>"), 0xffff);
			HAL_Delay(50);
	}
	
	for(int i = 0; i < 2 ; i ++){
			HAL_UART_Transmit(&huart2, "<ST_BAUD=921600>", strlen("<ST_BAUD=230400>"), 0xffff);
			HAL_Delay(100);
	}

	huart2.Init.BaudRate = 921600;
	if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
	

	
}




// TIM16定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		//定时器16的中断，说明要去让adc采集电压了
    if (htim == (&htim16)){
			HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);	//不选加速度和角速度 
			HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);    // ADC CS 拉高
			HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
			HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);   // START/SYNC拉低，以备下次
    }
		
		//如果是定时器3的中断,采集PPG信息
		if (htim == (&htim3)){
			if(Pulse_IIC_times == 600){
				//HAL_UART_Transmit_DMA(&huart2, Pulse_IIC_all, 1200); 
				Pulse_IIC_times = 0;
			}
			
			if(Pulse_IIC2_times == 600){
				//HAL_UART_Transmit_DMA(&huart2, Pulse_IIC2_all, 600); 
				Pulse_IIC2_times = 0;
			}
			
			MAX_ReadFIFOByte(Pulse_IIC, 3);
			MAX2_ReadFIFOByte(Pulse_IIC2, 3);
			
			Pulse_IIC_all[Pulse_IIC_times++] = Pulse_IIC[0] & 0x03;
			Pulse_IIC_all[Pulse_IIC_times++] = Pulse_IIC[1];
			Pulse_IIC_all[Pulse_IIC_times++] = Pulse_IIC[2];

			
			Pulse_IIC2_all[Pulse_IIC2_times++] = Pulse_IIC2[0] & 0x03;
			Pulse_IIC2_all[Pulse_IIC2_times++] = Pulse_IIC2[1];
			Pulse_IIC2_all[Pulse_IIC2_times++] = Pulse_IIC2[2];

		}
		
}


// DRDY下降沿中断，可以读取adc转换信息了，在这里面读取mimu的数据
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == DRDY_Pin){
		HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);    // CS拉低，选通
		ADC_RDATA();  //读取电压
		//HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);    // ADC CS 拉高 待会拉低
			
		
		if(ADC_1to4Voltage_flag != 4){
			switch(ADC_1to4Voltage_flag){
				case 0:
					ADC_1to4Voltages[0][0] = DOUT[1];
					ADC_1to4Voltages[0][1] = DOUT[2];
					ADC_1to4Voltages[0][2] = DOUT[3];				
					ADC_1to4Voltage_flag = 1;	
					HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
					ADC_WREG(ADC_MUX_REG, 0X1C);
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换		
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);   // START/SYNC拉低，以备下次
					break;
				case 1:
					ADC_1to4Voltages[1][0] = DOUT[1];
					ADC_1to4Voltages[1][1] = DOUT[2];
					ADC_1to4Voltages[1][2] = DOUT[3];		
					ADC_1to4Voltage_flag = 2;
					HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
					ADC_WREG(ADC_MUX_REG, 0X2C);
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);   // START/SYNC拉低，以备下次				
					break; 
				case 2:
					ADC_1to4Voltages[2][0] = DOUT[1];
					ADC_1to4Voltages[2][1] = DOUT[2];
					ADC_1to4Voltages[2][2] = DOUT[3];		
					ADC_1to4Voltage_flag = 3;
					HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
					ADC_WREG(ADC_MUX_REG, 0XCC);
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);   // START/SYNC拉高，开始转换
					HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);   // START/SYNC拉低，以备下次
					break; 
				case 3:
					ADC_1to4Voltages[3][0] = DOUT[1];
					ADC_1to4Voltages[3][1] = DOUT[2];
					ADC_1to4Voltages[3][2] = DOUT[3];		
					ADC_1to4Voltage_flag = 4;
					HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);
					ADC_WREG(ADC_MUX_REG, 0X0C);
					break; 
				default: 
					 ADC_1to4Voltage_flag = 5;
			}			
		}
		
		/* 读取MIMU数据 */
		if(ADC_1to4Voltage_flag == 4){
			ACC_6BytesRead();
			//GYRO_6BytesRead();  //GYRO有问题  ACC MAG没问题
			//MAG_6BytesRead();	
		}			
		
	}
}

//接收pc发的信号，进行传感器的设置
void Sensor_config(void){
	
	DR_ACC = rx_buffer[2];
	DR_PPG_COLOR = rx_buffer[3];
	DR_PPG_INTENSITY = rx_buffer[4];
	
	DR_PPG_SMPAVE = rx_buffer[5];
	DR_PPG_ADCRANGE = rx_buffer[6];
	DR_PPG_SR = rx_buffer[7];
	
	DR_PPG_PW = rx_buffer[8];
	DR_If_G5V = rx_buffer[9];
	
	//stop timer 3&16
	HAL_TIM_Base_Stop(&htim16);
	HAL_TIM_Base_Stop(&htim3);
	
	/* 加速度量程设置 */
//	switch(DR_ACC){
//		case 1:
//			ACC_GYRO_Write(CTRL_REG6_XL,0xC6);    //加速度寄存器写CTRL_REG6_XL (20h)，数据更新率952Hz，带宽100Hz，加速度范围+-2g
//			break;
//		case 2:
//			ACC_GYRO_Write(CTRL_REG6_XL,0xD6);    //加速度寄存器写CTRL_REG6_XL (20h)，数据更新率952Hz，带宽100Hz，加速度范围+-4g
//			break;
//		case 3:
//			ACC_GYRO_Write(CTRL_REG6_XL,0xDE);    //加速度寄存器写CTRL_REG6_XL (20h)，数据更新率952Hz，带宽100Hz，加速度范围+-8g
//			break;
//		default:
//			ACC_GYRO_Write(CTRL_REG6_XL,0xCE); //加速度寄存器写CTRL_REG6_XL (20h)，数据更新率952Hz，带宽100Hz，加速度范围+-16g
//	}

	/* PPG LED颜色设置 */
	switch(DR_PPG_COLOR){
		case 1: //红色  max30102
			MAX_WriteOneByte(0x11, 0x11);   //Multi-LED Mode Control Registers - SLOT1=011 其他SLOT=0
			MAX_WriteOneByte(0x12, 0x11);
			MAX2_WriteOneByte(0x11, 0x11);   //Multi-LED Mode Control Registers - SLOT1=011 其他SLOT=0
			MAX2_WriteOneByte(0x12, 0x11);
			break;
		case 2://2 红外
			MAX_WriteOneByte(0x11, 0x22);   //Multi-LED Mode Control Registers - SLOT1=001 其他SLOT=0
			MAX_WriteOneByte(0x12, 0x22);
			MAX2_WriteOneByte(0x11, 0x22);   //Multi-LED Mode Control Registers - SLOT1=001 其他SLOT=0
			MAX2_WriteOneByte(0x12, 0x22);
			break;
		default://红色
			MAX_WriteOneByte(0x11, 0x11);   //Multi-LED Mode Control Registers - SLOT1=010 其他SLOT=0
			MAX_WriteOneByte(0x12, 0x11);
			MAX2_WriteOneByte(0x11, 0x11);   //Multi-LED Mode Control Registers - SLOT1=010 其他SLOT=0
			MAX2_WriteOneByte(0x12, 0x11);
	}	
	
	/* PPG LED亮度设置 */	
	switch(DR_PPG_INTENSITY){
		case 1:
			MAX_WriteOneByte(LED1_PA_REG, 0x11);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x11);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x11);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x11);     //发光强度   green-end
			MAX2_WriteOneByte(LED1_PA_REG, 0x11);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x11);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x11);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x11);     //发光强度   green-end
			break;
		case 2:
			MAX_WriteOneByte(LED1_PA_REG, 0x31);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x31);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x31);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x31);
			MAX2_WriteOneByte(LED1_PA_REG, 0x31);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x31);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x31);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x31);
			break;
		case 3:
			MAX_WriteOneByte(LED1_PA_REG, 0x51);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x51);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x51);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x51);
			MAX2_WriteOneByte(LED1_PA_REG, 0x51);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x51);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x51);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x51);
			break;
		case 4:
			MAX_WriteOneByte(LED1_PA_REG, 0x71);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x71);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x71);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x71);
			MAX2_WriteOneByte(LED1_PA_REG, 0x71);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x71);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x71);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x71);
			break;
		case 5:
			MAX_WriteOneByte(LED1_PA_REG, 0x91);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x91);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x91);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x91);
			MAX2_WriteOneByte(LED1_PA_REG, 0x91);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x91);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x91);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x91);
			break;
		case 6:
			MAX_WriteOneByte(LED1_PA_REG, 0xB1);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0xB1);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0xB1);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0xB1);
			MAX2_WriteOneByte(LED1_PA_REG, 0xB1);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0xB1);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0xB1);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0xB1);
			break;
		case 7:
			MAX_WriteOneByte(LED1_PA_REG, 0xD1);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0xD1);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0xD1);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0xD1);
			MAX2_WriteOneByte(LED1_PA_REG, 0xD1);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0xD1);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0xD1);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0xD1);
			break;
		case 8:
			MAX_WriteOneByte(LED1_PA_REG, 0xE1);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0xE1);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0xE1);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0xE1);
			MAX2_WriteOneByte(LED1_PA_REG, 0xE1);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0xE1);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0xE1);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0xE1);
			break;
		default:
			MAX_WriteOneByte(LED1_PA_REG, 0x01);      //发光强度 red
			MAX_WriteOneByte(LED2_PA_REG, 0x01);      //发光强度  ir
			MAX_WriteOneByte(LED3_PA_REG, 0x01);      //发光强度  green-start
			MAX_WriteOneByte(LED4_PA_REG, 0x01);
			MAX2_WriteOneByte(LED1_PA_REG, 0x01);      //发光强度 red
			MAX2_WriteOneByte(LED2_PA_REG, 0x01);      //发光强度  ir
			MAX2_WriteOneByte(LED3_PA_REG, 0x01);      //发光强度  green-start
			MAX2_WriteOneByte(LED4_PA_REG, 0x01);
	}	
	
	/* PPG adc 量程 采样率 脉宽设置 */	
	switch(DR_PPG_ADCRANGE){
		case 1:
			PPG_uart_ADCRANGE = 0x00;
			break;
		case 2:
			PPG_uart_ADCRANGE = 0x20;
			break;
		case 3:
			PPG_uart_ADCRANGE = 0x40;
			break;
		default:
			PPG_uart_ADCRANGE = 0x60;
			break;
	}	
	
	switch(DR_PPG_SR){
		case 1:
			PPG_uart_SR = 0x00;    //50Hz
			break;
		case 2:
			PPG_uart_SR = 0x04;     //100
			break;
		case 3:
			PPG_uart_SR = 0x08;    //200
			break;
		case 4:
			PPG_uart_SR = 0x0C;    //400
			break;
		case 5:
			PPG_uart_SR = 0x10;    //800
			break;
		case 6:
			PPG_uart_SR = 0x14;    //1000	
			break;
			break;
		case 7:
			PPG_uart_SR = 0x18;    //1600
			break;
		default:
			PPG_uart_SR = 0x1C;    //3200
	}	
			
	
	switch(DR_PPG_PW){
		case 1:
			PPG_uart_PW = 0x00;    //69μs
			break;
		case 2:
			PPG_uart_PW = 0x01;     //118
			break;
		case 3:
			PPG_uart_PW = 0x02;    //215
			break;
		default:
			PPG_uart_PW = 0x03;   //411
	}	
	
	MAX_WriteOneByte(SPO2_CONFIG_REG, PPG_uart_ADCRANGE | PPG_uart_SR | PPG_uart_PW);  
	MAX2_WriteOneByte(SPO2_CONFIG_REG, PPG_uart_ADCRANGE | PPG_uart_SR | PPG_uart_PW);  
	
	/* PPG Sample Averaging设置 */			
	switch(DR_PPG_SMPAVE){
		case 1:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0X00|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //1
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0X00|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //1
			break;
		case 2:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0X20|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);     //2
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0X20|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);     //2
			break;
		case 3:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0X40|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //4
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0X40|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //4
			break;
		case 4:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0X60|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //8
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0X60|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);    //8
			break;
		case 5:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0X80|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);   //16
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0X80|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);   //16
			break;
		default:
			MAX_WriteOneByte(FIFO_CONFIG_REG,  0XA0|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);   //32
			MAX2_WriteOneByte(FIFO_CONFIG_REG,  0XA0|FIFO_CONFIG_REG_FIFO_ROLLOVER_EN|FIFO_CONFIG_REG_FIFO_ALL_FULL);   //32
	}	
	
	switch(DR_If_G5V){
		case 0:
			HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_RESET);	//关闭5.0V 
			break;
		default:
			HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_SET);	//开启5.0V 
	}	
	
//	switch(DR_If_G18V){
//		case 0:
//			//HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_RESET);	//关闭1.8V
//			break; 
//		default:
//			HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_SET);	//开启1.8V 
//	}	
//	
	HAL_Delay(100);
	//start timer 3&16
	HAL_TIM_Base_Start_IT(&htim16);   // enable TIM16 in interrupt mode 	必需的
	HAL_TIM_Base_Start_IT(&htim3);   // enable TIM3 in interrupt mode 	必需的
	
	uint8_t test_mes[3] = {0x12, 0x34, 0x51};
	HAL_UART_Transmit(&huart2, test_mes, 3,0xffff);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

