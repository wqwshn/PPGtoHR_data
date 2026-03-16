/* USER CODE BEGIN Header */
/**
 * @brief            : ADC + MIMU + PPG (绿光) 三合一采集程序 (完整功能版)
 * @details          :
 * 1. 恢复了所有传感器初始化和数据采集逻辑。
 * 2. 修正了初始化顺序，防止中断风暴导致死机。
 * 3. 包含了 DMA 串口发送逻辑。
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
#include "MIMU.h"
#include "MAX30101.h"
#include "MAX30101_2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 255
// 数据包: 2(头) + 8(ADC) + 3(ACC) + 3(绿光PPG 18-bit) + 1(样本计数) + 1(XOR) + 1(尾) = 19
#define PACKET_LEN 19
// PPG 数据起始索引
#define PPG_START_INDEX 13
// PPG 数据长度 (3字节绿光 + 1字节样本计数)
#define PPG_DATA_LEN 4
// XOR 校验长度: ADC(8) + ACC(3) + PPG(4) = 15 字节
#define XOR_CHECK_LEN 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 接收相关变量 */
uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_len = 0;
volatile uint8_t recv_end_flag = 0;

/* ----------------- 外部关联变量 ----------------- */
uint8_t ADC_1to4Voltage_flag = 0; // ADC采集状态机
uint8_t ACC_XYZ[6]  = {0};         // MIMU 原始数据
uint8_t GYRO_XYZ[6] = {0};
uint8_t MAG_XYZ[6]  = {0};
uint8_t DIN[4] = {0X12, 0, 0, 0};
uint8_t DOUT[4] = {0, 0, 0, 0};
/* ------------------------------------------------ */

/* 发送缓冲区 */
uint8_t allData[50] = {0};

/* ADC 相关 */
uint8_t Utop_times1 = 0;
uint8_t Utop_times2 = 5;

/* PPG 相关 */
// PPG 异步采样模式：不再使用中断缓存，主循环直接排空 FIFO

// 校验函数
uint8_t CheckXOR(uint8_t *Buf,uint8_t Len);
// PPG配置函数
static void PPG_Config_Green_Hardcoded(void);
// 蓝牙初始化函数
static void BLE_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

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
  uint8_t ADCbuff[] = "ADC ERROR!";
  uint8_t MIMUbuff[] = "MIMU ERROR!";
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // 1. 先初始化所有 MCU 内部外设
  MX_GPIO_Init();
  MX_DMA_Init();     // DMA 要早于串口/SPI
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  // TIM3 已删除 - PPG 异步采样重构  

  /* USER CODE BEGIN 2 */
  // 2. 初始化 MSP (通常由 HAL 自动调用，这里显式调用也可)
  HAL_UART_MspInit(&huart2);
  HAL_SPI_MspInit(&hspi1);
  HAL_SPI_MspInit(&hspi2);

  // 3. 开启传感器电源 (在 SPI 初始化之后再上电，避免引脚干扰)
  HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_SET);
  
  // 等待电源稳定，发送调试信息
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: Power On (5V & 1.8V)...\r\n", 32, 1000);
  HAL_Delay(500); // 增加延时确保电压稳定

  /* ====================================================================
   * 外设及通信模块初始化
   * ==================================================================== */

/* 蓝牙模块硬件复位 (高电平有效) */
//  HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);   // 拉高复位引脚
//  HAL_Delay(5);                                                      // 保持高电平至少 1ms (这里给 5ms 确保稳定)
//  HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET); // 拉低恢复正常运行状态
//  HAL_Delay(200);                                                    // 等待蓝牙模块系统启动完毕
  
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: BLE Hardware Reset OK\r\n", 30, 1000);

  /* 初始化蓝牙 */
  BLE_Init();
  HAL_Delay(200);
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: BLE Init OK\r\n", 21, 1000);

  /* ====================================================================
   * 模块在位检查 (Sensor Check Phase)
   * ==================================================================== */
  
  /* 1. MAX30101 检查 */
  if (MAX_Check() != 0) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MAX30101 Found!\r\n", 24, 100);
      HAL_Delay(100);
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: MAX30101 Check Failed!\r\n", 31, 100);
  }

  /* 2. ADC 检查 (阻塞式) */
  while (!ADC_check()) {
      HAL_UART_Transmit(&huart2, ADCbuff, sizeof(ADCbuff), 100); 
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
      HAL_Delay(500);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: ADC Found!\r\n", 19, 100);
  HAL_Delay(100);

  /* 3. MIMU 检查 (阻塞式) */
  while (!MIMU_check()) {
      HAL_UART_Transmit(&huart2, MIMUbuff, sizeof(MIMUbuff), 100);
      HAL_Delay(500);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MIMU Found!\r\n", 20, 100);
  HAL_Delay(100);
  
  /* ====================================================================
   * 模块初始化配置 (Sensor Init Phase)
   * ==================================================================== */
  
  /* 1. ADC 初始化 */
  ADC_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: ADC Init OK\r\n", 20, 1000);
  HAL_Delay(200);
    
  /* 2. MIMU 初始化 */
  MIMU_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MIMU Init OK\r\n", 21, 1000);
  HAL_Delay(200);

  /* 3. MAX30101 初始化及 PPG 配置 */
  MAX30101_Init(); 
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG Init OK\r\n", 20, 1000);
  HAL_Delay(200);

  PPG_Config_Green_Hardcoded();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG Configured.\r\n", 26, 100);

  /* ====================================================================
   * 系统启动准备
   * ==================================================================== */

  /* 初始化帧头帧尾 */
  allData[0] = 0xAA;
  allData[1] = 0xBB;

  /* 4. 启动定时器 (最后一步开启中断) */
  ADC_1to4Voltage_flag = 0;

  HAL_TIM_Base_Start_IT(&htim16); // ADC + MIMU tick
  // TIM3 已删除 - PPG 异步采样重构
  
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: System Start Loop...\r\n", 29, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 主循环逻辑：以 ADC 采集完成为基准触发发送
    if(ADC_1to4Voltage_flag == 4){

      // --- 1. 打包 ADC ---
      // 数据已由中断填充在 allData[2] ~ allData[9]

      // --- 2. 打包 ACC ---
      // 需求: 发送高位。传感器先发低后发高 -> 高位在 1, 3, 5
      allData[10] = ACC_XYZ[1]; // X High
      allData[11] = ACC_XYZ[3]; // Y High
      allData[12] = ACC_XYZ[5]; // Z High

      // --- 3. PPG FIFO 排空与均值计算 (核心逻辑) ---
      // 3.1 读取 FIFO 指针，计算可用样本数
      uint8_t wr_ptr = MAX_ReadOneByte(FIFO_WR_PTR_REG);
      uint8_t rd_ptr = MAX_ReadOneByte(RD_PTR_REG);
      uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;  // 通常为 1（硬件已平均）

      // 3.2 排空 FIFO 并累加（单绿光模式每样本3字节）
      static uint32_t last_avg_green = 0;  // 保留上次结果，防止 0 除
      uint32_t sum_green = 0;
      uint8_t buf[3];

      for (uint8_t i = 0; i < sample_count; i++) {
          MAX_ReadFIFO_Burst(buf, 3);
          // 左对齐处理：18-bit 数据在 buf[0..2] 的低 18 位
          uint32_t green_val = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
          sum_green += green_val;
      }

      // 3.3 装载至发送数组（3字节绿光18-bit数据 + 1字节样本计数）
      // 注意: sum_green 仍是累加和，上位机可自行除以 sample_count 得均值
      allData[PPG_START_INDEX]     = (sum_green >> 16) & 0xFF;  // 高字节
      allData[PPG_START_INDEX + 1] = (sum_green >> 8)  & 0xFF;  // 中字节
      allData[PPG_START_INDEX + 2] =  sum_green        & 0xFF;  // 低字节
      allData[PPG_START_INDEX + 3] = sample_count;                     // 样本计数

      // --- 4. 计算校验位 ---
      // 范围: ADC(8) + ACC(3) + PPG(4) = 15 字节
      allData[17] = CheckXOR(&allData[2], XOR_CHECK_LEN);

      // --- 5. 填充帧尾 ---
      allData[18] = 0xCC;

      // --- 6. DMA 发送 (17 字节) ---
      HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN);

      // --- 7. 清除标志位 ---
      ADC_1to4Voltage_flag = 0;
    }

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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
static void BLE_Init(void)
{
    static const uint8_t CMD_WAKE[]     = "<ST_WAKE=FOREVER>"; 
    static const uint8_t CMD_TX_POWER[] = "<ST_TX POWER=+2.5>";
    static const uint8_t CMD_NAME[]     = "<ST_NAME=HJ-131-LYX-5>";
	  static const uint8_t CMD_SECRET[]   = "<ST_SECRET=123456>";
    static const uint8_t CMD_BAUD[]     = "<ST_BAUD=115200>"; 
    static const uint8_t CMD_MIN_GAP[]  = "<ST_CON_MIN_GAP=75>"; 

    // 0. 唤醒序列：向 RX 引脚连续发送 0XAA 以中断唤醒 BLE
    uint8_t wakeup_seq[] = {0xAA, 0xAA, 0xAA, 0xAA};
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50); // 留出充裕时间等待模组完全唤醒
	
	  // 1. 锁定全速运行模式 (防睡眠)
    HAL_UART_Transmit(&huart2, CMD_WAKE, sizeof(CMD_WAKE)-1, 0xFFFF);
    HAL_Delay(50);
    
    // 2. 配置发射功率与名称
    HAL_UART_Transmit(&huart2, CMD_TX_POWER, sizeof(CMD_TX_POWER)-1, 0xFFFF);
    HAL_Delay(50);
    HAL_UART_Transmit(&huart2, CMD_NAME, sizeof(CMD_NAME)-1, 0xFFFF);
    HAL_Delay(50);

	  // 3. 设置连接密码，解禁 APP 配置权限
    HAL_UART_Transmit(&huart2, CMD_SECRET, sizeof(CMD_SECRET)-1, 0xFFFF);
    HAL_Delay(50);
	
    // 4. 发送波特率切换指令 (此时 MCU 仍在 19200bps)
    HAL_UART_Transmit(&huart2, CMD_BAUD, sizeof(CMD_BAUD)-1, 0xFFFF);
    HAL_Delay(50); // 给模组处理并反馈的时间

    // 5. MCU 切换到新波特率
    huart2.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(10); // 等待本地串口初始化稳定

    // 6. 用新的 115200bps 发送后续指令
    HAL_UART_Transmit(&huart2, CMD_MIN_GAP, sizeof(CMD_MIN_GAP)-1, 0xFFFF);
    HAL_Delay(50);

}

// 校验函数
uint8_t CheckXOR(uint8_t *Buf,uint8_t Len)
{
  uint8_t i =0;
  uint8_t x =0;
  for(i=0; i<Len; i++)
  {
    x = x^(*(Buf+i));
  }
  return x;
}

// PPG 硬编码配置 - 高性能单绿光模式 (1000Hz + 8次硬件平均 = 125Hz)
static void PPG_Config_Green_Hardcoded(void) {
    // 1. Mode Configuration (0x09) = 0x07: Multi-LED 模式
    //    (HR模式仅支持红光，单绿光必须使用Multi-LED模式)
    MAX_WriteOneByte(MODE_CONFIG_REG, 0x07);

    // 2. LED_CONTROL1 (0x11) = 0x03: SLOT1=LED3(Green), SLOT2禁用
    MAX_WriteOneByte(LED_CONTROL1, 0x03);

    // 3. LED_CONTROL2 (0x12) = 0x00: 禁用 SLOT3 和 SLOT4
    MAX_WriteOneByte(LED_CONTROL2, 0x00);

    // 4. LED3_PA_REG (0x0E) = 0x5F: 绿光亮度约 19mA
    MAX_WriteOneByte(LED3_PA_REG, 0x71);

    // 5. SPO2_CONFIG_REG (0x0A) = 0x77
    //    Bit7-5: 011 = 16384nA 量程
    //    Bit4-2: 111 = 1000sps 内部高频采样
    //    Bit1-0: 11 = 411us/18-bit 最长脉宽
    MAX_WriteOneByte(SPO2_CONFIG_REG, 0x77);

    // 6. FIFO_CONFIG_REG (0x08) = 0x5F
    //    Bit6: 1 (Sample Average = 4次硬件平均)
    //    Bit5: 1 (保留)
    //    Bit4: 1 (FIFO Rollover 使能)
    //    Bit0-3: 1111 (FIFO_A_FULL = 15)
    MAX_WriteOneByte(FIFO_CONFIG_REG, 0x5F);

    // 7. 清零 FIFO 指针
    MAX_WriteOneByte(FIFO_WR_PTR_REG, 0x00); // 写指针
    MAX_WriteOneByte(OVF_COUNTER_REG, 0x00); // 溢出计数
    MAX_WriteOneByte(RD_PTR_REG, 0x00);      // 读指针
}

/* ==============================================================================
 * 中断回调函数 - 已解放！
 * ============================================================================== */

// 定时器中断处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM16: 触发 ADC 采集 (节拍器)
    if (htim == (&htim16)){
        HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
        // 注意：确保 SPI1_NSS_Pin 在 MX_GPIO_Init 中已配置为 Output
        HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
    }
    // TIM3 已删除 - PPG 异步采样重构
}

// GPIO外部中断：读取 ADC 和 MIMU 数据
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == DRDY_Pin){
        // 此时 SPI 已经初始化完毕，可以安全调用
        HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);

        if(ADC_1to4Voltage_flag == 0){
            if(Utop_times1 != 10){
                if(Utop_times2 == 10) ADC_1to4Voltage_flag = 1;
                else                  ADC_1to4Voltage_flag = 2;
            }
        }

        if(ADC_1to4Voltage_flag != 4){
            switch(ADC_1to4Voltage_flag){
                case 0:
                    ADC_RDATA(&allData[8]);  // 填入桥中1
                    if(Utop_times2 == 10){
                        ADC_WREG(ADC_MUX_REG, 0X3C);
                        ADC_1to4Voltage_flag = 1;
                    } else {
                        ADC_WREG(ADC_MUX_REG, 0X0C);
                        ADC_1to4Voltage_flag = 2;
                    }
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    Utop_times1 = 0;
                    break;
                case 1:
                    ADC_RDATA(&allData[6]);  // 填入桥中2
                    ADC_WREG(ADC_MUX_REG, 0X0C);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    Utop_times2 = 0;
                    ADC_1to4Voltage_flag = 2;
                    break;
                case 2:
                    ADC_RDATA(&allData[4]);  // 填入桥顶1
                    ADC_1to4Voltage_flag = 3;
                    ADC_WREG(ADC_MUX_REG, 0X2C);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    break;
                case 3:
                    ADC_RDATA(&allData[2]);  // 填入桥顶2
                    ADC_1to4Voltage_flag = 4; // 标志一轮 ADC 完成！(触发 Main 发送)
                    Utop_times1++;
                    Utop_times2++;
                    if(Utop_times1 == 10)      ADC_WREG(ADC_MUX_REG, 0X1C);
                    else if(Utop_times2 == 10) ADC_WREG(ADC_MUX_REG, 0X3C);
                    else                       ADC_WREG(ADC_MUX_REG, 0X0C);
                    break;
                default:
                    ADC_1to4Voltage_flag = 5;
            }
        }

        // 读取 MIMU 数据
        if(ADC_1to4Voltage_flag == 4)
            ACC_6BytesRead();
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}