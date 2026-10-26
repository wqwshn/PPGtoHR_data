/* USER CODE BEGIN Header */
/**
 * @brief            : ADC + MIMU + PPG 双模式采集程序 (支持心率/血氧切换)
 * @details          :
 * 1. 通过修改 main.c 顶部配置开关切换工作模式和数据发送
 * 2. MAX30101使用FIFO异步采样模式，主循环批量读取并计算均值
 * 3. 心率模式：绿光单LED，血氧模式：红光+红外双LED
 * 4. 支持 1Hz HR 结果包和可选的原始数据包 (采样率由 sample_rate_config.h 配置)
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
#include <stdio.h>
#include "ADC_ADS124.h"
#include "MIMU.h"
#include "MAX30101.h"
#include "MAX30101_2.h"
#include "ppg_channel.h"
#include "hr_algorithm.h"
#include "sample_rate_config.h"
#include "board_check.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 255

/* 注意: 所有系统配置开关 (工作模式/数据发送/PPG通道/采样率)
 *       已统一移至 main.h 的 USER CODE 区域, 请在该文件修改 */

// PPG / GYRO / Raw序号偏移已在 main.h 中统一定义
// PPG_START_INDEX = 22, GYRO_START_INDEX = 16, RAW_SEQUENCE_START_INDEX = 31
#define PPG_AVG_FRAC_BITS 4U
#define PPG_AVG_SCALE     (1U << PPG_AVG_FRAC_BITS)
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
static uint8_t statusData[STATUS_PACKET_LEN] = {0};
static uint16_t raw_packet_seq = 0;
#if BLE_BATCH5
#include "raw_batch.h"
static RawBatch raw_batch;
#endif
static volatile uint8_t raw_diag_status_pending = 0;
static volatile uint8_t raw_diag_dma_tx_kind = 0;
static volatile uint32_t raw_diag_sample_counter = 0;
static volatile uint32_t raw_diag_adc_drdy_counter = 0;
static volatile uint32_t raw_diag_frame_counter = 0;
static volatile uint32_t raw_diag_tx_start_counter = 0;
static volatile uint32_t raw_diag_tx_done_counter = 0;
static volatile uint32_t raw_diag_tx_busy_counter = 0;
static volatile uint32_t raw_diag_tx_error_counter = 0;
static volatile uint32_t raw_diag_adc_error_counter = 0;
static volatile uint32_t raw_diag_imu_error_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_empty_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_overflow_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_sample_total_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_nonempty_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_single_sample_counter = 0;
static volatile uint32_t raw_diag_ppg_fifo_multi_sample_counter = 0;
/* 主循环初始化/重试期间禁止采集中断访问SPI2。 */
static volatile uint8_t mimu_ready = 0;
static volatile uint8_t mimu_state = MIMU_STATE_INITIALIZING;
#if ENABLE_MIMU_RECOVERY
static uint8_t mimu_recovery_warming = 0;
static uint32_t mimu_last_retry_ms = 0;
#endif

#include "ble_experiment_transport.h"
#include "ble_power_transport.h"

/* ADC 相关 */
uint8_t Utop_times1 = 0;
uint8_t Utop_times2 = 5;

/* PPG 相关变量 */
#if (CURRENT_WORK_MODE == MODE_SPO2)
// 血氧模式变量
static uint32_t last_red_avg_q4 = 0;
static uint32_t last_ir_avg_q4 = 0;
#else
// 心率模式无额外缓存变量 (sum_green 直接计算, 无需缓存上次均值)
#endif

/* 温度补偿相关 - 1Hz非阻塞测温状态机 (仅 SpO2 模式使用) */
#if (CURRENT_WORK_MODE == MODE_SPO2)
static uint8_t die_temp_int = 0;
static uint8_t die_temp_frac = 0;
#endif

/* --- 在线心率算法相关 --- */
static HR_Config_t hr_config;
static HR_State_t  hr_state;
static uint8_t algorithm_initialized = 0;

// 校验函数
uint8_t CheckXOR(uint8_t *Buf, uint8_t Len);
static void WriteU32BE(uint8_t *dst, uint32_t value);
static void BuildStatusFrame(void);
static void Acquisition_DebugPoll(void);
static void Acquisition_DebugADC(const char *phase);
static void MIMU_Startup(void);
static void MIMU_PollRecovery(void);
#if (PPG_DEFAULT_CHANNEL != 0)
static uint8_t Acquisition_DebugPPG(uint8_t configured);
#endif

// PPG配置函数
#if (CURRENT_WORK_MODE == MODE_SPO2)
static void PPG_Config_SpO2_Hardcoded(void);
#endif
static void PPG_Config_Green_Hardcoded(void);

#if (ENABLE_BLE_CONFIG && !BLE_RF_DISABLED)
static void BLE_Init(void);
static void BLE_ResetModule(void);
static void BLE_SendConfigCommand(const uint8_t *cmd, uint16_t len);
static void BLE_SetUartBaud(uint32_t baud_rate);
#endif
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
  char ADCbuff[] = "ADC ERROR!";
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
#if ENABLE_BOARD_CHECK
#if ENABLE_BLE_CONFIG
  /* Reuse the acquisition firmware's hardware-verified first-board setup. */
  HAL_Delay(500);
  {
      static const uint8_t note[] = "BOARD CHECK: BLE factory/project setup starting; verify after setup.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)note, sizeof(note) - 1, 1000);
  }
  BLE_Init();
#endif
  BoardCheck_Run();
  while (1) {
      HAL_Delay(5000);
      BoardCheck_PrintSummary();
  }
#endif
  // 2. 初始化 MSP (通常由 HAL 自动调用，这里显式调用也可)
  HAL_UART_MspInit(&huart2);
  HAL_SPI_MspInit(&hspi1);
  HAL_SPI_MspInit(&hspi2);

  // 3. 开启传感器电源 (在 SPI 初始化之后再上电，避免引脚干扰)
  HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_SET);

  // 等待电源稳定，发送调试信息
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: Power On (5V & 1.8V)...\r\n", 32, 1000);
  HAL_Delay(500);

  /* ====================================================================
   * 蓝牙模块初始化
   * ==================================================================== */
#if (ENABLE_BLE_CONFIG && !BLE_RF_DISABLED)
  BLE_Init();
  HAL_Delay(200);
  {
      static const uint8_t msg[] = "DEBUG: BLE Config OK\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 1000);
  }
#else
  {
#if BLE_RF_DISABLED
      static const uint8_t msg[] = "DEBUG: BLE RF OFF - hardware reset held HIGH\r\n";
#else
      static const uint8_t msg[] = "DEBUG: BLE RF ON - existing module config\r\n";
#endif
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 1000);
  }
#endif

  /* ====================================================================
   * 模块检测阶段 (Sensor Check Phase)
   * ==================================================================== */

  /* 1. MAX30101 检测 (通过通道接口) */
#if BLE_FIXED_MINUS10 || BLE_BATCH5 || BLE_SINGLE25
  /* Reset/release BLE before configuring, prior to ADC/Raw streaming.
   * Hardware reset preserves settings; no factory reset or readback. */
  {
      static const uint8_t wake[] = {0xAA,0xAA,0xAA,0xAA};
#if BLE_BATCH5 || BLE_SINGLE25
      static const uint8_t power[] = "<ST_TX_POWER=2.5>";
#if BLE_SINGLE25
      static const uint8_t note[] = "DEBUG: BLE target +2.5 dBm UNVERIFIED; RAW single frame / 10 ms\r\n";
#else
      static const uint8_t note[] = "DEBUG: BLE target +2.5 dBm UNVERIFIED; RAW batch 5 / 50 ms\r\n";
#endif
#else
      static const uint8_t power[] = "<ST_TX_POWER=-10>";
      static const uint8_t note[] = "DEBUG: BLE target -10 dBm written; UNVERIFIED\r\n";
#endif
      HAL_GPIO_WritePin(BLE_RST_GPIO_Port,BLE_RST_Pin,GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(BLE_RST_GPIO_Port,BLE_RST_Pin,GPIO_PIN_RESET);
      HAL_Delay(300);
      if (HAL_UART_Transmit(&huart2,(uint8_t *)wake,sizeof(wake),100)!=HAL_OK) Error_Handler();
      HAL_Delay(2);
      if (HAL_UART_Transmit(&huart2,(uint8_t *)power,sizeof(power)-1,100)!=HAL_OK) Error_Handler();
      HAL_Delay(100);
      HAL_UART_Transmit(&huart2,(uint8_t *)note,sizeof(note)-1,100);
  }
#endif
  PPG_SetChannel(PPG_DEFAULT_CHANNEL);
#if (PPG_DEFAULT_CHANNEL != 0)
  if (Acquisition_DebugPPG(0)) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MAX30101 Found!\r\n", 24, 100);
      HAL_Delay(100);
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: MAX30101 Check Failed!\r\n", 31, 100);
  }

#endif /* PPG enabled */

  /* 2. ADC 检测 (阻塞式) */
  if (ENABLE_ACQUISITION_DEBUG) Acquisition_DebugADC("pre-init");
  while (!ADC_check()) {
      HAL_UART_Transmit(&huart2, (uint8_t*)ADCbuff, sizeof(ADCbuff), 100);
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
      HAL_Delay(500);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: ADC Found!\r\n", 19, 100);
  HAL_Delay(100);

  /* ====================================================================
   * 模块初始化配置 (Sensor Init Phase)
   * ==================================================================== */

  /* 1. ADC 初始化 */
  ADC_Init();
  {
      static const uint8_t note[] = "[DEBUG-acq] ADC init writes done; verification pending\r\n";
      if (ENABLE_ACQUISITION_DEBUG) HAL_UART_Transmit(&huart2, (uint8_t *)note, sizeof(note) - 1, 1000);
  }
  Acquisition_DebugADC("post-init");
  HAL_Delay(200);

  /* 2. MIMU 初始化 */
  MIMU_Startup();

#if (PPG_DEFAULT_CHANNEL != 0)
  /* 3. MAX30101 初始化 (通过通道接口) */
  PPG_Init();
  {
      static const uint8_t note[] = "[DEBUG-acq] PPG init writes done; verification pending\r\n";
      if (ENABLE_ACQUISITION_DEBUG) HAL_UART_Transmit(&huart2, (uint8_t *)note, sizeof(note) - 1, 1000);
  }
  HAL_Delay(200);

  /* 4. 根据当前模式配置 PPG 参数 */
#if (CURRENT_WORK_MODE == MODE_SPO2)
  PPG_Config_SpO2_Hardcoded();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG SpO2 Mode Config.\r\n", 30, 100);
#else
  PPG_Config_Green_Hardcoded();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG HR Mode Config.\r\n", 28, 100);
#endif

  /* STOP 后留出空闲时间，再检查 MCU 是否释放及引脚实际电平。 */
  HAL_Delay(2);
  Acquisition_DebugPPG(1);
#endif /* PPG enabled */

  /* ====================================================================
   * 心率在线算法初始化 (仅在线心率模式)
   * ==================================================================== */
#if (!ENABLE_RAW_DATA_PACKET)
  HR_GetDefaultConfig(&hr_config);
  HR_Init(&hr_config, &hr_state);
  algorithm_initialized = 1;
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: HR Algorithm Init OK\r\n", 29, 1000);
#endif

  /* ====================================================================
   * 系统准备就绪
   * ==================================================================== */

  /* 初始化帧头帧尾 */
  allData[0] = 0xAA;
  allData[1] = 0xBB;

  /* 4. 启动定时器 (最后一步开启中断) */
  ADC_1to4Voltage_flag = 0;

#if BLE_RF_EXPERIMENT || BLE_POWER_EXPERIMENT
  RF_Start();
#endif
  {
      HAL_StatusTypeDef timer_status = HAL_TIM_Base_Start_IT(&htim16);
      char line[128];
      snprintf(line, sizeof(line), "[SYSTEM] TIM16 start=%u PPG=%u rate=%u raw=%u mode=%u; UART=115200 8N1\r\n",
               (unsigned)timer_status, PPG_DEFAULT_CHANNEL, PPG_SAMPLE_RATE,
               ENABLE_RAW_DATA_PACKET, CURRENT_WORK_MODE);
      HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MIMU_PollRecovery();
    Acquisition_DebugPoll();
#if BLE_RF_EXPERIMENT || BLE_POWER_EXPERIMENT
    RF_Poll();
#endif
#if (!ENABLE_RAW_DATA_PACKET)
    /* ---- 在线心率模式: 每秒执行一次解算并发送 HR 结果包 ---- */
    if (algorithm_initialized && hr_state.flag_1s_ready) {
        float bpm = HR_RunSolver(&hr_state);
        if (bpm > 0) {
            /* 构建心率结果数据包 (31 字节) */
            uint8_t hr_packet[31];
            hr_packet[0] = 0xAA;       /* 帧头 */
            hr_packet[1] = 0xCC;

            /* 融合心率 (x10, 0.1 BPM 精度) */
            uint16_t hr_x10 = (uint16_t)(bpm * 10.0f + 0.5f);
            hr_packet[2] = (hr_x10 >> 8) & 0xFF;
            hr_packet[3] = hr_x10 & 0xFF;

            /* 运动标志 */
            hr_packet[4] = hr_state.is_motion;

            /* 窗口填充状态 */
            hr_packet[5] = hr_state.win_filled;

            /* LMS-HF 路径 BPM (x10) */
            uint16_t hf_x10 = (uint16_t)(hr_state.hr_lms_hf * 600.0f + 0.5f);
            hr_packet[6] = (hf_x10 >> 8) & 0xFF;
            hr_packet[7] = hf_x10 & 0xFF;

            /* LMS-ACC 路径 BPM (x10) */
            uint16_t acc_x10 = (uint16_t)(hr_state.hr_lms_acc * 600.0f + 0.5f);
            hr_packet[8] = (acc_x10 >> 8) & 0xFF;
            hr_packet[9] = acc_x10 & 0xFF;

            /* FFT 路径 BPM (x10) */
            uint16_t fft_x10 = (uint16_t)(hr_state.hr_fft * 600.0f + 0.5f);
            hr_packet[10] = (fft_x10 >> 8) & 0xFF;
            hr_packet[11] = fft_x10 & 0xFF;

            /* PPG 信号均值 */
            float ppg_mean = 0;
            for (int i = 0; i < HR_STEP_SAMPLES; i++) ppg_mean += hr_state.buf_1s_ppg[i];
            ppg_mean /= HR_STEP_SAMPLES;
            uint16_t ppg_u16 = (uint16_t)(ppg_mean + 0.5f);
            hr_packet[12] = (ppg_u16 >> 8) & 0xFF;
            hr_packet[13] = ppg_u16 & 0xFF;

            /* 运动校准状态 */
            hr_packet[14] = hr_state.motion_calibrated;

            /* 时间戳 (秒, 低16位) */
            static uint16_t ts = 0;
            ts++;
            hr_packet[15] = (ts >> 8) & 0xFF;
            hr_packet[16] = ts & 0xFF;

            /* 校准进度 (0-8) */
            hr_packet[17] = (hr_state.calib_windows_done < 8) ? hr_state.calib_windows_done : 8;

            /* 采样率 (Hz) - 由算法头文件定义 */
            hr_packet[18] = HR_FS;

            /* HF1 AC 幅值 (x100 mV, LSB std -> mV) -- 桥顶1 */
            float hf1_mv = hr_state.hf1_signal_std * HR_HF_LSB_TO_MV;
            uint16_t hf1_mv_x100 = (uint16_t)(hf1_mv * 100.0f + 0.5f);
            hr_packet[19] = (hf1_mv_x100 >> 8) & 0xFF;
            hr_packet[20] = hf1_mv_x100 & 0xFF;

            /* HF1-PPG 相关系数 (x10000, signed) */
            int16_t hf1_corr_x10000 = (int16_t)(hr_state.hf1_ppg_corr * 10000.0f);
            hr_packet[21] = (uint8_t)((hf1_corr_x10000 >> 8) & 0xFF);
            hr_packet[22] = (uint8_t)(hf1_corr_x10000 & 0xFF);

            /* ACC-PPG 相关系数 (x10000, signed, 最优轴) */
            int16_t acc_corr_x10000 = (int16_t)(hr_state.acc_ppg_corr * 10000.0f);
            hr_packet[23] = (uint8_t)((acc_corr_x10000 >> 8) & 0xFF);
            hr_packet[24] = (uint8_t)(acc_corr_x10000 & 0xFF);

            /* HF2 AC 幅值 (x100 mV) -- 桥顶2 */
            float hf2_mv = hr_state.hf2_signal_std * HR_HF_LSB_TO_MV;
            uint16_t hf2_mv_x100 = (uint16_t)(hf2_mv * 100.0f + 0.5f);
            hr_packet[25] = (hf2_mv_x100 >> 8) & 0xFF;
            hr_packet[26] = hf2_mv_x100 & 0xFF;

            /* HF2-PPG 相关系数 (x10000, signed) */
            int16_t hf2_corr_x10000 = (int16_t)(hr_state.hf2_ppg_corr * 10000.0f);
            hr_packet[27] = (uint8_t)((hf2_corr_x10000 >> 8) & 0xFF);
            hr_packet[28] = (uint8_t)(hf2_corr_x10000 & 0xFF);

            /* XOR 校验 (覆盖 bytes[2..28]) */
            uint8_t xor_val = 0;
            for (int i = 2; i < 29; i++) xor_val ^= hr_packet[i];
            hr_packet[29] = xor_val;

            /* 帧尾 */
            hr_packet[30] = 0xCC;

            HAL_UART_Transmit_DMA(&huart2, hr_packet, 31);
        }
    }
#endif /* !ENABLE_RAW_DATA_PACKET */

    /* ---- 数据采集: ADC 完成后触发 ---- */
    if(ADC_1to4Voltage_flag == 4){

      /* --- 1. 打包 ADC (保留原逻辑) --- */
      // 数据已由中断填充在 allData[2] ~ allData[9]

      /* --- 2. 打包 ACC 完整 16 位 (X/Y/Z 各2字节, 大端, Z轴取反) --- */
      allData[10] = ACC_XYZ[1];
      allData[11] = ACC_XYZ[0];
      allData[12] = ACC_XYZ[3];
      allData[13] = ACC_XYZ[2];
      {
          int16_t az = -(int16_t)((ACC_XYZ[5] << 8) | ACC_XYZ[4]);
          allData[14] = (uint8_t)(az >> 8);
          allData[15] = (uint8_t)(az & 0xFF);
      }

      /* --- 2.5 打包 GYRO 完整 16 位 (扣除零偏) --- */
      {
          int16_t gx = (int16_t)((GYRO_XYZ[1] << 8) | GYRO_XYZ[0]) - gyro_offset[0];
          int16_t gy = (int16_t)((GYRO_XYZ[3] << 8) | GYRO_XYZ[2]) - gyro_offset[1];
          int16_t gz = (int16_t)((GYRO_XYZ[5] << 8) | GYRO_XYZ[4]) - gyro_offset[2];
          allData[GYRO_START_INDEX]     = (uint8_t)(gx >> 8);
          allData[GYRO_START_INDEX + 1] = (uint8_t)(gx & 0xFF);
          allData[GYRO_START_INDEX + 2] = (uint8_t)(gy >> 8);
          allData[GYRO_START_INDEX + 3] = (uint8_t)(gy & 0xFF);
          allData[GYRO_START_INDEX + 4] = (uint8_t)(gz >> 8);
          allData[GYRO_START_INDEX + 5] = (uint8_t)(gz & 0xFF);
      }

      /* --- 3. PPG 数据采集；禁用时固定填充9字节零，不访问FIFO --- */
#if (PPG_DEFAULT_CHANNEL == 0)
      memset(&allData[PPG_START_INDEX], 0, 9);
#else
      uint8_t wr_ptr = PPG_ReadOneByte(FIFO_WR_PTR_REG);
      uint8_t rd_ptr = PPG_ReadOneByte(FIFO_RD_PTR_REG);
      uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;
      if (sample_count == 0) {
          raw_diag_ppg_fifo_empty_counter++;
      } else {
          raw_diag_ppg_fifo_sample_total_counter += sample_count;
          raw_diag_ppg_fifo_nonempty_counter++;
          if (sample_count == 1U) {
              raw_diag_ppg_fifo_single_sample_counter++;
          } else {
              raw_diag_ppg_fifo_multi_sample_counter++;
          }
          if (sample_count >= 31U) {
              raw_diag_ppg_fifo_overflow_counter++;
          }
      }

#if (CURRENT_WORK_MODE == MODE_SPO2)
      /* --- 3.1 血氧模式: Red+IR Q4 均值采集 --- */
      uint32_t sum_red = 0, sum_ir = 0;
      uint8_t buf[6];  /* Red 3B + IR 3B, 18-bit 左对齐 */

      for (uint8_t i = 0; i < sample_count; i++) {
          PPG_ReadFIFO_Burst(buf, 6);
          uint32_t red_val = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
          uint32_t ir_val = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) & 0x03FFFF;
          sum_red += red_val;
          sum_ir += ir_val;
      }

      if (sample_count > 0) {
          last_red_avg_q4 =
              ((sum_red << PPG_AVG_FRAC_BITS) + (sample_count / 2U)) / sample_count;
          last_ir_avg_q4 =
              ((sum_ir << PPG_AVG_FRAC_BITS) + (sample_count / 2U)) / sample_count;
      }

      /* 打包与心率模式统一布局: Green=0, Red Q4, IR Q4 (上位机无需改动) */
      allData[PPG_START_INDEX + 0] = 0x00;  /* Green 高位=0 */
      allData[PPG_START_INDEX + 1] = 0x00;  /* Green 中位=0 */
      allData[PPG_START_INDEX + 2] = 0x00;  /* Green 低位=0 */
      allData[PPG_START_INDEX + 3] = (last_red_avg_q4 >> 16) & 0xFF;
      allData[PPG_START_INDEX + 4] = (last_red_avg_q4 >> 8)  & 0xFF;
      allData[PPG_START_INDEX + 5] = last_red_avg_q4         & 0xFF;
      allData[PPG_START_INDEX + 6] = (last_ir_avg_q4 >> 16)  & 0xFF;
      allData[PPG_START_INDEX + 7] = (last_ir_avg_q4 >> 8)   & 0xFF;
      allData[PPG_START_INDEX + 8] = last_ir_avg_q4          & 0xFF;

#else
      /* --- 3.3 心率模式: 三通道 PPG 采集 (Green+Red+IR) --- */
      uint32_t sum_green = 0, sum_red = 0, sum_ir = 0;
      uint8_t buf[MAX30101_FIFO_SAMPLE_BYTES]; /* 9 字节 */

      /* 缓存上次有效均值, 防止 FIFO 空读时跳变到 0 */
      static uint32_t last_green_avg_q4 = 0;
      static uint32_t last_red_avg_q4 = 0;
      static uint32_t last_ir_avg_q4 = 0;

      for (uint8_t i = 0; i < sample_count; i++) {
          PPG_ReadFIFO_Burst(buf, MAX30101_FIFO_SAMPLE_BYTES);
          /* PPG有效位宽由 sample_rate_config.h 控制: 17-bit右移1位, 18-bit不右移 */
          uint32_t raw;
          raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
          sum_green += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
          raw = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
          sum_red += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
          raw = ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 8) | buf[8];
          sum_ir += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
      }

      if (sample_count > 0) {
          last_green_avg_q4 =
              ((sum_green << PPG_AVG_FRAC_BITS) + (sample_count / 2U)) / sample_count;
          last_red_avg_q4 =
              ((sum_red << PPG_AVG_FRAC_BITS) + (sample_count / 2U)) / sample_count;
          last_ir_avg_q4 =
              ((sum_ir << PPG_AVG_FRAC_BITS) + (sample_count / 2U)) / sample_count;
      }

      /* 打包 PPG 三通道 Q4 平均值 (各3字节, 高位在前) */
      allData[PPG_START_INDEX]     = (last_green_avg_q4 >> 16) & 0xFF;
      allData[PPG_START_INDEX + 1] = (last_green_avg_q4 >> 8)  & 0xFF;
      allData[PPG_START_INDEX + 2] =  last_green_avg_q4        & 0xFF;
      allData[PPG_START_INDEX + 3] = (last_red_avg_q4 >> 16)   & 0xFF;
      allData[PPG_START_INDEX + 4] = (last_red_avg_q4 >> 8)    & 0xFF;
      allData[PPG_START_INDEX + 5] =  last_red_avg_q4          & 0xFF;
      allData[PPG_START_INDEX + 6] = (last_ir_avg_q4 >> 16)    & 0xFF;
      allData[PPG_START_INDEX + 7] = (last_ir_avg_q4 >> 8)     & 0xFF;
      allData[PPG_START_INDEX + 8] =  last_ir_avg_q4           & 0xFF;

      /* 算法数据推送 (仅在线心率模式, 暂用绿光通道) */
#if (!ENABLE_RAW_DATA_PACKET)
      if (algorithm_initialized && sample_count > 0) {
          float ppg_val = (float)sum_green / (float)sample_count;

          /* ACC 完整 16 位数据 */
          int16_t ax_raw = (int16_t)((ACC_XYZ[1] << 8) | ACC_XYZ[0]);
          int16_t ay_raw = (int16_t)((ACC_XYZ[3] << 8) | ACC_XYZ[2]);
          int16_t az_raw = (int16_t)((ACC_XYZ[5] << 8) | ACC_XYZ[4]);

          /* HF1/ADC: 桥顶1 (allData[4..5]) */
          int16_t hf1_raw = (int16_t)((allData[4] << 8) | allData[5]);
          /* HF2/ADC: 桥顶2 (allData[2..3]) */
          int16_t hf2_raw = (int16_t)((allData[2] << 8) | allData[3]);

          HR_PushSample(&hr_state, ppg_val,
                        (float)ax_raw, (float)ay_raw, (float)az_raw,
                        (float)hf1_raw, (float)hf2_raw);
      }
#endif /* !ENABLE_RAW_DATA_PACKET */
#endif

#endif /* PPG enabled */

      /* --- 4. Raw采样序号 (bytes[31..32], 固件侧采样周期) --- */
      allData[RAW_SEQUENCE_START_INDEX] = (uint8_t)(raw_packet_seq >> 8);
      allData[RAW_SEQUENCE_START_INDEX + 1] = (uint8_t)(raw_packet_seq & 0xFF);

      /* --- 5. 校验位 (bytes[2..32], 共31字节) --- */
      allData[33] = CheckXOR(&allData[2], XOR_CHECK_LEN);

      /* --- 6. 帧尾 --- */
      allData[34] = 0xCC;

      /* --- 7. DMA 发送 (35 字节) --- */
#if (ENABLE_RAW_DATA_PACKET)
      raw_diag_frame_counter++;
#if BLE_BATCH5
      /* Short critical section: serialize UART completion and batch ownership.
       * DMA continues during subsequent acquisition; never transmit allData. */
      uint32_t irq_mask = __get_PRIMASK();
      __disable_irq();
      int batch_ready = RawBatch_Push(&raw_batch, allData, huart2.gState == HAL_UART_STATE_READY);
      if (batch_ready == 1) {
          raw_diag_dma_tx_kind = 1;
          HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&huart2, raw_batch.tx, sizeof(raw_batch.tx));
          if (result == HAL_OK) raw_diag_tx_start_counter += 5;
          else {
              raw_diag_dma_tx_kind = 0;
              if (result == HAL_BUSY) raw_diag_tx_busy_counter += 5;
              else raw_diag_tx_error_counter += 5;
          }
      } else if (batch_ready < 0) raw_diag_tx_busy_counter += 5;
      __set_PRIMASK(irq_mask);
#else
      HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN);
      if (tx_status == HAL_OK) {
          raw_diag_tx_start_counter++;
          raw_diag_dma_tx_kind = 1;
      } else if (tx_status == HAL_BUSY) {
          raw_diag_tx_busy_counter++;
      } else {
          raw_diag_tx_error_counter++;
      }
#endif /* BLE_BATCH5 */
      raw_packet_seq++;
#endif

      /* --- 8. 清除标志位 --- */
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

// 蓝牙配置。当前硬件调试口与 BLE 共用 UART, 不在固件内读回应答。
#if (ENABLE_BLE_CONFIG && !BLE_RF_DISABLED)
static void BLE_Init(void)
{
    static const uint8_t CMD_FACTORY[]     = "<ST_FACTORY>";
    static const uint8_t CMD_CLEAR_SECRET[] = "<ST_CLEAR_SECRET>";
    static const uint8_t CMD_WAKE[]        = "<ST_WAKE=FOREVER>";
    static const uint8_t CMD_TX_POWER[]    = "<ST_TX_POWER=+2.5>";
    static const uint8_t CMD_NAME[]        = "<ST_NAME=HJ-131-LYX>";
    static const uint8_t CMD_OWN_MAC[]     = "<ST_OWN_MAC=" BLE_CUSTOM_MAC ">";
    static const uint8_t CMD_MIN_GAP[]     = "<ST_CON_MIN_GAP=75>";
    static const uint8_t CMD_BAUD[]        = "<ST_BAUD=115200>";

    BLE_ResetModule();

    // 0. 发送唤醒序列。若模块已是 FOREVER 模式，该步骤无副作用。
    uint8_t wakeup_seq[] = {0xAA, 0xAA, 0xAA, 0xAA};
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_FACTORY, sizeof(CMD_FACTORY) - 1);
    HAL_Delay(500);
    BLE_SetUartBaud(19200);
    BLE_ResetModule();
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_FACTORY, sizeof(CMD_FACTORY) - 1);
    HAL_Delay(500);
    BLE_ResetModule();
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_CLEAR_SECRET, sizeof(CMD_CLEAR_SECRET) - 1);
    BLE_SendConfigCommand(CMD_WAKE, sizeof(CMD_WAKE) - 1);
    BLE_SendConfigCommand(CMD_TX_POWER, sizeof(CMD_TX_POWER) - 1);
    BLE_SendConfigCommand(CMD_NAME, sizeof(CMD_NAME) - 1);
    BLE_SendConfigCommand(CMD_MIN_GAP, sizeof(CMD_MIN_GAP) - 1);
    /* 设置自定义 MAC 地址: 模块自动重启, 耗时约 500ms */
    BLE_SendConfigCommand(CMD_OWN_MAC, sizeof(CMD_OWN_MAC) - 1);
    HAL_Delay(600);
    /* MAC 设置后模块重启, 波特率可能回到默认 19200, 重新发送唤醒序列 */
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);
    /* 波特率设置必须在最后 (MAC 重启可能恢复默认波特率) */
    BLE_SendConfigCommand(CMD_BAUD, sizeof(CMD_BAUD) - 1);
    HAL_Delay(100);
    BLE_SetUartBaud(115200);
}

static void BLE_ResetModule(void)
{
    HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);
}

static void BLE_SendConfigCommand(const uint8_t *cmd, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, len, 1000);
    HAL_Delay(50);
}

// 校验函数
// BLE factory reset falls back to 19200bps; switch MCU UART before continuing config.
static void BLE_SetUartBaud(uint32_t baud_rate)
{
    HAL_UART_DeInit(&huart2);
    huart2.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

#endif /* ENABLE_BLE_CONFIG */

// XOR check helper.
uint8_t CheckXOR(uint8_t *Buf, uint8_t Len)
{
  uint8_t i = 0;
  uint8_t x = 0;
  for(i = 0; i < Len; i++)
  {
    x = x ^ (*(Buf + i));
  }
  return x;
}

static void WriteU32BE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)((value >> 24) & 0xFF);
  dst[1] = (uint8_t)((value >> 16) & 0xFF);
  dst[2] = (uint8_t)((value >> 8) & 0xFF);
  dst[3] = (uint8_t)(value & 0xFF);
}

static void BuildStatusFrame(void)
{
  uint8_t pos = 3;

  statusData[0] = 0xAA;
  statusData[1] = STATUS_HEADER_BYTE_1;
  statusData[2] = RAW_DIAG_PROTOCOL_VERSION;

  WriteU32BE(&statusData[pos], HAL_GetTick()); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_sample_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_adc_drdy_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_frame_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_tx_start_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_tx_done_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_tx_busy_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_tx_error_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_adc_error_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_imu_error_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_empty_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_overflow_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_sample_total_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_nonempty_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_single_sample_counter); pos += 4;
  WriteU32BE(&statusData[pos], raw_diag_ppg_fifo_multi_sample_counter); pos += 4;
  WriteU32BE(&statusData[pos], mimu_state);

  statusData[STATUS_XOR_INDEX] = CheckXOR(&statusData[2], STATUS_XOR_CHECK_LEN);
  statusData[STATUS_FOOTER_INDEX] = 0xCC;
}

static void MIMU_Startup(void)
{
#if ENABLE_MIMU_RECOVERY
    static const uint8_t settling[] = "[MIMU] power settling 1000 ms; startup retry enabled\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)settling, sizeof(settling) - 1, 1000);
    HAL_Delay(1000);
    const unsigned attempts = 5;
#else
    const unsigned attempts = 1;
#endif
    for (unsigned attempt = 1; attempt <= attempts; ++attempt) {
        if (MIMU_Init()) {
            static const uint8_t ok[] = "DEBUG: MIMU Init OK (AG identity/config verified)\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)ok, sizeof(ok) - 1, 1000);
            HAL_Delay(200);
            MIMU_LoadGyroOffset();
            MIMU_GyroCalibrate();
            mimu_ready = 1;
            mimu_state = MIMU_STATE_BOOT_READY;
            return;
        }
        raw_diag_imu_error_counter++;
        char line[96];
        snprintf(line, sizeof(line), "[MIMU] init FAIL attempt=%u/%u; identity/config not verified\r\n", attempt, attempts);
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
        if (attempt < attempts) HAL_Delay(500);
    }
#if ENABLE_MIMU_RECOVERY
    mimu_last_retry_ms = HAL_GetTick();
    mimu_state = MIMU_STATE_RETRYING;
    static const uint8_t failed[] = "[MIMU] offline; ACC/GYRO zero; calibration skipped; retry every 2000 ms\r\n";
#else
    mimu_state = MIMU_STATE_OFFLINE;
    static const uint8_t failed[] = "[MIMU] offline; ACC/GYRO zero; calibration skipped; recovery disabled\r\n";
#endif
    HAL_UART_Transmit(&huart2, (uint8_t *)failed, sizeof(failed) - 1, 1000);
}

static void MIMU_PollRecovery(void)
{
#if ENABLE_MIMU_RECOVERY
    if (mimu_ready) return;
    uint32_t now = HAL_GetTick();
    if (mimu_recovery_warming) {
        if ((uint32_t)(now - mimu_last_retry_ms) < 200U || huart2.gState != HAL_UART_STATE_READY) return;
        /* 不在运行中执行10秒标定/Flash写入；有效历史偏置不存在时沿用零偏0。 */
        MIMU_LoadGyroOffset();
        ACC_6BytesRead();
        GYRO_6BytesRead();
        mimu_recovery_warming = 0;
        mimu_ready = 1;
        mimu_state = MIMU_STATE_RECOVERED;
        static const uint8_t recovered[] = "[MIMU] recovered; identity/config verified; saved offset if valid; live calibration skipped\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)recovered, sizeof(recovered) - 1, 100);
        return;
    }
    if ((uint32_t)(now - mimu_last_retry_ms) < 2000U || huart2.gState != HAL_UART_STATE_READY) return;
    mimu_last_retry_ms = now;
    if (!MIMU_Init()) {
        raw_diag_imu_error_counter++;
        static const uint8_t failed[] = "[MIMU] recovery FAIL; retry in 2000 ms\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)failed, sizeof(failed) - 1, 100);
        return;
    }
    /* 时间戳暖机期间让ADC/PPG继续采集，MIMU中断读取仍关闭。 */
    mimu_recovery_warming = 1;
    mimu_last_retry_ms = HAL_GetTick();
#endif
}

/* 新板取证：仅启动读回和无组帧心跳，不在 ISR 打印或改写器件配置。 */
static void Acquisition_DebugADC(const char *phase)
{
    const uint8_t regs[] = {ADC_ID_REG, ADC_STATUS_REG, ADC_MUX_REG,
                           ADC_GSET_REG, ADC_RATE_REG, ADC_RCON_REG, ADC_SCON_REG};
    char line[224];
    if (ENABLE_ACQUISITION_DEBUG) {
    snprintf(line, sizeof(line), "[DEBUG-acq] ADC %s RESET=%u RESET_ODR=%u START=%u DRDY=%u MISO=%u CE5=%u CE1V8=%u\r\n",
        phase, (unsigned)HAL_GPIO_ReadPin(AD_RESET_GPIO_Port, AD_RESET_Pin),
        (unsigned)((AD_RESET_GPIO_Port->ODR & AD_RESET_Pin) != 0),
        (unsigned)HAL_GPIO_ReadPin(START_CONV_GPIO_Port, START_CONV_Pin),
        (unsigned)HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin),
        (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6),
        (unsigned)HAL_GPIO_ReadPin(V5_0_CE_GPIO_Port, V5_0_CE_Pin),
        (unsigned)HAL_GPIO_ReadPin(V1_8_CE_GPIO_Port, V1_8_CE_Pin));
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
    }
    /* CS 上升沿使事务从明确边界开始；不更改 SPI 模式或器件寄存器。 */
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    uint8_t identity_ok = 0, config_ok = 1;
    const uint8_t expected[] = {0, 0, 0x3C, 0x00, 0x2C, 0x3A, 0x10};
    for (unsigned i = 0; i < sizeof(regs); ++i) {
        uint8_t tx[] = {(uint8_t)(0x20U | regs[i]), 0, 0}, rx[3] = {0};
        HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 50);
        HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
        if (i == 0) identity_ok = status == HAL_OK && (rx[2] & 0x07U) == 0x01U;
        if (i >= 2 && (status != HAL_OK || rx[2] != expected[i])) config_ok = 0;
        if (ENABLE_ACQUISITION_DEBUG) {
        snprintf(line, sizeof(line), "[DEBUG-acq] ADC %s reg=0x%02X SPI=%u RX=%02X/%02X/%02X value=0x%02X transport_ok=%u\r\n",
                 phase, regs[i], (unsigned)status, rx[0], rx[1], rx[2], rx[2], status == HAL_OK);
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
        }
    }
    snprintf(line, sizeof(line), "[ADC] %s identity=%s config=%s (pre-init config is informational)\r\n",
             phase, identity_ok ? "PASS" : "FAIL", identity_ok && config_ok ? "PASS" : "FAIL");
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
}

#if (PPG_DEFAULT_CHANNEL != 0)
/* 0=成功，1=总线低，2=写地址NACK，3=寄存器地址NACK，4=读地址NACK。 */
static uint8_t Acquisition_ReadPPG(uint8_t reg, uint8_t *value)
{
    uint8_t second = PPG_GetChannel() == PPG_CH2;
    void (*start)(void) = second ? IIC2_Start : IIC_Start;
    void (*stop)(void) = second ? IIC2_Stop : IIC_Stop;
    void (*send)(uint8_t) = second ? IIC2_Send_Byte : IIC_Send_Byte;
    uint8_t (*ack)(void) = second ? IIC2_Wait_Ack : IIC_Wait_Ack;
    uint8_t (*read)(uint8_t) = second ? IIC2_Read_Byte : IIC_Read_Byte;
    GPIO_TypeDef *port = second ? I2C_SCL2_GPIO_Port : I2C_SCL1_GPIO_Port;
    uint16_t scl = second ? I2C_SCL2_Pin : I2C_SCL1_Pin;
    uint16_t sda = second ? I2C_SDA2_Pin : I2C_SDA1_Pin;
    if (!HAL_GPIO_ReadPin(port, scl) || !HAL_GPIO_ReadPin(port, sda)) return 1;
    uint8_t result = 2;
    start();
    send(0xAE);
    if (ack()) goto done;
    result = 3;
    send(reg);
    if (ack()) goto done;
    result = 4;
    start();
    send(0xAF);
    if (ack()) goto done;
    *value = read(0);
    result = 0;
done:
    stop();
    return result;
}

static uint8_t Acquisition_DebugPPG(uint8_t configured)
{
    /* FE/FIFO 仅展示；其余寄存器逐项与当前模式目标值比较。 */
    const uint8_t regs[] = {0xFF, 0xFE, MODE_CONFIG_REG, SPO2_CONFIG_REG,
        FIFO_CONFIG_REG, LED1_PA_REG, LED2_PA_REG, LED3_PA_REG,
        LED_CONTROL1, LED_CONTROL2, FIFO_WR_PTR_REG, FIFO_RD_PTR_REG, OVF_COUNTER_REG};
#if (CURRENT_WORK_MODE == MODE_SPO2)
    const uint8_t expected[] = {0x15, 0, 0x03, 0x77, 0x5F, 0xC0, 0xC0};
#else
    const uint8_t expected[] = {0x15, 0, 0x07, MAX30101_SPO2_CONFIG_VAL,
        MAX30101_FIFO_CONFIG_VAL, 0xC0, 0xC0, 0xE0,
        MAX30101_MULTI_LED_CTRL1_VAL, MAX30101_MULTI_LED_CTRL2_VAL};
#endif
    uint8_t second = PPG_GetChannel() == PPG_CH2;
    GPIO_TypeDef *port = second ? I2C_SCL2_GPIO_Port : I2C_SCL1_GPIO_Port;
    char line[224];
    if (ENABLE_ACQUISITION_DEBUG) {
    snprintf(line, sizeof(line), "[DEBUG-acq] PPG GPIO %s ODR=0x%04lX IDR=0x%04lX MODER=0x%08lX OTYPER=0x%04lX (ODR bit0/1: 1=release in OD mode)\r\n",
        configured ? "post-config" : "pre-init", (unsigned long)port->ODR,
        (unsigned long)port->IDR, (unsigned long)port->MODER, (unsigned long)port->OTYPER);
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
    snprintf(line, sizeof(line), "[DEBUG-acq] PPG%u %s SCL=%u SDA=%u (pins=%s)\r\n",
        (unsigned)PPG_GetChannel(), configured ? "post-config" : "pre-init",
        (unsigned)HAL_GPIO_ReadPin(port, second ? I2C_SCL2_Pin : I2C_SCL1_Pin),
        (unsigned)HAL_GPIO_ReadPin(port, second ? I2C_SDA2_Pin : I2C_SDA1_Pin),
        second ? "PB0/PB1" : "PA0/PA1");
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
    }
    uint8_t verified = 1;
    for (unsigned i = 0; i < (configured ? sizeof(regs) : 2U); ++i) {
        uint8_t value = 0;
        uint8_t status = Acquisition_ReadPPG(regs[i], &value);
        if (status) {
            snprintf(line, sizeof(line), "[PPG] reg=0x%02X FAIL stage=%u (1=BUS_LOW 2=ADDR_W 3=REG 4=ADDR_R); value=NA\r\n",
                     regs[i], status);
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
            verified = 0;
            break;
        }
        uint8_t mismatch = 0;
        if (i != 1 && i < sizeof(expected)) {
            uint8_t match = value == expected[i];
            verified &= match;
            mismatch = !match;
            snprintf(line, sizeof(line), "[PPG] reg=0x%02X value=0x%02X expected=0x%02X %s\r\n",
                     regs[i], value, expected[i], match ? "OK" : "MISMATCH");
        } else {
            snprintf(line, sizeof(line), "[PPG] reg=0x%02X value=0x%02X ACK\r\n", regs[i], value);
        }
        if (ENABLE_ACQUISITION_DEBUG || mismatch)
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
    }
    snprintf(line, sizeof(line), "[PPG] %s %s\r\n",
             configured ? "config" : "identity", verified ? "VERIFIED" : "FAIL");
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
    return verified;
}

/* PPG 从未产出样本时保留延后证据，供晚连接的蓝牙接收；只读，不重配置。 */
static void Acquisition_DebugPPGLive(uint32_t now)
{
    if (!ENABLE_ACQUISITION_DEBUG) return;
    static uint32_t last_ms;
    if (raw_diag_ppg_fifo_nonempty_counter != 0 || huart2.gState != HAL_UART_STATE_READY ||
        (uint32_t)(now - last_ms) < 10000U) return;
    last_ms = now;
    uint8_t second = PPG_GetChannel() == PPG_CH2;
    GPIO_TypeDef *port = second ? I2C_SCL2_GPIO_Port : I2C_SCL1_GPIO_Port;
    const uint8_t regs[] = {0xFF, MODE_CONFIG_REG, SPO2_CONFIG_REG, FIFO_CONFIG_REG,
        LED1_PA_REG, LED2_PA_REG, LED3_PA_REG, LED_CONTROL1, LED_CONTROL2,
        FIFO_WR_PTR_REG, FIFO_RD_PTR_REG, OVF_COUNTER_REG};
    char line[320];
    unsigned pos = (unsigned)snprintf(line, sizeof(line),
        "[DEBUG-ppg-live] ms=%lu ch=%u SCL=%u SDA=%u ODR=0x%04lX ",
        (unsigned long)now, (unsigned)PPG_GetChannel(),
        (unsigned)HAL_GPIO_ReadPin(port, second ? I2C_SCL2_Pin : I2C_SCL1_Pin),
        (unsigned)HAL_GPIO_ReadPin(port, second ? I2C_SDA2_Pin : I2C_SDA1_Pin),
        (unsigned long)port->ODR);
    for (unsigned i = 0; i < sizeof(regs); ++i) {
        uint8_t value = 0;
        uint8_t stage = Acquisition_ReadPPG(regs[i], &value);
        if (stage) {
            pos += (unsigned)snprintf(line + pos, sizeof(line) - pos,
                "FAIL reg=0x%02X stage=%u value=NA ", regs[i], stage);
            break;
        }
        pos += (unsigned)snprintf(line + pos, sizeof(line) - pos, "%02X=%02X ", regs[i], value);
    }
    snprintf(line + pos, sizeof(line) - pos, "nonempty=%lu empty=%lu\r\n",
        (unsigned long)raw_diag_ppg_fifo_nonempty_counter, (unsigned long)raw_diag_ppg_fifo_empty_counter);
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
}
#endif

static void Acquisition_DebugPoll(void)
{
    if (!ENABLE_ACQUISITION_DEBUG) return;
    static uint32_t last_ms, last_frames, last_done;
    static uint8_t started;
    uint32_t now = HAL_GetTick();
#if (PPG_DEFAULT_CHANNEL != 0)
    /* 到期后持续等待首个UART空闲窗口，避免固定2秒检查点与DMA反复重合。 */
    Acquisition_DebugPPGLive(now);
#endif
    if (!started) {
        started = 1;
        last_ms = now;
        last_frames = raw_diag_frame_counter;
        last_done = raw_diag_tx_done_counter;
    }
    if ((uint32_t)(now - last_ms) < 2000U) return;
    last_ms = now;
#if ENABLE_RAW_DATA_PACKET
    uint8_t no_frame = raw_diag_frame_counter == last_frames;
    uint8_t no_done = raw_diag_tx_done_counter == last_done;
    last_frames = raw_diag_frame_counter;
    last_done = raw_diag_tx_done_counter;
    /* 不抢占 Raw/STATUS DMA，也不终止挂起的发送。 */
    if ((!no_frame && !no_done) || huart2.gState != HAL_UART_STATE_READY) return;
    char line[288];
    snprintf(line, sizeof(line), "[DEBUG-acq] %s ms=%lu tick=%lu drdy=%lu adc_state=%u DRDYpin=%u frame=%lu tx=%lu done=%lu busy=%lu err=%lu uart_err=%lu ppg_empty=%lu\r\n",
        no_frame ? "NO_FRAME" : "NO_TX_DONE", (unsigned long)now, (unsigned long)raw_diag_sample_counter,
        (unsigned long)raw_diag_adc_drdy_counter, ADC_1to4Voltage_flag,
        (unsigned)HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin),
        (unsigned long)raw_diag_frame_counter, (unsigned long)raw_diag_tx_start_counter,
        (unsigned long)raw_diag_tx_done_counter, (unsigned long)raw_diag_tx_busy_counter,
        (unsigned long)raw_diag_tx_error_counter, (unsigned long)huart2.ErrorCode,
        (unsigned long)raw_diag_ppg_fifo_empty_counter);
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100);
#endif
}

// PPG 血氧模式硬件配置 (仅 SpO2 模式编译)
// 最优参数: 0x77/0x5F = 1000sps/411us/18-bit, 4x 平均, 有效输出 250sps
// 双通道时序: 2x411us=822us < 1000us(1/1000sps), 余量 17.8%
#if (CURRENT_WORK_MODE == MODE_SPO2)
static void PPG_Config_SpO2_Hardcoded(void)
{
    /* --- 1. Mode Configuration = 0x03: SpO2 (Red + IR) --- */
    PPG_WriteOneByte(MODE_CONFIG_REG, 0x03);

    /* --- 2. LED 电流 --- */
    PPG_WriteOneByte(LED1_PA_REG, 0xC0);  /* Red  ~29mA */
    PPG_WriteOneByte(LED2_PA_REG, 0xC0);  /* IR   ~29mA */

    /* --- 3. SPO2_CONFIG = 0x77 ---
     * ADC_RGE=16384nA(011) | SR=1000sps(101) | PW=411us/18-bit(11) */
    PPG_WriteOneByte(SPO2_CONFIG_REG, 0x77);

    /* --- 4. FIFO_CONFIG = 0x5F ---
     * SMP_AVE=4x(010) | ROLLOVER(1) | A_FULL=15(1111) */
    PPG_WriteOneByte(FIFO_CONFIG_REG, 0x5F);

    /* --- 5. 清除 FIFO 指针 --- */
    PPG_WriteOneByte(FIFO_WR_PTR_REG, 0x00);
    PPG_WriteOneByte(OVF_COUNTER_REG, 0x00);
    PPG_WriteOneByte(FIFO_RD_PTR_REG, 0x00);
}
#endif /* MODE_SPO2 */

// PPG 多光路配置 (Green+Red+IR, 参数由 sample_rate_config.h 决定)
static void PPG_Config_Green_Hardcoded(void)
{
    /* --- 1. Mode Configuration = 0x07: Multi-LED 模式 --- */
    PPG_WriteOneByte(MODE_CONFIG_REG, 0x07);

    /* --- 2. 多光路时隙: SLOT1=Green, SLOT2=Red, SLOT3=IR --- */
    PPG_WriteOneByte(LED_CONTROL1, MAX30101_MULTI_LED_CTRL1_VAL); /* 0x13 */
    PPG_WriteOneByte(LED_CONTROL2, MAX30101_MULTI_LED_CTRL2_VAL); /* 0x02 */

    /* --- 3. 保持已验证的LED电流，采样参数为0x6B/0x3F --- */
    PPG_WriteOneByte(LED3_PA_REG, 0xE0);  /* Green */ //2、4、6、8、A、C、E、FF对应1-8挡
    PPG_WriteOneByte(LED1_PA_REG, 0xC0);  /* Red */
    PPG_WriteOneByte(LED2_PA_REG, 0xC0);  /* IR */

    /* --- 4. SPO2_CONFIG: RGE + SR + PW (由 sample_rate_config.h 决定) --- */
    PPG_WriteOneByte(SPO2_CONFIG_REG, MAX30101_SPO2_CONFIG_VAL);

    /* --- 5. FIFO_CONFIG: SMP_AVE + ROLLOVER + A_FULL --- */
    PPG_WriteOneByte(FIFO_CONFIG_REG, MAX30101_FIFO_CONFIG_VAL);

    /* --- 6. 清除 FIFO 指针 --- */
    PPG_WriteOneByte(FIFO_WR_PTR_REG, 0x00);
    PPG_WriteOneByte(OVF_COUNTER_REG, 0x00);
    PPG_WriteOneByte(FIFO_RD_PTR_REG, 0x00);
}

/* ==============================================================================
 * 中断回调函数
 * ============================================================================== */

// 定时器中断处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM16: 触发 ADC 采集 (固定频率)
    if (htim == (&htim16)){
        raw_diag_sample_counter++;
        if ((raw_diag_sample_counter % PPG_SAMPLE_RATE) == 0) {
            raw_diag_status_pending = 1;
        }
        if (mimu_ready)
            HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
    }
}

// GPIO外部中断：读取 ADC 和 MIMU 数据
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
#if ENABLE_BOARD_CHECK
    /* Communication checks must never enter the ADC acquisition ISR. */
    (void)GPIO_Pin;
    return;
#endif
    if(GPIO_Pin == DRDY_Pin){
        raw_diag_adc_drdy_counter++;
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
                    raw_diag_adc_error_counter++;
                    ADC_1to4Voltage_flag = 5;
            }
        }

        // 读取 MIMU 数据
        if(ADC_1to4Voltage_flag == 4 && mimu_ready) {
            ACC_6BytesRead();
            GYRO_6BytesRead();
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
#if BLE_POWER_EXPERIMENT
        BP_TxDone();
#endif
        if (raw_diag_dma_tx_kind == 1) {
            raw_diag_tx_done_counter += BLE_BATCH5 ? 5 : 1;
            raw_diag_dma_tx_kind = 0;
            if (raw_diag_status_pending) {
                BuildStatusFrame();
                if (HAL_UART_Transmit_DMA(&huart2, statusData, STATUS_PACKET_LEN) == HAL_OK) {
                    raw_diag_dma_tx_kind = 2;
                    raw_diag_status_pending = 0;
                }
            }
#if BLE_RF_EXPERIMENT || BLE_POWER_EXPERIMENT
            else { RF_TryTransmit(); }
#endif
        } else {
            raw_diag_dma_tx_kind = 0;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        if (raw_diag_dma_tx_kind == 1) {
            raw_diag_tx_error_counter += BLE_BATCH5 ? 5 : 1;
        }
        raw_diag_dma_tx_kind = 0;
    }
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
#if BLE_RF_EXPERIMENT
    RF_Poll();
#endif
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

