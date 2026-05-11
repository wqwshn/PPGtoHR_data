/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_OneBoard_HalfDuplex_DMA/Src/main.c
  * @author  MCD Application Team
  * @version V3.0.1
  * @date    2026-04-02
  * @brief   This example describes how to send/receive bytes over SPI IP using
  *          the STM32L4xx SPI LL API.
  ******************************************************************************
  * @attention
  *
  * �ض�����������������STM32L476��SWD��дģʽ����ʹ��SWCLK SWDIO NRST VDD GND ��5���ߣ�ʹ��NRST����ʱ��NUCLEO�������ϵ�SB12���ű���Ͽ���
  * ����KEIL��options for target>>debug>>settings>>debug>>Connect & reset options�������£�Connect: with Pre-reset; Reset: Autodetect.
  * ʹ��NUCLEO�������������£�Connect: under reset; Reset: Autodetect.
  * ע�⣺SPIд��֮��һ��Ҫ������Ȼ������ȡ�������
  *
  ******************************************************************************
  */

  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

/* Private typedef for USART2 Rx -----------------------------------------------------------*/
__IO uint8_t RXBUFFER [20];
__IO uint8_t Frame_head = 0;
__IO uint8_t Frame_complete = 1;
__IO uint8_t RxDataNUM = 0;
/* Private typedef for DRIVE and REFERENCE signals -----------------------------------------------------------*/

  /* Configure the phace of REF_gyro signals */
uint16_t PH_XGYRO = 54;
uint16_t PH_YGYRO = 54;
uint16_t PH_ZGYRO = 54;
/* Configure the phace of REF_acc signals */
uint16_t PH_XACC = 0;
uint16_t PH_YACC = 0;
uint16_t PH_ZACC = 0;

/* Configure the frequence of Drive signals */
uint16_t FREQ = 2000;
/* Configure the Smoothing filter Ratio*/
uint8_t SF_Ratio = 20;
uint8_t SF_Ratio_true = 0;//�˲�������ֵ����ֹ�������ݸ���Ƶ�ʴ��ڴ��ڷ���Ƶ�ʵ�����
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

uint8_t LoopCounter = 0;
uint8_t DisplayCounter = 0;

/* Buffer used for SPI2 transmission */
uint16_t SPI2_aTxBuffer[ ] = { 0x0000,0x1111,0x2222,0x0000,0x1111,0x2222,0x0000,0x1111 };
uint8_t SPI2_ubNbDataToTransmit = sizeof(SPI2_aTxBuffer);
uint8_t SPI2_ubTransmissionComplete = 0;

/* Buffer used for SPI2 reception */
uint16_t aRxBuffer [sizeof(SPI2_aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(SPI2_aTxBuffer);
uint32_t RXdata [6];
double RXdata_float [sizeof(SPI2_aTxBuffer)];
//float RXdata_float_Queue[6][5001];
long double RXdata_float_QueueSum [6];
double RXdata_float_QueueMean [6];
double RXdata_display_Queue [6][151];
long double RXdata_display_QueueSum [6];
double RXdata_display_QueueMean [6];
__IO uint8_t ubReceptionComplete = 0;

/* SPI2���ͺͽ��ջ����������ݳ��� */
/* Buffer used for SPI2 transmission */
uint16_t SPI2_TxBuffer[ ] = { 0x0000,0x1111,0x2222,0x0000,0x1111,0x2222,0x0000,0x1111,0x2222 };
uint8_t SPI2_datalength = sizeof(SPI2_TxBuffer);
/* Buffer used for SPI2 reception */
int16_t SPI2_RxBuffer [sizeof(SPI2_TxBuffer)];
int16_t SPI1_RxBuffer [sizeof(SPI2_TxBuffer)];
uint8_t SPI2_TxFinish = 0;
uint8_t SPI2_RxFinish = 0;
double SPI2_data [sizeof(SPI2_TxBuffer)];
double SPI2_HotfilmFloat [sizeof(SPI2_TxBuffer)];
double SPI1_HotfilmFloat [sizeof(SPI2_TxBuffer)];
int32_t SPI2_HotfilmInt32 [sizeof(SPI2_TxBuffer)];
int32_t SPI1_HotfilmInt32 [sizeof(SPI2_TxBuffer)];
int32_t SPI2_HotfilmTX [sizeof(SPI2_TxBuffer)];
int32_t SPI1_HotfilmTX [sizeof(SPI2_TxBuffer)];

/* SPI3���ݳ��� */
uint16_t SPI3_RxBuffer [5];

uint16_t SPI3_GyroData [4];
uint16_t SPI3_AccData [4];
uint16_t SPI3_MagData [4];

float SPI3_GyroFloat [4];
float SPI3_AccFloat [4];
float SPI3_MagFloat [4];
int32_t SPI3_GyroInt32 [4];
int32_t SPI3_AccInt32 [4];
int32_t SPI3_MagInt32 [4];
int32_t SPI3_GyroTX [4];
int32_t SPI3_AccTX [4];
int32_t SPI3_MagTX [4];

/* 4�Ű��������� */
float AccBias[ ] = { 0,0,0 };//��λg
float GyroBias[ ] = { 0,0,0 };//��λ100degree/s
float MagBias[ ] = { 0,0,0 };//��λgauss
float Acc_FS[ ] = { 1,1,1 };//ʵ��1���������ٶ��¼��ٶȼ����,��λg
float Mag_FS[ ] = { 0.4584,0.4467,0.4344 };//ʵ�ʵشų����ֵ��Ӧ�������λgauss

///* 3�Ű��������� */
//float AccBias[]={-0.003123,0.024613,0.015404};//��λg
//float GyroBias[]={0.020677,0.013153,-0.013201};//��λ100degree/s
//float MagBias[]={-0.267707,-0.006555,0.061274};//��λgauss
//float Acc_FS[]={0.994140,0.992523,1.001249};//ʵ��1���������ٶ��¼��ٶȼ����,��λg
//float Mag_FS[]={0.4584,0.4467,0.4344};//ʵ�ʵشų����ֵ��Ӧ�������λgauss
///* 2�Ű��������� */
//float AccBias[]={-0.003123,0.024613,0.015404};//��λg
//float GyroBias[]={0.020677,0.013153,-0.013201};//��λ100degree/s
//float MagBias[]={-0.267707,-0.006555,0.061274};//��λgauss
//float Acc_FS[]={0.994140,0.992523,1.001249};//ʵ��1���������ٶ��¼��ٶȼ����,��λg
//float Mag_FS[]={0.4584,0.4467,0.4344};//ʵ�ʵشų����ֵ��Ӧ�������λgauss
__IO uint8_t SPI3_datalength = 6;
/* SPI2���պͷ�����ɱ�־ */
__IO uint8_t SPI2_ReceptionComplete = 0;
__IO uint8_t SPI2_TransmissionComplete = 0;
/* SPI3���պͷ�����ɱ�־ */
__IO uint8_t SPI3_ReceptionComplete = 0;
__IO uint8_t SPI3_TransmissionComplete = 0;
/* MIMU���̵�λ��ʼֵ */
uint8_t DR_ACC = 2;
uint8_t DR_GYRO = 2;
uint8_t DR_MAG = 1;

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE void     Set_SmoothingFilterRatio(void);

__STATIC_INLINE void     StartHSE(void);
__STATIC_INLINE void     SystemClock_Config_HSE(void);
__STATIC_INLINE void     SystemClock_Config_MSI(void);
__STATIC_INLINE void     UserButton_Init(void);
__STATIC_INLINE void     PWM_GPIO_Init(void);
__STATIC_INLINE void     BTModule_Init(void);
__STATIC_INLINE void     Configure_USART_PINs(void);
__STATIC_INLINE void     Configure_USART_BAUD19200(void);
__STATIC_INLINE void     Configure_USART_BAUD115200(void);
__STATIC_INLINE void     G5VPower_Init(void);
__STATIC_INLINE void     ADC_Init(void);
__STATIC_INLINE void     MIMU_Init(void);
__STATIC_INLINE void     Configure_DMA(void);
__STATIC_INLINE void     Configure_SPI2(void);
__STATIC_INLINE void     Configure_SPI3(void);
__STATIC_INLINE void     ConfigureSampRate_ADC_MIMU_byTIM3(void);
__STATIC_INLINE void     Wait_SPI2TxRx_Finish(void);
double     CalculateFloat(double RData);
void     ACC_GYRO_Write(uint8_t RegAdress, uint8_t RegData);
void     MAG_Write(uint8_t RegAdress, uint8_t RegData);
uint16_t     ACC_GYRO_Read(uint8_t RegAdress);
void     ACC_GYRO_MultipleRead(uint16_t Data[ ], uint8_t N_of_Data, uint8_t RegAdress);
void     MAG_MultipleRead(uint16_t Data[ ], uint8_t N_of_Data, uint8_t RegAdress);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {
  /* Start HSE */
  StartHSE( );
  /* Configure the system clock to 80 MHz */
  SystemClock_Config_HSE( );
  LL_mDelay(10);
  G5VPower_Init( );
  Configure_USART_PINs( );
  //first config BT
//	Configure_USART_BAUD19200();
//	BTModule_Init();//����ģ��
  Configure_USART_BAUD115200( );
  Configure_SPI2( );
  ADC_Init( );  //Select ADC Low-Speed mode
  //	Configure_DMA();
  UserButton_Init( );
  Configure_SPI3( );
  MIMU_Init( );
  PWM_GPIO_Init( );//����PA.8Ϊ������ţ��������ü��㣬����������
  ConfigureSampRate_ADC_MIMU_byTIM3( );
  LL_mDelay(10);
  char jj = 0;//��������Ҫ������ѭ�����⣬��Ȼ��ѭ���Ῠס��ԭ��δ֪
  //	Send_String("\nTXOK\n");

    /* Infinite loop */
  while (1) {
    /* Wait for the end of the transfer */
    Wait_SPI2TxRx_Finish( );
    /* ��ֵ�˲� */
    if (LoopCounter >= SF_Ratio) {
      SF_Ratio_true = LoopCounter;
      for (jj = 0;jj <= 3;jj++) {
        SPI2_HotfilmTX [jj] = SPI2_HotfilmInt32 [jj] / SF_Ratio_true;
        SPI1_HotfilmTX [jj] = SPI1_HotfilmInt32 [jj] / SF_Ratio_true;
      }
      for (jj = 0;jj <= 2;jj++) {
        SPI3_AccTX [jj] = SPI3_AccInt32 [jj] / SF_Ratio_true;
        SPI3_GyroTX [jj] = SPI3_GyroInt32 [jj] / SF_Ratio_true;
        SPI3_MagTX [jj] = SPI3_MagInt32 [jj] / SF_Ratio_true;
      }
      /* ��ֵ�˲��������ʹ�����������������ʱ���㣬�������жϳ����м����ɼ������� */
      LoopCounter = 0;
      for (jj = 0;jj <= 3;jj++) {
        SPI2_HotfilmInt32 [jj] = 0;
        SPI1_HotfilmInt32 [jj] = 0;
      }
      for (jj = 0;jj <= 2;jj++) {
        SPI3_AccInt32 [jj] = 0;
        SPI3_GyroInt32 [jj] = 0;
        SPI3_MagInt32 [jj] = 0;
      }
      /* ����֡ͷ */
      while (!LL_USART_IsActiveFlag_TXE(USART2));
      LL_USART_TransmitData8(USART2, 0xAA);
      while (!LL_USART_IsActiveFlag_TXE(USART2));
      LL_USART_TransmitData8(USART2, 0xBB);
      /* ������Ĥ���� */
      for (jj = 0;jj <= 3;jj++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI2_HotfilmTX [jj] >> 8);
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI2_HotfilmTX [jj] & 0xFF);
      }
      for (jj = 0;jj <= 3;jj++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI1_HotfilmTX [jj] >> 8);
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI1_HotfilmTX [jj] & 0xFF);
      }
      /* ���ͼ��ٶ����� */
      for (jj = 0;jj <= 2;jj++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_AccTX [jj] >> 8);
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_AccTX [jj] & 0xFF);
      }
      /* ������������ */
      for (jj = 0;jj <= 2;jj++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_GyroTX [jj] >> 8);
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_GyroTX [jj] & 0xFF);
      }
      /* ���͵ش����� */
      for (jj = 0;jj <= 2;jj++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_MagTX [jj] >> 8);
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, SPI3_MagTX [jj] & 0xFF);
      }
      /* ����֡β */
      while (!LL_USART_IsActiveFlag_TXE(USART2));
      LL_USART_TransmitData8(USART2, 0xCC);
      while (!LL_USART_IsActiveFlag_TXE(USART2));
      LL_USART_TransmitData8(USART2, 0xDD);
    }
  }
}

/**
  * @brief  Configures the G5V power source.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void G5VPower_Init(void) {
  /* ��������  */
  /* Enable the GPIOC Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /* Configure PC.1/HV_CE in output push-pull mode  */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_2, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);

  /* PC.2/HV_CE ��ʼֵ���ߣ���5V��Դ */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_2);
}

/**
  * @brief  Configures the Smoothing Filter Ratio.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void     Set_SmoothingFilterRatio(void) {
  int i, j;
  LoopCounter = 0;
  DisplayCounter = 0;
  for (j = 0;j < 6;j++) {
    RXdata [j] = 0;
    RXdata_float [j] = 0;
    RXdata_float_QueueSum [j] = 0;
    RXdata_float_QueueMean [j] = 0;
    RXdata_float_QueueMean [j] = 0;
    RXdata_display_QueueSum [j] = 0;
    RXdata_display_QueueMean [j] = 0;
    for (i = 0;i < SF_Ratio;i++) {
      RXdata_display_Queue [j][i] = 0;
    }
  }
}

/**
  * @brief  Configures the HSE as clock source.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void     StartHSE(void) {
  register uint32_t frequency = 0;
  /* Configure Systick to 1 ms with the current frequency which should be MSI */
  frequency = __LL_RCC_CALC_MSI_FREQ(LL_RCC_MSI_IsEnabledRangeSelect( ), \
    (LL_RCC_MSI_IsEnabledRangeSelect( ) ? \
      LL_RCC_MSI_GetRange( ) : \
      LL_RCC_MSI_GetRangeAfterStandby( )));
  LL_Init1msTick(frequency);
  /* Configure NVIC for RCC */
  NVIC_EnableIRQ(RCC_IRQn);
  NVIC_SetPriority(RCC_IRQn, 0);
  /* Enable interrupt on HSE ready */
  /* Enable the CSS
     Enable the HSE and set HSEBYP to use the external clock
     instead of an oscillator
     Enable HSE */
     /* Note : the clock is switched to HSE in the RCC_IRQHandler ISR */
  LL_RCC_EnableIT_HSERDY( );
  LL_RCC_HSE_EnableCSS( );
  LL_RCC_HSE_EnableBypass( );
  LL_RCC_HSE_Enable( );
  /* Check that the condition is met */
  while (LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_HSE) { };
}

/**
  * @brief  Configure the bluetooth module.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void     BTModule_Init(void) {
  /* ��������  */
  /* Enable the GPIOC Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /* Configure PC.4/BTState in input push-pull mode  */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
  /* Configure PC.5/BTReset in output push-pull mode  */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_5, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);
  /* �ϵ����ʱ1500ms */
  LL_mDelay(1500);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);
  //wakeup BT
  LL_USART_TransmitData8(USART2, 0xAA);
  LL_USART_TransmitData8(USART2, 0xAA);
  LL_USART_TransmitData8(USART2, 0xAA);
  LL_USART_TransmitData8(USART2, 0xAA);
  LL_mDelay(50);
  //set wake mode
  LL_UART_SendString(USART2, "<ST_WAKE=FOREVER>");
  LL_mDelay(50);
  //set transmitted power
  LL_UART_SendString(USART2, "<ST_TX_POWER=+2.5>");
  LL_mDelay(50);
  //set BT name
  LL_UART_SendString(USART2, "<ST_NAME=HJ-131-HB-1>");
  LL_mDelay(50);
  //set password
  LL_UART_SendString(USART2, "<ST_SECRET=123456>");
  LL_mDelay(50);
  //switch baud
  LL_UART_SendString(USART2, "<ST_BAUD=115200>");
  LL_mDelay(50);
  //reset
//	LL_UART_SendString(USART2,"<ST_RESET_BLE>");
//	LL_mDelay(1500);
  //set min gap
  LL_UART_SendString(USART2, "<ST_CON_MIN_GAP=75>");
  LL_mDelay(50);
  /* �ȴ��������������ӳɹ�*/
//	while(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_4));
}

/**
  * @brief  Configures the timer to generate Driving and Reference signals.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void  ConfigureSampRate_ADC_MIMU_byTIM3(void) {
  /* Configure the NVIC to handle TIM3 interrupt */
  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  //  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    /* Time base configuration */
    /* Counter mode: select up-counting mode */
  LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
  /* Set the pre-scaler value to have TIM3 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM3, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
  /* Set the auto-reload value to have a counter frequency of FREQ Hz */
  LL_TIM_SetAutoReload(TIM3, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM3), FREQ));

  /* Output waveform configuration */
  /* Set compare value for output channel 1 (TIMx_CCR1) as 0 of the auto-reload value */
  LL_TIM_OC_SetCompareCH1(TIM3, 0);
  /* TIM2 interrupts set-up */
  /* Enable the capture/compare interrupt for channel 1*/
  LL_TIM_EnableIT_CC1(TIM3);
  /* Start output signal generation */
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM3);
  /* Enable counter */
  LL_TIM_EnableCounter(TIM3);
}

/**
  * @brief  Initialize Driving and Reference signals output pins.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void PWM_GPIO_Init(void) {
  /* Enable the heartbeat LED !!!*/
  /* Enable the GPIOA Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /* Configure PA.8/AC_DRIVE_IN2 in output push-pull mode  */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_DOWN);
  /* Set PA.8/AC_DRIVE_IN2 to high level on dedicated gpio port  */
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
}

/**
  * @brief  This function converts the ADC output data to float value
  * @param   Rdata
  * @retval  float value
  */
double CalculateFloat(double RData) {
  double Rfloat;
  if (RData <= 0x7FFFFF) {
    Rfloat = (double) RData / (double) 0x7FFFFF * 2.5;
  }
  else if (RData >= 0x800000) {
    Rfloat = ((double) RData - (double) 0xFFFFFF - 1) / (double) 0x7FFFFF * 2.5;
  }
  return Rfloat;
}


/**
  * @brief  This function transmits string to PC
  * @param   Tdata
  * @retval  none
  */
void Send_String(char* TData) {
  while (*TData != 0) {
    while (!LL_USART_IsActiveFlag_TXE(USART2)) {
    }
    LL_USART_TransmitData8(USART2, *TData);
    TData++;

  }
}


/**
  * @brief  This function transmits float data to PC
  * @param   Tdata
  * @retval  none
  */
void DisplayFloat(double TData) {
  unsigned char Decimal_Display [6];
  int i;
  unsigned int decimal_part, integer_part;
  //UartA0_Transmit(0xFD);
  //UartA0_Transmit(0xFD);
  //UartA0_Transmit(0xFD);
  while (!LL_USART_IsActiveFlag_TXE(USART2)) {
  }
  if (TData < 0) {
    LL_USART_TransmitData8(USART2, 45);
    TData = -1 * TData;
  }
  else LL_USART_TransmitData8(USART2, 32);
  integer_part = TData / 1;
  decimal_part = (TData - integer_part) * 1000000 / 1;

  while (!LL_USART_IsActiveFlag_TXE(USART2)) {
  }
  LL_USART_TransmitData8(USART2, (integer_part + 48));

  while (!LL_USART_IsActiveFlag_TXE(USART2)) {
  }
  LL_USART_TransmitData8(USART2, 46);
  for (i = 5;i >= 0;i--) {
    Decimal_Display [i] = decimal_part % 10;
    decimal_part /= 10;
  }
  for (i = 0;i <= 5;i++) {

    while (!LL_USART_IsActiveFlag_TXE(USART2)) {
    }
    LL_USART_TransmitData8(USART2, (Decimal_Display [i] + 48));
  }

}
/**
  * @brief  This function configures the DMA Channels for SPI1 and SPI3
  * @note  This function is used to :
  *        -1- Enable DMA1 and DMA2 clock
  *        -2- Configure NVIC for DMA1 and DMA2 transfer complete/error interrupts
  *        -3- Configure the DMA1_Channel3 functional parameters
  *        -4- Configure the DMA2_Channel1 functional parameters
  *        -5- Enable DMA1 and DMA2 interrupts complete/error
  * @param   None
  * @retval  None
  */
__STATIC_INLINE void Configure_DMA(void) {
  /* ����DMA1 ����SPI2���ݴ��� */
  /* (1) Enable the clock of DMA1  */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  /* (2) Configure NVIC for DMA transfer/receive complete/error interrupts */
  NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* (3) Configure the DMA1_Channel4/SPI2_RX functional parameters */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4,
    LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
    LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
    LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, LL_SPI_DMA_GetRegAddr(SPI2), (uint32_t) SPI2_RxBuffer,
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, SPI2_datalength);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_1);
  /* (4) Configure the DMA1_Channel5/SPI2_TX functional parameters */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5,
    LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
    LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
    LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, (uint32_t) SPI2_TxBuffer, LL_SPI_DMA_GetRegAddr(SPI2),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, SPI2_datalength);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_1);
  /* (5) Enable DMA interrupts complete/error */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

  //	/* ����DMA2 ����SPI3���ݴ��� */
  //	/* (1) Enable the clock of DMA2  */
  //  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  // /* (2) Configure NVIC for DMA transfer/receive complete/error interrupts */	
  //	NVIC_SetPriority(DMA2_Channel2_IRQn, 0);
  //  NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  //	NVIC_SetPriority(DMA2_Channel1_IRQn, 0);
  //  NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  //	
  //  /* (3) Configure the DMA2_Channel1/SPI3_RX functional parameters */
  //  LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_1,
  //                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | 
  //                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | 
  //                        LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
  //  LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_1, LL_SPI_DMA_GetRegAddr(SPI3), (uint32_t)aRxBuffer, 
  //                         LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_1));
  //  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, ubNbDataToReceive);
  //  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_3);
  //	/* (4) Configure the DMA2_Channel2/SPI3_TX functional parameters */
  //  LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_2,
  //                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | 
  //                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | 
  //                        LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
  //  LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_2, (uint32_t)SPI2_aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI3), 
  //                         LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_2));
  //  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, ubNbDataToReceive);
  //  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_3);

  //  /* (5) Enable DMA interrupts complete/error */
  //  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);
  //  LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_1);
  //	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_2);
  //  LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_2);
}


/**
  * @brief  This function configures SPI2.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI3 pins.
  *        -2- Configure SPI3 functional parameters.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_SPI2(void) {
  /* ��PB.12 ����ΪƬѡ����SPI2 NSS */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /* Configure pin PB.12 in output push-pull mode for SPI2 NSS */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_12, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);
  /* Set SPI2 NSS high */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);

  /* ����SPI2********************/
  /* ����SPI2������ ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /* Configure SCK Pin PB.13 */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_DOWN);
  /* Configure MISO Pin PB.14 */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_14, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_DOWN);
  /* Configure MOSI Pin PB.15 */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_15, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);
  /* ����SPI2�Ĺ���ģʽ ********************************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  /* Configure SPI2 communication */
  LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV2);
  LL_SPI_SetTransferDirection(SPI2, LL_SPI_FULL_DUPLEX);
  //  LL_SPI_SetHalfDuplexDirection(SPI2,LL_SPI_DIRECTION_HALF_DUPLEX_RX); 
  LL_SPI_SetClockPhase(SPI2, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(SPI2, LL_SPI_POLARITY_HIGH);
  LL_SPI_SetTransferBitOrder(SPI2, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_16BIT);
  LL_SPI_SetNSSMode(SPI2, LL_SPI_NSS_SOFT);
  LL_SPI_SetMode(SPI2, LL_SPI_MODE_MASTER);
  LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_HALF);

  /* ��SPI2 ���պͷ��͵�DMA���� */
  /* Enable DMA RX and TX request */
//  LL_SPI_EnableDMAReq_RX(SPI2);
//	LL_SPI_EnableDMAReq_TX(SPI2);

  /* ����SPI1********************/
  /* ����SPI1������ ********************/
  /* Enable the peripheral clock of GPIOA */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /* Configure SCK Pin PA.5 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);
  /* Configure MISO Pin PA.6 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
  /* Configure MOSI Pin PA.7 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);
  /* ����SPI1�Ĺ���ģʽ ********************************/
  /* Enable the peripheral clock of GPIOA */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  /* Configure SPI2 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV2);
  LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
  //  LL_SPI_SetHalfDuplexDirection(SPI1,LL_SPI_DIRECTION_HALF_DUPLEX_RX); 
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);//ע�⣺SPI1��SPI2��˫ͨ��ͨѶ����λ���ò�ͬ
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
  LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_SLAVE);

  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
  /* Enable SPI2 */
  LL_SPI_Enable(SPI2);
}

/**
  * @brief  This function configures SPI3.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI3 pins.
  *        -2- Configure SPI3 functional parameters.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_SPI3(void) {
  /* ��PB.7��PB.8 ����ΪƬѡ����SPI3_CS_M��SPI3_CS_A/G */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /* Configure pin PB.7 in output push-pull mode for SPI3_CS_M */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
  /* Configure pin PB.8 in output push-pull mode for SPI3_CS_A/G */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
  /* Set SPI3_CS_M and SPI3_CS_A/G high */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7 | LL_GPIO_PIN_8);

  /* ����SPI3********************/
  /* ����SPI3������ ********************/
  /* Enable the peripheral clock of GPIOC */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /* Configure SCK Pin PC.10 */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure MISO Pin PC.11 */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);
  /* Configure MOSI Pin PC.12 */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_12, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_12, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);

  /* ����SPI3�Ĺ���ģʽ ********************************/
  /* Enable the peripheral clock of GPIOC */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  /* Configure SPI3 communication */
  LL_SPI_SetBaudRatePrescaler(SPI3, LL_SPI_BAUDRATEPRESCALER_DIV4);
  LL_SPI_SetTransferDirection(SPI3, LL_SPI_FULL_DUPLEX);
  //LL_SPI_SetHalfDuplexDirection(SPI2,LL_SPI_DIRECTION_HALF_DUPLEX_RX); 
  LL_SPI_SetClockPhase(SPI3, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI3, LL_SPI_POLARITY_HIGH);
  LL_SPI_SetTransferBitOrder(SPI3, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI3, LL_SPI_DATAWIDTH_16BIT);
  LL_SPI_SetNSSMode(SPI3, LL_SPI_NSS_SOFT);
  LL_SPI_SetMode(SPI3, LL_SPI_MODE_MASTER);
  LL_SPI_SetRxFIFOThreshold(SPI3, LL_SPI_RX_FIFO_TH_HALF);

  //  /* ��SPI3 ���պͷ��͵�DMA���� */
  //  /* Enable DMA RX and TX request */
  //  LL_SPI_EnableDMAReq_RX(SPI3);
  //	LL_SPI_EnableDMAReq_TX(SPI3);
    /* Enable SPI3 */
  LL_SPI_Enable(SPI3);
  LL_mDelay(10);//SPI��ʼ����ɺ�Ҫ�ȴ�һ��ʱ�������
}

/**
  * @brief  This function configures USARTx Instance.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_USART_PINs(void) {
  /* Enables GPIO clock and configures the USARTx pins (TX RX) */
  /* Enable the peripheral clock of GPIO Port */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  //  USARTx_SET_TX_GPIO_AF();
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
}

/**
  * @brief  This function configures USARTx Instance.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_USART_BAUD19200(void) {
  /* �ȹرս����ж� */
  LL_USART_DisableIT_RXNE(USART2);
  /* Configure IT */
  /*  Set priority for USARTx_IRQn */
  /*  Enable USARTx_IRQn */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);
  /* Enable the peripheral clock for USART */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* Configure USART */
  /* Disable USART prior modifying configuration registers */
  LL_USART_Disable(USART2);
  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  /* Oversampling by 16 */
  LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* This frequency can also be calculated through LL RCC macro */
  /* Ex :
      pllclk = __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6), LL_RCC_PLLM_DIV1, 40, LL_RCC_PLLR_DIV2);
      hclk = __LL_RCC_CALC_HCLK_FREQ(pllclk, LL_RCC_GetAHBPrescaler());
      periphclk = __LL_RCC_CALC_PCLKx_FREQ(hclk, LL_RCC_GetAPBxPrescaler());  x=1 or 2 depending on USART instance
      periphclk is expected to be equal to 80000000 Hz
      In this example, Peripheral Clock is equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 19200);
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while ((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2)))) { };
  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_mDelay(10);
  //  LL_USART_EnableIT_ERROR(USART2);
}


/**
  * @brief  This function configures USARTx Instance.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_USART_BAUD115200(void) {
  /* �ȹرս����ж� */
  LL_USART_DisableIT_RXNE(USART2);
  /* Configure IT */
  /*  Set priority for USARTx_IRQn */
  /*  Enable USARTx_IRQn */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);
  /* Enable the peripheral clock for USART */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* Configure USART */
  /* Disable USART prior modifying configuration registers */
  LL_USART_Disable(USART2);
  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  /* Oversampling by 16 */
  LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* This frequency can also be calculated through LL RCC macro */
  /* Ex :
      pllclk = __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6), LL_RCC_PLLM_DIV1, 40, LL_RCC_PLLR_DIV2);
      hclk = __LL_RCC_CALC_HCLK_FREQ(pllclk, LL_RCC_GetAHBPrescaler());
      periphclk = __LL_RCC_CALC_PCLKx_FREQ(hclk, LL_RCC_GetAPBxPrescaler());  x=1 or 2 depending on USART instance

      periphclk is expected to be equal to 80000000 Hz

      In this example, Peripheral Clock is equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200);
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while ((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2)))) { };
  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_mDelay(10);
  //  LL_USART_EnableIT_ERROR(USART2);
}

/**
  * @brief  Initialize adc mode selection pins.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void ADC_Init(void) {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /* Configure pin PA.9 in output push-pull mode for ADC OS2 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);
  /* Configure pin PA.10 in output push-pull mode for ADC OS1 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
  /* Configure pin PA.11 in output push-pull mode for ADC OS0 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
  /* Set ADC OS ratio 32 (OS[2:0] 101) */
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9 | LL_GPIO_PIN_11);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /* Configure pin PC.5 in output push-pull mode */
  /* Configure pin PC.7 in input mode for ADC BUSY X */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
  /* Configure pin PC.8 in output push-pull mode for AD_RESET */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_8, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
  /* Configure pin PC.9 in output push-pull mode for ADC CONVST */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_9, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);

  /* Reset ADC after power-up through pin PC.8 */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
  LL_mDelay(1);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
}

/**
  * @brief  Initialize adc mode selection pins.
  * @param  None
  * @retval None
  */
void ADC_Start(void) {
  /* ���� PC.9/ADC CONVST */
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
  /* ���� PC.9/ADC CONVST */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
}

/**
  * @brief  Initialize MIMU mode selection pins.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void MIMU_Init(void) {
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /* Configure pin PA.15 in output push-pull mode for DEN_A/G */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_15, LL_GPIO_SPEED_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_NO);
  /* Set DEN_A/G high to enable Gyro and Acc data*/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /* Configure pin PB.5 in input mode for MIMU INT_M */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
  /* Configure pin PB.3 in input mode for MIMU INT1_A/G */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  /* Configure pin PD.2 in input mode for MIMU INT2_A/G */
  LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);

  //  ACC_GYRO_Write(0x20,0xD6); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-4g
  ACC_GYRO_Write(0x20, 0xCE); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-16g
  ACC_GYRO_Write(0x22, 0x44); //���ٶ����ݼĴ���дCTRL_REG8 (22h)��������ٸ�������
  ACC_GYRO_Write(0x23, 0x04); //���ٶ����ݼĴ���дCTRL_REG9 (23h)������I2C

  //	ACC_GYRO_Write(0x10,0xC8); //���ݼĴ���дCTRL_REG1_G (10h),���ݸ�����952Hz������500 dps��LPF1����100Hz
  ACC_GYRO_Write(0x10, 0xD8); //���ݼĴ���дCTRL_REG1_G (10h),���ݸ�����952Hz��LPF1����100Hz������2000 dps

  MAG_Write(0x20, 0xFC); //�żĴ���дCTRL_REG1_M (20h)�����²���XY���UHPģʽ��ODR=80Hz
  MAG_Write(0x21, 0x00); //�żĴ���дCTRL_REG2_M (21h)������+-4gauss
  MAG_Write(0x22, 0x80); //�żĴ���дCTRL_REG3_M (22h)������I2C��SPI��д���ܣ�����ת��ģʽ
  MAG_Write(0x23, 0x0C); //�żĴ���дCTRL_REG4_M (23h)��Z���UHPģʽ
  MAG_Write(0x24, 0x40); //�żĴ���дCTRL_REG5_M (24h)�����ݶ������ٸ���
}

void ACC_GYRO_Write(uint8_t RegAdress, uint8_t RegData) {
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, ((RegAdress << 8) | RegData));//���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  (void) LL_SPI_ReceiveData16(SPI3);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_7);
}

void MAG_Write(uint8_t RegAdress, uint8_t RegData) {
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, ((RegAdress << 8) | RegData));//���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  (void) LL_SPI_ReceiveData16(SPI3);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_7);
}

uint16_t ACC_GYRO_Read(uint8_t RegAdress) {
  uint16_t ReadData;
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, ((0x80 | RegAdress) << 8));
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  ReadData = LL_SPI_ReceiveData16(SPI3);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_7);
  return ReadData;
}

void ACC_GYRO_MultipleRead(uint16_t Data[ ], uint8_t N_of_Data, uint8_t RegAdress) {
  int i;
  /* ����PB.8/SPI3_CS_A/G  */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, ((0x80 | RegAdress) << 8));
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  Data [0] = LL_SPI_ReceiveData16(SPI3);
  if (N_of_Data > 1) {
    for (i = 0;i < (N_of_Data - 1);i++) {
      while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
      LL_SPI_TransmitData16(SPI3, 0x0000);//��ȡAcc_Y
      while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
      Data [i + 1] = LL_SPI_ReceiveData16(SPI3);//X_L+Y_H
    }
  }
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, 0x0000);//��ȡAcc_Y
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  Data [N_of_Data] = LL_SPI_ReceiveData16(SPI3);//X_L+Y_H
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_7);
  /* �������� */
  for (i = 0;i < N_of_Data;i++) {
    Data [i] = (Data [i] & 0xFF) | (Data [i + 1] & 0xFF00);
  }
}

void MAG_MultipleRead(uint16_t Data[ ], uint8_t N_of_Data, uint8_t RegAdress) {
  int i;
  /* ����PB.8/SPI3_CS_A/G  */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, ((0xC0 | RegAdress) << 8));
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  Data [0] = LL_SPI_ReceiveData16(SPI3);
  if (N_of_Data > 1) {
    for (i = 0;i < (N_of_Data - 1);i++) {
      while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
      LL_SPI_TransmitData16(SPI3, 0x0000);//��ȡAcc_Y
      while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
      Data [i + 1] = LL_SPI_ReceiveData16(SPI3);//X_L+Y_H
    }
  }
  while (!LL_SPI_IsActiveFlag_TXE(SPI3)) { };
  LL_SPI_TransmitData16(SPI3, 0x0000);//��ȡAcc_Y
  while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) { };
  Data [N_of_Data] = LL_SPI_ReceiveData16(SPI3);//X_L+Y_H
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_7);
  /* �������� */
  for (i = 0;i < N_of_Data;i++) {
    Data [i] = (Data [i] & 0xFF) | (Data [i + 1] & 0xFF00);
  }
}

/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void UserButton_Init(void) {
  /* ���ý���ADC�����ж�*/
  /* Enable the BUTTON Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
  /* Connect External Line to the GPIO*/
//  USER_BUTTON_SYSCFG_SET_EXTI();
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE7);
  /* Enable a rising trigger External line 13 Interrupt */
//  USER_BUTTON_EXTI_LINE_ENABLE();
//  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, 0x03);
}

/**
  * @brief  Wait end of transfer and check if received Data are well.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Wait_SPI2TxRx_Finish(void) {
  /* 1 - Wait end of SPI2 transmission */
  while (SPI2_TxFinish != 1) { };
  SPI2_TxFinish = 0;
  /* Disable DMA1SPI2 Tx Channel */
//  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  /* 1 - Wait end of SPI2 reception */
  while (SPI2_RxFinish != 1) { };
  SPI2_RxFinish = 0;
  /* Disable DMA1SPI2 Rx Channel */
//  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
__STATIC_INLINE void SystemClock_Config_HSE(void) {
  /* MSI configuration and activation */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  //  LL_RCC_MSI_Enable();
  //  while(LL_RCC_MSI_IsReady() != 1) 
  //  {
  //  };

    /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable( );
  LL_RCC_PLL_EnableDomain_SYS( );
  while (LL_RCC_PLL_IsReady( ) != 1) { };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) { };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 80MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6), LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2)*/
  LL_Init1msTick(80000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(80000000);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
__STATIC_INLINE void SystemClock_Config_MSI(void) {
  /* MSI configuration and activation */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  LL_RCC_MSI_Enable( );
  while (LL_RCC_MSI_IsReady( ) != 1) { };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable( );
  LL_RCC_PLL_EnableDomain_SYS( );
  while (LL_RCC_PLL_IsReady( ) != 1) { };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) { };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 80MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6),
                                  LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2)*/
  LL_Init1msTick(80000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(80000000);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
  * @brief  Function to manage User push-button
  * @param  None
  * @retval None
  */
void UserButton_Callback(void) {
  /* Update User push-button variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
  //	//WaitAndCheckEndOfTransfer();
  //	/* DMA1 set datalength for SPI1 AND SPI2 */
  //	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ubNbDataToReceive);
  //	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ubNbDataToReceive);
  //		
  //	/* Active SPI2 */
  //	Activate_SPI2();



}


/**
  * @brief  Function called from DMA2 IRQ Handler when Rx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1SPI2_ReceiveComplete_Callback(void) {
  /* DMA Rx transfer completed */
  ubReceptionComplete = 1;
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

}
/**
  * @brief  Function called from DMA2 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1SPI2_TransmitComplete_Callback(void) {
  /* DMA Tx transfer completed */
  SPI2_ubTransmissionComplete = 1;
  /* Disable DMA2 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
}
/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI_TransferError_Callback(void) {
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

  /* Disable DMA2 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

}

/**
  * @brief  This function handles the HSE ready detection (called in RCC_IRQHandler)
  * @param  None
  * @retval None
  */
void HSEReady_Callback(void) {
  /* Switch the system clock to HSE */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  /* 1ms config with HSE 8MHz*/
  LL_Init1msTick(8000000);
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void) {
  __IO uint8_t received_char;

  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  received_char = LL_USART_ReceiveData8(USART2);

  if ((received_char == 0xAA) && (Frame_complete == 1)) {
    Frame_head = 1;
    return;
  }
  if (Frame_head == 1) {
    if (received_char == 0xBB) {
      Frame_complete = 0;
      Frame_head = 2;
    }
    else
      Frame_head = 0;
    return;
  }
  if ((Frame_head == 2) && (RxDataNUM < 5)) {
    RXBUFFER [RxDataNUM] = received_char;
    //LL_USART_TransmitData8(USARTx_INSTANCE, RXBUFFER[RxDataNUM]);	
    RxDataNUM++;

    //LL_USART_TransmitData8(USARTx_INSTANCE, received_char);
  }
  if (RxDataNUM == 5) {
    if ((RXBUFFER [3] == 0xCC) && (RXBUFFER [4] == 0X55)) {
      //			/* Echo received character on TX */
      //			uint32_t j;
      //			for(j=0;j<5;j++)
      //			{
      //				while (!LL_USART_IsActiveFlag_TXE(USART2))
      //				{
      //				}
      //				LL_USART_TransmitData8(USART2, RXBUFFER[j]);	
      //				
      //			}	
      DR_ACC = RXBUFFER [0];
      DR_GYRO = RXBUFFER [1];
      DR_MAG = RXBUFFER [2];

      /* Disable counter */

      LL_TIM_DisableCounter(TIM3);//ֹͣ��ʱ����ֹͣADC��MIMU����

      /* ���ٶ��������� */
      if (DR_ACC == 1)//���ٶ�����+-2g
      {
        ACC_GYRO_Write(0x20, 0xC6); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-2g
      }
      else if (DR_ACC == 2)//���ٶ�����+-4g
      {
        ACC_GYRO_Write(0x20, 0xD6); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-4g
      }
      else if (DR_ACC == 3)//���ٶ�����+-8g
      {
        ACC_GYRO_Write(0x20, 0xDE); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-8g
      }
      else if (DR_ACC == 4)//���ٶ�����+-16g
      {
        ACC_GYRO_Write(0x20, 0xCE); //���ٶȼĴ���дCTRL_REG6_XL (20h)�����ݸ�����952Hz������100Hz�����ٶȷ�Χ+-16g
      }
      /* ������������ */
      if (DR_GYRO == 1)//����������+-245dps
      {
        ACC_GYRO_Write(0x10, 0xC0); //���ݼĴ���дCTRL_REG1_G (10h),���ݸ�����952Hz��LPF1����100Hz������245 dps
      }
      else if (DR_GYRO == 2)//��������+-500dps
      {
        ACC_GYRO_Write(0x10, 0xC8); //���ݼĴ���дCTRL_REG1_G (10h),���ݸ�����952Hz��LPF1����100Hz������500 dps
      }
      else if (DR_GYRO == 3)//��������+-2000dps
      {
        ACC_GYRO_Write(0x10, 0xD8); //���ݼĴ���дCTRL_REG1_G (10h),���ݸ�����952Hz��LPF1����100Hz������2000 dps
      }
      /* ���������� */
      if (DR_MAG == 1)//������+-4gauss
      {
        MAG_Write(0x21, 0x00); //�żĴ���дCTRL_REG2_M (21h)������+-4gauss
      }
      else if (DR_MAG == 2)//������+-8gauss
      {
        MAG_Write(0x21, 0x20); //�żĴ���дCTRL_REG2_M (21h)������+-8gauss
      }
      else if (DR_MAG == 3)//������+-12gauss
      {
        MAG_Write(0x21, 0x40); //�żĴ���дCTRL_REG2_M (21h)������+-12gauss
      }
      else if (DR_MAG == 4)//������+-16gauss
      {
        MAG_Write(0x21, 0x60); //�żĴ���дCTRL_REG2_M (21h)������+-16gauss
      }
      LL_mDelay(10);//��ʱ�ȴ�MIMU�ȶ�
      LL_TIM_EnableCounter(TIM3);//������ʱ������ʼADC��MIMU����

      /* Enable counter */
//			LL_TIM_DisableCounter(TIM2);
//			LL_TIM_DisableCounter(TIM3);
//			Set_TIMOutputCompare_PH0to360();
//			Set_SmoothingFilterRatio();
    }

    Frame_complete = 1;
    Frame_head = 0;
    RxDataNUM = 0;
  }

}

void LL_UART_SendString_Len(USART_TypeDef* UARTx, const char* str, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(UARTx)) { };
    LL_USART_TransmitData8(UARTx, (uint8_t) str [i]);
  }
  while (!LL_USART_IsActiveFlag_TC(UARTx)) { };
}

void LL_UART_SendString(USART_TypeDef* UARTx, const char* str) {
  uint16_t len = 0;
  const char* p = str;
  while (*p++ != '\0') len++;
  LL_UART_SendString_Len(UARTx, str, len);
}

/**
  * @}
  */

  /**
    * @}
    */

    /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
