/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C_SCL1_Pin GPIO_PIN_0
#define I2C_SCL1_GPIO_Port GPIOA
#define I2C_SDA1_Pin GPIO_PIN_1
#define I2C_SDA1_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define I2C_SCL2_Pin GPIO_PIN_0
#define I2C_SCL2_GPIO_Port GPIOB
#define I2C_SDA2_Pin GPIO_PIN_1
#define I2C_SDA2_GPIO_Port GPIOB
#define BLE_STATE_Pin GPIO_PIN_12
#define BLE_STATE_GPIO_Port GPIOB
#define BLE_RST_Pin GPIO_PIN_13
#define BLE_RST_GPIO_Port GPIOB
#define V5_0_CE_Pin GPIO_PIN_8
#define V5_0_CE_GPIO_Port GPIOA
#define V1_8_CE_Pin GPIO_PIN_9
#define V1_8_CE_GPIO_Port GPIOA
#define START_CONV_Pin GPIO_PIN_10
#define START_CONV_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_11
#define DRDY_GPIO_Port GPIOA
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define AD_RESET_Pin GPIO_PIN_12
#define AD_RESET_GPIO_Port GPIOA
#define DEN_A_G_Pin GPIO_PIN_15
#define DEN_A_G_GPIO_Port GPIOA
#define INT2_A_G_Pin GPIO_PIN_3
#define INT2_A_G_GPIO_Port GPIOB
#define INT1_A_G_Pin GPIO_PIN_4
#define INT1_A_G_GPIO_Port GPIOB
#define INT2_M_Pin GPIO_PIN_5
#define INT2_M_GPIO_Port GPIOB
#define DRDY_M_Pin GPIO_PIN_6
#define DRDY_M_GPIO_Port GPIOB
#define CS_A_G_Pin GPIO_PIN_7
#define CS_A_G_GPIO_Port GPIOB
#define CS_M_Pin GPIO_PIN_8
#define CS_M_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* ============================================================
 * 系统配置开关 (统一修改区, 改后重新编译烧录即可生效)
 * ============================================================ */

/* 工作模式选择 */
#define MODE_HEART_RATE 0
#define MODE_SPO2       1
#ifndef CURRENT_WORK_MODE
#define CURRENT_WORK_MODE    MODE_HEART_RATE   /* 0=心率模式, 1=血氧模式 */
#endif

/* 数据发送模式选择 (两种模式互斥, 不会同时发送)
 * 0 = 在线心率模式: 运行算法, 仅发送 1Hz HR 结果包 (0xAA 0xCC) [仅125Hz]
 * 1 = 原始数据模式: 仅发送原始数据包 (0xAA 0xBB), 不运行算法 [所有采样率]
 */
#ifndef ENABLE_RAW_DATA_PACKET
#define ENABLE_RAW_DATA_PACKET  1
#endif

/* HJ-131IMH BLE 配置开关:
 * 0 = 上电不发送 BLE 配置指令
 * 1 = 上电复位 BLE 模块后发送固定配置指令
 */
/* RF-off experiment: hold HJ-131IMH active-high hardware reset.
 * Does not gate sensor power or the shared wired UART. */
#ifndef BLE_RF_EXPERIMENT
#define BLE_RF_EXPERIMENT 0
#endif
#ifndef BLE_POWER_EXPERIMENT
#define BLE_POWER_EXPERIMENT 0
#endif
#ifndef BLE_POWER_READ_ONLY
#define BLE_POWER_READ_ONLY 0
#endif
#ifndef BLE_POWER_WRITE_ONLY
#define BLE_POWER_WRITE_ONLY 0
#endif
#ifndef BLE_SINGLE25
#define BLE_SINGLE25 0
#endif
#if BLE_SINGLE25 && (BLE_BATCH5 || BLE_FIXED_MINUS10 || BLE_POWER_EXPERIMENT || BLE_RF_EXPERIMENT || BLE_RF_DISABLED || ENABLE_BLE_CONFIG)
#error "Single +2.5 mode excludes other BLE modes"
#endif
#ifndef BLE_BATCH5
#define BLE_BATCH5 0
#endif
#if BLE_BATCH5 && (BLE_FIXED_MINUS10 || BLE_POWER_EXPERIMENT || BLE_RF_EXPERIMENT || BLE_RF_DISABLED || ENABLE_BLE_CONFIG || !ENABLE_RAW_DATA_PACKET)
#error "Batch mode requires Raw and excludes other BLE modes"
#endif
#ifndef BLE_FIXED_MINUS10
#define BLE_FIXED_MINUS10 0
#endif
#if BLE_FIXED_MINUS10 && (BLE_POWER_EXPERIMENT || BLE_RF_EXPERIMENT || BLE_RF_DISABLED || ENABLE_BLE_CONFIG)
#error "Fixed -10 mode must preserve other BLE settings and exclude experiments"
#endif
#if BLE_POWER_WRITE_ONLY && (!BLE_POWER_EXPERIMENT || BLE_POWER_READ_ONLY)
#error "Write-only mode requires power experiment and excludes read-only mode"
#endif
#if BLE_POWER_READ_ONLY && !BLE_POWER_EXPERIMENT
#error "BLE_POWER_READ_ONLY requires BLE_POWER_EXPERIMENT"
#endif
#if BLE_RF_EXPERIMENT && !ENABLE_RAW_DATA_PACKET
#error "RF experiment requires Raw output"
#endif
#ifndef BLE_RF_DISABLED
#define BLE_RF_DISABLED 0
#endif
#if (BLE_RF_DISABLED != 0) && (BLE_RF_DISABLED != 1)
#error "BLE_RF_DISABLED must be 0 or 1"
#endif

/* 自定义 BLE MAC 地址 (12 字节 HEX, 大端)
 * HJ-380 将只连接此 MAC 地址的 HJ-131 设备
 * 与上位机 tools/monitor/protocol.py 中 BLE_CUSTOM_MAC 保持一致
 */
#define BLE_CUSTOM_MAC  "784128c58150"

#ifndef ENABLE_BLE_CONFIG
#define ENABLE_BLE_CONFIG       0
#endif

#if BLE_RF_EXPERIMENT && (BLE_RF_DISABLED || ENABLE_BLE_CONFIG)
#error "RF experiment must preserve module config and start released"
#endif
#if BLE_POWER_EXPERIMENT && (BLE_RF_EXPERIMENT || BLE_RF_DISABLED || ENABLE_BLE_CONFIG || !ENABLE_RAW_DATA_PACKET)
#error "Power experiment requires Raw, released BLE and preserved module settings"
#endif
#if BLE_POWER_EXPERIMENT && PPG_SAMPLE_RATE != 100 && defined(PPG_SAMPLE_RATE)
#error "Power experiment requires 100 Hz Raw output"
#endif
void BP_RxIRQ(void);

/* PPG 通道选择: 0=禁用PPG（两路IIC关闭，PPG字段填0）, 1=PPG1(IIC1总线), 2=PPG2(IIC2总线) */
#ifndef PPG_DEFAULT_CHANNEL
#define PPG_DEFAULT_CHANNEL     2
#endif

/* PPG 采样率 (Hz): 50 / 100 / 125
 * 50Hz:  内部 800sps / 16x 硬件平均 = 50sps
 * 100Hz: 内部 400sps / 2x 硬件平均 = 200sps, MCU 100Hz 读取
 * 125Hz: 内部 1000sps / 4x 硬件平均 = 250sps, MCU 125Hz 读取
 */
#ifndef PPG_SAMPLE_RATE
#define PPG_SAMPLE_RATE         100
#endif

/* ============================================================ */

/* 扩展数据包长度: 35字节 (含三通道PPG + 完整ACC + 陀螺仪 + Raw序号) */
#define PACKET_LEN 35
#define XOR_CHECK_LEN 31  /* 校验区域: ADC(8) + ACC(6) + GYRO(6) + PPG(9) + SEQ(2) = 31 */

/* Raw 链路诊断 STATUS 帧: 69字节, 1Hz, 帧头 0xAA 0xDD */
#define STATUS_HEADER_BYTE_1 0xDD
#define RAW_DIAG_PROTOCOL_VERSION 2
#define STATUS_PACKET_LEN 69
#define STATUS_XOR_CHECK_LEN 65  /* protocol_version(1) + uint32计数器(16*4) */
#define STATUS_XOR_INDEX 67
#define STATUS_FOOTER_INDEX 68

/* 数据段偏移定义 */
#define GYRO_START_INDEX  16   /* 2(头) + 8(ADC) + 6(ACC) = 16 */
#define PPG_START_INDEX   22   /* 2(头) + 8(ADC) + 6(ACC) + 6(GYRO) = 22 */
#define RAW_SEQUENCE_START_INDEX  31

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
