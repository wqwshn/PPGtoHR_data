#include "MIMU.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern uint8_t ACC_XYZ[];
extern uint8_t GYRO_XYZ[];
extern uint8_t MAG_XYZ[];

/* ── SPI 单字节写寄存器 ── */
void ACC_GYRO_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);  // 选中ACC/GYRO
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);        // 取消磁力计

	spiTxData[0] = RegAddress & 0x7F;  // bit7=0: 写操作
	spiTxData[1] = txData;

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);    // 释放CS
}


/* ── SPI 单字节读寄存器 ── */
uint8_t ACC_GYRO_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	spiTxData[0] = 0x80 | RegAddress;  // bit7=1: 读操作

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);

	return spiRxData[1];
}


/* ── MAG 写寄存器 ── */
void MAG_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	spiTxData[0] = RegAddress & 0x7F;
	spiTxData[1] = txData;

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);
}


/* ── MAG 读寄存器 ── */
uint8_t MAG_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	spiTxData[0] = 0x80 | RegAddress;

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	return spiRxData[1];
}


/* ── 加速度计6字节突发读取 (小端序, OUT_X_L_XL=0x28) ── */
void ACC_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0x80 | 0x28;  // 读 OUT_X_L_XL, IF_ADD_INC 自动递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		ACC_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);  // 读取完毕释放CS
}


/* ── 陀螺仪6字节突发读取 (小端序, OUT_X_L_G=0x18) ── */
void GYRO_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0x80 | 0x18;  // 读 OUT_X_L_G, IF_ADD_INC 自动递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		GYRO_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);  // 读取完毕释放CS
}


/* ── 磁力计6字节突发读取 ── */
void MAG_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0xC0 | 0x28;  // 读 OUT_X_L_M, 地址递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		MAG_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);
}


/*
 * MIMU 初始化
 *
 * 初始化流程 (参考 LSM9DS1.md):
 *   1. 软复位 CTRL_REG8=0x05, 延时20ms
 *   2. 启用 BDU + IF_ADD_INC: CTRL_REG8=0x44
 *   3. 关闭 FIFO 和硬件中断
 *   4. 配置陀螺仪: 119Hz, ±500dps, BW=14Hz, 高通滤波
 *   5. 配置加速度计: 119Hz, ±4g, 高分辨率, 数字滤波
 *   6. 配置磁力计
 */
void MIMU_Init(void){
	/* Step-1: 软复位 + IF_ADD_INC, 等待寄存器重装完成 */
	ACC_GYRO_Write(CTRL_REG8, 0x05);   // SW_RESET=1 + IF_ADD_INC=1
	HAL_Delay(20);

	/* Step-2: BDU(防撕裂) + IF_ADD_INC(突发读取地址递增) */
	ACC_GYRO_Write(CTRL_REG8, 0x44);   // BDU=1 + IF_ADD_INC=1

	/* Step-3: 关闭 FIFO 和硬件中断 */
	ACC_GYRO_Write(CTRL_REG9, 0x04);   // 禁用FIFO, 禁用I2C

	/* Step-4: 陀螺仪配置
	 * CTRL_REG1_G=0x68: ODR=119Hz, FS=±500dps, BW=14Hz
	 * CTRL_REG3_G=0x46: HP_EN=1, HPCF=0b0110 (约0.1Hz截止)
	 */
	ACC_GYRO_Write(CTRL_REG1_G, 0x68);
	ACC_GYRO_Write(CTRL_REG3_G, 0x46);  // 开启高通滤波消除零偏漂移

	/* Step-5: 加速度计配置
	 * CTRL_REG6_XL=0x73: ODR=119Hz, FS=±4g, BW_XL=11(50Hz防混叠)
	 * CTRL_REG7_XL=0xC4: HR=1高分辨率, DCF=ODR/9(13.2Hz), FDS=1滤波输出
	 * 注: HR和FDS必须同时启用, 仅设FDS=1会导致ACC输出趋零
	 */
	ACC_GYRO_Write(CTRL_REG6_XL, 0x73);  // ODR=119Hz, +-4g, BW_XL=11(50Hz防混叠)
	ACC_GYRO_Write(CTRL_REG7_XL, 0xC4);  // HR=1, DCF=ODR/9(13.2Hz), FDS=1(滤波输出)
	// 注: HR和FDS必须同时启用, 仅设FDS=1而不设HR=1会导致ACC输出趋零

	/* Step-6: 磁力计配置 (保持原有配置不变) */
	MAG_Write(CTRL_REG1_M, 0xFC);  // 磁温补, XY轴UHP模式, ODR=80Hz
	MAG_Write(CTRL_REG2_M, 0x00);  // 量程±4gauss
	MAG_Write(CTRL_REG3_M, 0x80);  // 禁用I2C, SPI读写, 连续转换模式
	MAG_Write(CTRL_REG4_M, 0x0C);  // Z轴UHP模式
	MAG_Write(CTRL_REG5_M, 0x00);  // 连续转换模式
}


/* ── 陀螺仪零偏标定 (暖机丢弃 + 800样本 + std校验 + Flash存储) ── */
int16_t gyro_offset[3] = {0, 0, 0};

/* Flash 存储: 使用最后一页 (512KB Flash 的 page 255) */
#define CALIB_FLASH_PAGE_ADDR  0x0807F800U
#define CALIB_MAGIC            0xAA55CC77U

/* Flash 数据结构: magic(4) + offset[3](6) + reserved(1) + xor(1) = 12 bytes */
typedef struct __attribute__((packed)) {
	uint32_t magic;
	int16_t  offset[3];
	uint8_t  reserved;
	uint8_t  xor_check;
} GyroCalibFlash_t;

static uint8_t _calc_calib_xor(const GyroCalibFlash_t *d) {
	const uint8_t *p = (const uint8_t *)d;
	uint8_t x = 0;
	for (int i = 0; i < (int)sizeof(GyroCalibFlash_t) - 1; i++)
		x ^= p[i];
	return x;
}

void MIMU_LoadGyroOffset(void) {
	const GyroCalibFlash_t *flash = (const GyroCalibFlash_t *)CALIB_FLASH_PAGE_ADDR;
	if (flash->magic == CALIB_MAGIC && _calc_calib_xor(flash) == flash->xor_check) {
		memcpy(gyro_offset, flash->offset, sizeof(gyro_offset));
	}
}

static void _save_gyro_offset_to_flash(void) {
	GyroCalibFlash_t data = {
		.magic = CALIB_MAGIC,
		.reserved = 0xFF,
	};
	memcpy(data.offset, gyro_offset, sizeof(gyro_offset));
	data.xor_check = _calc_calib_xor(&data);

	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Banks     = FLASH_BANK_1,
		.Page      = (CALIB_FLASH_PAGE_ADDR - 0x08000000U) / 2048U,
		.NbPages   = 1,
	};
	uint32_t err = 0;
	HAL_FLASHEx_Erase(&erase, &err);

	const uint64_t *src = (const uint64_t *)&data;
	for (int i = 0; i < 2; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
		                  CALIB_FLASH_PAGE_ADDR + i * 8, src[i]);
	}
	HAL_FLASH_Lock();
}

/* 标定过程中通过 UART 输出进度 (需 extern huart2) */
extern UART_HandleTypeDef huart2;

static void _calib_print(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

#define CALIB_WARMUP   200   // 暖机丢弃样本数 (~2s)
#define CALIB_SAMPLES  800   // 有效采集样本数 (~8s)
#define CALIB_STD_MAX  3.0f  // 标准差阈值 (dps), 超过则拒绝本次标定

void MIMU_GyroCalibrate(void) {
	char buf[64];
	int32_t sum_x = 0, sum_y = 0, sum_z = 0;

	_calib_print("GYRO CALIB: warming up...\r\n");
	HAL_Delay(500);

	/* 阶段1: 暖机丢弃 */
	for (int i = 0; i < CALIB_WARMUP; i++) {
		GYRO_6BytesRead();
		HAL_Delay(10);
	}

	_calib_print("GYRO CALIB: collecting...\r\n");

	/* 阶段2: 有效采集 */
	for (int i = 0; i < CALIB_SAMPLES; i++) {
		GYRO_6BytesRead();
		sum_x += (int16_t)((GYRO_XYZ[1] << 8) | GYRO_XYZ[0]);
		sum_y += (int16_t)((GYRO_XYZ[3] << 8) | GYRO_XYZ[2]);
		sum_z += (int16_t)((GYRO_XYZ[5] << 8) | GYRO_XYZ[4]);
		HAL_Delay(10);

		/* 每200样本输出一次进度 */
		if ((i + 1) % 200 == 0) {
			snprintf(buf, sizeof(buf),
			         "  %d/%d mean: X=%d Y=%d Z=%d\r\n",
			         i + 1, CALIB_SAMPLES,
			         (int)(sum_x / (i + 1)), (int)(sum_y / (i + 1)), (int)(sum_z / (i + 1)));
			_calib_print(buf);
		}
	}

	int16_t ox = (int16_t)(sum_x / CALIB_SAMPLES);
	int16_t oy = (int16_t)(sum_y / CALIB_SAMPLES);
	int16_t oz = (int16_t)(sum_z / CALIB_SAMPLES);

	/* 阶段3: 标准差校验 (二次遍历计算方差) */
	/* 需要重新采集一遍算方差 -- 用最近采集的偏差估计 */
	/* 简化: 使用均值和最后若干样本的偏差估计 */
	/* 更精确: 再采集100个样本算标准差 */
	_calib_print("GYRO CALIB: validating...\r\n");
	int32_t var_x = 0, var_y = 0, var_z = 0;
	int n_std = 200;
	for (int i = 0; i < n_std; i++) {
		GYRO_6BytesRead();
		int16_t gx = (int16_t)((GYRO_XYZ[1] << 8) | GYRO_XYZ[0]);
		int16_t gy = (int16_t)((GYRO_XYZ[3] << 8) | GYRO_XYZ[2]);
		int16_t gz = (int16_t)((GYRO_XYZ[5] << 8) | GYRO_XYZ[4]);
		int32_t dx = gx - ox, dy = gy - oy, dz = gz - oz;
		var_x += dx * dx;
		var_y += dy * dy;
		var_z += dz * dz;
		HAL_Delay(10);
	}
	/* 标准差 (dps), 灵敏度 17.50 mdps/LSB */
	float std_x = sqrtf((float)var_x / n_std) * 17.50f / 1000.0f;
	float std_y = sqrtf((float)var_y / n_std) * 17.50f / 1000.0f;
	float std_z = sqrtf((float)var_z / n_std) * 17.50f / 1000.0f;
	float std_max = std_x > std_y ? std_x : std_y;
	if (std_max < std_z) std_max = std_z;

	snprintf(buf, sizeof(buf),
	         "  std: X=%.2f Y=%.2f Z=%.2f dps (threshold=%.1f)\r\n",
	         std_x, std_y, std_z, CALIB_STD_MAX);
	_calib_print(buf);

	if (std_max > CALIB_STD_MAX) {
		snprintf(buf, sizeof(buf),
		         "GYRO CALIB: REJECTED (std too high, keeping flash default)\r\n");
		_calib_print(buf);
		return;
	}

	/* 阶段4: 更新零偏并写入 Flash */
	gyro_offset[0] = ox;
	gyro_offset[1] = oy;
	gyro_offset[2] = oz;
	_save_gyro_offset_to_flash();

	snprintf(buf, sizeof(buf),
	         "GYRO CALIB: OK  X=%d Y=%d Z=%d (saved to flash)\r\n",
	         ox, oy, oz);
	_calib_print(buf);
}


/* ── MIMU 连接检查 ── */
uint8_t MIMU_check(void){
	uint8_t mimu_id[2] = {0};
	mimu_id[0] = ACC_GYRO_Read(WHO_AM_I);
	mimu_id[1] = MAG_Read(WHO_AM_I_M);

	if((mimu_id[0] == 0x68) && (mimu_id[1] == 0x3D))
		return 1;
	else
		return 0;
}
