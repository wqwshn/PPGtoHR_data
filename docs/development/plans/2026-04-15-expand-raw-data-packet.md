# 扩展原始数据包 - 多光谱PPG + 陀螺仪 + UI重构

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 扩展原始数据模式(0xAA 0xBB)为多光谱(绿/红/红外)PPG采集 + 完整16-bit ACC + 三轴陀螺仪，同步重构上位机PPG波形显示。

**Architecture:**
- MCU: MAX30101 切换为 Multi-LED 三光路模式(Green+Red+IR)，LED脉宽降至215us(17-bit)以满足三通道时序约束；LSM9DS1 陀螺仪数据同步读取；数据包从21字节扩展至33字节
- PC: 新增 RawDataPacket 协议解析，dashboard 添加实时三通道PPG波形区域(Green左侧，Red/IR右侧上下分布)

**Tech Stack:** C + STM32 HAL + CMSIS-DSP (MCU), Python + PyQt5 + pyqtgraph (PC)

---

## 新数据包格式 (33 字节)

```
偏移   字段              类型        说明
────────────────────────────────────────────────────────────
0-1    帧头              uint8 x2    0xAA, 0xBB
2-3    ADC 桥顶2(HF2)    uint16 BE   ADS124S06 Bridge2
4-5    ADC 桥顶1(HF1)    uint16 BE   ADS124S06 Bridge1
6-7    ADC 桥中2         uint16 BE   ADS124S06 Mid2
8-9    ADC 桥中1         uint16 BE   ADS124S06 Mid1
10-11  ACC_X             int16 BE    加速度计 X (完整16位)
12-13  ACC_Y             int16 BE    加速度计 Y (完整16位)
14-15  ACC_Z             int16 BE    加速度计 Z (完整16位)
16-17  GYRO_X            int16 BE    陀螺仪 X (角速度)
18-19  GYRO_Y            int16 BE    陀螺仪 Y (角速度)
20-21  GYRO_Z            int16 BE    陀螺仪 Z (角速度)
22-24  PPG Green         3 bytes     17-bit 原始ADC值
25-27  PPG Red           3 bytes     17-bit 原始ADC值
28-30  PPG IR            3 bytes     17-bit 原始ADC值
31     XOR 校验           uint8       bytes[2..30] 异或
32     帧尾              uint8       0xCC
```

**带宽:** 33 bytes x 100Hz x 10 = 33 kbps (115200 bps 内)

---

## Task 1: sample_rate_config.h 多光路配置适配

**Files:**
- Modify: `Core/Inc/sample_rate_config.h`

- [ ] **Step 1: 添加 100Hz 三光路 SPO2_CONFIG 值**

在 `PPG_SAMPLE_RATE == 100` 分支中，将 SPO2_CONFIG 从 0x73 改为 0x52:
```c
#elif (PPG_SAMPLE_RATE == 100)
/* ADC=8192nA(10) | SR=800sps(100) | PW=215us(10) = 0x52 */
/* 三光路约束: 3 LEDs x 215us = 645us < 1250us (1/800sps) */
#define MAX30101_SPO2_CONFIG_VAL    0x52
```

- [ ] **Step 2: 添加多光路时隙配置宏**

在文件末尾 (#endif 之前) 添加:
```c
/* ============================================================
 * 多光路时隙配置 (Multi-LED Mode)
 * SLOT1=GREEN(011), SLOT2=RED(001), SLOT3=IR(010), SLOT4=Disabled
 * CTRL1: SLOT1=011 | SLOT2=001 = 0x13
 * CTRL2: SLOT3=010 | SLOT4=000 = 0x02
 * ============================================================ */
#define MAX30101_MULTI_LED_CTRL1_VAL   0x13U
#define MAX30101_MULTI_LED_CTRL2_VAL   0x02U
#define MAX30101_PPG_CHANNELS          3U

/* 17-bit 分辨率掩码 (LED_PW=10) */
#define MAX30101_PPG_RIGHT_SHIFT       1U
#define MAX30101_PPG_VALID_MASK        0x01FFFFU
```

- [ ] **Step 3: 更新采样率信息字符串**
```c
#elif (PPG_SAMPLE_RATE == 100)
#define PPG_RATE_STR    "100Hz (800/8x, Multi-LED G+R+IR)"
```

- [ ] **Step 4: Commit**

```bash
git add Core/Inc/sample_rate_config.h
git commit -m "feat: 多光路三通道配置参数 (G+R+IR, 17-bit, 100Hz)"
```

---

## Task 2: main.h 数据包常量更新

**Files:**
- Modify: `Core/Inc/main.h`

- [ ] **Step 1: 更新数据包长度常量**

将 `PACKET_LEN` 从 21 改为 33，`XOR_CHECK_LEN` 从 17 改为 29:
```c
/* 扩展数据包长度: 33字节 (含三通道PPG + 完整ACC + 陀螺仪) */
#define PACKET_LEN 33
#define XOR_CHECK_LEN 29  /* 校验区域: ADC(8) + ACC(6) + GYRO(6) + PPG(9) */
```

- [ ] **Step 2: 添加数据偏移索引宏**

替换原有的 `PPG_START_INDEX` 和 `TEMP_START_INDEX`:
```c
/* 数据段偏移定义 */
#define PPG_START_INDEX   22   /* 2(头) + 8(ADC) + 6(ACC) + 6(GYRO) = 22 */
#define GYRO_START_INDEX  16   /* 2(头) + 8(ADC) + 6(ACC) = 16 */
```

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/main.h
git commit -m "feat: 扩展原始数据包常量定义 (33字节)"
```

---

## Task 3: main.c MAX30101 多光路配置 + 数据采集重构

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: 修改 PPG_Config_Green_Hardcoded 为多光路配置**

将函数 `PPG_Config_Green_Hardcoded()` 重构为三光路初始化:
```c
static void PPG_Config_Green_Hardcoded(void)
{
    /* --- 1. Mode Configuration = 0x07: Multi-LED 模式 --- */
    PPG_WriteOneByte(MODE_CONFIG_REG, 0x07);

    /* --- 2. 多光路时隙: SLOT1=Green(011), SLOT2=Red(001), SLOT3=IR(010) --- */
    PPG_WriteOneByte(LED_CONTROL1, MAX30101_MULTI_LED_CTRL1_VAL); /* 0x13 */
    PPG_WriteOneByte(LED_CONTROL2, MAX30101_MULTI_LED_CTRL2_VAL); /* 0x02 */

    /* --- 3. LED 电流: 三通道统一 12.6mA (0x3F) --- */
    PPG_WriteOneByte(LED3_PA_REG, 0x3F);  /* Green */
    PPG_WriteOneByte(LED1_PA_REG, 0x3F);  /* Red */
    PPG_WriteOneByte(LED2_PA_REG, 0x3F);  /* IR */

    /* --- 4. SPO2_CONFIG: RGE + SR + PW (由 sample_rate_config.h 决定) --- */
    PPG_WriteOneByte(SPO2_CONFIG_REG, MAX30101_SPO2_CONFIG_VAL);

    /* --- 5. FIFO_CONFIG: SMP_AVE + ROLLOVER + A_FULL --- */
    PPG_WriteOneByte(FIFO_CONFIG_REG, MAX30101_FIFO_CONFIG_VAL);

    /* --- 6. 清除 FIFO 指针 --- */
    PPG_WriteOneByte(FIFO_WR_PTR_REG, 0x00);
    PPG_WriteOneByte(OVF_COUNTER_REG, 0x00);
    PPG_WriteOneByte(FIFO_RD_PTR_REG, 0x00);
}
```

- [ ] **Step 2: 更新 PPG 数据采集为三通道**

在 `ADC_1to4Voltage_flag == 4` 分支中，将心率模式的 PPG 采集逻辑替换为三通道:
```c
      /* --- 3.3 心率模式: 三通道 PPG 采集 --- */
      uint32_t sum_green = 0, sum_red = 0, sum_ir = 0;
      uint8_t buf[9];  /* 三通道 x 3 字节 */

      static uint32_t last_green_avg = 0;
      static uint32_t last_red_avg = 0;
      static uint32_t last_ir_avg = 0;

      for (uint8_t i = 0; i < sample_count; i++) {
          PPG_ReadFIFO_Burst(buf, 9);
          /* 17-bit 数据，右移1位对齐 (LED_PW=215us) */
          uint32_t raw;
          raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
          sum_green += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
          raw = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
          sum_red += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
          raw = ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 8) | buf[8];
          sum_ir += (raw >> MAX30101_PPG_RIGHT_SHIFT) & MAX30101_PPG_VALID_MASK;
      }

      if (sample_count > 0) {
          last_green_avg = sum_green / sample_count;
          last_red_avg   = sum_red / sample_count;
          last_ir_avg    = sum_ir / sample_count;
      }
```

- [ ] **Step 3: 添加完整 ACC + GYRO 打包**

替换原有的 `allData[10..12]` 3字节 ACC 高位为 16-bit 完整数据，并添加陀螺仪:
```c
      /* --- 2. 打包 ACC 完整 16 位 --- */
      allData[10] = ACC_XYZ[1];  /* X_H */
      allData[11] = ACC_XYZ[0];  /* X_L */
      allData[12] = ACC_XYZ[3];  /* Y_H */
      allData[13] = ACC_XYZ[2];  /* Y_L */
      allData[14] = ACC_XYZ[5];  /* Z_H */
      allData[15] = ACC_XYZ[4];  /* Z_L */

      /* --- 2.5 打包 GYRO 完整 16 位 --- */
      /* 陀螺仪数据已在 GYRO_6BytesRead() 中填充到 GYRO_XYZ[] */
      allData[GYRO_START_INDEX]     = GYRO_XYZ[1];  /* GX_H */
      allData[GYRO_START_INDEX + 1] = GYRO_XYZ[0];  /* GX_L */
      allData[GYRO_START_INDEX + 2] = GYRO_XYZ[3];  /* GY_H */
      allData[GYRO_START_INDEX + 3] = GYRO_XYZ[2];  /* GY_L */
      allData[GYRO_START_INDEX + 4] = GYRO_XYZ[5];  /* GZ_H */
      allData[GYRO_START_INDEX + 5] = GYRO_XYZ[4];  /* GZ_L */
```

- [ ] **Step 4: PPG 三通道打包**

替换原有的单通道 PPG 打包:
```c
      /* --- 3. 打包 PPG 三通道 --- */
      allData[PPG_START_INDEX]     = (last_green_avg >> 16) & 0xFF;
      allData[PPG_START_INDEX + 1] = (last_green_avg >> 8)  & 0xFF;
      allData[PPG_START_INDEX + 2] =  last_green_avg        & 0xFF;
      allData[PPG_START_INDEX + 3] = (last_red_avg >> 16)   & 0xFF;
      allData[PPG_START_INDEX + 4] = (last_red_avg >> 8)    & 0xFF;
      allData[PPG_START_INDEX + 5] =  last_red_avg          & 0xFF;
      allData[PPG_START_INDEX + 6] = (last_ir_avg >> 16)    & 0xFF;
      allData[PPG_START_INDEX + 7] = (last_ir_avg >> 8)     & 0xFF;
      allData[PPG_START_INDEX + 8] =  last_ir_avg           & 0xFF;
```

- [ ] **Step 5: 更新校验和与帧尾**

```c
      /* --- 4. 校验位 (bytes[2..30], 共29字节) --- */
      allData[31] = CheckXOR(&allData[2], XOR_CHECK_LEN);

      /* --- 5. 帧尾 --- */
      allData[32] = 0xCC;

      /* --- 6. DMA 发送 (33 字节) --- */
#if (ENABLE_RAW_DATA_PACKET)
      HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN);
#endif
```

- [ ] **Step 6: 在 EXTI 回调中添加陀螺仪读取**

在 `HAL_GPIO_EXTI_Callback` 中，`ACC_6BytesRead()` 调用后添加:
```c
        // 读取 MIMU 数据
        if(ADC_1to4Voltage_flag == 4) {
            ACC_6BytesRead();
            GYRO_6BytesRead();  /* 新增: 同步读取陀螺仪 */
        }
```

- [ ] **Step 7: 更新帧头初始化注释**

将 `allData[0..1]` 初始化处的注释更新，说明新数据包格式。

- [ ] **Step 8: 编译验证**

Run: `cd build && cmake --build . 2>&1 | tail -20`
Expected: 编译成功，0 errors, 0 warnings

- [ ] **Step 9: Commit**

```bash
git add Core/Src/main.c
git commit -m "feat: 多光谱PPG采集 + 陀螺仪 + 扩展数据包(33字节)"
```

---

## Task 4: PC 端协议解析 - RawDataPacket

**Files:**
- Modify: `tools/hr_monitor/protocol.py`

- [ ] **Step 1: 添加原始数据包常量**

在文件顶部的常量区域后添加:
```python
# 原始数据帧常量
RAW_HEADER_BYTE_0 = 0xAA
RAW_HEADER_BYTE_1 = 0xBB
RAW_FOOTER_BYTE = 0xCC

RAW_PACKET_LEN = 33
RAW_XOR_START = 2
RAW_XOR_END = 30  # 含
```

- [ ] **Step 2: 添加 RawDataPacket 数据类**

```python
@dataclass
class RawDataPacket:
    """解析后的多光谱原始传感器数据包 (33 字节)"""
    # ADC 桥路
    adc_hf2: int          # ADS124S06 桥顶2
    adc_hf1: int          # ADS124S06 桥顶1
    adc_mid2: int         # ADS124S06 桥中2
    adc_mid1: int         # ADS124S06 桥中1
    # 加速度计 (完整 16-bit)
    acc_x: int            # X 轴加速度 (原始 ADC 值)
    acc_y: int            # Y 轴加速度
    acc_z: int            # Z 轴加速度
    # 陀螺仪 (角速度)
    gyro_x: int           # X 轴角速度 (原始 ADC 值)
    gyro_y: int           # Y 轴角速度
    gyro_z: int           # Z 轴角速度
    # PPG 多光谱
    ppg_green: int        # 绿光 17-bit 原始值
    ppg_red: int          # 红光 17-bit 原始值
    ppg_ir: int           # 红外光 17-bit 原始值
```

- [ ] **Step 3: 添加 parse_raw_packet 函数**

```python
def _decode_int16(raw: int) -> int:
    """大端 uint16 转 int16 有符号"""
    if raw >= 0x8000:
        return raw - 0x10000
    return raw


def parse_raw_packet(data: bytes) -> Optional[RawDataPacket]:
    """
    解析 33 字节多光谱原始数据包.

    Args:
        data: 完整的 33 字节原始帧

    Returns:
        RawDataPacket 解析成功, None 校验失败
    """
    if len(data) != RAW_PACKET_LEN:
        return None

    if data[0] != RAW_HEADER_BYTE_0 or data[1] != RAW_HEADER_BYTE_1:
        return None

    if data[32] != RAW_FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..30]
    xor_val = 0
    for i in range(RAW_XOR_START, RAW_XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[31]:
        return None

    # ADC (uint16 BE, 无符号)
    adc_hf2  = (data[2] << 8)  | data[3]
    adc_hf1  = (data[4] << 8)  | data[5]
    adc_mid2 = (data[6] << 8)  | data[7]
    adc_mid1 = (data[8] << 8)  | data[9]

    # ACC (int16 BE, 有符号)
    acc_x = _decode_int16((data[10] << 8) | data[11])
    acc_y = _decode_int16((data[12] << 8) | data[13])
    acc_z = _decode_int16((data[14] << 8) | data[15])

    # GYRO (int16 BE, 有符号)
    gyro_x = _decode_int16((data[16] << 8) | data[17])
    gyro_y = _decode_int16((data[18] << 8) | data[19])
    gyro_z = _decode_int16((data[20] << 8) | data[21])

    # PPG 三通道 (3 bytes each, 17-bit)
    ppg_green = ((data[22] << 16) | (data[23] << 8) | data[24]) & 0x01FFFF
    ppg_red   = ((data[25] << 16) | (data[26] << 8) | data[27]) & 0x01FFFF
    ppg_ir    = ((data[28] << 16) | (data[29] << 8) | data[30]) & 0x01FFFF

    return RawDataPacket(
        adc_hf2=adc_hf2, adc_hf1=adc_hf1, adc_mid2=adc_mid2, adc_mid1=adc_mid1,
        acc_x=acc_x, acc_y=acc_y, acc_z=acc_z,
        gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
        ppg_green=ppg_green, ppg_red=ppg_red, ppg_ir=ppg_ir,
    )
```

注意: 如果 `_decode_int16` 已存在（HR 包解析中），直接复用，不要重复定义。

- [ ] **Step 4: Commit**

```bash
git add tools/hr_monitor/protocol.py
git commit -m "feat: 多光谱原始数据包协议解析 (33字节)"
```

---

## Task 5: PC 端串口读取 - 双包类型支持

**Files:**
- Modify: `tools/hr_monitor/serial_reader.py`

- [ ] **Step 1: 更新导入和常量**

```python
from protocol import (
    HEADER_BYTE_0, HEADER_BYTE_1, PACKET_LEN,
    parse_hr_packet, HRPacket,
    RAW_HEADER_BYTE_1, RAW_PACKET_LEN,
    parse_raw_packet, RawDataPacket,
)
```

- [ ] **Step 2: 添加 raw_packet_received 信号**

在 `SerialReader` 类中添加新信号:
```python
class SerialReader(QThread):
    packet_received = pyqtSignal(HRPacket)
    raw_packet_received = pyqtSignal(RawDataPacket)  # 新增
    error_occurred = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
```

- [ ] **Step 3: 重构状态机支持双包类型**

将状态机改为三态 (等待0xAA -> 判断第二字节 -> 收集 payload):
```python
        state = 0
        buf = bytearray()
        packet_type = None  # 'hr' or 'raw'

        try:
            while self._running:
                raw = self._serial.read(64)
                if not raw:
                    continue

                for byte in raw:
                    if state == 0:
                        if byte == HEADER_BYTE_0:  # 0xAA
                            buf = bytearray([byte])
                            state = 1
                    elif state == 1:
                        if byte == HEADER_BYTE_1:  # 0xCC -> HR packet
                            buf.append(byte)
                            packet_type = 'hr'
                            state = 2
                        elif byte == RAW_HEADER_BYTE_1:  # 0xBB -> Raw packet
                            buf.append(byte)
                            packet_type = 'raw'
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        expected_len = PACKET_LEN if packet_type == 'hr' else RAW_PACKET_LEN
                        if len(buf) == expected_len:
                            if packet_type == 'hr':
                                pkt = parse_hr_packet(bytes(buf))
                                if pkt is not None:
                                    self.packet_received.emit(pkt)
                            else:
                                pkt = parse_raw_packet(bytes(buf))
                                if pkt is not None:
                                    self.raw_packet_received.emit(pkt)
                            state = 0
                            buf = bytearray()
                            packet_type = None
```

- [ ] **Step 4: Commit**

```bash
git add tools/hr_monitor/serial_reader.py
git commit -m "feat: 串口读取支持双包类型 (HR + 原始数据)"
```

---

## Task 6: PC 端仪表盘 - PPG 多光谱波形显示

**Files:**
- Modify: `tools/hr_monitor/dashboard.py`
- Modify: `tools/hr_monitor/main.py`

- [ ] **Step 1: 在 Dashboard 中添加原始数据缓存**

在 `__init__` 方法中添加:
```python
        # 原始数据波形缓存 (最近 5 秒, 100Hz = 500 点)
        RAW_WAVE_SECONDS = 5
        RAW_WAVE_POINTS = RAW_WAVE_SECONDS * 100
        self._raw_ppg_green = deque(maxlen=RAW_WAVE_POINTS)
        self._raw_ppg_red = deque(maxlen=RAW_WAVE_POINTS)
        self._raw_ppg_ir = deque(maxlen=RAW_WAVE_POINTS)
        self._raw_time = deque(maxlen=RAW_WAVE_POINTS)
        self._raw_start_time = time.time()
        self._raw_pkt_count = 0
        self._is_raw_mode = False  # 当前是否为原始数据模式
```

- [ ] **Step 2: 构建 PPG 波形卡片**

添加新方法 `_build_ppg_waveform_card()`:
```python
    def _build_ppg_waveform_card(self) -> QFrame:
        """构建三通道 PPG 波形显示卡片"""
        card = QFrame()
        card.setObjectName("card")
        layout = QHBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        pg.setConfigOptions(antialias=True, background=COLOR_CARD, foreground=COLOR_TEXT_DIM)

        # 左侧: 绿光波形
        left = QVBoxLayout()
        lbl_green = QLabel("PPG Green")
        lbl_green.setStyleSheet(f"color: #22C55E; font-size: 12px; font-weight: bold;")
        left.addWidget(lbl_green)
        self._plot_green = pg.PlotWidget()
        self._plot_green.setYRange(0, 131072)  # 17-bit range
        self._plot_green.showGrid(y=True, alpha=0.15)
        self._plot_green.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_green.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._curve_green = self._plot_green.plot(pen=pg.mkPen("#22C55E", width=1.5))
        left.addWidget(self._plot_green)
        layout.addLayout(left, 1)

        # 右侧: 红光 + 红外上下排列
        right = QVBoxLayout()
        right.setSpacing(4)

        # 红光
        lbl_red = QLabel("PPG Red")
        lbl_red.setStyleSheet(f"color: #EF4444; font-size: 12px; font-weight: bold;")
        right.addWidget(lbl_red)
        self._plot_red = pg.PlotWidget()
        self._plot_red.setYRange(0, 131072)
        self._plot_red.showGrid(y=True, alpha=0.15)
        self._plot_red.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_red.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._curve_red = self._plot_red.plot(pen=pg.mkPen("#EF4444", width=1.5))
        right.addWidget(self._plot_red, 1)

        # 红外
        lbl_ir = QLabel("PPG IR")
        lbl_ir.setStyleSheet(f"color: #F97316; font-size: 12px; font-weight: bold;")
        right.addWidget(lbl_ir)
        self._plot_ir = pg.PlotWidget()
        self._plot_ir.setYRange(0, 131072)
        self._plot_ir.showGrid(y=True, alpha=0.15)
        self._plot_ir.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_ir.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._curve_ir = self._plot_ir.plot(pen=pg.mkPen("#F97316", width=1.5))
        right.addWidget(self._plot_ir, 1)

        layout.addLayout(right, 1)

        return card
```

- [ ] **Step 3: 修改 _init_ui 支持模式切换**

在 `_init_ui` 中，替换中间的趋势图为可切换区域:
```python
        # 中间: 趋势图 (HR模式) 或 PPG 波形 (Raw模式)
        self._trend_card = self._build_trend_card()
        self._ppg_wave_card = self._build_ppg_waveform_card()

        # 容器布局，用于切换显示
        self._middle_layout = QHBoxLayout()
        self._middle_layout.setContentsMargins(0, 0, 0, 0)
        self._middle_layout.addWidget(self._trend_card)
        root.addLayout(self._middle_layout, 4)

        # 默认显示趋势图
        self._ppg_wave_card.hide()
```

- [ ] **Step 4: 添加 update_raw_data 方法**

```python
    def update_raw_data(self, pkt: RawDataPacket):
        """接收并展示一个原始传感器数据包"""
        self._raw_pkt_count += 1
        elapsed = time.time() - self._raw_start_time

        # 首次收到原始数据时切换到 Raw 模式
        if not self._is_raw_mode:
            self._switch_to_raw_mode()

        # 追加波形数据
        self._raw_ppg_green.append(pkt.ppg_green)
        self._raw_ppg_red.append(pkt.ppg_red)
        self._raw_ppg_ir.append(pkt.ppg_ir)
        self._raw_time.append(elapsed)

        # 更新波形曲线
        t_list = list(self._raw_time)
        self._curve_green.setData(t_list, list(self._raw_ppg_green))
        self._curve_red.setData(t_list, list(self._raw_ppg_red))
        self._curve_ir.setData(t_list, list(self._raw_ppg_ir))

        # 自动滚动 X 轴
        if t_list:
            t_max = t_list[-1]
            t_min = max(0, t_max - 5)  # 显示最近 5 秒
            for pw in (self._plot_green, self._plot_red, self._plot_ir):
                pw.setXRange(t_min, max(t_max, t_min + 1))

            # 自动缩放 Y 轴 (基于绿光数据范围)
            g_list = list(self._raw_ppg_green)
            if g_list:
                y_min = max(0, min(g_list) - 5000)
                y_max = min(131072, max(g_list) + 5000)
                for pw in (self._plot_green, self._plot_red, self._plot_ir):
                    pw.setYRange(y_min, y_max)

        # 状态栏
        self._status_label.setText(
            f"Raw: {self._raw_pkt_count} pts  |  "
            f"Green: {pkt.ppg_green}  Red: {pkt.ppg_red}  IR: {pkt.ppg_ir}  |  "
            f"ACC: ({pkt.acc_x}, {pkt.acc_y}, {pkt.acc_z})  "
            f"GYRO: ({pkt.gyro_x}, {pkt.gyro_y}, {pkt.gyro_z})"
        )
```

- [ ] **Step 5: 添加模式切换方法**

```python
    def _switch_to_raw_mode(self):
        """切换到原始数据显示模式"""
        self._is_raw_mode = True
        # 替换趋势图为 PPG 波形卡
        self._middle_layout.removeWidget(self._trend_card)
        self._trend_card.hide()
        self._middle_layout.addWidget(self._ppg_wave_card)
        self._ppg_wave_card.show()

    def _switch_to_hr_mode(self):
        """切换到心率算法显示模式"""
        self._is_raw_mode = False
        self._middle_layout.removeWidget(self._ppg_wave_card)
        self._ppg_wave_card.hide()
        self._middle_layout.addWidget(self._trend_card)
        self._trend_card.show()
```

- [ ] **Step 6: 更新 clear_screen 方法**

在 `_clear_screen()` 中添加原始数据缓存清除:
```python
        # 清除原始数据
        self._raw_ppg_green.clear()
        self._raw_ppg_red.clear()
        self._raw_ppg_ir.clear()
        self._raw_time.clear()
        self._raw_pkt_count = 0
        self._raw_start_time = time.time()

        # 切换回 HR 模式
        if self._is_raw_mode:
            self._switch_to_hr_mode()

        # 清除波形曲线
        if hasattr(self, '_curve_green'):
            self._curve_green.setData([], [])
            self._curve_red.setData([], [])
            self._curve_ir.setData([], [])
```

- [ ] **Step 7: 在 main.py 中连接 raw_packet_received 信号**

在 `AppController._connect()` 中添加:
```python
        self._reader = SerialReader(port, baudrate=115200)
        self._reader.packet_received.connect(self._dash.update_data)
        self._reader.raw_packet_received.connect(self._dash.update_raw_data)
        self._reader.error_occurred.connect(self._on_error)
        self._reader.connection_changed.connect(self._dash.set_connected)
        self._reader.start()
```

- [ ] **Step 8: 测试 Python 语法**

Run: `cd tools/hr_monitor && python -c "from protocol import RawDataPacket, parse_raw_packet; print('OK')"`
Expected: `OK`

Run: `cd tools/hr_monitor && python -c "from dashboard import Dashboard; print('OK')"`
Expected: `OK`

- [ ] **Step 9: Commit**

```bash
git add tools/hr_monitor/dashboard.py tools/hr_monitor/main.py
git commit -m "feat: 上位机多光谱PPG波形显示 + 双模式切换"
```

---

## Task 7: 模拟模式更新 + 集成测试

**Files:**
- Modify: `tools/hr_monitor/dashboard.py`

- [ ] **Step 1: 添加 raw data 模拟定时器**

在 `start_simulation` 中添加原始数据模拟支持:
```python
    def start_simulation(self, raw_mode=False):
        """启动模拟数据模式"""
        t = TRANSLATIONS[self._lang]
        self._sim_step = 0
        if raw_mode:
            self._sim_raw_timer = QTimer(self)
            self._sim_raw_timer.timeout.connect(self._sim_raw_tick)
            self._sim_raw_timer.start(10)  # 100Hz 模拟
            self.set_connected(True)
            self._conn_label.setText(f"{t['simulated']} (Raw 100Hz)")
        else:
            self._sim_timer.start(1000)  # 1Hz HR 模拟
            self.set_connected(True)
            self._conn_label.setText(t["simulated"])

    def _sim_raw_tick(self):
        """生成一个模拟原始数据包 (100Hz)"""
        import math
        self._sim_step += 1
        t = self._sim_step * 0.01  # 时间轴

        # 模拟 PPG 信号 (正弦波 + 噪声)
        base = 50000
        pulse = 8000 * math.sin(2 * math.pi * 1.2 * t)  # ~72 BPM
        noise = (hash(t * 1000) % 1000 - 500)
        green = int(base + pulse + noise) & 0x01FFFF
        red = int(base * 0.8 + pulse * 0.6 + noise * 0.8) & 0x01FFFF
        ir = int(base * 0.7 + pulse * 0.5 + noise * 0.7) & 0x01FFFF

        pkt = RawDataPacket(
            adc_hf2=1000, adc_hf1=2000, adc_mid2=500, adc_mid1=800,
            acc_x=100, acc_y=-50, acc_z=16384,
            gyro_x=10, gyro_y=-5, gyro_z=3,
            ppg_green=green, ppg_red=red, ppg_ir=ir,
        )
        self.update_raw_data(pkt)
```

- [ ] **Step 2: 更新 main.py 的 --simulate 参数**

添加 `--raw` 模拟模式选项:
```python
    parser.add_argument(
        "--raw", action="store_true",
        help="Simulate raw multi-spectral data at 100Hz",
    )
```

在启动部分:
```python
    if args.simulate:
        win.start_simulation(raw_mode=False)
    elif args.raw:
        win.start_simulation(raw_mode=True)
    else:
        ctrl = AppController(win)
        app.aboutToQuit.connect(ctrl.cleanup)
```

- [ ] **Step 3: 测试模拟模式**

Run: `cd tools/hr_monitor && python main.py --raw`
Expected: 窗口正常显示三通道 PPG 波形，绿光左侧，红光/红外右侧上下排列

- [ ] **Step 4: Commit**

```bash
git add tools/hr_monitor/dashboard.py tools/hr_monitor/main.py
git commit -m "feat: 原始数据模拟模式 (--raw) 用于UI测试"
```

---

## Task 8: 文档更新

**Files:**
- Modify: `docs/在线心率算法实施文档.md`
- Modify: `docs/上位机UI说明文档.md`

- [ ] **Step 1: 更新算法文档**

在 `docs/在线心率算法实施文档.md` 的变更记录章节添加:
```
| 2026-04-15 | 扩展原始数据包至33字节: 多光谱PPG(G+R+IR)+完整ACC(16-bit)+陀螺仪 | 主要文件: main.c, sample_rate_config.h, main.h |
```

- [ ] **Step 2: 更新UI文档**

在 `docs/上位机UI说明文档.md` 的变更日志添加新条目:
```
| 2026-04-15 | 新增多光谱PPG波形显示(Green/Red/IR三通道) | 支持原始数据包33字节解析，双模式自动切换 | protocol.py, dashboard.py, serial_reader.py |
```

- [ ] **Step 3: Commit**

```bash
git add docs/在线心率算法实施文档.md docs/上位机UI说明文档.md
git commit -m "docs: 更新数据包扩展相关文档"
```

---

## 关键约束与注意事项

1. **LED 脉宽时序**: 三光路 100Hz 时 LED_PW 必须 <= 215us (17-bit)，否则违反 MAX30101 硬件时序约束
2. **FIFO 读取长度**: 三通道每样本 9 字节 (3 x 3)，不是之前的 3 字节
3. **数据对齐**: 17-bit 数据需要右移 1 位 (`raw >> 1 & 0x01FFFF`)，不同于之前的 18-bit 直接 `& 0x03FFFF`
4. **模式互斥**: 原始数据模式和 HR 算法模式互斥 (main.h 的 `ENABLE_RAW_DATA_PACKET` 开关)
5. **上位机自适应**: 仪表盘根据接收到的包类型自动切换显示模式，无需手动选择
6. **GYRO_6BytesRead 已存在**: MIMU.c 中已有陀螺仪读取函数，只需在 main.c 中调用
