---
description: LSM9DS1 IMU传感器调试指南 -- 陀螺仪/加速度计数据异常排查与修复, 涵盖寄存器配置、SPI通信、上位机解算全链路
---

# LSM9DS1 IMU 传感器调试指南

本项目的 MIMU (LSM9DS1) 通过 SPI2 连接 (CS_A_G=PB7, CS_M=PB8), 经历过陀螺仪数据撕裂和加速度计归零两次重大调试。本技能记录了完整的问题现象、根因分析和修复方案。

## 关键文件

| 文件 | 职责 |
|------|------|
| `Core/Src/MIMU.c` | SPI驱动 + 传感器初始化 + 数据读取 |
| `Core/Inc/MIMU.h` | 寄存器地址定义 + 函数声明 |
| `Core/Src/main.c` | 数据打包 (allData[]) + EXTI回调中触发读取 |
| `tools/monitor/protocol.py` | 上位机协议解析 (ACC/GYRO灵敏度转换) |
| `tools/monitor/raw_data_panel.py` | 波形显示面板 |
| `docs/another-referrence/LSM9DS1.md` | 寄存器完整参考文档 |
| `docs/another-referrence/LSM9DS1.c/.h` | 参考驱动 (DMA + 合并读取) |

## 当前工作配置

```
CTRL_REG8  = 0x44  (BDU=1 + IF_ADD_INC=1)
CTRL_REG9  = 0x04  (FIFO off + I2C disable)
CTRL_REG1_G = 0x68 (GYRO: 119Hz, +/-500dps, BW=14Hz)
CTRL_REG3_G = 0x46 (GYRO HP filter, HPCF=6)
CTRL_REG6_XL = 0x70 (ACC: 119Hz, +/-4g, auto BW)
CTRL_REG7_XL = 0x00 (ACC: LPF2 disabled -- FDS=1会导致输出趋零!)
```

## 已解决问题清单

### 问题1: 陀螺仪数据撕裂 (2026-04-16)

**现象**: 陀螺仪数据出现随机大幅跳变, 静态下噪声极大

**根因**: CTRL_REG8 未启用 BDU (Block Data Update), SPI 突发读取 6 字节时高低字节来自不同采样周期

**修复**:
1. 软复位 `CTRL_REG8 = 0x05` + 延时 20ms
2. 启用 BDU + IF_ADD_INC: `CTRL_REG8 = 0x44`
3. 陀螺仪降频至 119Hz, 启用高通滤波

**排查方法**: 检查数据是否出现"量化台阶"异常 -- 如果 LSB 步进不连续, 多为撕裂

### 问题2: 陀螺仪静态零偏 (2026-04-17)

**现象**: 静态下陀螺仪三轴有固定偏移 (X=-4.6dps, Y=+1.2dps, Z=-6.5dps)

**根因**: MEMS 陀螺仪固有零偏, 在 LSM9DS1 规格范围内 (典型 +/-2dps, 最大 +/-10dps)

**发现**: CTRL_REG2_G 默认 0x00 (OUT_SEL=00) 导致 HP 滤波器输出未路由到数据寄存器, 即使 HP_EN=1 也无效

**修复**: 软件零偏标定 -- 启动时静态采集 200 样本取均值, 后续扣除偏移

### 问题3: ACC 三轴归零 (2026-04-17)

**现象**: ACC 三轴均接近 0g, Z轴应为 ~1g

**排查过程** (关键步骤, 可复用):

1. **上位机解析验证**: RANGE_ACC = 4.0/32767.0 (0.122 mg/LSB), 字节偏移正确 -- 排除
2. **数据打包验证**: ADC_RDATA 只写 2 字节, 不会溢出到 ACC 区域 -- 排除
3. **NVIC 优先级检查**: TIM16 和 EXTI 同为 (0,0), 不会互相抢占 -- 排除 CS 竞争
4. **寄存器回读**: CTRL_REG6_XL 回读 0x70 -- 配置正确, 不是写入失败
5. **分步测试** (最终定位):
   - 滤波配置 (CTRL_REG7=0xC4): X=195, Y=-33, Z=138 --> 趋零!
   - 禁用滤波 (CTRL_REG7=0x00): X=-5858, Y=-1065, Z=-5624 --> 矢量模~1.0g

**根因**: `CTRL_REG7_XL = 0xC4` 中 HR=1 + FDS=1 启用的 LPF2 数字滤波器导致 ACC 输出趋零

**修复**: `CTRL_REG7_XL = 0x00` (禁用 HR 和 LPF2)

**教训**: LSM9DS1 的 LPF2 滤波器 (FDS=1) 在单独读取 ACC 时可能存在异常行为, 参考代码使用合并 12 字节突发读取可能规避了此问题

### 问题4: ACC Z轴方向 (2026-04-17)

**现象**: Z 轴显示 -1g (应为 +1g)

**根因**: 传感器在 PCB 上的安装方向导致 Z 正轴朝下

**修复**: main.c 数据打包时对 Z 轴取反: `int16_t az = -(int16_t)((ACC_XYZ[5] << 8) | ACC_XYZ[4])`

## 调试方法论

### 1. 寄存器回读验证
```c
// 在 MIMU_Init() 后回读关键寄存器
uint8_t reg6 = ACC_GYRO_Read(0x20);  // CTRL_REG6_XL
uint8_t reg8 = ACC_GYRO_Read(0x22);  // CTRL_REG8
uint8_t stat = ACC_GYRO_Read(0x27);  // STATUS_REG (XLDA)
// 通过 UART 打印验证
```

### 2. STATUS_REG 分析
STATUS_REG (0x27) 比特位:
- bit 0 (XDA): X 轴新数据
- bit 1 (YDA): Y 轴新数据
- bit 2 (ZDA): Z 轴新数据
- bit 3 (ZYXDA): 三轴均有新数据
- bit 7-4: 对应 overrun 标志

如果只有部分轴有数据 (如 ZDA=1 但 XDA=0), 检查 CTRL_REG5_XL (0x1F) 轴使能位

### 3. 灵敏度验证
| 传感器 | 量程 | 灵敏度 | 1g/1dps 对应 LSB |
|--------|------|--------|-----------------|
| ACC +/-2g | +/-2g | 0.061 mg/LSB | ~16393 |
| ACC +/-4g | +/-4g | 0.122 mg/LSB | ~8192 |
| GYRO +/-500dps | +/-500dps | 17.50 mdps/LSB | ~57 |
| GYRO +/-2000dps | +/-2000dps | 70.0 mdps/LSB | ~14 |

### 4. SPI 通信检查清单
- WHO_AM_I (0x0F) 应返回 0x68
- 读操作: bit7=1, 写操作: bit7=0
- 突发读取: bit6=0 (地址递增), 依赖 IF_ADD_INC (CTRL_REG8 bit2)
- CS 管理: 每次传输前后显式控制, 中断回调中注意 CS 竞争

### 5. 上位机解析验证
- ACC: `RANGE_ACC = 4.0 / 32767.0` (g/LSB)
- GYRO: `RANGE_GYRO = 17.50 / 1000.0` (dps/LSB)
- 数据包字节序: 固件打包为大端 (高字节在前), 上位机按大端解析
- 传感器输出为小端 (OUT_X_L 在前), 打包时交换 H/L

## 参考资源

- `docs/another-referrence/LSM9DS1.md` -- 完整寄存器映射和推荐初始化流程
- `docs/another-referrence/LSM9DS1.c` -- DMA 合并读取参考实现 (12 字节 GYRO+ACC)
- `bug/ACC三轴归零问题排查记录.md` -- ACC 归零的完整排查记录
