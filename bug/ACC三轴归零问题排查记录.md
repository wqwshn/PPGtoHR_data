# ACC 三轴加速度计数据归零问题排查记录

**日期**: 2026-04-17
**状态**: 待修复
**现象**: 静态水平放置, ACC 三轴均接近 0g, Z 轴应为 ~1g

---

## 1. 实测数据统计

从 `一次采集数据-存在问题.csv` (9774 样本, ~100s):

| 轴   | 均值 (g)      | 标准差 (g)   | 原始 LSB 偏移 |
|------|---------------|--------------|---------------|
| AccX | -0.000001     | 0.000927     | ~0 LSB        |
| AccY | +0.000001     | 0.000412     | ~0 LSB        |
| AccZ | +0.000002     | 0.001181     | ~0 LSB        |

**预期**: AccX/Y ~0g, AccZ ~1g (约 8192 LSB). 实际三轴均为 0 LSB 级别.

---

## 2. 排除项

### 2.1 上位机解析 -- 排除

- `protocol.py` 中 `RANGE_ACC = 4.0 / 32767.0` (0.122 mg/LSB), 匹配 +/-4g 量程
- 字节偏移 data[10..15] 与固件 allData[10..15] 一一对应
- GYRO 数据解析正确 (同文件同结构), 证明帧同步和校验无误

### 2.2 数据打包覆盖 -- 排除

- ADC_RDATA(&allData[n]) 只写 2 字节, 不会溢出到 allData[10+] 区域
- ACC 打包代码 `allData[10] = ACC_XYZ[1]` 在 `if(flag==4)` 块内, 位置正确
- ACC_XYZ[] 除 ACC_6BytesRead() 外无其他写入点

### 2.3 竞争条件 -- 低概率

- TIM16 定时器回调中 `HAL_GPIO_WritePin(CS_A_G_..., SET)` 会释放 MIMU CS
- 定时器周期 10ms (100Hz), DRDY 中断滞后约 5ms, ACC/GYRO 读取耗时约 200us
- 理论上定时器不会在 ACC/GYRO SPI 传输期间抢占 (间隔 5ms)
- **但**: 如果中断优先级配置不当, TIM16 抢占 EXTI 回调会导致 SPI 传输中断
- 需确认 NVIC 优先级: EXTI 应高于 (数值小于) TIM16

---

## 3. 高度可疑项

### 3.1 ACC 可能未正确启用 (最可能)

固件写入 `CTRL_REG6_XL(0x20) = 0x70` 配置 119Hz +/-4g, 但无法确认写入是否生效:
- 没有回读验证
- SW_RESET 后寄存器可能未完全复位
- 信号链路上可能存在干扰

**验证方法**: 在 MIMU_Init() 后回读 CTRL_REG6_XL, 通过 UART 打印确认值

### 3.2 CTRL_REG7_XL 数字滤波器交互

`CTRL_REG7_XL = 0xC4`:
- HR=1 (高分辨率), FDS=1 (滤波后输出), DCF=10
- 如果 ACC 数据路径未正确建立, 滤波器输出可能为零
- **验证**: 尝试将 CTRL_REG7_XL 设为 0x00 (旁路滤波器) 观察 ACC 是否恢复

### 3.3 LSM9DS1 HP 滤波器未路由到输出 (已确认)

`CTRL_REG2_G(0x11)` 默认 0x00, OUT_SEL[1:0]=00:
- HP 滤波器输出 **未路由到数据输出寄存器**
- 仅 LPF1 (BW=14Hz) 实际作用于 GYRO 输出数据
- 这解释了 GYRO 零偏为何未被 HP 滤波器消除

---

## 4. 建议排查步骤

1. **回读 ACC 配置寄存器**: 在 MIMU_Init() 结束后读 CTRL_REG6_XL, 通过 UART 打印
2. **读 STATUS_REG (0x27)**: 检查 bit[3] XYZDA 是否为 1 (ACC 新数据就绪)
3. **旁路数字滤波器**: 将 CTRL_REG7_XL 临时设为 0x00, 观察 ACC 是否恢复
4. **裸读 ACC 输出**: 在 MIMU_Init() 后直接调用 ACC_6BytesRead() 并打印 ACC_XYZ[] 原始字节
5. **检查 NVIC 优先级**: 确认 EXTI 优先级高于 TIM16

---

## 5. 涉及文件

- `Core/Src/MIMU.c` -- ACC_6BytesRead(), MIMU_Init()
- `Core/Src/main.c` -- 数据打包 (line 383-389), EXTI 回调 (line 781-784)
- `tools/monitor/protocol.py` -- ACC 解析 (RANGE_ACC)
