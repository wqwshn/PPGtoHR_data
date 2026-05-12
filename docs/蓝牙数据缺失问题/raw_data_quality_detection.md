# 原始数据模式数据质量检测体系说明

本文档阐述单片机原始数据模式下，系统如何对发送数据的质量进行多层级的链路诊断。

## 1. 体系总览

质量检测覆盖从 MCU 采样触发到 PC 端文件保存的全路径，分为三个层面：

- **MCU 固件侧**：12 路诊断计数器 + STATUS 状态帧
- **UART/DMA 物理层**：发送状态监控 + 调度策略
- **PC 上位机侧**：帧校验 + 序号缺口 + STATUS 对照 + 三类输出文件

核心诊断思路是**计数器守恒关系**：全链路各环节的计数增量应该相等，差值即定位丢失环节。

```
TIM16 采样 → ADC DRDY → 主循环组帧 → DMA 启动 → DMA 完成 → UART/BLE → PC 接收
  ↑            ↑           ↑           ↑           ↑              ↑
sample     adc_drdy    frame      tx_start    tx_done       PC valid frame
counter     counter    counter    counter     counter
```

## 2. MCU 固件侧：12 路诊断计数器

定义于 `Core/Src/main.c:74-85`，12 个 `volatile uint32_t` 计数器：

| 计数器 | 递增位置 | 语义 |
|---|---|---|
| `sample_counter` | `HAL_TIM_PeriodElapsedCallback()` | MCU 触发 AD 采样的总次数 (TIM16 周期，100Hz) |
| `adc_drdy_counter` | `HAL_GPIO_EXTI_Callback()` DRDY | ADC 完成单次转换的回调次数 |
| `frame_counter` | 主循环组帧后 | Raw DATA 帧实际组帧并尝试发送的次数 |
| `tx_start_counter` | DMA 启动成功 (HAL_OK) | `HAL_UART_Transmit_DMA()` 返回 HAL_OK |
| `tx_done_counter` | `HAL_UART_TxCpltCallback()` | Raw DATA 帧 DMA 发送完成回调 |
| `tx_busy_counter` | DMA 启动失败 (HAL_BUSY) | DMA 通道被占用时新帧尝试发送 |
| `tx_error_counter` | DMA 返回 HAL_ERROR + UART ErrorCallback | 发送错误累计 |
| `adc_error_counter` | ADC 状态机 default 分支 | ADC 读取状态机进入未知状态 |
| `imu_error_counter` | (预留) | IMU 读取异常 |
| `ppg_fifo_empty_counter` | 主循环 PPG FIFO 读取 | FIFO 写指针与读指针差为 0 |
| `ppg_fifo_overflow_counter` | 主循环 PPG FIFO 读取 | FIFO 可用样本数 >= 31，逼近溢出 |

### 2.1 计数器递增细节

**采样触发级** (`main.c:792-797`)：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim16) {
        raw_diag_sample_counter++;
        if ((raw_diag_sample_counter % PPG_SAMPLE_RATE) == 0) {
            raw_diag_status_pending = 1;  // 每秒标记一次 STATUS 待发
        }
        // 触发 ADC 转换 ...
    }
}
```

**ADC DRDY 级** (`main.c:807-860`)：每次 ADC 数据就绪中断递增 `adc_drdy_counter`。状态机 default 分支递增 `adc_error_counter`。

**PPG FIFO 级** (`main.c:437-444`)：

```c
uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;
if (sample_count == 0)       → ppg_fifo_empty_counter++;
else if (sample_count >= 31) → ppg_fifo_overflow_counter++;
```

FIFO 空读时固件复用上一有效均值，避免跳变到 0。该现象被计数器记录，用于后诊断而非实时阻断。

**组帧与发送级** (`main.c:570-582`)：

```c
raw_diag_frame_counter++;
HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN);
if (tx_status == HAL_OK)       → tx_start_counter++
else if (tx_status == HAL_BUSY) → tx_busy_counter++
else                           → tx_error_counter++
raw_packet_seq++;
```

**DMA 完成与错误回调** (`main.c:871-900`)：`HAL_UART_TxCpltCallback` 中按 `raw_diag_dma_tx_kind` 区分帧类型，仅对 DATA 帧递增 `tx_done_counter`。`HAL_UART_ErrorCallback` 中递增 `tx_error_counter`。

## 3. STATUS 状态帧

### 3.1 帧格式

STATUS 帧头为 `0xAA 0xDD`，固定 53 字节 (`main.h:141`)，1Hz 上报。

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0-1 | Header | 0xAA, 0xDD |
| 2 | protocol_version | uint8 (当前 = 1) |
| 3-6 | mcu_time_ms | uint32 BE |
| 7-10 | sample_counter | uint32 BE |
| 11-14 | adc_drdy_counter | uint32 BE |
| 15-18 | frame_counter | uint32 BE |
| 19-22 | tx_start_counter | uint32 BE |
| 23-26 | tx_done_counter | uint32 BE |
| 27-30 | tx_busy_counter | uint32 BE |
| 31-34 | tx_error_counter | uint32 BE |
| 35-38 | adc_error_counter | uint32 BE |
| 39-42 | imu_error_counter | uint32 BE |
| 43-46 | ppg_fifo_empty_counter | uint32 BE |
| 47-50 | ppg_fifo_overflow_counter | uint32 BE |
| 51 | XOR | bytes[2..50] 异或 |
| 52 | 0xCC | 帧尾 |

### 3.2 STATUS 发送调度

STATUS 帧不主动抢占 UART DMA。调度策略 (`main.c:871-887`)：

1. Raw DATA 帧 DMA 完成后，回调检查 `raw_diag_status_pending`
2. 若有待发 STATUS，在 DATA 完成后顺带启动 STATUS DMA
3. `raw_diag_dma_tx_kind` 标记帧类型 (1=DATA, 2=STATUS)

此设计消除了早期版本中 STATUS 主动抢占 DMA 导致的约 1Hz 系统性 `tx_busy` 缺口。

## 4. Raw DATA 帧内嵌质量信息

每个 35 字节 Raw DATA 帧 (`0xAA 0xBB`) 包含：

| 字段 | 说明 |
|---|---|
| `raw_packet_seq` (bytes[31..32], uint16 BE) | 固件侧采样序号，0-65535 后回绕 |
| XOR 校验 (byte[33]) | bytes[2..32] 异或 |
| 帧尾 0xCC (byte[34]) | 帧尾标识 |

## 5. 计数器守恒诊断法

`docs/raw_data_link_upgrade_plan.md` 第 8.2 节定义的判断规则：

| 计数差值 | 可能原因 | 优先排查方向 |
|---|---|---|
| `sample_counter` > `frame_counter` | 采样了但未组帧 | 主循环阻塞、状态机卡死 |
| `frame_counter` > `tx_start_counter + tx_busy_counter + tx_error_counter` | 组帧了但发送启动路径不完整 | 计数器覆盖完整性 |
| `tx_busy_counter` 持续增长 | DMA 发送启动被阻塞 | 双缓冲或小环形队列 |
| `tx_error_counter` 增长 | UART/DMA 错误 | UART ErrorCallback、DMA IRQ |
| `tx_start_counter` 与 `tx_done_counter` 不追平 | DMA 完成回调异常 | HAL_UART_TxCpltCallback 和 DMA 中断 |
| `tx_done_counter` > PC received | MCU 发完但 PC 未收到 | UART/BLE 透传链路或 PC 解析 |
| gap 为 1-3 样本 | 短暂拥塞 | 可先靠缓冲优化，后续短缺失插值 |
| gap 成片出现 | 吞吐不足或阻塞 | 优先解决链路瓶颈 |

### 5.1 PC 侧支持判断的额外指标

| 指标 | 计算方式 | 用途 |
|---|---|---|
| `PcMissingAfterTxDone` | `tx_done_delta - pc_received_delta` (基线化) | MCU DMA 完成后到 PC 解析成功之间的缺失量 |
| `PcRawInvalidDelta` | 相邻 STATUS 间新增无效候选帧 | 区分"字节未到"还是"到了但校验失败" |
| `tx_inflight` | `tx_start_counter - tx_done_counter` | 有多少启动的发送尚未完成 |

判断逻辑：

- 若周期性 gap 出现时 `PcRawInvalidDelta` 同步增加 → 字节到达 PC 但帧损坏或状态机失同步，应升级帧保护
- 若周期性 gap 出现时 `PcRawInvalidDelta` 仍为 0 → PC 未形成对应候选帧，丢失在 UART/BLE 透传或底层驱动

## 6. 上位机侧质量检测

### 6.1 帧级校验 (`protocol.py:211-273`)

每帧必须通过四级校验：

1. 长度 = 35 字节
2. 帧头 = 0xAA 0xBB
3. 帧尾 = 0xCC
4. XOR 校验 (bytes[2..32])

任一失败则帧被丢弃并计入 `_raw_invalid`。

### 6.2 序号缺口统计 (`raw_quality.py:54-72`)

支持 uint16 回绕 (模 65536)，计算：

- `missing_before = (sequence - last_sequence) % 65536 - 1`
- `loss_rate = missing_count / expected_count`

### 6.3 三类输出文件

| 文件 | 内容 |
|---|---|
| `raw_data_*_raw.csv` | 仅保存真实收到并通过校验的帧，`Seq` + `MissingBefore` |
| `raw_data_*_timeline.csv` | 按 100Hz 样本轴展开规则时间轴，缺失行填 `NaN` + `ValidFlag=0` |
| `raw_data_*_quality_events.csv` | 每条 gap 记录起点、长度、后继序号 |
| `raw_data_*_status.csv` | 1Hz STATUS 快照 + PC 端链路统计 |

### 6.4 `NaN` 语义

Timeline CSV 中的 `NaN` 表示"该样本索引没有收到有效传感器样本"。它不是物理 0 值，也不是传感器饱和值。算法处理斐时应注意：

- 需要固定 100Hz 时间轴时优先读取 `_timeline.csv`
- 滤波、FFT、模型输入默认排除 `ValidFlag=0` 行
- 覆盖 BLE gap 的心率窗口应标记为低可信或在 gap 两侧切分

## 7. 实际诊断案例

来自 2026-04-30 的两次 15 分钟采集反馈分析：

### 7.1 第一次采集

| 指标 | 值 | 诊断 |
|---|---|---|
| `tx_busy_counter` 增量 | ~900 | STATUS 帧主动抢占 UART DMA |
| 1 样本 gap 数量 | 883 | 对应 ~1Hz 的系统性 busy 缺口 |
| `Loss` | 3.56% | |

**修复**：将 STATUS 帧改为 DMA 顺带发送 (`main.c:871-887`)。

### 7.2 第二次采集 (修复后)

| 指标 | 值 | 诊断 |
|---|---|---|
| `tx_busy_counter` | 0 | 系统性 busy 已消除 |
| `Loss` | 2.36% (下降约 1 个百分点) | |
| 成片 gap (30-50 样本) | 周期性约 14.7s | 与 HJ-380 BLE DONGLE 蓝灯熄灭吻合 |
| MCU 侧所有计数连续 | 采样/组帧/DMA 无异常 | 剩余丢失在 UART/BLE 透传链路 |

**结论**：剩余缺失优先定位为 HJ-131IMH + HJ-380 BLE 透传链路的周期性短时空窗，非 MCU 固件问题。

### 7.3 2026-05-12 根因确认

后续实测确认，14-15s 周期性断联与 HJ-131IMH 蓝牙从机连接密码认证有关。手册说明：若模块设置了连接密码，主机连接后默认 15s 内必须通过 APP->BLE 通道发送密码，失败或超时会断开连接。通过 `<ST_FACTORY>` 恢复出厂设置并 `<ST_CLEAR_SECRET>` 清空密码后，周期性断联问题消失。

当前固件 BLE 配置策略：
- 临时将 `ENABLE_BLE_CONFIG=1` 烧录一次：先兼容 115200bps 发送 `<ST_FACTORY>`，再切到出厂默认 19200bps 完成恢复出厂、清密码、`FOREVER` 模式、名称、功率、连接间隔和 115200bps 配置。
- 配置完成后将 `ENABLE_BLE_CONFIG=0` 作为正常采集默认值，避免每次采集前重启或重配 BLE；`ST_WAKE=FOREVER` 为掉电保存配置。

## 8. 相关文件索引

| 文件 | 内容 |
|---|---|
| `Core/Inc/main.h` | STATUS 帧常量、协议开关、数据包偏移定义 |
| `Core/Src/main.c` | 诊断计数器声明、递增逻辑、BuildStatusFrame、DMA 回调 |
| `tools/monitor/protocol.py` | StatusPacket 解析、parse_status_packet() |
| `tools/monitor/raw_quality.py` | RawQualityStats 序号缺口统计、STATUS 对照分析 |
| `docs/raw_data_link_upgrade_plan.md` | 链路诊断与时间轴补救完整方案 |
| `docs/raw_data_file_structure.md` | 录制文件构成说明 (raw/timeline/quality_events/status) |

---

该体系已成功实现"先诊断、后修复、再补救"的策略闭环：先通过计数器守恒定位丢失环节，再针对性地消除 DMA busy 等固件侧问题，最后通过三类输出文件对剩余链路缺失进行显式标记和时间轴补救。
