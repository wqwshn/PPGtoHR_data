# Raw 数据录制文件构成说明

本文说明当前原始数据面板一次录制会生成哪些文件，以及文件中有效样本、缺失样本和 `NaN` 占空值的语义。

## 文件集合

每次录制使用同一个时间戳前缀，只生成两个 CSV：

- `raw_data_YYYYMMDD_HHMMSS.csv`：主原始数据文件，按 100Hz 设备样本轴展开；真实样本写入传感器数值，缺失样本显式写入 `NaN` 占位。
- `raw_data_YYYYMMDD_HHMMSS_status.csv`：保存 1Hz 固件 STATUS 诊断快照和 PC 端链路统计，用于复盘发送异常、解析异常和链路状态。

不再额外生成 `_timeline.csv` 和 `_quality_events.csv`。原 `_timeline.csv` 的时间轴补齐语义已经合并到无后缀主 CSV；序号 gap 信息由主 CSV 的 `ValidFlag`、`GapLen`、`MissingBefore` 和 `_status.csv` 中的统计量共同描述。

## `raw_data_YYYYMMDD_HHMMSS.csv`

这是后续算法优先读取的原始数据文件。字段语义：

- `Time(s)`：设备样本轴时间，由 100Hz 样本索引换算得到，不是 PC 接收 wall-clock。
- `SampleIndex`：100Hz 设备样本轴索引。
- `Seq`：固件侧 16 位 Raw 序号，超过 `65535` 后回绕。
- `ValidFlag`：`1` 表示真实收到并通过校验的 Raw 样本；`0` 表示由序号缺口补出的缺失样本行。
- `InterpFlag`：当前仍为 `0`，表示缺失行只占位，不做自动插值。
- `GapLen`：缺失行所属 gap 的长度；真实样本行填 `0`。
- `MissingBefore`：当前真实样本前由 `Seq` 缺口推断的缺失样本数。
- 传感器字段：桥压、加速度、陀螺仪和 PPG 三通道。缺失样本行统一写 `NaN`，不是物理 0 值，也不是上一有效样本延续。

蓝牙密码认证问题清除后，实测丢包率已经明显降低；因此多数录制中主 CSV 基本就是连续 100Hz 时间轴。若仍出现少量缺失，仍沿用 `NaN` 占位逻辑。

## `raw_data_YYYYMMDD_HHMMSS_status.csv`

该文件保存 1Hz STATUS 快照和 PC 端解析统计，主要用于链路诊断，不应直接作为生理信号输入。

关键字段：

- 固件计数器：`SampleCounter`、`FrameCounter`、`TxStartCounter`、`TxDoneCounter`、`TxBusyCounter`、`TxErrorCounter`、传感器错误计数和 PPG FIFO 计数等。
- PC 统计：Raw 接收数、期望数、缺失数、Raw 候选帧数、无效候选帧数和相邻 STATUS 间无效候选帧增量。
- `PcMissingAfterTxDone`：从本次录制首个 STATUS 建立基线后，对比固件已完成发送的 Raw 帧增量与 PC 已解析 Raw 帧增量，用于估计 MCU 发送完成后到 PC 解析成功之间的缺失量。

该文件应随 Raw CSV 一起保存，便于后续复盘数据质量和链路状态。

## `NaN` 对算法的含义

主 CSV 中的 `NaN` 表示“该样本索引没有收到有效传感器样本”。建议算法默认排除 `ValidFlag=0` 行，除非显式设计了缺失数据策略。覆盖较长 gap 的心率窗口应标记为低可信，或在 gap 两侧切分。
