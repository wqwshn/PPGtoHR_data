# Raw 数据录制文件构成说明

本文说明当前原始数据面板一次录制会生成哪些文件，以及这些文件中有效样本、缺失样本和 `NaN` 占空值的语义。后续心率算法 agent 在读取数据前应先理解本文件。

## 文件集合

每次录制使用同一个时间戳前缀：

- `raw_data_YYYYMMDD_HHMMSS.csv`：只保存 PC 端真实收到且通过校验的 Raw DATA 帧。
- `raw_data_YYYYMMDD_HHMMSS_status.csv`：保存 1Hz 固件 STATUS 诊断快照和 PC 端链路统计。
- `raw_data_YYYYMMDD_HHMMSS_timeline.csv`：按 100Hz 设备样本轴展开的规则时间轴，显式插入缺失行。
- `raw_data_YYYYMMDD_HHMMSS_quality_events.csv`：每个 Raw 序号缺口记录一行质量事件。

## `raw_data_YYYYMMDD_HHMMSS.csv`

这是紧凑的原始接收文件，只包含已到达 PC 并通过帧头、帧尾和 XOR 校验的 Raw 帧。

关键字段：

- `Time(s)`：设备样本轴时间，由 100Hz 样本索引换算得到；不是 PC 接收 wall-clock，也不是 CSV 行号。
- `Seq`：固件侧 16 位 Raw 序号，超过 `65535` 后回绕。
- `MissingBefore`：当前行前由 `Seq` 缺口推断的缺失样本数。
- 传感器字段：桥压、加速度、陀螺仪和 PPG 三通道等真实收到的帧内容。

该文件不为缺失样本插入占位行。发生 BLE gap 后，`Time(s)` 会跳过缺失段，`MissingBefore` 记录该段长度。

## `raw_data_YYYYMMDD_HHMMSS_timeline.csv`

这是需要规则 100Hz 时间轴的算法优先使用的文件。

行语义：

- 真实收到的样本行：`ValidFlag=1`，`InterpFlag=0`。
- 缺失样本占位行：`ValidFlag=0`，`InterpFlag=0`。
- 缺失行的传感器数值写为 `NaN`，不是 0。
- `GapLen` 记录该缺失行所属缺失段的长度。

当前 BLE 周期性缺失常见长度为 30-50 个样本。该长度已经属于中长 gap，当前版本不自动插值（No automatic interpolation）。未来如果需要，可另行生成只针对 1-3 样本短 gap 的插值派生文件，但插值行仍应保留 `ValidFlag=0`，并设置 `InterpFlag=1`。

## `raw_data_YYYYMMDD_HHMMSS_quality_events.csv`

该文件记录由 Raw `Seq` 不连续推断出的 gap 事件。

典型字段：

- `EventType`：当前为 `seq_gap`。
- `GapStartSampleIndex`：缺失段在 100Hz 设备样本轴上的起点。
- `GapLen`：缺失样本数量。
- `NextSeq`：缺失段后第一个真实收到 Raw 帧的序号。
- `PcMissingRaw`：记录该事件时 PC 端累计缺失样本数。

心率算法可用该文件提前生成缺失掩码，对跨越中长 gap 的窗口执行跳过、切分或低可信标记。

## `raw_data_YYYYMMDD_HHMMSS_status.csv`

该文件保存 1Hz STATUS 快照和 PC 端解析统计，主要用于链路诊断，不应当直接作为生理信号输入。

关键字段：

- 固件计数器：`sample_counter`、`frame_counter`、`tx_start_counter`、`tx_done_counter`、`tx_busy_counter`、`tx_error_counter`、传感器错误计数和 PPG FIFO 计数等。
- PC 统计：Raw 接收数、期望数、缺失数、丢失率、Raw 候选帧数、无效候选帧数和相邻 STATUS 间无效候选帧增量。
- `PCMissingAfterTxDone`：从本次录制首个 STATUS 建立基线后，对比固件已完成发送的 Raw 帧增量与 PC 已解析 Raw 帧增量，用于估计 MCU 发送完成后到 PC 解析成功之间的缺失量。

该文件应随 Raw CSV 一起保存，便于后续复盘数据质量和链路状态。

## `NaN` 对算法的含义

`_timeline.csv` 中的 `NaN` 表示“该样本索引没有收到有效传感器样本”。它不是物理 0 值，不是传感器饱和值，也不是上一有效样本的延续。

建议算法处理方式：

- 需要固定 100Hz 时间轴时优先读取 `_timeline.csv`。
- 滤波、FFT、模型输入或窗口统计默认排除 `ValidFlag=0` 行，除非算法显式设计了缺失数据策略。
- 覆盖 30-50 样本 BLE gap 的心率窗口应标记为低可信，或在 gap 两侧切分。
- 使用 `_quality_events.csv` 预先构建缺失掩码。
- 只有在算法已经能处理非均匀时间间隔时，才直接读取紧凑的 `raw_data_YYYYMMDD_HHMMSS.csv`，并必须使用 `Time(s)` 和 `MissingBefore` 判断缺口。
