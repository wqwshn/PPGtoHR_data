# PPG Monitor 统一上位机 UI 说明文档

## 当前统一入口（2026-09）

根目录 `start.bat` 或旧 `tools/monitor/start_monitor.bat` 均进入 `Workbench`。
“原始数据采集”嵌入原始数据窗口；“固件配置与烧录”提供 IIC 通道、蓝牙初始化、采集光模式和 SWD 速度选择。
烧录页使用 QProcess 顺序执行 CMake 配置、构建和 OpenOCD；失败停止后续操作，进程日志落到 `.local/logs`。
当前输出固定 Raw 100Hz，与采集时间轴一致。保存选择到 `config/firmware.json`，通过 CMake 宏生效，不改写源码。
采集默认保存至项目 `recordings/`，模拟入口在工作台右上角。硬件连接检查和烧录前需要停止录制，并断开采集；窗口关闭统一释放串口和 CSV。
详见 [统一工作台使用说明](统一工作台使用说明.md)。

> 本文档记录统一上位机 monitor 的设计、功能、协议交互，供开发和调试参考。
> 由原 hr_monitor 和 data_monitor 融合而来。

---

## 1. 概述

基于 PyQt5 + pyqtgraph 构建的暗色主题统一监测上位机，通过串口/蓝牙接收 STM32 单片机数据，当前 UI 只显示原始数据页面；下文在线心率面板章节保留为历史实现参考，不再提供入口。协议解析层仍可识别 HR 包，但不连接独立 HR 展示和录制页面。

历史面板与当前原始数据功能：
- **在线心率面板**: 1Hz 心率结果包 (31字节, 0xAA 0xCC)，实时展示融合心率、三路径对比、趋势曲线
- **原始数据面板**: 100Hz 原始传感器包 (35字节, 0xAA 0xBB)，实时展示 PPG 波形、热膜桥压、加速度计、陀螺仪和链路质量；同时接收 1Hz Raw 链路诊断 STATUS 包 (69字节, 0xAA 0xDD)

### 1.1 运行方式

```bash
# 实际串口模式 (自动识别两种协议包)
python tools/monitor/main.py

# 原始数据模拟（兼容旧 --simulate 参数）
python tools/monitor/main.py --simulate

# 原始数据模拟模式 (100Hz)
python tools/monitor/main.py --raw-simulate
```

### 1.2 依赖

- Python >= 3.9
- PyQt5
- pyqtgraph
- pyserial

安装: `pip install -r tools/monitor/requirements.txt`

---

## 2026-09-15 合并后的页面与录制

- 工作台采用远程双页结构：“原始数据采集”和“固件配置与烧录”；不创建独立 HR 页面。
- 字体沿用远程 `Arial` / `SimSun` 字体族；工具栏为 10pt、30 逻辑像素高度，状态信息 9pt，启用高 DPI 缩放。
- 工具栏第一行放热膜显示切换、串口、连接/断开、刷新和连接状态；第二行放清屏、保存路径、实验信息、Marker 与录制。
- 状态区第一行显示采样率、丢包和计数；第二行单独显示实时 FFT 心率、置信信息、频谱 SNR（dB）与计算耗时；第三行显示链路诊断与设备消息。长信息允许换行。
- 保留本地 PPG nA 显示和曲线均值；CSV 的 PPG 原始 ADC 码值不变。
- 保留“实验信息”命名与按需 Marker，禁止覆盖同名文件。热膜原始值、中值列及 `Median3Valid` 一起记录，另存 `_processing.json`；RF 事件按需写入 `_rf_events.csv`。
- 清屏停止录制并重置按钮、计数和曲线；切换热膜显示不会中断录制。
- 保留 HJ-380 MAC 绑定和帧内字节保护，新增 RF/功率事件解析。`--simulate` 与 `--raw-simulate` 都进入 Raw 模拟。
- 双路采集中的“有线参考”窗口仅接收，不发送 HJ-380 绑定命令；“无线链路”窗口保留自动绑定，两个窗口分别保存。
- 烧录页支持垂直滚动，较小窗口仍可访问全部配置和日志。

录制字段见[文件结构说明](acquisition/原始数据录制文件结构说明.md)。以下双面板章节是历史参考，不作为当前页面操作说明。

## 2. 功能模块（历史双面板设计参考）

### 2.1 工具栏

| 控件 | 功能 |
|------|------|
| 状态指示灯 | 绿色脉冲=已连接, 红色=未连接 |
| 串口选择 | 自动枚举系统可用串口 |
| 连接/断开 | 管理串口连接 (115200 bps) |
| 刷新 | 重新扫描串口列表 |
| **面板切换** | "在线心率" / "原始数据" 双面板切换按钮 |
| **清屏** | 清除当前面板的所有数据、曲线、记录缓冲区 |
| **保存路径** | 显示当前保存目录名称, 点击可切换目录, 默认桌面 |
| **实验信息** | 为 Raw 录制设置保存根目录、场景、试次、受试者缩写和实验日期，并预览目标路径 |
| **录制** | Raw 面板需先确认实验信息后开始录制；再次点击停止并保存 |
| **语言切换** | 中文/English 即时切换，默认中文 |
| 连接状态 | 文字显示当前连接状态 |

### 2.2 心率卡片

- 大字显示当前融合心率 BPM
- 颜色阈值: <60 蓝色, 60-100 绿色, 100-140 橙色, >140 红色
- 运动状态标签: 静息(绿色) / 运动(橙色)

### 2.3 算法路径卡片

三路径 BPM 对比进度条:
- **LMS-HF**: LMS 自适应滤波 + HF 桥臂参考 (青色)
- **LMS-ACC**: LMS 自适应滤波 + ACC 参考计 (紫色)
- **FFT**: 纯 FFT + Hamming 窗 (橙色)

附加信息:
- PPG 信号均值 (信号强度参考)
- 校准进度 (0-8, 运动阈值自动标定)
- 数据窗口状态 (填充中/就绪)

### 2.4 趋势图

- 显示最近 60 秒心率曲线
- 绿色参考区域: 60-100 BPM 正常心率范围
- X/Y 轴自动缩放

### 2.5 原始数据面板

顶部信息条显示: 当前模式(HR/SpO2) | 实际采样率 | 丢包率 | 数据包总数

**PPG 波形区** (模式切换):
- HR 模式: 绿光 PPG 波形 (100Hz 实时)
- SpO2 模式: 左侧红光+红外 PPG 上下排列 / 右侧温度曲线 + SpO2 预留

**通用波形** (始终显示):
- 桥顶电压: Ut1 (橙色) + Ut2 (紫色) 水平并排
- 桥中电压: Uc1 (红色) + Uc2 (蓝色) 水平并排
- 三轴加速度: AccX(红) + AccY(绿) + AccZ(蓝) 单图三线

波形以 33ms (30FPS) 定时刷新，缓冲区 2000 点，可见窗口 800 点 (8 秒)。信息条按帧率更新而非逐包刷新。

原始数据面板顶部信息条新增 `实时心率/Realtime HR`: 每 1 秒从绿光 PPG 缓冲区取最近 8 秒数据执行轻量纯 FFT 估计，仅用于静息采集时快速判断佩戴位置与绿光信号是否可解算。该计算不进入串口读取线程，也不在逐包 CSV 写入路径中执行；界面同步显示 `SNR xx.x dB` 和单次计算耗时 `Calc xx ms`。

实时 FFT 先去均值、加 Hamming 窗，并在 0.7–4.0 Hz（42–240 BPM）范围取功率最大峰。绿光 PPG 去均值后的峰峰值低于 50 原始码值（约 3.125 nA）时，仍显示“信号弱”而不输出 BPM。SNR 采用 `10 × log10(主峰功率 / 噪声底功率)`；噪声底为排除主峰前后各 2 个 FFT bin 后，其余频点功率的中位数。SNR 当前仅作同步质量显示，不参与“信号弱”拦截，便于区分低幅但频谱清晰的信号与频谱不清晰的信号。

原始数据面板顶部信息条新增 `诊断/Diag`: 显示固件 1Hz STATUS 帧中的关键链路诊断摘要。中文模式显示 `发送忙 x | 发送错误 y | 发送后缺口 z | FIFO空/溢出 empty/overflow | PPG均值 n.nn`，英文模式显示 `Busy x | Err y | PCGap z | FIFO empty/overflow | PPGAvg n.nn`。其中 `PPG均值/PPGAvg` 表示每个非空 PPG FIFO 读取周期的平均样本数，用于判断 100Hz Raw 周期内是单样本读取还是多样本均值读取。

### 2.6 状态栏

显示: 数据包计数 | 运行时间 | [录制数据点数(仅录制时)] | PPG 均值 | 校准进度

---

## 3. 数据录制

### 3.1 录制机制

- 工具栏"保存路径"按钮显示当前保存目录，点击可选择其他目录，默认桌面
- Raw 面板点击“实验信息”后，以模态对话框填写保存根目录、实验场景、正整数试次、1–8 位大写 ASCII 受试者缩写和实验日期；界面实时预览目录和主文件名
- Raw 场景采用中文显示与稳定 token 分离的方式保存。新增“开机”场景 token `kaiji`，用于手动记录传感器开机阶段的信号；不会在连接或首个 Raw 包到达时自动开始
- 点击 Raw “录制”按钮时先校验元数据和主/`_status`/`_markers` 三个潜在路径；任一路径已存在即拒绝录制，不静默覆盖
- 录制期间每收到一个心率结果包 (1Hz)，记录所有解算字段到内存缓冲区
- 时间列采用相对时间 (从录制开始的第一个数据包计，单位秒，精度毫秒)
- 再次点击按钮停止录制，数据自动写入之前选择的 CSV 文件
- 原始数据面板录制: 逐包实时写入 CSV，每 100 包刷盘一次 (避免 GUI 线程阻塞导致丢包)
- 原始数据面板主文件使用 `<scenario><trial>_<SUBJECT>_<MMDD>.csv`，保存在 `<保存根目录>\YYYYMM-multiperson\MMDD-SUBJECT\`；例如 `202607-multiperson\0710-LYX\kaiji1_LYX_0710.csv`
- Raw 录制开始时生成主 Raw CSV 和同 stem 的 `_status.csv`；`_markers.csv` 仅在至少一次点击标记按钮后生成
- 无后缀主 Raw CSV 默认按 100Hz 设备样本轴展开；真实样本 `ValidFlag=1`，若仍有缺失则插入 `ValidFlag=0` 且传感器字段为 `NaN` 的占位行
- Raw `Seq` 仅在向前递增或 65535 后真实回绕时推进时间轴；重复 Seq 或小幅回退/乱序包会被计为异常并跳过，不再展开成 655xx 行 `NaN`
- `_status.csv`: 记录 1Hz STATUS 计数器快照、PPG FIFO 样本数分布、PC 端 Raw 接收/缺失统计、Raw 候选帧解析统计和 `PCMissingAfterTxDone`，用于采集后诊断链路瓶颈
- 串口读取线程采用 0.01s timeout + 最多 4 个 Raw 包的小块读取，避免 4096 字节读取造成约 124 包批量进入 UI

### 3.2 CSV 格式

在线心率面板仍使用默认文件名 `hr_data_YYYYMMDD_HHMMSS.csv`。Raw 面板使用 `<scenario><trial>_<SUBJECT>_<MMDD>.csv`，保存路径在录制前通过“实验信息”确定。

CSV 列定义:

| 列名 | 类型 | 说明 |
|------|------|------|
| time | float | 相对时间 (秒) |
| fused_bpm | float | 融合心率 (BPM) |
| is_motion | int | 运动标志 (0/1) |
| win_filled | int | 窗口填充状态 (0/1) |
| hr_lms_hf | float | LMS-HF 路径 BPM |
| hr_lms_acc | float | LMS-ACC 路径 BPM |
| hr_fft | float | FFT 路径 BPM |
| ppg_mean | int | PPG 信号均值 (缩放值 = 原始均值 / 8) |
| motion_calibrated | int | 运动阈值是否已校准 (0/1) |
| timestamp | int | 单片机秒计数器 |
| calib_progress | int | 校准进度 (0-8) |
| sampling_rate | int | 采样率 (Hz) |

**注意**: `ppg_mean` 为缩放后的值，真实 PPG 均值需乘以 8。正常佩戴时应在 5000-15000 范围，对应原始值 40000-120000。

### 3.3 清屏行为

点击"清屏"按钮会同时清除记录缓冲区并重置录制状态。如需保留数据，请先停止录制再清屏。

---

## 4. 语言切换

支持中文和 English，默认中文。点击语言切换按钮即时切换所有 UI 文本:

- 窗口标题
- 工具栏按钮文字
- 算法路径卡片标题和信息标签
- 运动状态标签 (静息/运动 or REST/MOTION)
- 校准和窗口状态文本
- 状态栏文本

---

## 5. 通信协议

### 5.1 心率结果包 (31 字节, 1Hz, 帧头 0xAA 0xCC)

```
偏移  字段                类型        说明
0-1   帧头                uint8 x2    0xAA, 0xCC
2-3   融合心率 BPM        uint16 BE   x10 精度 (72.5 = 725)
4     运动标志             uint8       0=静息, 1=运动
5     窗口填充状态         uint8       0=未满, 1=已满
6-7   LMS-HF 路径 BPM     uint16 BE   x10 精度
8-9   LMS-ACC 路径 BPM    uint16 BE   x10 精度
10-11 FFT 路径 BPM        uint16 BE   x10 精度
12-13 PPG 信号均值         uint16 BE   信号强度
14    运动校准状态         uint8       0=未校准, 1=已校准
15-16 时间戳              uint16 BE   秒计数器
17    校准窗口进度         uint8       0-8
18    采样率              uint8       固定 100
19    XOR 校验             uint8       bytes[2..18] 异或
20    帧尾                uint8       0xCC
```

### 5.2 原始传感器包 (35 字节, 100Hz, 帧头 0xAA 0xBB)

当前 Raw 包格式如下。该格式自 2026-04-24 起用于链路质量评估，旧 21/33 字节格式仅作为历史记录保留在变更记录中。

```
偏移   字段                类型        说明
0-1    帧头                uint8 x2    0xAA, 0xBB
2-3    桥顶2 (HF2)         uint16 BE   ADS124S06 24bit高16bit
4-5    桥顶1 (HF1)         uint16 BE   ADS124S06 24bit高16bit
6-7    桥中2               uint16 BE   ADS124S06 24bit高16bit
8-9    桥中1               uint16 BE   ADS124S06 24bit高16bit
10-15  ACC X/Y/Z           int16 BE    LSM9DS1完整16bit, X/Y/Z
16-21  GYRO X/Y/Z          int16 BE    LSM9DS1完整16bit, X/Y/Z
22-24  PPG Green           3 bytes     Q4均值, 上位机除以16还原为原始码值
25-27  PPG Red             3 bytes     Q4均值, 上位机除以16还原为原始码值
28-30  PPG IR              3 bytes     Q4均值, 上位机除以16还原为原始码值
31-32  Seq                 uint16 BE   固件侧Raw采样序号, 0xFFFF后回绕
33     XOR 校验             uint8       bytes[2..32] 异或
34     帧尾                uint8       0xCC
```

链路质量评估:
- `接收 Hz / RX Hz`: PC 端每秒解析成功的 Raw 包数。
- `设备 Hz / DEV Hz`: 按 `Seq` 推断的设备侧 Raw 采样周期数, 正常应接近 100Hz。
- `Loss`: `missing_count / expected_count`, 其中 `missing_count` 来自 `Seq` 缺口。
- `MissingBefore`: CSV 中每个包前的缺失样本数。例如序号 11 后直接收到 15, 则序号 15 行的 `MissingBefore=3`。
- `Time(s)`: 按设备样本轴生成，即样本位置 `/100Hz`，不再按已接收行号压缩。发生丢包后，Raw CSV 会插入 `NaN` 占位行保持时间轴连续。
- 为避免把短距离乱序误判为 16 位回绕，PC 端只接受半个 uint16 周期以内的前向 `Seq` 差值。`delta > 32768` 的情况视为重复/乱序异常，不更新 `MissingBefore`，也不写入主 Raw CSV 时间轴。

### 3.3 Raw 时间轴补齐

当前无后缀主 Raw CSV 已经合并原 `_timeline.csv` 的功能，按 100Hz 设备样本轴展开。真实样本行 `ValidFlag=1, InterpFlag=0`; 缺失样本行 `ValidFlag=0, InterpFlag=0`, 传感器值写 `NaN`, `GapLen` 记录该缺失段长度。当前不自动插值。

不再生成 `_timeline.csv` 和 `_quality_events.csv`。详细文件构成和 NaN 占空语义见 `docs/acquisition/原始数据录制文件结构说明.md`。

### 5.3 Raw 链路诊断 STATUS 包 (69 字节, 1Hz, 帧头 0xAA 0xDD)

STATUS 包用于第一阶段链路诊断，不改变 35 字节 Raw DATA 包格式。

```
偏移   字段                         类型        说明
0-1    帧头                         uint8 x2    0xAA, 0xDD
2      protocol_version             uint8       当前为 2
3-6    mcu_time_ms                  uint32 BE   MCU HAL_GetTick()
7-10   sample_counter               uint32 BE   TIM16 采样 tick 总数
11-14  adc_drdy_counter             uint32 BE   ADC DRDY 中断次数
15-18  frame_counter                uint32 BE   Raw DATA 组帧次数
19-22  tx_start_counter             uint32 BE   Raw DATA UART DMA 启动成功次数
23-26  tx_done_counter              uint32 BE   Raw DATA UART DMA 完成回调次数
27-30  tx_busy_counter              uint32 BE   Raw DATA 发送时 HAL_BUSY 次数
31-34  tx_error_counter             uint32 BE   Raw DATA UART/DMA 错误次数
35-38  adc_error_counter            uint32 BE   ADC 状态异常次数
39-42  imu_error_counter            uint32 BE   IMU 运行期异常次数
43-46  ppg_fifo_empty_counter       uint32 BE   PPG FIFO 空读次数
47-50  ppg_fifo_overflow_counter    uint32 BE   PPG FIFO 接近满/溢出风险次数
51-54  ppg_fifo_sample_total_counter uint32 BE  PPG FIFO 非空周期累计读取样本数
55-58  ppg_fifo_nonempty_counter    uint32 BE   PPG FIFO 非空读取周期次数
59-62  ppg_fifo_single_sample_counter uint32 BE 单样本读取周期次数
63-66  ppg_fifo_multi_sample_counter uint32 BE  多样本均值读取周期次数
67     XOR 校验                      uint8       bytes[2..66] 异或
68     帧尾                         uint8       0xCC
```

录制时 `_status.csv` 的关键派生列:
- `PcReceivedRaw` / `PcExpectedRaw` / `PcMissingRaw`: 上位机按 Raw `Seq` 统计的接收、期望和缺失样本数。
- `PcMissingAfterTxDone`: 从本次录制首个 STATUS 建立基线后，计算 `tx_done_counter` 增量与 `PcReceivedRaw` 增量的差值，用于估算 MCU 已完成发送但 PC 未解析成功的帧数。
- `TxInflight`: `tx_start_counter - tx_done_counter` 的非负部分，用于观察 DMA 是否长期未完成。
- `PcRawTotalCandidates` / `PcRawInvalidCandidates` / `PcRawInvalidDelta`: PC 串口状态机收满的 Raw 候选帧总数、累计无效候选帧数和相邻 STATUS 之间新增无效候选帧数。若 gap 周期内该 delta 增长，优先怀疑字节损坏、帧尾/XOR 校验失败或帧同步问题；若该 delta 不增长，优先怀疑下游串口/BLE 透传丢失。
- `PpgFifoSampleTotalCounter` / `PpgFifoNonemptyCounter` / `PpgFifoSingleSampleCounter` / `PpgFifoMultiSampleCounter`: 统计 PPG FIFO 每次非空读取拿到的样本数分布。`SampleTotal / Nonempty` 越接近当前内部过采样预期，说明 Raw 周期中真实做了多样本均值；若长期接近 1 或空读增长，说明仍存在输出率/读取节拍不匹配。

历史旧格式摘录如下，当前版本不再使用:

```
偏移  字段                类型        说明
0-1   帧头                uint8 x2    0xAA, 0xBB
2-3   桥顶2 (HF2)         uint16 BE   24bit 高16bit+8bit
4-5   桥顶1 (HF1)         uint16 BE   24bit 高16bit+8bit
6-7   桥中2               uint16 BE   24bit 高16bit+8bit
8-9   桥中1               uint16 BE   24bit 高16bit+8bit
10    ACC X 高字节         uint8
11    ACC Y 高字节         uint8
12    ACC Z 高字节         uint8
13-16 PPG 数据            4 bytes     模式相关 (见下)
17-18 扩展数据            2 bytes     模式相关 (见下)
19    XOR 校验             uint8       bytes[2..18] 异或
20    帧尾                uint8       0xCC

HR 模式:   bytes[13-15]=24bit绿光累加, byte[16]=采样计数
           bytes[17]=0x00, byte[18]=0xFF (模式标记)
SpO2 模式: bytes[13-14]=16bit红光均值, bytes[15-16]=16bit红外均值
           byte[17]=温度整数(有符号), byte[18]=温度小数
           温度公式: die_temp_int + die_temp_frac * 0.0625 + 2.4 (LED温升补偿)
```

### 5.4 帧解析状态机

串口读取线程使用双协议状态机:
1. 等待帧头 0xAA
2. 等待第二帧头字节区分协议:
   - 0xCC -> 31字节 HR 结果包 -> `parse_hr_packet()` -> `HRPacket`
   - 0xBB -> 35字节 原始传感器包 -> `parse_raw_packet()` -> `RawDataPacket`
   - 0xDD -> 69字节 Raw 链路诊断包 -> `parse_status_packet()` -> `StatusPacket`
3. 收集 payload 直到满对应长度
4. XOR 校验 + 帧尾验证
5. 通过不同的 pyqtSignal 发射给对应面板

---

## 6. 文件结构

```
tools/monitor/
  main.py              # 程序入口, AppController 连接 MonitorWindow 和 SerialReader
  dashboard.py         # MonitorWindow(外壳+工具栏) + HRPanel(在线心率面板) + 翻译表/配色
  raw_data_panel.py    # RawDataPanel(原始数据面板) - PPG/ACC/桥压/SpO2 波形
  realtime_hr.py       # 原始数据面板绿光PPG实时纯FFT心率估计
  protocol.py          # HRPacket(31字节) + RawDataPacket(35字节) + StatusPacket(69字节) 协议定义与解析
  serial_reader.py     # 多协议串口读取线程 (QThread + 状态机)
  requirements.txt     # Python 依赖
  start_monitor.bat    # Windows 一键启动脚本
```

---

## 7. 变更记录

| 日期 | 变更内容 |
|------|----------|
| 2026-03-28 | 初始版本: 实时心率仪表盘, 串口通信, 三路径对比, 趋势图 |
| 2026-03-28 | 新增清屏功能: 清除所有数据和曲线 |
| 2026-03-28 | 新增数据保存: CSV 导出解算后数据 (相对时间, UTF-8-BOM) |
| 2026-03-28 | 新增语言切换: 中/English 双语支持, 默认中文 |
| 2026-03-28 | 新增采样率显示: 算法路径卡片增加采样率字段, HR结果包扩展至21字节 |
| 2026-03-29 | 协议升级至31字节: 新增HF2 AC幅值、HF2-PPG相关系数字段; 上位机新增HF1/HF2信号质量显示 (AC幅值+相关系数) |
| 2026-04-01 | 录制功能重构: "保存"按钮改为"录制"按钮, 点击先选路径(默认桌面)再开始录制, 停止时自动保存, 状态栏显示录制数据点数 |
| 2026-04-05 | 融合 data_monitor 原始数据功能: 新建 tools/monitor/ 统一上位机, 支持面板切换(在线心率/原始数据); 双协议串口状态机(31字节HR+21字节Raw); 原始数据暗色科技风面板(PPG/桥压/ACC/SpO2, 50ms刷新); 扩展 i18n |
| 2026-04-15 | 多光谱原始数据包支持(33字节): 新增 RawDataPacket 解析(Green+Red+IR三通道PPG + 16-bit ACC + 陀螺仪); 串口双包类型自动检测(0xAA0xBB/0xAA0xCC); PPG波形固定三通道布局(左侧绿光, 右侧红光+红外上下); 新增陀螺仪角速度波形; 移除HR/SpO2模式切换; tools/monitor/ 协议/串口/面板同步更新 |
| 2026-04-18 | UI优化: 1)录制按钮旁新增保存路径设置(默认桌面, 点击录制直接开始无需选路径); 2)修复数据录制丢包(移除逐包flush改为每100包刷盘, 串口读取缓冲区从128字节增至4096); 3)绘图性能优化(启用pyqtgraph裁剪视图+自动降采样, 刷新频率从20FPS降至15FPS) |
| 2026-04-18 | 绘图流畅度优化: 缓冲区翻倍至2000点, 可见窗口缩减为800点(8秒), 每帧渲染量-20%; 刷新率提升至30FPS(33ms); 信息条从逐包(100Hz)降频为按帧率(30FPS)更新, 消除冗余QLabel重绘 |
| 2026-04-24 | 原始数据录制缺点排查修复: 串口读取由 4096 字节大块读取改为低延迟小块读取; Raw CSV 时间列改为按 100Hz 样本序号生成，消除批处理造成的时间戳跳变和点击录制尾部时延 |
| 2026-04-24 | Raw链路质量评估: 原始数据包扩展为35字节, 新增固件侧 `Seq`; 上位机显示 `接收/RX Hz`、`设备/DEV Hz` 和 `Loss`, CSV新增 `Seq` 与 `MissingBefore` |
| 2026-04-26 | 原始数据面板新增绿光 PPG 实时纯 FFT 心率估计: 8 秒窗口、1Hz 更新、显示计算耗时; 计算从已有缓冲读取，不影响串口解析和 Raw CSV 逐包保存链路 |
| 2026-04-30 | Raw链路诊断第一阶段: 新增 53 字节 STATUS 包 (`0xAA 0xDD`) 与 `StatusPacket` 解析; Raw 面板显示 `诊断/Diag` 摘要; 录制时同步生成 `_status.csv`，用于采集后定位 MCU 采样/组帧/UART DMA/PC 接收解析问题 |
| 2026-04-30 | 阶段B首次采集反馈修正: `PCMissingAfterTxDone` 改为基线增量计算; 固件 STATUS 改为 Raw DMA 完成后顺带发送，避免 1Hz STATUS 抢占 DMA 造成系统性 `HAL_BUSY` |
| 2026-04-30 | 阶段B第二次采集反馈: `_status.csv` 新增 PC 端 Raw 候选帧解析统计列，用于区分“字节到达但解析/校验失败”和“下游链路未形成候选帧” |
| 2026-05-12 | Raw录制文件简化: 无后缀主 CSV 合并时间轴补齐和 NaN 缺失行，只额外保留 `_status.csv`; 实时绿光 FFT 心率搜索下限降至 0.7Hz |
| 2026-05-20 | Raw Seq 异常保护: 重复 Seq 与小幅回退/乱序包不再按 `% 65536` 展开为大规模缺失，主 CSV 避免出现 655xx 行误判 `NaN` |
| 2026-05-12 | 原始数据面板初始标签中文化: `_build_info_bar` 中 Mode/Loss/Packets 初始占位文本改为中文，与默认 zh 语言一致 |
| 2026-05-18 | 曲线标题实时显示最近10点平均值(精度1位小数, 33ms刷新); 新增 Marker 标记按钮, 录制时点击生成 `_markers.csv` 同步记录, 按钮显示累计次数 |
| 2026-05-27 | 三路同步 PPG 采集配置更新: Raw 包 PPG 三通道改为 Q4 定点均值，上位机除以 16 保存小数；STATUS 升级到 v2/69字节，新增 PPG FIFO 累计样本数、非空周期、单样本周期和多样本周期计数，`诊断/Diag` 显示 `PPG均值/PPGAvg` |
| 2026-07-10 | Raw 录制新增实验信息命名：按场景、试次、受试者与日期构造算法兼容目录和文件名；新增“开机”`kaiji` 场景；仅在至少一次点击 marker 后生成 `_markers.csv` |
| 2026-07-11 | Raw 实时绿光 FFT 心率增加频谱 SNR（dB）显示；峰峰值弱信号门槛由 100 降至 50 原始码值，SNR 暂仅显示不拦截 BPM |

---

**最后更新**: 2026-07-11
**对应分支**: main

## 不使用 PPG（通道 0）

固件配置页的 PPG/IIC 通道新增“不使用 PPG”。对应 `PPG_DEFAULT_CHANNEL=0`：两路软件 IIC 引脚设为模拟输入/高阻，跳过 MAX30101 检测、初始化、模式配置、FIFO 和温度访问。光模式选择此时禁用、不生效。
原始包仍为 35 字节，PPG Green/Red/IR 所占的 bytes[22..30] 每帧固定填 0；ADC、ACC、GYRO、序号、XOR 和 STATUS 发送保留。PPG 未启用时不增加 FIFO 空/溢出计数。上位机 PPG/CSV 显示 0 是占位，不是测得的信号；协议不携带禁用标记，因此 FFT 可能显示信号弱/无结果，不能从全零数据独立判断硬件配置。默认通道仍为 2，需要选择通道 0 并重新编译烧录后才影响板子。


## 2026-09-13 功率分档与双路采集（独立工作树）

固件页面增加功率七段实验选项（ble_rf_disabled=3）及“打开双路采集窗口”按钮。两个窗口分别选择板载有线 COM 与 HJ-380 COM，默认保存到 recordings/wired 与 recordings/wireless；也可通过 --capture-link wired/wireless 打开。AA EE 事件新增功率阶段和读回确认，事件 CSV 追加 Experiment、TargetPower(dBm)、PowerVerified。Raw/STATUS 布局不变。同秒同目录录制不再覆盖已有文件；停止录制会关闭事件文件。实验过程及失败恢复见 [蓝牙功率分档实验](experiments/archive/蓝牙功率分档实验.md)。


## 2026-09-13 功率指令应答自检

固件页面新增“只读功率自检”（ble_rf_disabled=4）。支持 AA EF v1 53字节诊断帧（与 STATUS 同长，以帧头区分）。事件 CSV 增加 DiagReason、RxBytes、RxErrors、CommandTxDone、RxTailHex、RxTailAscii、ReadOnly；UI 展示接收字节及错误概况。原 AA EE、Raw、STATUS 布局不变。只读阶段码13–16，不设置目标功率或伪报 PowerVerified。


## 2026-09-13 用户选择只写功率实验

新增只写功率七段选项（ble_rf_disabled=5），事件25/26及阶段17–21显示未读回验证；CSV Experiment=power_writeonly、PowerVerified=0、目标功率不等于实测值。双路录制方式不变。详见蓝牙功率分档实验.md。


## 2026-09-13 固定-10 dBm连续采集

新增固件选项ble_rf_disabled=6，对应BLE_FIXED_MINUS10=1。启动采样前只写一次ST_TX_POWER=-10，排除全量初始化及所有循环实验。不读回、不更改ADC，不在结束后恢复功率。详细操作见[固定负10dBm采集](experiments/archive/固定负10dBm采集.md)。


## 固定-10配置前增加硬件复位

按用户要求，BLE_FIXED_MINUS10启动时先PB13拉高100 ms、释放等待300 ms，再唤醒并写入-10。不恢复出厂、不改变其他参数，仍未读回验证。UI固定-10选项已明确标注“启动复位后配置”；启动时应等待HJ-380重新连接后录制。详见固定负10dBm采集.md。


## 2026-09-14：批量发送对照版本

实验工作树模式 7 / BLE_BATCH5=1：维持 Raw 100 Hz、35 字节协议和采样序号，每累计 5 帧发送 175 字节，标称间隔 50 ms。独立 staging/tx 缓冲区保证 DMA 期间数据不被覆盖。批次边界 UART 忙则丢弃该批、busy 计数增加 5，序号继续递增；不阻塞采样。发送开始/完成诊断按帧计数（每批加 5）。状态包仍为 1 Hz，在 Raw 批次完成后发送。

启动复位 HIGH 100 ms、释放 LOW 等待 300 ms、唤醒后写入 <ST_TX_POWER=2.5>，不读取验证，不执行其他功率/复位实验。PPG 默认关闭，沿用此前热膜实验配置。串口批量不保证射频连接事件减少。

工作树 start.bat → 固件页面 → “批量 5 帧 / 50 ms · +2.5 dBm / 持续采集” → 编译并烧录。独立编译：python scripts/build_power_experiment.py --batch5，产物 build/ble-batch5/L452CEU6.elf、hex、bin 和校验清单。

上位机沿用逐帧序号恢复 100 Hz 时间轴，不能把一批 5 帧视作同一采样时刻。有线和无线均受批量发送影响。建议连接稳定、做好防风后连续录制 3 分钟；HJ-380 全程连接，保持电池、USB、距离和姿态一致。优先有线录制，条件允许同步录制无线，检查丢帧及校验错误。目标功率未经读回验证。此版本不自动切换发送方式，也不会生成射频阶段事件。


## 2026-09-14：单帧 +2.5 dBm 与热膜双数据录制

工作树 start.bat 固件页默认模式8：“默认单帧100 Hz · +2.5 dBm”。启动硬件复位后写入功率，不读回确认；关闭批量及实验阶段切换。PPG保持关闭，与此前热膜实验一致。独立产物 build/ble-single25；编译 scripts/build_power_experiment.py --single25。

Raw页新增“热膜显示：3点中值 / 原始”按钮，默认中值，仅处理Uc1/Uc2/Ut1/Ut2。PPG及惯性数据保持原样。滤波为100 Hz逐样本尾随3点中值，不依赖UI刷新率；相对居中结果延后1点（10 ms），列时间仍标记当前样本时刻。

CSV原字段完整保留，末尾新增 Uc1_Median3(mV)、Uc2_Median3(mV)、Ut1_Median3(mV)、Ut2_Median3(mV)、Median3Valid。显示开关不影响任一录制列。开启录制/清屏/序号不连续会重置窗口；不足3点保存NaN、有效标记0；缺失行也保存NaN。_processing.json记录滤波参数及时间对齐规则。原始与中值处于同一CSV、同一序号，避免两个文件错位。

Uc1/Uc2原先低速更新并保持重复值，中值3不能将它们变成新的100 Hz独立采样。静置数据无法确定滤波是否损失真实热膜响应；后续使用静置→佩戴→手腕运动数据验证。此次未加入带通/低通，也未实际烧录。


## 2026-09-14：汇报界面整理

移除采集页独立的平滑按钮行，工具栏端口旁“原始数据”改为无边框切换按钮。默认未按下=热膜3点中值，按下高亮=纯原始；提示文字说明当前状态。CSV双数据保存不变。工具栏控件30px、上下边距6px，标签页与诊断区同步收紧。字体统一Arial优先、中文回退SimSun（宋体），按钮10pt、诊断9pt、图标题10pt、刻度9pt。

固件页仅显示标准单帧+2.5和自定义初始化，保留PPG/IIC、光模式、SWD速度。旧实验配置载入时映射到标准模式，实验源代码与历史构建脚本保留。不会在打开页面时自动烧录或改写磁盘配置。
