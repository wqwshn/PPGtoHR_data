# PPG 与热膜原始数据采集系统

STM32L452CEU6 固件与 PyQt5 工作台，用于 MAX30101 三路 PPG、热膜桥压和 LSM9DS1 运动信号的同步采集、记录与心率验证。

## 快速开始

双击根目录 **`start.bat`**，打开“原始数据采集”和“固件配置与烧录”两个页面。
旧入口 `tools/monitor/start_monitor.bat` 仍可使用。启动器优先使用项目虚拟环境，也支持已激活的 Conda/PATH Python；可用 `PPG_PYTHON` 指定解释器。

```powershell
python -m pip install -r tools/monitor/requirements.txt
python tools/monitor/main.py --raw-simulate
```

实际采集时选择 COM 口并连接；先填写“实验信息”，再点击“录制”。操作说明见[统一工作台使用说明](docs/统一工作台使用说明.md)。

## 当前功能

- **采集页面**：PPG、四路桥压、ACC/GYRO 波形；实时 FFT 心率、频谱 SNR、计算耗时与链路诊断。没有独立在线心率页面。
- **显示与数据**：PPG 波形和均值使用 nA，CSV 保留原始 ADC 码值。热膜默认显示 3 点中值，可切换原始显示；CSV 同时保存原始值、中值和有效标记。
- **实验录制**：按场景、试次、受试者和日期命名，保留 Marker；缺帧用 NaN 占位，同名文件禁止覆盖。
- **固件页面**：PPG 通道 0/1/2、三路 PPG或红光/红外模式、蓝牙配置、SWD 速度，支持仅编译和 ST-Link 烧录。
- **固件与链路**：保留本地三路 PPG FIFO 精度/连续性改进、BLE MAC 配置和 HJ-380 前缀解析修复；合入远程热膜与蓝牙实验支持。

工作台输出固定为 Raw 100 Hz、35 字节单帧。无本机配置时选择标准蓝牙模式：启动复位后写入 +2.5 dBm，功率未读回验证。也可选择“自定义初始化”保留模块配置。直接命令行构建以头文件默认值或显式 CMake 参数为准。

## 项目目录

```text
Core/                 STM32 应用、采集、算法与驱动接口
Drivers/              STM32 HAL 与 CMSIS 第三方依赖
tools/monitor/        采集、绘图、录制和固件工作台
tools/firmware/       ST-Link / OpenOCD 配置
scripts/              启动入口、构建和历史实验脚本
tests/                Python 与固件回归测试
docs/                 按使用、硬件、协议、实验和开发分类的文档
config/               本机工作台配置（firmware.json 不提交）
build/                构建、测试与验证产物（不提交）
recordings/            原始录制（不提交）
reports/               本机分析报告（不提交）
```

## 构建与测试

构建需要 CMake、Ninja、ARM GCC；烧录另需 OpenOCD 与 ST-Link。

```powershell
cmake -S . -B build -G Ninja
cmake --build build -j4
python -m pytest -q tests
```

工作台的构建产物单独存放在 `build/desktop/`，本机选择保存在 `config/firmware.json`；保存参数后需重新编译烧录才能影响设备。

## 文档入口

- [完整文档导航](docs/README.md)
- [上位机 UI 说明](docs/上位机UI说明文档.md)
- [采集与 CSV 格式](docs/acquisition/README.md)
- [硬件配置](docs/hardware/README.md)
- [热膜与蓝牙实验总结](docs/experiments/热膜与蓝牙实验总结.md)
- [固件与在线算法实施记录](docs/在线心率算法实施文档.md)

历史实验参数和算法设计不等于当前默认采集配置；详细硬件参数以代码和相应配置说明为准。
