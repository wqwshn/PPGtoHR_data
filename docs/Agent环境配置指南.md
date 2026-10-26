# Agent 环境配置指南

用途：在新的Windows电脑或现有仓库中配置数据采集上位机；需要编译时再增加固件工具。安装与验证在项目内完成。读取根目录 `AGENTS.md` 后按本文执行，结束时报告真实验证结果。

## 1. 确认位置和Python

仓库根目录应包含 `start.bat`、`Core/`、`tools/monitor/` 和本文。已有工作区时先查看Git状态，保留用户修改、录制和PCB原件。

使用64位Python 3.11，或3.10/3.12；依赖锁定在已验证的NumPy 1.26.4等版本，当前安装流程不适用于Python 3.13及以上。用 `python -c "import sys,struct; print(sys.executable,sys.version,struct.calcsize('P')*8)"` 核对实际解释器。没有适用解释器时，从[Python官方Windows下载页](https://www.python.org/downloads/windows/)选择对应版本，不改动已有Conda环境。

## 2. 安装项目环境

在根目录运行PowerShell，默认仅安装上位机运行依赖：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\setup_monitor.ps1
```

需要运行测试、编译固件时使用：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\setup_monitor.ps1 -Development
```

该命令创建或复用项目 `.venv`，通过其中的Python安装依赖，并执行离屏模拟检查。可重复运行；不删除旧环境，不安装全局Python包，不写入板子。`-Development`增加pytest、CMake和Ninja，**不包含ARM GCC、OpenOCD及设备驱动**。

PATH中有多个Python时显式传入路径，例如 `-PythonExecutable "C:\Python311\python.exe"`。脚本失败时以出错步骤为准：解释器错误先选对Python，pip错误先检查网络和代理，Qt错误检查依赖及图形运行环境；不用修改采集算法来解决安装问题。

完成标准：脚本退出码0，并打印 `Environment ready`。随后始终使用项目解释器，避免把Conda/PATH Python与`.venv`混用。

## 3. 验证上位机

```powershell
.\.venv\Scripts\python.exe scripts/check_environment.py --gui-smoke-test --json
.\.venv\Scripts\python.exe tools/monitor/main.py --raw-simulate
```

第一个命令检查依赖，离屏创建工作台并处理模拟样本，JSON顶层 `ok` 应为true。第二个命令显示实际窗口；确认两个页面、模拟波形和烧录参数界面正常，然后关闭。离屏检查不证明真实串口、BLE或传感器正常，也不作为100Hz吞吐验收。

只采集已有板子数据时不要求固件编译工具；环境报告中的OPTIONAL不阻止上位机运行。配置环境阶段不自动选择COM口、发送AT指令、连接ST-Link或烧录。

## 4. 按需增加固件工具

需要编译/烧录才执行本节。已装工具先复用；版本和下载来源记录在交付说明。

- CMake 3.23及以上、Ninja由开发依赖提供。
- ARM GCC包含 `arm-none-eabi-gcc`、objcopy、size等工具，可从[xPack ARM GCC发布页](https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases)获取Windows x64包。
- OpenOCD可从[xPack OpenOCD发布页](https://github.com/xpack-dev-tools/openocd-xpack/releases)获取。实际烧录另需ST-Link驱动和用户确认的接线、供电。

工作台按以下目录结构发现便携工具，也支持PATH已有工具：

```text
.local-tools/
  xpack-arm-none-eabi-gcc-<version>/bin/arm-none-eabi-gcc.exe
  xpack-openocd-<version>/bin/openocd.exe
```

```powershell
.\.venv\Scripts\python.exe scripts/check_environment.py --firmware --gui-smoke-test
.\.venv\Scripts\python.exe -m pytest -q tests -p no:cacheprovider --basetemp build/pytest-environment
```

`--firmware`要求四种工具均能发现，但不执行硬件连接。Clang和Node用于固件WASM仿真测试，缺少时相关测试会SKIP；须在报告中列出跳过项，不能声称全部固件测试已运行。首次构建建议通过固件页“仅编译”，生成当前用途的产物与清单，禁止把“仅编译成功”写成“已烧录成功”。

固件用途、首次蓝牙配置及日常版本切换统一读取[固件烧录与版本使用手册](固件烧录与版本使用手册.md)。

## 5. 交付标准

报告项目路径、Python绝对路径和版本、环境检查结果、模拟窗口结果；开发配置还报告工具路径、测试通过/失败/跳过数及构建结果。明确哪些步骤未运行。

运行环境和产物留在 `.venv/`、`.local/`、`.local-tools/`、`build/`；录制留在 `recordings/`。这些本机文件不提交到仓库。不要删除已有配置来强制显示默认值；已有固件选择应保留。当前支持Windows工作台，其他系统的虚拟环境、Qt图形依赖与启动方式需另行验证。


## 6. 本次复现记录（2026-10-26）

在仓库内新建隔离虚拟环境，使用Python3.11.7 x64执行安装脚本；首次安装和重复执行均退出0，离屏模拟处理100条样本通过。该环境中完整pytest为226项通过，无跳过。固件另验证日常、详细诊断、新板验收三个用途的独立构建，不执行烧录；安装环境通过不代表器件或真实蓝牙链路已验收。
