# BLE MAC 地址绑定实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MCU 配置 HJ-131 自定义 MAC 地址，上位机连接时向 HJ-380 发送指定连接指令，实现设备绑定隔离。

**Architecture:** MCU 端在现有的 `BLEC_Init()` 序列中增加 `<ST_OWN_MAC=...>` 命令。PC 端在 `SerialReader` 中增加前缀剥离状态机和 HJ-380 连接握手，`dashboard.py` 中增加 MAC 配置状态显示。

**Tech Stack:** C (STM32 HAL), Python/PyQt5, serial (pyserial)

---

### Task 1: MCU 端 — 新增 BLE_CUSTOM_MAC 宏并启用 BLE 配置

**Files:**
- Modify: `Core/Inc/main.h:116-120`

- [ ] **Step 1: 新增 BLE_CUSTOM_MAC 宏并将 ENABLE_BLE_CONFIG 改为 1**

在 `main.h` 第 116-120 行区域修改:

```c
/* HJ-131IMH BLE 配置开关:
 * 0 = 上电不发送 BLE 配置指令
 * 1 = 上电复位 BLE 模块后发送固定配置指令
 */
#define ENABLE_BLE_CONFIG       1

/* 自定义 BLE MAC 地址 (12 字节 HEX, 大端)
 * HJ-380 将只连接此 MAC 地址的 HJ-131 设备
 */
#define BLE_CUSTOM_MAC  "784128c58150"
```

注意: `BLE_CUSTOM_MAC` 中的 MAC 地址需要与实际 HJ-131 模块的目标 MAC 一致。当前使用占位值，实际部署时替换。

- [ ] **Step 2: 编译验证**

```bash
cd build && cmake --build . --target all 2>&1 | tail -20
```
Expected: 编译成功，无错误。

- [ ] **Step 3: 提交 MCU 端改动**

```bash
git add Core/Inc/main.h
git commit -m "feat: 新增BLE_CUSTOM_MAC宏，启用BLE配置
- main.h 新增 BLE_CUSTOM_MAC 宏 (12字节HEX MAC地址)
- ENABLE_BLE_CONFIG 改为 1, 上电时自动配置HJ-131"
```

---

### Task 2: MCU 端 — BLEC_Init() 中增加 ST_OWN_MAC 命令

**Files:**
- Modify: `Core/Src/main.c:645-683`

- [ ] **Step 1: 在 BLEC_Init() 命令序列中增加 MAC 设置命令**

在 `main.c` 第 645-683 行的 `BLE_Init()` 中，新增 `CMD_OWN_MAC` 并在配置序列末尾发送（在 `CMD_BAUD` 之前，因为设置 MAC 后模块自动重启，波特率设置应在最后）:

```c
#if (ENABLE_BLE_CONFIG)
static void BLE_Init(void)
{
    static const uint8_t CMD_FACTORY[]     = "<ST_FACTORY>";
    static const uint8_t CMD_CLEAR_SECRET[] = "<ST_CLEAR_SECRET>";
    static const uint8_t CMD_WAKE[]        = "<ST_WAKE=FOREVER>";
    static const uint8_t CMD_TX_POWER[]    = "<ST_TX_POWER=+2.5>";
    static const uint8_t CMD_NAME[]        = "<ST_NAME=HJ-131-LYX>";
    static const uint8_t CMD_OWN_MAC[]     = "<ST_OWN_MAC=" BLE_CUSTOM_MAC ">";
    static const uint8_t CMD_MIN_GAP[]     = "<ST_CON_MIN_GAP=75>";
    static const uint8_t CMD_BAUD[]        = "<ST_BAUD=115200>";

    BLE_ResetModule();

    // 0. 发送唤醒序列。若模块已是 FOREVER 模式，该步骤无副作用。
    uint8_t wakeup_seq[] = {0xAA, 0xAA, 0xAA, 0xAA};
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_FACTORY, sizeof(CMD_FACTORY) - 1);
    HAL_Delay(500);
    BLE_SetUartBaud(19200);
    BLE_ResetModule();
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_FACTORY, sizeof(CMD_FACTORY) - 1);
    HAL_Delay(500);
    BLE_ResetModule();
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    BLE_SendConfigCommand(CMD_CLEAR_SECRET, sizeof(CMD_CLEAR_SECRET) - 1);
    BLE_SendConfigCommand(CMD_WAKE, sizeof(CMD_WAKE) - 1);
    BLE_SendConfigCommand(CMD_TX_POWER, sizeof(CMD_TX_POWER) - 1);
    BLE_SendConfigCommand(CMD_NAME, sizeof(CMD_NAME) - 1);
    BLE_SendConfigCommand(CMD_MIN_GAP, sizeof(CMD_MIN_GAP) - 1);
    /* 设置自定义 MAC 地址 (模块自动重启, 耗时约 500ms) */
    BLE_SendConfigCommand(CMD_OWN_MAC, sizeof(CMD_OWN_MAC) - 1);
    HAL_Delay(600);
    /* MAC 设置后模块重启, 需要重新发送唤醒序列 */
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);
    /* 波特率设置必须在最后 (MAC 设置后的重启可能恢复默认波特率) */
    BLE_SendConfigCommand(CMD_BAUD, sizeof(CMD_BAUD) - 1);
    HAL_Delay(100);
    BLE_SetUartBaud(115200);
}
```

注意: `<ST_OWN_MAC>` 设置后 HJ-131 会自动重启，重启后波特率可能回到默认 19200。所以在 MAC 设置后重新发送唤醒序列，确保后续 `CMD_BAUD` 能正确发送。

- [ ] **Step 2: 编译验证**

```bash
cd build && cmake --build . --target all 2>&1 | tail -20
```
Expected: 编译成功，无错误。

- [ ] **Step 3: 提交**

```bash
git add Core/Src/main.c
git commit -m "feat: BLEC_Init()增加ST_OWN_MAC自定义MAC地址设置
- 新增CMD_OWN_MAC命令, 使用main.h中BLE_CUSTOM_MAC宏
- MAC设置后模块自动重启, 增加600ms等待和唤醒序列
- CMD_BAUD移至序列末尾, 确保MAC重启后波特率正确"
```

---

### Task 3: PC 端 — protocol.py 新增 BLE 常量

**Files:**
- Modify: `tools/monitor/protocol.py`

- [ ] **Step 1: 新增 BLE MAC 和 HJ-380 命令常量**

在 `protocol.py` 的帧常量区域（第 51 行前）新增:

```python
# HJ-131 / HJ-380 BLE MAC 绑定常量
BLE_CUSTOM_MAC = "784128c58150"                     # 与固件 main.h 中 BLE_CUSTOM_MAC 一致
HJ380_PREFIX_FROM = b"From"                         # HJ-380 数据前缀标识
HJ380_MAX_PREFIX_LEN = 64                           # 前缀最大字节数 (From + 空格 + 12 MAC + ":  " < 30)
HJ380_CMD_CON_MAC_FMT = "<ST_CON_MAC={}>"           # HJ-380 连接指定 MAC 命令模板
HJ380_RSP_CON_MAC_OK = "st_con_mac="                # HJ-380 连接成功应答前缀
```

注意: `BLE_CUSTOM_MAC` 必须与 `Core/Inc/main.h` 中的值完全一致。12 字节小写 HEX。

- [ ] **Step 2: 提交**

```bash
git add tools/monitor/protocol.py
git commit -m "feat: protocol.py新增BLE MAC绑定常量"
```

---

### Task 4: PC 端 — serial_reader.py 增加前缀剥离与 HJ-380 握手

**Files:**
- Modify: `tools/monitor/serial_reader.py`

这是核心改动。需要在 `SerialReader` 中增加:
1. 串口写入能力 (发送 HJ-380 AT 命令)
2. 前缀剥离状态机 (`From    mac:  ` 识别)
3. AT 响应解析 (`<st_con_mac=...>` 识别)
4. MAC 配置状态信号

- [ ] **Step 1: 更新 import 和新增信号**

修改 `serial_reader.py` 第 10-52 行:

```python
"""
PPG Monitor - 串口读取线程

在独立 QThread 中运行串口读取, 使用状态机逐字节解析帧.
支持双协议:
  - 31 字节心率结果包 (0xAA 0xCC) -> hr_packet_received
  - 35 字节多光谱原始传感器包 (0xAA 0xBB) -> raw_packet_received
  - 53 字节 Raw 链路诊断状态包 (0xAA 0xDD) -> status_packet_received

HJ-380 BLE 适配:
  - 识别并剥离 From mac: 数据前缀
  - 解析 HJ-380 AT 命令响应
  - 连接时自动发送 ST_CON_MAC 绑定指令
"""
from __future__ import annotations

import serial
import serial.tools.list_ports
import time
from typing import List, Optional
from PyQt5.QtCore import QThread, pyqtSignal

from protocol import (
    HEADER_BYTE_0, HEADER_BYTE_1, PACKET_LEN,
    RAW_HEADER_BYTE_1, RAW_PACKET_LEN,
    STATUS_HEADER_BYTE_1, STATUS_PACKET_LEN,
    parse_hr_packet, HRPacket,
    parse_raw_packet, RawDataPacket,
    parse_status_packet, StatusPacket,
    BLE_CUSTOM_MAC,
    HJ380_PREFIX_FROM, HJ380_MAX_PREFIX_LEN,
    HJ380_CMD_CON_MAC_FMT, HJ380_RSP_CON_MAC_OK,
)

SERIAL_READ_TIMEOUT_S = 0.01
SERIAL_READ_CHUNK_BYTES = max(PACKET_LEN, RAW_PACKET_LEN, STATUS_PACKET_LEN) * 4
HJ380_HANDSHAKE_TIMEOUT_S = 3.0   # HJ-380 连接握手超时
```

在 `SerialReader` 类中新增信号（第 40-53 行后追加）:

```python
class SerialReader(QThread):
    """串口读取与双协议帧解析线程 (含 HJ-380 BLE 适配)"""

    # ... 现有信号保持不变 ...

    # 信号: HJ-380 MAC 配置结果 (mac, success)
    mac_configured = pyqtSignal(str, bool)

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._running = False
        self._serial: Optional[serial.Serial] = None
        self._raw_total = 0
        self._raw_invalid = 0
```

- [ ] **Step 2: 增加串口发送和握手方法**

在 `SerialReader` 类中增加方法:

```python
    def _send_command(self, cmd: str) -> bool:
        """向 HJ-380 发送 AT 命令, 返回是否发送成功"""
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(cmd.encode("ascii"))
                self._serial.flush()
                return True
        except (serial.SerialException, OSError):
            pass
        return False

    def _do_hj380_handshake(self) -> bool:
        """HJ-380 连接握手: 发送 ST_CON_MAC 命令, 返回是否发送成功"""
        cmd = HJ380_CMD_CON_MAC_FMT.format(BLE_CUSTOM_MAC)
        return self._send_command(cmd)
```

- [ ] **Step 3: 重写 run() 方法 — 加入握手流程和前缀剥离状态机**

完整替换 `run()` 方法 (第 65-166 行):

```python
    def run(self):
        """线程主循环: 打开串口 -> HJ-380 握手 -> 状态机解析 (含前缀剥离)"""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
            )
            self._running = True
            self._raw_total = 0
            self._raw_invalid = 0
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open {self._port}: {e}")
            self.connection_changed.emit(False)
            return

        # HJ-380 连接握手: 发送 ST_CON_MAC 绑定指令
        if not self._do_hj380_handshake():
            self.error_occurred.emit("HJ-380: 配置指令发送失败")
        self._handshake_deadline = time.time() + HJ380_HANDSHAKE_TIMEOUT_S
        self._mac_configured = False

        # 帧状态机: 0=等待帧头0(0xAA), 1=等待帧头1, 2=收集payload
        state = 0
        buf = bytearray()
        expected_len = 0
        # ASCII 文本行缓冲 (标定状态等)
        text_buf = bytearray()
        # HJ-380 前缀匹配缓冲
        prefix_buf = bytearray()
        # HJ-380 AT 响应缓冲
        at_buf = bytearray()

        try:
            while self._running:
                raw = read_serial_chunk(self._serial)
                if not raw:
                    continue

                for byte in raw:
                    # ── AT 响应收集 (独立于帧状态机) ──
                    if len(at_buf) > 0:
                        at_buf.append(byte)
                        if byte == 0x3E:  # '>'
                            self._parse_at_response(bytes(at_buf))
                            at_buf.clear()
                        elif len(at_buf) > 128:
                            at_buf.clear()
                        continue

                    # ── 前缀匹配 (独立于帧状态机) ──
                    if len(prefix_buf) > 0:
                        if self._feed_prefix(byte, prefix_buf):
                            # 前缀匹配完成 (成功或失败), prefix_buf 已清空
                            # 当前 byte 已在 _feed_prefix 内处理, continue
                            continue
                        else:
                            # 前缀匹配进行中, continue
                            continue

                    # ── 检测 AT 响应起始 ──
                    if byte == 0x3C:  # '<'
                        at_buf = bytearray([byte])
                        continue

                    # ── 检测 From 前缀起始 ──
                    if byte == HJ380_PREFIX_FROM[0]:  # 'F'
                        prefix_buf = bytearray([byte])
                        continue

                    # ── 原有帧状态机 ──
                    if state == 0:
                        if byte == HEADER_BYTE_0:  # 0xAA
                            buf = bytearray([byte])
                            state = 1
                        elif 0x20 <= byte <= 0x7E or byte in (0x0D, 0x0A):
                            if byte == 0x0A:
                                line = text_buf.decode("ascii", errors="ignore").strip()
                                if line:
                                    self.calib_status_received.emit(line)
                                text_buf.clear()
                            elif byte != 0x0D:
                                if len(text_buf) < 256:
                                    text_buf.append(byte)
                        else:
                            text_buf.clear()
                    elif state == 1:
                        if byte == HEADER_BYTE_1:  # 0xCC -> HR 结果包
                            buf.append(byte)
                            expected_len = PACKET_LEN  # 31
                            state = 2
                        elif byte == RAW_HEADER_BYTE_1:  # 0xBB -> 原始数据包
                            buf.append(byte)
                            expected_len = RAW_PACKET_LEN  # 35
                            state = 2
                        elif byte == STATUS_HEADER_BYTE_1:  # 0xDD -> 诊断状态包
                            buf.append(byte)
                            expected_len = STATUS_PACKET_LEN  # 69
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        if len(buf) == expected_len:
                            if expected_len == PACKET_LEN:
                                pkt = parse_hr_packet(bytes(buf))
                                if pkt is not None:
                                    self.hr_packet_received.emit(pkt)
                            elif expected_len == RAW_PACKET_LEN:
                                self._raw_total += 1
                                pkt = parse_raw_packet(bytes(buf))
                                if pkt is not None:
                                    self.raw_packet_received.emit(pkt)
                                else:
                                    self._raw_invalid += 1
                            else:
                                status = parse_status_packet(bytes(buf))
                                if status is not None:
                                    self.raw_parse_stats_received.emit(
                                        self._raw_total,
                                        self._raw_invalid,
                                    )
                                    self.status_packet_received.emit(status)
                            state = 0
                            buf = bytearray()
        except serial.SerialException as e:
            if self._running:
                self.error_occurred.emit(f"Serial error: {e}")
        finally:
            # 握手超时检测
            if not self._mac_configured:
                self.mac_configured.emit(BLE_CUSTOM_MAC, False)
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.connection_changed.emit(False)
```

- [ ] **Step 4: 增加前缀匹配方法 `_feed_prefix()`**

在 `SerialReader` 类中增加:

```python
    def _feed_prefix(self, byte: int, prefix_buf: bytearray) -> bool:
        """
        HJ-380 From 前缀逐字节匹配.
        
        匹配模式: From<whitespace><12-hex-MAC>:<whitespace>
        例如: "From    784128c58150:  "
        
        Returns:
            True  - 匹配完成 (成功或失败), prefix_buf 已清空
            False - 匹配进行中, 需要更多字节
        """
        prefix_buf.append(byte)

        # 超长保护: 前缀不可能超过 HJ380_MAX_PREFIX_LEN
        if len(prefix_buf) > HJ380_MAX_PREFIX_LEN:
            # 不是有效前缀, 将缓冲内容作为文本行输出
            self._flush_prefix_as_text(prefix_buf)
            return True

        # 检测分隔符 ":  " (冒号 + 空格) 或 ": " (冒号 + 单个空格)
        # 在收到 ':' 之后紧跟空格时认为前缀结束
        buf_len = len(prefix_buf)
        if buf_len >= 2:
            # 查找 ":" 后跟空格的位置
            for i in range(buf_len - 1):
                if prefix_buf[i] == 0x3A and prefix_buf[i + 1] == 0x20:  # ':' + ' '
                    # 尝试解析前缀
                    prefix_str = bytes(prefix_buf[:i + 1]).decode("ascii", errors="ignore")
                    rest = bytes(prefix_buf[i + 1:])  # ":  " 之后的部分 (含空格)
                    mac = self._extract_mac_from_prefix(prefix_str)
                    if mac:
                        # 成功提取 MAC
                        self._mac_configured = True
                        self.mac_configured.emit(mac, True)
                    # 剥离前缀, 将剩余字节 (跳过开头的空格) 暂存
                    # 注意: 剩余字节 (rest) 开头的空格需要跳过,
                    # 后续字节才可能是二进制帧数据
                    # 但因为我们逐字节处理, rest 中的剩余字节已被消费在当前循环中
                    # 前缀已完成, 后续字节在下一轮循环中自然被处理
                    prefix_buf.clear()
                    return True

        return False  # 需要更多字节
```

- [ ] **Step 5: 增加辅助方法**

```python
    def _extract_mac_from_prefix(self, prefix: str) -> Optional[str]:
        """
        从 "From    784128c58150" 格式中提取 12 位 MAC 地址.
        
        Returns:
            12 字节 HEX MAC 字符串, 或 None (格式不匹配)
        """
        import re
        # 匹配: "From" + 空格 + 12 HEX + 可选 ":"
        m = re.match(r'^From\s+([0-9a-fA-F]{12}):?$', prefix)
        if m:
            return m.group(1).lower()
        return None

    def _flush_prefix_as_text(self, prefix_buf: bytearray):
        """将无效前缀缓冲作为标定文本行输出"""
        try:
            line = bytes(prefix_buf).decode("ascii", errors="ignore").strip()
            if line:
                self.calib_status_received.emit(line)
        except Exception:
            pass
        prefix_buf.clear()

    def _parse_at_response(self, data: bytes):
        """
        解析 HJ-380 AT 响应.
        
        格式: <st_con_mac=xxxxxxxxxxxx>  连接成功
              <st_con_mac=error>         连接失败
              <st_scan_device_on=timeout> 扫描超时
              <SCAN_DEVICE=mac,data,rssi> 扫描结果
              其他...
        """
        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            return

        # 去头尾尖括号
        if text.startswith("<") and text.endswith(">"):
            inner = text[1:-1]
        else:
            return

        # 连接 MAC 响应
        if inner.startswith(HJ380_RSP_CON_MAC_OK):
            mac = inner[len(HJ380_RSP_CON_MAC_OK):]
            if len(mac) == 12:
                self._mac_configured = True
                self.mac_configured.emit(mac, True)
            elif mac == "error":
                self._mac_configured = True
                self.mac_configured.emit(BLE_CUSTOM_MAC, False)
            else:
                self.mac_configured.emit(BLE_CUSTOM_MAC, False)
        else:
            # 其他 AT 响应作为调试文本输出
            self.calib_status_received.emit(text)
```

- [ ] **Step 6: 提交**

```bash
git add tools/monitor/serial_reader.py
git commit -m "feat: SerialReader增加HJ-380前缀剥离与MAC绑定握手
- 新增mac_configured信号, 发射MAC配置结果
- 连接时自动发送ST_CON_MAC命令
- From前缀逐字节匹配与剥离, 提取源MAC
- AT响应解析, 支持st_con_mac成功/失败检测
- 握手超时 (3秒) 检测与失败通知"
```

---

### Task 5: PC 端 — dashboard.py 增加 MAC 配置状态显示

**Files:**
- Modify: `tools/monitor/dashboard.py`

- [ ] **Step 1: 翻译表中新增 MAC 状态文案**

在 `TRANSLATIONS` 字典的 `"zh"` 和 `"en"` 中分别追加 3 行（在 `"disconnected"` 之后的位置）:

```python
    "zh": {
        # ... 现有条目 ...
        "disconnected": "未连接",
        "mac_configuring": "正在配置设备绑定...",
        "mac_configured": "已绑定设备 MAC: ",
        "mac_config_failed": "MAC 配置失败",
```

```python
    "en": {
        # ... 现有条目 ...
        "disconnected": "Disconnected",
        "mac_configuring": "Configuring device binding...",
        "mac_configured": "Bound MAC: ",
        "mac_config_failed": "MAC config failed",
```

- [ ] **Step 2: MAC 配置状态处理**

在 `MonitorWindow` 类中增加处理 MAC 配置信号的方法。找到 `show_error` 方法（约第 580 行）附近新增:

```python
    def on_mac_configured(self, mac: str, success: bool):
        """HJ-380 MAC 绑定配置结果回调"""
        t = TRANSLATIONS[self._lang]
        if success:
            self._conn_label.setText(t["mac_configured"] + mac.upper())
            self._conn_label.setStyleSheet(
                f"color: {COLOR_GREEN}; font-size: 12px; font-weight: bold;"
            )
        else:
            self._conn_label.setText(t["mac_config_failed"])
            self._conn_label.setStyleSheet(
                f"color: {COLOR_RED}; font-size: 12px; font-weight: bold;"
            )
```

- [ ] **Step 3: 连接 MAC 配置信号**

在 `main.py` 的 `AppController._connect()` 方法中（约第 40 行附近），连接新增的 `mac_configured` 信号:

找到 main.py 第 37-48 行，在信号连接区域增加一行:

```python
    def _connect(self):
        port = self._win._combo_port.currentText()
        if not port or "No ports" in port:
            self._win.show_error("No valid port selected")
            return

        self._reader = SerialReader(port, baudrate=115200)
        # 双协议信号连接
        self._reader.hr_packet_received.connect(self._win._hr_panel.update_data)
        self._reader.raw_packet_received.connect(self._win._raw_panel.handle_raw_data)
        self._reader.raw_parse_stats_received.connect(
            self._win._raw_panel.handle_raw_parse_stats
        )
        self._reader.status_packet_received.connect(self._win._raw_panel.handle_status_data)
        self._reader.calib_status_received.connect(self._win._raw_panel.handle_calib_status)
        self._reader.error_occurred.connect(self._on_error)
        self._reader.connection_changed.connect(self._win.set_connected)
        self._reader.mac_configured.connect(self._win.on_mac_configured)  # 新增
        self._reader.start()
```

- [ ] **Step 4: 连接状态文案支持 MAC 配置状态**

修改 `set_connected()` 方法（约第 568 行），连接时显示"配置中":

```python
    def set_connected(self, connected: bool):
        """更新连接状态 UI"""
        t = TRANSLATIONS[self._lang]
        self._status_dot.set_connected(connected)
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._combo_port.setEnabled(not connected)
        if connected:
            self._conn_label.setText(t["mac_configuring"])
            self._conn_label.setStyleSheet(
                f"color: {COLOR_ORANGE}; font-size: 12px;"
            )
        else:
            self._conn_label.setText(t["disconnected"])
            self._conn_label.setStyleSheet(
                f"color: {COLOR_TEXT_DIM}; font-size: 12px;"
            )
```

- [ ] **Step 5: 提交**

```bash
git add tools/monitor/dashboard.py tools/monitor/main.py
git commit -m "feat: 上位机UI增加HJ-380 MAC配置状态显示
- dashboard.py: 新增on_mac_configured回调, 连接时显示配置中状态
- main.py: 连接mac_configured信号到UI
- 翻译表新增mac_configuring/mac_configured/mac_config_failed文案"
```

---

### Task 6: 集成验证

- [ ] **Step 1: 验证 Python 代码无语法错误**

```bash
cd tools/monitor && python -c "from serial_reader import SerialReader; from protocol import BLE_CUSTOM_MAC; print('OK:', BLE_CUSTOM_MAC)"
```
Expected: `OK: 784128c58150`

- [ ] **Step 2: 验证上位机 GUI 可启动**

```bash
cd tools/monitor && python main.py --simulate &
# 检查窗口是否正常启动, 然后关闭
```
Expected: 窗口正常显示, 无异常。

- [ ] **Step 3: 验证 MCU 固件编译**

```bash
cd build && cmake --build . --target all 2>&1
```
Expected: 编译成功, 无警告。

- [ ] **Step 4: 提交最终验证结果 (如有微调)**

```bash
git diff --stat
# 如有微调, 提交
```

---

### 注意事项

1. **MAC 地址同步**: `Core/Inc/main.h` 和 `tools/monitor/protocol.py` 中的 `BLE_CUSTOM_MAC` 必须一致。当前使用 `784128c58150` 为占位值，实际部署时需替换为 HJ-131 模块的目标 MAC。

2. **数据格式风险**: 本文档假设 HJ-380 在 `<ST_CON_MAC>` 连接后输出 `From    mac:  data` 格式。若实际数据格式与数据手册描述有差异，需调整 `_feed_prefix()` 中的匹配逻辑。

3. **HJ-131 MAC 持久化**: 数据手册未明确说明 `<ST_OWN_MAC>` 设置是否掉电保存。若不保存，MCU 上电初始化流程会重新设置。

4. **多设备场景**: 此方案确保 HJ-380 只连接指定 MAC 的 HJ-131。如果周围存在其他 HJ-131 设备，它们不会被连接。
