"""
PPG Monitor - 串口读取线程

在独立 QThread 中运行串口读取, 使用状态机逐字节解析帧.
支持双协议:
  - 31 字节心率结果包 (0xAA 0xCC) -> hr_packet_received
  - 35 字节多光谱原始传感器包 (0xAA 0xBB) -> raw_packet_received
  - 69 字节 Raw 链路诊断状态包 (0xAA 0xDD) -> status_packet_received

HJ-380 BLE 适配:
  - 连接握手: 自动发送 ST_CON_MAC 绑定指令
  - 前缀剥离: 识别并剥离 From mac: 数据前缀
  - AT 响应解析: 解析 HJ-380 命令应答, 发射 MAC 配置状态
"""
from __future__ import annotations

import re
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
    HJ380_HANDSHAKE_TIMEOUT_S,
)

from rf_events import RFEvent, RF_PACKET_LEN, RF_HEADER, parse_rf_event
from rf_events import POWER_DIAG_HEADER, POWER_DIAG_LEN, parse_power_diagnostic

SERIAL_READ_TIMEOUT_S = 0.01
SERIAL_READ_CHUNK_BYTES = max(PACKET_LEN, RAW_PACKET_LEN, STATUS_PACKET_LEN) * 4


def read_serial_chunk(serial_port) -> bytes:
    """Read a small buffered chunk so 100Hz packets are delivered with low latency."""
    waiting = getattr(serial_port, "in_waiting", 0)
    read_size = min(max(waiting, 1), SERIAL_READ_CHUNK_BYTES)
    return serial_port.read(read_size)


class SerialReader(QThread):
    """串口读取与双协议帧解析线程 (含 HJ-380 BLE 适配)"""

    # 信号: 心率结果包 (1Hz)
    hr_packet_received = pyqtSignal(HRPacket)
    # 信号: 原始传感器包 (100Hz)
    raw_packet_received = pyqtSignal(RawDataPacket)
    # 信号: Raw 链路诊断状态包 (1Hz)
    status_packet_received = pyqtSignal(StatusPacket)
    rf_event_received = pyqtSignal(RFEvent)
    # 信号: PC 端 Raw 候选帧解析统计 (总候选帧, 无效候选帧)
    raw_parse_stats_received = pyqtSignal(int, int)
    # 信号: 陀螺仪标定状态文本
    calib_status_received = pyqtSignal(str)
    # 信号: 错误信息
    error_occurred = pyqtSignal(str)
    # 信号: 连接状态变化
    connection_changed = pyqtSignal(bool)
    # 信号: HJ-380 MAC 配置结果 (mac, success)
    mac_configured = pyqtSignal(str, bool)

    def __init__(self, port: str, baudrate: int = 115200, parent=None, *, bind_mac: bool = True):
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._bind_mac = bind_mac
        self._running = False
        self._serial: Optional[serial.Serial] = None
        # 原始包统计 (用于丢包率计算)
        self._raw_total = 0
        self._raw_invalid = 0
        # HJ-380 握手状态
        self._handshake_deadline = 0.0
        self._mac_configured = False

    # ── HJ-380 握手与 AT 命令 ─────────────────────────────

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
        """HJ-380 连接握手: 断开已有连接 -> 绑定目标 MAC"""
        # 步骤1: 断开所有已有连接 (避免 "已连接" 状态下 ST_CON_MAC 被忽略)
        self._send_command("<ST_CENTER_LINK=ALL>")
        time.sleep(0.15)
        # 步骤2: 发送 ST_CON_MAC 绑定指令
        cmd = HJ380_CMD_CON_MAC_FMT.format(BLE_CUSTOM_MAC)
        return self._send_command(cmd)

    # ── HJ-380 From 前缀剥离 ──────────────────────────────

    def _feed_prefix(self, byte: int, prefix_buf: bytearray) -> bool:
        """
        HJ-380 From 前缀逐字节匹配.

        匹配模式: From<whitespace><12-hex-MAC>:<whitespace>
        例如: "From    784128c58150:  "

        Returns:
            True  - 匹配完成 (成功或失败), prefix_buf 已清空
            False - 匹配进行中, 需要更多字节

        注意: 不在每次匹配时发射 mac_configured 信号 (100Hz 开销).
              仅通过 AT 响应 (_parse_at_response) 确认初始握手状态.
        """
        prefix_buf.append(byte)

        # 超长保护
        if len(prefix_buf) > HJ380_MAX_PREFIX_LEN:
            self._flush_prefix_as_text(prefix_buf)
            return True

        # 检测分隔符 ": " (冒号 + 空格), 认为前缀结束
        buf_len = len(prefix_buf)
        if buf_len >= 2:
            for i in range(buf_len - 1):
                if prefix_buf[i] == 0x3A and prefix_buf[i + 1] == 0x20:  # ':' + ' '
                    prefix_str = bytes(prefix_buf[:i + 1]).decode("ascii", errors="ignore")
                    mac = self._extract_mac_from_prefix(prefix_str)
                    if mac:
                        self._mac_configured = True
                    prefix_buf.clear()
                    return True

        return False  # 需要更多字节

    # 预编译: From 前缀 MAC 提取正则 (避免每帧重复编译)
    _PREFIX_MAC_RE = re.compile(r'^From\s+([0-9a-fA-F]{12}):?$')

    def _extract_mac_from_prefix(self, prefix: str) -> Optional[str]:
        """
        从 "From    784128c58150" 格式中提取 12 位 MAC 地址.

        Returns:
            12 字节小写 HEX MAC 字符串, 或 None (格式不匹配)
        """
        m = self._PREFIX_MAC_RE.match(prefix)
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

    # ── HJ-380 AT 响应解析 ─────────────────────────────────

    def _parse_at_response(self, data: bytes):
        """
        解析 HJ-380 AT 响应.

        格式: <st_con_mac=xxxxxxxxxxxx>  连接成功
              <st_con_mac=error>         连接失败
              其他 AT 响应作为调试文本输出
        """
        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            return

        if text.startswith("<") and text.endswith(">"):
            inner = text[1:-1]
        else:
            return

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
            self.calib_status_received.emit(text)

    # ── 主循环 ────────────────────────────────────────────

    def run(self):
        """线程主循环: 打开串口 -> HJ-380 握手 -> 带前缀剥离的状态机解析"""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
                write_timeout=0.5,   # 写超时 500ms, 防止阻塞
            )
            self._running = True
            self._raw_total = 0
            self._raw_invalid = 0
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open {self._port}: {e}")
            self.connection_changed.emit(False)
            return

        # CH340 稳定等待: 串口打开后 DTR/RTS 可能触发设备复位, 需等待稳定
        time.sleep(0.3)

        # HJ-380 连接握手: 先断开已有连接, 再绑定目标 MAC
        if self._bind_mac:
            if not self._do_hj380_handshake():
                self.error_occurred.emit("HJ-380: 配置指令发送失败")
            self._handshake_deadline = time.time() + HJ380_HANDSHAKE_TIMEOUT_S
        else:
            # 有线参考窗口只读取数据，不向共享 UART 发送蓝牙绑定命令。
            self._handshake_deadline = 0.0
        self._mac_configured = not self._bind_mac

        # 帧状态机: 0=等待帧头0(0xAA), 1=等待帧头1区分协议, 2=收集 payload
        state = 0
        buf = bytearray()
        expected_len = 0
        # ASCII 文本行缓冲 (用于标定状态等调试文本)
        text_buf = bytearray()
        # HJ-380 From 前缀匹配缓冲
        prefix_buf = bytearray()
        # HJ-380 AT 响应缓冲
        at_buf = bytearray()

        try:
            while self._running:
                # 小块低延迟读取, 避免4096字节约124个原始包批量进入UI线程.
                raw = read_serial_chunk(self._serial)
                if not raw:
                    # 握手超时检测
                    if (not self._mac_configured and
                            self._handshake_deadline > 0 and
                            time.time() > self._handshake_deadline):
                        self._handshake_deadline = 0.0
                        self.mac_configured.emit(BLE_CUSTOM_MAC, False)
                    continue

                for byte in raw:
                    # ── AT 响应收集 (进行中) ──
                    if len(at_buf) > 0:
                        at_buf.append(byte)
                        if byte == 0x3E:  # '>'
                            self._parse_at_response(bytes(at_buf))
                            at_buf.clear()
                        elif len(at_buf) > 128:
                            at_buf.clear()
                        continue

                    # ── From 前缀匹配 (进行中) ──
                    if len(prefix_buf) > 0:
                        if self._feed_prefix(byte, prefix_buf):
                            continue
                        else:
                            continue

                    # ── 仅在帧状态机 IDLE 时检测 AT/Prefix 起始 ──
                    # 关键: 不能拦截帧 payload 内部的 0x3C/0x46 字节
                    if state == 0:
                        if byte == 0x3C:  # '<' -> AT 响应起始
                            at_buf = bytearray([byte])
                            continue

                        if byte == HJ380_PREFIX_FROM[0]:  # 'F' -> From 前缀起始
                            prefix_buf = bytearray([byte])
                            continue

                    # ── 帧状态机 ──
                    if state == 0:
                        if byte == HEADER_BYTE_0:  # 0xAA
                            buf = bytearray([byte])
                            state = 1
                        elif 0x20 <= byte <= 0x7E or byte in (0x0D, 0x0A):
                            # 可打印 ASCII 或换行: 收集文本行
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
                        elif byte == STATUS_HEADER_BYTE_1:  # 0xDD -> Raw链路诊断状态包
                            buf.append(byte)
                            expected_len = STATUS_PACKET_LEN  # 69
                            state = 2
                        elif byte == RF_HEADER:
                            buf.append(byte)
                            expected_len = RF_PACKET_LEN
                            state = 2
                        elif byte == POWER_DIAG_HEADER:
                            buf.append(byte)
                            expected_len = POWER_DIAG_LEN
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            # 连续 0xAA, 重新开始
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        if len(buf) == expected_len:
                            # 收集满一帧, 按类型解析
                            if buf[1] == POWER_DIAG_HEADER:
                                event = parse_power_diagnostic(bytes(buf))
                                if event is not None:
                                    self.rf_event_received.emit(event)
                            elif expected_len == RF_PACKET_LEN:
                                event = parse_rf_event(bytes(buf))
                                if event is not None:
                                    self.rf_event_received.emit(event)
                            elif expected_len == PACKET_LEN:
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
                            # 重置状态机
                            state = 0
                            buf = bytearray()
        except serial.SerialException as e:
            if self._running:
                self.error_occurred.emit(f"Serial error: {e}")
        finally:
            # 握手超时最终检测
            if not self._mac_configured:
                self.mac_configured.emit(BLE_CUSTOM_MAC, False)
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.connection_changed.emit(False)

    def stop(self):
        """安全停止线程"""
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def get_raw_stats(self) -> tuple[int, int]:
        """返回原始数据包统计 (总包数, 无效包数)"""
        return self._raw_total, self._raw_invalid

    @staticmethod
    def list_ports() -> List[str]:
        """返回系统可用串口列表"""
        return [p.device for p in serial.tools.list_ports.comports()]
