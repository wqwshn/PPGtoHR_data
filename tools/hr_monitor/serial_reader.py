"""
PPG Heart Rate Monitor - 串口读取线程

在独立 QThread 中运行串口读取, 使用状态机逐字节解析帧,
支持两种帧类型:
  - HR 结果包 (0xAA 0xCC, 31 字节)
  - 多光谱原始数据包 (0xAA 0xBB, 33 字节)
解析成功后通过 pyqtSignal 发射给 UI 线程.
"""
from __future__ import annotations

import serial
import serial.tools.list_ports
from typing import List, Optional
from PyQt5.QtCore import QThread, pyqtSignal

from protocol import (
    HEADER_BYTE_0, HEADER_BYTE_1, PACKET_LEN,
    parse_hr_packet, HRPacket,
    RAW_HEADER_BYTE_1, RAW_PACKET_LEN,
    parse_raw_packet, RawDataPacket,
)


class SerialReader(QThread):
    """串口读取与帧解析线程"""

    # 信号: 解析成功一个 HR 结果帧
    packet_received = pyqtSignal(HRPacket)
    # 信号: 解析成功一个原始数据帧
    raw_packet_received = pyqtSignal(RawDataPacket)
    # 信号: 错误信息
    error_occurred = pyqtSignal(str)
    # 信号: 连接状态变化
    connection_changed = pyqtSignal(bool)

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._running = False
        self._serial: Optional[serial.Serial] = None

    def run(self):
        """线程主循环: 打开串口 -> 逐字节状态机解析"""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            self._running = True
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open {self._port}: {e}")
            self.connection_changed.emit(False)
            return

        # 状态机: 0=等待帧头0xAA, 1=判断第二字节, 2=收集 payload
        state = 0
        buf = bytearray()
        packet_type = None  # 'hr' or 'raw'
        max_len = max(PACKET_LEN, RAW_PACKET_LEN)

        try:
            while self._running:
                # 非阻塞读取, timeout=0.1s 保证能检查 _running 标志
                raw = self._serial.read(64)
                if not raw:
                    continue

                for byte in raw:
                    if state == 0:
                        if byte == HEADER_BYTE_0:  # 0xAA
                            buf = bytearray([byte])
                            state = 1
                    elif state == 1:
                        if byte == HEADER_BYTE_1:  # 0xCC -> HR packet
                            buf.append(byte)
                            packet_type = 'hr'
                            state = 2
                        elif byte == RAW_HEADER_BYTE_1:  # 0xBB -> Raw packet
                            buf.append(byte)
                            packet_type = 'raw'
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            # 连续 0xAA, 重新开始
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        # 超长保护: 防止噪声导致 buf 无限增长
                        if len(buf) > max_len:
                            state = 0
                            buf = bytearray()
                            packet_type = None
                            continue
                        expected_len = PACKET_LEN if packet_type == 'hr' else RAW_PACKET_LEN
                        if len(buf) == expected_len:
                            # 收集满一帧, 尝试解析
                            if packet_type == 'hr':
                                pkt = parse_hr_packet(bytes(buf))
                                if pkt is not None:
                                    self.packet_received.emit(pkt)
                            else:
                                pkt = parse_raw_packet(bytes(buf))
                                if pkt is not None:
                                    self.raw_packet_received.emit(pkt)
                            # 重置状态机
                            state = 0
                            buf = bytearray()
                            packet_type = None
        except serial.SerialException as e:
            if self._running:
                self.error_occurred.emit(f"Serial error: {e}")
        finally:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.connection_changed.emit(False)

    def stop(self):
        """安全停止线程"""
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:
            self._serial.close()

    @staticmethod
    def list_ports() -> List[str]:
        """返回系统可用串口列表"""
        return [p.device for p in serial.tools.list_ports.comports()]
