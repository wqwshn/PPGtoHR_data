"""
PPG Heart Rate Monitor - 串口读取线程

在独立 QThread 中运行串口读取, 使用状态机逐字节解析帧,
解析成功后通过 pyqtSignal 发射 HRPacket 给 UI 线程.
"""
from __future__ import annotations

import serial
import serial.tools.list_ports
from typing import List, Optional
from PyQt5.QtCore import QThread, pyqtSignal

from protocol import (
    HEADER_BYTE_0, HEADER_BYTE_1, PACKET_LEN,
    parse_hr_packet, HRPacket,
)


class SerialReader(QThread):
    """串口读取与帧解析线程"""

    # 信号: 解析成功一个完整帧
    packet_received = pyqtSignal(HRPacket)
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

        # 状态机: 0=等待帧头0, 1=等待帧头1, 2=收集 payload
        state = 0
        buf = bytearray()

        try:
            while self._running:
                # 非阻塞读取, timeout=0.1s 保证能检查 _running 标志
                raw = self._serial.read(64)
                if not raw:
                    continue

                for byte in raw:
                    if state == 0:
                        if byte == HEADER_BYTE_0:
                            buf = bytearray([byte])
                            state = 1
                    elif state == 1:
                        if byte == HEADER_BYTE_1:
                            buf.append(byte)
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            # 连续 0xAA, 重新开始
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        if len(buf) == PACKET_LEN:
                            # 收集满一帧, 尝试解析
                            pkt = parse_hr_packet(bytes(buf))
                            if pkt is not None:
                                self.packet_received.emit(pkt)
                            # 重置状态机
                            state = 0
                            buf = bytearray()
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
