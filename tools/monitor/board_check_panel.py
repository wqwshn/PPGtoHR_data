"""Read-only USB-TTL report viewer for the firmware communication checker."""
from __future__ import annotations

import re
from dataclasses import dataclass

import serial
import serial.tools.list_ports
from debug_logging import DebugSessionLog, LogTools, append_log_text
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QComboBox,
                            QPushButton, QLabel, QTableWidget, QTableWidgetItem,
                            QHeaderView, QAbstractItemView, QPlainTextEdit)

CHECK_ITEMS = ("MIMU ACC/GYRO", "MIMU MAG", "ADC ADS124S06", "PPG2 MAX30101", "BLE HJ-131IMH")


@dataclass(frozen=True)
class CheckMessage:
    kind: str
    item: str = ""
    status: str = ""
    detail: str = ""
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    manual: int = 0


class BoardCheckParser:
    """Bounded ASCII line parser; tolerate baud-switch garbage before a valid line."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[CheckMessage]:
        messages = []
        for byte in data:
            if byte != 10:
                self._buffer.append(byte)
                if len(self._buffer) > 512:
                    del self._buffer[:-256]
                continue
            line = self._buffer.decode("ascii", errors="replace").rstrip("\r")
            self._buffer.clear()
            if "BOARD COMMUNICATION CHECK /" in line:
                messages.append(CheckMessage("start"))
            elif line.endswith("=== BOARD COMMUNICATION CHECK SUMMARY ==="):
                messages.append(CheckMessage("summary_start"))
            elif match := re.search(r"STEP ([1-5])/5: (.+)$", line):
                if match[2] in CHECK_ITEMS and CHECK_ITEMS[int(match[1]) - 1] == match[2]:
                    messages.append(CheckMessage("step", item=match[2]))
            elif match := re.search(r"\[(PASS|FAIL|SKIP|MANUAL)\] ([^:]+): (.*)$", line):
                if match[2] in CHECK_ITEMS:
                    messages.append(CheckMessage("result", match[2], match[1], match[3]))
            elif match := re.search(r"TOTAL: PASS=([0-5]) FAIL=([0-5])(?: SKIP=([0-5]))?(?: MANUAL=([0-5]))?; PPG1=NOT_TESTED$", line):
                skipped = int(match[3] or 0)
                manual = int(match[4] or 0)
                if int(match[1]) + int(match[2]) + skipped + manual == len(CHECK_ITEMS):
                    messages.append(CheckMessage("total", passed=int(match[1]), failed=int(match[2]), skipped=skipped, manual=manual))
        return messages


class BoardCheckReader(QThread):
    opened = pyqtSignal()
    message_received = pyqtSignal(object)
    log_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, parent=None, *, session_log=None):
        super().__init__(parent)
        self.port = port
        self.session_log = session_log

    def run(self):
        port = None
        parser = BoardCheckParser()
        try:
            if self.session_log:
                self.session_log.event("CONNECT_ATTEMPT", self.port)
            port = serial.Serial(port=self.port, baudrate=115200, timeout=0.1,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, rtscts=False, dsrdtr=False)
            if self.session_log:
                self.session_log.event("CONNECTED", self.port)
            self.opened.emit()
            while not self.isInterruptionRequested():
                chunk = port.read(min(max(port.in_waiting, 1), 1024))
                if not chunk:
                    continue
                if self.session_log:
                    self.session_log.receive(chunk)
                self.log_received.emit(chunk.decode("ascii", errors="replace"))
                for message in parser.feed(chunk):
                    self.message_received.emit(message)
        except (serial.SerialException, OSError, ValueError) as exc:
            if self.session_log:
                try:
                    self.session_log.event("ERROR", str(exc))
                except OSError:
                    pass
            if not self.isInterruptionRequested():
                self.error_occurred.emit(f"检查串口或日志异常：{exc}")
        finally:
            if port is not None:
                try:
                    port.close()
                except (serial.SerialException, OSError):
                    pass
            if self.session_log:
                try:
                    self.session_log.close()
                except OSError as exc:
                    self.error_occurred.emit(f"检查日志保存失败：{exc}")


class BoardCheckPanel(QGroupBox):
    active_changed = pyqtSignal(bool)

    def __init__(self, prepare_hardware, parent=None, *, metadata_provider=None):
        super().__init__("新板检查结果 · USB-TTL 115200 / 8N1", parent)
        self.prepare_hardware = prepare_hardware
        self.metadata_provider = metadata_provider or (lambda: {})
        self._reader = None
        self._error = ""
        self._results = {}
        root = QVBoxLayout(self)
        row = QHBoxLayout()
        self.port = QComboBox()
        self.port.setMinimumWidth(150)
        self.refresh_button = QPushButton("刷新串口")
        self.connect_button = QPushButton("连接检查串口")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setEnabled(False)
        row.addWidget(QLabel("USB-TTL"))
        for widget in (self.port, self.refresh_button, self.connect_button, self.disconnect_button):
            row.addWidget(widget)
        row.addStretch()
        root.addLayout(row)
        self.status = QLabel("选择 USB-TTL 串口并连接，然后复位板子；结果将在此自动更新。")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        self.table = QTableWidget(len(CHECK_ITEMS), 3)
        self.table.setHorizontalHeaderLabels(["检查项目", "状态", "器件应答 / 原因"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.setMinimumHeight(195)
        for row, name in enumerate(CHECK_ITEMS):
            self.table.setItem(row, 0, QTableWidgetItem(name))
        self._reset_results()
        root.addWidget(self.table)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setMaximumHeight(150)
        self.log.setMinimumHeight(90)
        self.log.setPlaceholderText("检查日志；蓝牙需通过 HJ-380 确认连接和数据接收。")
        root.addWidget(self.log)
        self.log_tools = LogTools(self.log, self)
        root.addWidget(self.log_tools)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.start_listening)
        self.disconnect_button.clicked.connect(self.stop_listening)
        self.refresh_ports()

    @property
    def listening(self):
        return self._reader is not None

    def refresh_ports(self):
        selected = self.port.currentData()
        self.port.clear()
        for port in serial.tools.list_ports.comports():
            self.port.addItem(f"{port.device} · {port.description}", port.device)
        if not self.port.count():
            self.port.addItem("未发现串口", None)
        index = self.port.findData(selected)
        if index >= 0:
            self.port.setCurrentIndex(index)

    def _reset_results(self):
        self._results.clear()
        for row in range(len(CHECK_ITEMS)):
            self.table.setItem(row, 1, QTableWidgetItem("待检查"))
            self.table.setItem(row, 2, QTableWidgetItem(""))

    def start_listening(self):
        if self._reader is not None:
            return
        port = self.port.currentData()
        if not port:
            self.status.setText("未选择有效 USB-TTL 串口，请插入适配器并刷新串口。")
            return
        if not self.prepare_hardware():
            return
        try:
            session = DebugSessionLog(port, self.metadata_provider())
        except (OSError, ValueError) as exc:
            self.status.setText(f"无法创建检查日志，尚未打开串口：{exc}")
            return
        self.log_tools.set_path(session.path)
        self._error = ""
        self._reset_results()
        self.log.clear()
        reader = BoardCheckReader(port, self, session_log=session)
        self._reader = reader
        # Ignore delayed events from a disconnected session after reconnecting.
        reader.opened.connect(self._reader_opened)
        reader.message_received.connect(self._reader_message)
        reader.log_received.connect(self._reader_log)
        reader.error_occurred.connect(self._reader_error)
        reader.finished.connect(self._reader_finished)
        self._set_connected(True)
        self.status.setText(f"正在打开 {port}…")
        self.active_changed.emit(True)
        reader.start()

    @pyqtSlot()
    def _reader_opened(self):
        reader = self.sender()
        if reader is not None and self._reader is reader:
            self._opened(reader)

    @pyqtSlot(object)
    def _reader_message(self, message):
        if self._reader is not None and self._reader is self.sender():
            self._on_message(message)

    @pyqtSlot(str)
    def _reader_log(self, text):
        if self._reader is not None and self._reader is self.sender():
            self._append(text)

    @pyqtSlot(str)
    def _reader_error(self, text):
        if self._reader is not None and self._reader is self.sender():
            self._on_error(text)

    @pyqtSlot()
    def _reader_finished(self):
        reader = self.sender()
        if reader is not None and self._reader is reader:
            self._finished(reader)

    def _opened(self, reader):
        if self._reader is reader:
            self.status.setText(f"已连接 {reader.port}，请复位板子；若检查已完成，约5秒内会收到汇总。")

    def stop_listening(self) -> bool:
        reader = self._reader
        if reader is None:
            return True
        reader.requestInterruption()
        if not reader.wait(1500):
            self.status.setText("正在等待串口读取结束，请稍后重试。")
            return False
        self._finished(reader)
        return True

    def _finished(self, reader):
        if self._reader is not reader:
            return
        self._reader = None
        reader.deleteLater()
        self._set_connected(False)
        self.status.setText(self._error or "已断开检查串口；表格保留上次收到的结果。")
        self.active_changed.emit(False)

    def _set_connected(self, connected):
        self.port.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)

    def _on_error(self, text):
        self._error = text
        self.status.setText(text)

    def _append(self, text):
        # A non-checking firmware must not leave an arbitrarily long text block.
        text = text.replace("\r", "")
        if self.log.document().lastBlock().length() > 512:
            text = "\n" + text
        append_log_text(self.log, text)

    def _on_message(self, message: CheckMessage):
        if message.kind in ("start", "summary_start") or (message.kind == "step" and message.item == CHECK_ITEMS[0]):
            self._reset_results()
            self.status.setText("正在检查…" if message.kind != "summary_start" else "正在接收本轮汇总…")
        if message.kind in ("step", "result"):
            row = CHECK_ITEMS.index(message.item)
            status = "检查中" if message.kind == "step" else {"PASS": "通过", "FAIL": "失败", "SKIP": "跳过", "MANUAL": "待人工确认"}[message.status]
            self.table.setItem(row, 1, QTableWidgetItem(status))
            self.table.item(row, 1).setForeground(QColor(
                "#FBBF24" if message.kind == "step" or message.status == "MANUAL" else "#34D399" if message.status == "PASS" else
                "#9BAFC3" if message.status == "SKIP" else "#F87171"))
            self.table.setItem(row, 2, QTableWidgetItem(message.detail))
            self.table.item(row, 2).setToolTip(message.detail)
            if message.kind == "result":
                self._results[message.item] = message.status
        elif message.kind == "total":
            passed = sum(status == "PASS" for status in self._results.values())
            skipped = sum(status == "SKIP" for status in self._results.values())
            manual = sum(status == "MANUAL" for status in self._results.values())
            if (len(self._results) == len(CHECK_ITEMS) and passed == message.passed
                    and skipped == message.skipped and manual == message.manual):
                pending = f"，待人工确认 {message.manual} 项；请用 HJ-380 确认蓝牙连接及数据接收" if manual else ""
                self.status.setText(f"自动检查完成：通过 {message.passed} 项，失败 {message.failed} 项，跳过 {message.skipped} 项{pending}；PPG1未检查。复位板子可重测。")
            else:
                self.status.setText("本轮汇总不完整或计数不一致，等待下次报告；未收到的项目保持待检查。")

    def closeEvent(self, event):
        if not self.stop_listening():
            event.ignore()
            return
        super().closeEvent(event)
