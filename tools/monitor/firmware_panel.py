"""Nonblocking configure/build/verify/flash workflow using QProcess."""
from __future__ import annotations

import hashlib
import json
from datetime import datetime

from PyQt5.QtCore import QProcess, QProcessEnvironment, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
                            QLabel, QComboBox, QPushButton, QPlainTextEdit,
                            QMessageBox, QGroupBox)

from firmware_build import (ROOT, BUILD, FirmwareSettings, tool_environment,
                            find_tools, flash_args, OPENOCD_CONFIG)
from dashboard import ui_font


class FirmwarePanel(QWidget):
    busy_changed = pyqtSignal(bool)

    def __init__(self, prepare_hardware, parent=None):
        super().__init__(parent)
        self.setFont(ui_font())
        self.prepare_hardware = prepare_hardware
        self.busy = False
        self._queue = []
        self._flash = False
        self._log_file = None
        self._phase = ""
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 16)
        root.setSpacing(14)
        title = QLabel("固件配置与烧录")
        title.setStyleSheet("font-size: 18pt; font-weight: 600;")
        root.addWidget(title)
        intro = QLabel("选择板子参数，然后编译或通过 ST-Link 烧录。目标芯片：STM32L452CEU6。")
        intro.setWordWrap(True)
        root.addWidget(intro)
        self.form_box = QGroupBox("采集参数")
        form = QFormLayout(self.form_box)
        form.setSpacing(14)
        self.channel = self._combo([("不使用 PPG · 两路 IIC 关闭 / PPG 填 0", 0), ("PPG 1 · IIC1", 1), ("PPG 2 · IIC2", 2)])
        self.ble = self._combo([("关闭 · 保留模块现有配置", 0), ("开启 · 每次上电初始化蓝牙", 1)])
        self.mode = self._combo([("绿光采集（心率光模式）", 0), ("红光 + 红外（血氧光模式）", 1)])
        self.speed = self._combo([(f"{x} kHz", x) for x in (100, 400, 1000, 1800)])
        form.addRow("PPG / IIC 通道", self.channel)
        form.addRow("蓝牙初始化", self.ble)
        form.addRow("采集光模式", self.mode)
        form.addRow("ST-Link SWD 速度", self.speed)
        form.addRow("输出与采样率", QLabel("原始数据 · 100 Hz（与采集页时间轴一致）"))
        root.addWidget(self.form_box)
        note = QLabel("蓝牙初始化开启后，每次上电都会发送固件中预设的模块配置指令。\n"
                      "SWD 接线：SWDIO → PA13，SWCLK → PA14，GND 共地，建议接 NRST。\n"
                      "按板子要求供电，ST-Link 的 VTref 是目标电压参考；串口采集仍需 UART / 蓝牙连接。")
        note.setWordWrap(True)
        note.setStyleSheet("color: #9BAFC3; font-size: 9pt;")
        root.addWidget(note)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet("color: #42C7DC; padding: 10px; background: #1A2332;")
        root.addWidget(self.summary)
        row = QHBoxLayout()
        self.save_button = QPushButton("保存选择")
        self.probe_button = QPushButton("检查 ST-Link 连接")
        self.build_button = QPushButton("仅编译")
        self.flash_button = QPushButton("编译并烧录")
        self.flash_button.setStyleSheet("background: #DC4655;")
        for button in (self.save_button, self.probe_button, self.build_button, self.flash_button):
            button.setMinimumHeight(38)
            row.addWidget(button)
        row.addStretch()
        root.addLayout(row)
        self.status = QLabel("就绪 · 尚未操作硬件")
        root.addWidget(self.status)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(4000)
        self.log.setMinimumHeight(150)
        self.log.setStyleSheet("background: #0A1520; border: 1px solid #2A3A4E; padding: 8px; font-size: 9pt;")
        root.addWidget(self.log, 1)
        try:
            settings = FirmwareSettings.load()
        except (ValueError, TypeError, OSError) as exc:
            settings = FirmwareSettings()
            self.status.setText(f"配置文件无效，已显示默认值（原文件未覆盖）：{exc}")
        for combo, value in ((self.channel, settings.channel), (self.ble, settings.ble_init),
                             (self.mode, settings.work_mode), (self.speed, settings.swd_speed)):
            combo.setCurrentIndex(combo.findData(value))
            combo.currentIndexChanged.connect(self._update_summary)
        self._update_summary()
        self.save_button.clicked.connect(self._save)
        self.build_button.clicked.connect(lambda: self.start_build(False))
        self.flash_button.clicked.connect(lambda: self.start_build(True))
        self.probe_button.clicked.connect(self.probe)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._error)

    @staticmethod
    def _combo(options):
        combo = QComboBox()
        for label, value in options:
            combo.addItem(label, value)
        combo.setMinimumHeight(36)
        return combo

    def settings(self):
        return FirmwareSettings(self.channel.currentData(), self.ble.currentData(),
                                self.mode.currentData(), self.speed.currentData())

    def _update_summary(self):
        self.mode.setEnabled(self.channel.currentData() != 0)
        self.summary.setText(self.settings().summary())

    def _save(self):
        try:
            self.settings().save()
            self.status.setText("选择已保存 · 编译并烧录后才会应用到板子")
        except OSError as exc:
            self.status.setText(f"保存失败：{exc}")

    def _prepare(self):
        env = tool_environment()
        self.tools = find_tools(env)
        qenv = QProcessEnvironment()
        for key, value in env.items():
            qenv.insert(key, value)
        self.process.setProcessEnvironment(qenv)
        self.process.setWorkingDirectory(str(ROOT))

    def start_build(self, flash=False):
        if self.busy:
            return
        settings = self.settings()
        if flash:
            answer = QMessageBox.question(self, "确认烧录目标", settings.summary() +
                "\n\n将覆盖所连接 STM32L452CEU6 的程序。请确认芯片型号、供电与 SWD 接线。\n"
                "烧录期间请勿断电或拔线。是否继续？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes or not self.prepare_hardware():
                return
        try:
            self._prepare()
            settings.save()
            self._settings = settings
            self._flash = flash
            self._queue = [("配置工程", self.tools["cmake"], settings.cmake_args()),
                           ("编译固件", self.tools["cmake"], ["--build", str(BUILD), "-j", "4"])]
            if flash:
                self._queue.append(("烧录与校验", self.tools["openocd"], flash_args(settings)))
            self._begin()
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def probe(self):
        if self.busy or not self.prepare_hardware():
            return
        try:
            self._prepare()
            self._flash = False
            self._queue = [("检查 ST-Link", self.tools["openocd"],
                            ["-f", str(OPENOCD_CONFIG), "-c", f"adapter speed {self.settings().swd_speed}",
                             "-c", "init; shutdown"])]
            self._begin()
        except OSError as exc:
            self.status.setText(str(exc))

    def _begin(self):
        logs = ROOT / ".local" / "logs"
        logs.mkdir(parents=True, exist_ok=True)
        self._log_file = (logs / f"firmware-{datetime.now():%Y%m%d-%H%M%S-%f}.log").open("w", encoding="utf-8")
        self.log.clear()
        self._set_busy(True)
        self._next()

    def _set_busy(self, value):
        self.busy = value
        self.form_box.setEnabled(not value)
        for button in (self.save_button, self.probe_button, self.build_button, self.flash_button):
            button.setEnabled(not value)
        self.busy_changed.emit(value)

    def _next(self):
        self._phase, program, args = self._queue.pop(0)
        self.status.setText(self._phase + "…")
        self._append(f"\n[{self._phase}] {program} {' '.join(args)}\n")
        self.process.start(program, args)

    def _append(self, text):
        self.log.moveCursor(self.log.textCursor().End)
        self.log.insertPlainText(text)
        if self._log_file:
            self._log_file.write(text)
            self._log_file.flush()

    def _read_output(self):
        self._append(bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace"))

    def _finished(self, code, exit_status):
        if not self.busy:
            return
        self._read_output()
        if code != 0 or exit_status != QProcess.NormalExit:
            self._complete(f"{self._phase}失败（退出码 {code}），已停止后续操作。请查看日志。")
            return
        if self._phase == "编译固件":
            try:
                elf = BUILD / "L452CEU6.elf"
                metadata = {"settings": self._settings.__dict__,
                            "sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
                            "built_at": datetime.now().isoformat()}
                (BUILD / "firmware-manifest.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            except OSError as exc:
                self._complete(f"构建产物检查失败：{exc}，未继续烧录。")
                return
        if self._queue:
            self._next()
        else:
            message = ("烧录、校验与复位完成" if self._flash else
                       "ST-Link 与目标芯片连接正常" if self._phase == "检查 ST-Link" else
                       "编译成功 · build/desktop/L452CEU6.elf（尚未烧录）")
            self._complete(message)

    def _error(self, error):
        if error == QProcess.FailedToStart and self.busy:
            self._complete(f"无法启动{self._phase}：{self.process.errorString()}")

    def _complete(self, message):
        self._queue.clear()
        self.status.setText(message)
        self._append("\n" + message + "\n")
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        self._set_busy(False)
