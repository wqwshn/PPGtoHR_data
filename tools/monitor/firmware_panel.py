"""Nonblocking configure/build/verify/flash workflow using QProcess."""
from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime

from PyQt5.QtCore import QProcess, QProcessEnvironment, pyqtSignal, Qt, QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
                            QLabel, QComboBox, QPushButton, QPlainTextEdit,
                            QMessageBox, QGroupBox, QScrollArea, QCheckBox, QToolButton,
                            QListView)

from firmware_build import (ROOT, BUILD, FirmwareSettings, tool_environment,
                            find_tools, flash_args, OPENOCD_CONFIG)
from dashboard import ui_font, COMBO_POPUP_QSS
from board_check_panel import BoardCheckPanel
from debug_logging import LogTools, append_log_text


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
        self._log_path = None
        self._phase = ""
        self.setObjectName("firmwarePage")
        self.setStyleSheet("""
            QWidget#firmwarePage { background: #101923; color: #DAE5EF; }
            QGroupBox { border: 1px solid #2A3D4E; border-radius: 10px;
                margin-top: 12px; padding: 18px 16px 12px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #A9C6DC; }
            QComboBox { background: #1B2A39; border: 1px solid #344B60;
                border-radius: 6px; padding: 6px 10px; color: #E4EDF5; }
            QComboBox:focus { border-color: #54B3CC; }
            QPushButton { background: #1A2B3C; border: 1px solid #36516A;
                border-radius: 6px; padding: 7px 13px; color: #DCE9F4; }
            QPushButton:hover { background: #243E54; border-color: #64B5CC; }
            QPushButton:disabled { color: #657B90; border-color: #263646; }
            QToolButton { color: #8CBFD8; padding: 8px 2px; border: none; text-align: left; }
            QCheckBox { color: #DCE9F4; spacing: 8px; }
        """ + COMBO_POPUP_QSS)
        root = QVBoxLayout(self)
        root.setContentsMargins(22, 18, 22, 16)
        root.setSpacing(12)
        header = QHBoxLayout()
        heading = QVBoxLayout()
        title = QLabel("固件配置与烧录")
        title.setStyleSheet("font-size: 18pt; font-weight: 600; color: #E8F2FA;")
        heading.addWidget(title)
        intro = QLabel("选择用途，检查参数，再编译或烧录。STM32L452CEU6 · ST-Link")
        intro.setStyleSheet("color: #9AB0C4;")
        intro.setWordWrap(True)
        heading.addWidget(intro)
        header.addLayout(heading, 1)
        self.manual_button = QPushButton("使用手册")
        self.manual_button.clicked.connect(lambda: QDesktopServices.openUrl(
            QUrl.fromLocalFile(str(ROOT / "docs" / "固件烧录与版本使用手册.md"))))
        header.addWidget(self.manual_button)
        root.addLayout(header)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setStyleSheet("QScrollArea { background: transparent; }")
        content = QWidget()
        content.setObjectName("firmwareContent")
        content.setStyleSheet("QWidget#firmwareContent { background: transparent; }")
        body = QVBoxLayout(content)
        body.setContentsMargins(0, 0, 8, 0)
        body.setSpacing(12)
        scroll.setWidget(content)
        root.addWidget(scroll, 1)
        self.form_box = QGroupBox("01  选择固件用途与采集参数")
        form = QFormLayout(self.form_box)
        self._form = form
        form.setSpacing(12)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        form.setRowWrapPolicy(QFormLayout.WrapLongRows)
        self.purpose = self._combo([("日常采集 · 稳定运行", 0), ("新板验收 · 通信检查与首次配置", 1)])
        self.channel = self._combo([("不使用 PPG · 双 IIC 关闭 / PPG 填 0", 0),
                                   ("PPG 1 · IIC1", 1), ("PPG 2 · IIC2", 2)])
        self.ble = self._combo([("保留模块现有配置（推荐）", 0),
                               ("首次配置 · 每次上电恢复出厂并写项目参数", 1)])
        self.mode = self._combo([("绿光 + 红光 + 红外 · 三路采集", 0),
                                ("红光 + 红外 · 血氧光模式", 1)])
        form.addRow("固件用途", self.purpose)
        form.addRow("PPG / IIC 通道", self.channel)
        form.addRow("采集光模式", self.mode)
        form.addRow("蓝牙初始化", self.ble)
        self.output_note = QLabel()
        self.output_note.setWordWrap(True)
        self.output_note.setStyleSheet("color: #98B5CB;")
        form.addRow("输出方式", self.output_note)
        body.addWidget(self.form_box)
        self.note = QLabel()
        self.note.setWordWrap(True)
        self.note.setStyleSheet("color: #A9BED0; padding: 3px 4px;")
        body.addWidget(self.note)
        self.advanced_toggle = QToolButton()
        self.advanced_toggle.setText("高级设置与实验")
        self.advanced_toggle.setCheckable(True)
        self.advanced_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.advanced_toggle.setArrowType(Qt.RightArrow)
        body.addWidget(self.advanced_toggle)
        self.advanced_box = QGroupBox("高级参数 · 对照实验")
        advanced = QFormLayout(self.advanced_box)
        self._advanced_form = advanced
        advanced.setSpacing(10)
        advanced.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        advanced.setRowWrapPolicy(QFormLayout.WrapLongRows)
        self.rf = self._combo([
            ("日常运行 · 保留模块配置", 0),
            ("射频关闭 · 保持模块硬件复位", 1),
            ("射频复位三轮 · 连接 / 复位对照", 2),
            ("功率七段 · 写入并尝试读回", 3),
            ("功率自检 · 只读，不设置功率", 4),
            ("功率七段 · 只写，不读回", 5),
            ("固定 -10 dBm · 启动复位后写入", 6),
            ("批量5帧 / 50 ms · 启动写 +2.5 dBm", 7),
            ("单帧100 Hz · 启动复位并写 +2.5 dBm", 8)])
        self.mimu = self._combo([("稳健启动 · 延时重试与后台恢复（推荐）", 1),
                                 ("常规启动 · 单次检测（对照）", 0)])
        self.mimu.setToolTip("检测前等待1秒，最多5次；失败后每2秒重试，后台恢复使用历史零偏。")
        self.speed = self._combo([(f"{x} kHz", x) for x in (100, 400, 1000, 1800)])
        self.debug = QCheckBox("启用详细诊断（寄存器读回与故障探测）")
        self.debug.setToolTip("只增加诊断详情；日常版本仍核验身份、配置并记录重要错误。")
        advanced.addRow("MIMU 启动方案", self.mimu)
        advanced.addRow("蓝牙策略 / 实验", self.rf)
        advanced.addRow("诊断日志", self.debug)
        advanced.addRow("ST-Link SWD 速度", self.speed)
        self.dual_button = QPushButton("打开双路采集窗口 · 有线参考 / 无线链路")
        self.dual_button.clicked.connect(self._open_dual_capture)
        advanced.addRow(self.dual_button)
        body.addWidget(self.advanced_box)
        self.advanced_toggle.toggled.connect(self._toggle_advanced)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet("color: #BCE6ED; padding: 12px; background: #152C3A; "
                                  "border-left: 3px solid #50B1C5; border-radius: 6px;")
        body.addWidget(self.summary)
        self.artifact_note = QLabel()
        self.artifact_note.setWordWrap(True)
        self.artifact_note.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.artifact_note.setStyleSheet("color: #8FA9BD; padding-left: 4px;")
        body.addWidget(self.artifact_note)
        row = QHBoxLayout()
        self.save_button = QPushButton("保存选择")
        self.probe_button = QPushButton("检查 ST-Link")
        self.build_button = QPushButton("仅编译")
        self.flash_button = QPushButton("编译并烧录")
        self.flash_button.setStyleSheet("QPushButton { background: #237F9B; border: 1px solid #5BBAD2; "
                                       "font-weight: 600; } QPushButton:hover { background: #2C96B4; }")
        for button in (self.save_button, self.probe_button, self.build_button, self.flash_button):
            button.setMinimumHeight(38)
            row.addWidget(button)
        body.addLayout(row)
        self.status = QLabel("就绪 · 选择参数后可编译；保存选择不会更改板上程序")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #9CB6C9; padding: 3px;")
        body.addWidget(self.status)
        self.check_panel = BoardCheckPanel(self.prepare_hardware, self,
            metadata_provider=lambda: {"settings": self.settings().__dict__,
                                      "build_log": str(self._log_path) if self._log_path else None})
        body.addWidget(self.check_panel)
        log_box = QGroupBox("02  编译与烧录日志")
        log_layout = QVBoxLayout(log_box)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(4000)
        self.log.setMinimumHeight(140)
        self.log.setMaximumHeight(220)
        self.log.setStyleSheet("background: #0B1520; border: none; padding: 8px; color: #B6CBDC;")
        log_layout.addWidget(self.log)
        self.log_tools = LogTools(self.log, self)
        log_layout.addWidget(self.log_tools)
        body.addWidget(log_box)
        body.addStretch()
        try:
            settings = FirmwareSettings.load()
        except (ValueError, TypeError, OSError) as exc:
            settings = FirmwareSettings()
            self.status.setText(f"配置文件无效，已显示默认值（原文件未覆盖）：{exc}")
        selections = ((self.channel, settings.channel), (self.ble, settings.ble_init),
                      (self.mode, settings.work_mode), (self.speed, settings.swd_speed),
                      (self.rf, settings.ble_rf_disabled), (self.purpose, settings.board_check),
                      (self.mimu, settings.mimu_recovery))
        for combo, value in selections:
            combo.setCurrentIndex(combo.findData(value))
        self.debug.setChecked(bool(settings.acquisition_debug))
        self.advanced_toggle.setChecked(bool(settings.ble_rf_disabled or settings.acquisition_debug or
                                              not settings.mimu_recovery or settings.swd_speed != 1000))
        self._toggle_advanced(self.advanced_toggle.isChecked())
        for combo, _ in selections:
            combo.currentIndexChanged.connect(self._update_summary)
        self.debug.toggled.connect(self._update_summary)
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

    def _toggle_advanced(self, expanded):
        self.advanced_box.setVisible(expanded)
        self.advanced_toggle.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)

    @staticmethod
    def _combo(options):
        combo = QComboBox()
        combo.setView(QListView(combo))
        for label, value in options:
            combo.addItem(label, value)
        combo.setMinimumHeight(34)
        combo.setMinimumContentsLength(18)
        combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        return combo

    def _open_dual_capture(self):
        for link in ('wired', 'wireless'):
            ok = QProcess.startDetached(sys.executable,
                 [str(ROOT / 'tools/monitor/main.py'), '--capture-link', link], str(ROOT))
            if isinstance(ok, tuple):
                ok = ok[0]
            if not ok:
                self.status.setText(f"无法打开 {link} 窗口，请查看 Python 环境。")
                return
        self.status.setText("双路窗口已打开 · 各自选择不同 COM 口并录制；保存到 recordings/wired 与 wireless")

    def settings(self):
        if self.purpose.currentData():
            return FirmwareSettings(channel=self.channel.currentData(), ble_init=self.ble.currentData(),
                                    swd_speed=self.speed.currentData(), board_check=1, mimu_recovery=0)
        return FirmwareSettings(self.channel.currentData(), self.ble.currentData(),
                                self.mode.currentData(), self.speed.currentData(), self.rf.currentData(),
                                mimu_recovery=self.mimu.currentData(), acquisition_debug=int(self.debug.isChecked()))

    def _update_summary(self):
        checking = bool(self.purpose.currentData())
        if checking and self.channel.currentData() == 1:
            self.channel.setCurrentIndex(self.channel.findData(2))
            return
        self.channel.model().item(self.channel.findData(1)).setEnabled(not checking)
        self.note.setText(
            "新板先按需完成蓝牙首次配置，再验收并换回日常采集固件。检查不运行采集或陀螺标定。\n"
            "USB-TTL RX → PA2、GND 共地；TX 与蓝牙 TX 硬件并接 PA3，电脑端勿发送。\n"
            "MAG 固定跳过，未接小板可跳过 PPG。蓝牙连接与数据需用 HJ-380 人工确认。"
            if checking else
            "推荐：MIMU 稳健启动，蓝牙保留现有配置。保存选择后需重新编译烧录才会生效。\n"
            "ST-Link：SWDIO → PA13，SWCLK → PA14，GND 共地；建议连接 NRST，主板独立供电。")
        if not checking and not self.check_panel.stop_listening():
            self.purpose.setCurrentIndex(self.purpose.findData(1))
            return
        self.check_panel.setVisible(checking)
        self.channel.setEnabled(True)
        self.mode.setEnabled(not checking and self.channel.currentData() != 0)
        for widget in (self.rf, self.mimu, self.debug):
            widget.setVisible(not checking)
            widget.setEnabled(not checking)
            self._advanced_form.labelForField(widget).setVisible(not checking)
        self.mode.setVisible(not checking)
        self._form.labelForField(self.mode).setVisible(not checking)
        self.dual_button.setVisible(not checking)
        self.ble.setEnabled(checking or self.rf.currentData() == 0)
        if not checking and self.rf.currentData() != 0 and self.ble.currentData() != 0:
            self.ble.setCurrentIndex(self.ble.findData(0))
        self.output_note.setText("通信检查结果 · USB-TTL 115200 / 8N1 · MAG 跳过" if checking else
                                 "Raw 100 Hz · 持续状态上报 · 自动保存接收日志")
        chosen = self.settings()
        self.summary.setText(chosen.summary())
        self.artifact_note.setText(f"构建产物：{chosen.build_dir.relative_to(ROOT).as_posix()}/L452CEU6.elf / .hex / .bin")

    def _save(self):
        try:
            self._preferences().save()
            self.status.setText("选择已保存 · 编译并烧录后才会应用到板子")
        except OSError as exc:
            self.status.setText(f"保存失败：{exc}")

    def _preferences(self):
        # 验收构建覆盖运行参数，但保存时保留用户隐藏的高级选择。
        return FirmwareSettings(channel=self.channel.currentData(), ble_init=self.ble.currentData(),
                                work_mode=self.mode.currentData(), swd_speed=self.speed.currentData(),
                                ble_rf_disabled=self.rf.currentData(), board_check=self.purpose.currentData(),
                                mimu_recovery=self.mimu.currentData(), acquisition_debug=int(self.debug.isChecked()))

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
            self._preferences().save()
            self._settings = settings
            self._flash = flash
            self._queue = [("配置工程", self.tools["cmake"], settings.cmake_args()),
                           ("编译固件", self.tools["cmake"], ["--build", str(settings.build_dir), "-j", "4"])]
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
        if not self.check_panel.stop_listening():
            raise OSError("检查串口尚未释放，请稍后重试。")
        logs = ROOT / ".local" / "logs"
        logs.mkdir(parents=True, exist_ok=True)
        self._log_path = logs / f"firmware-{datetime.now():%Y%m%d-%H%M%S-%f}.log"
        self._log_file = self._log_path.open("w", encoding="utf-8")
        self.log_tools.set_path(self._log_path)
        self.log.clear()
        self._set_busy(True)
        self._next()

    def _set_busy(self, value):
        self.busy = value
        self.form_box.setEnabled(not value)
        self.advanced_box.setEnabled(not value)
        self.advanced_toggle.setEnabled(not value)
        self.dual_button.setEnabled(not value)
        self.check_panel.setEnabled(not value)
        for button in (self.save_button, self.probe_button, self.build_button, self.flash_button):
            button.setEnabled(not value)
        self.busy_changed.emit(value)

    def _next(self):
        self._phase, program, args = self._queue.pop(0)
        self.status.setText(self._phase + "…")
        self._append(f"\n[{self._phase}] {program} {' '.join(args)}\n")
        self.process.start(program, args)

    def _append(self, text):
        append_log_text(self.log, text)
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
                elf = self._settings.build_dir / "L452CEU6.elf"
                metadata = {"settings": self._settings.__dict__,
                            "sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
                            "built_at": datetime.now().isoformat(),
                            "profile": self._settings.profile, "elf_path": str(elf)}
                text = json.dumps(metadata, indent=2)
                (self._settings.build_dir / "firmware-manifest.json").write_text(text, encoding="utf-8")
                (BUILD / "firmware-manifest.json").write_text(text, encoding="utf-8")
            except OSError as exc:
                self._complete(f"构建产物检查失败：{exc}，未继续烧录。")
                return
        if self._queue:
            self._next()
        else:
            message = ("烧录、校验与复位完成" if self._flash else
                       "ST-Link 与目标芯片连接正常" if self._phase == "检查 ST-Link" else
                       f"编译成功 · {self._settings.build_dir.relative_to(ROOT).as_posix()}/L452CEU6.elf（尚未烧录）")
            self._complete(message)
            if self._flash and self._settings.board_check:
                self.check_panel.start_listening()

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

    def closeEvent(self, event):
        if not self.check_panel.stop_listening():
            event.ignore()
            return
        super().closeEvent(event)
