"""Single desktop entry point for acquisition and firmware management."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QTabWidget, QMessageBox, QPushButton, QScrollArea

from dashboard import MonitorWindow, DARK_QSS, ui_font
from firmware_panel import FirmwarePanel
from firmware_build import ROOT


class Workbench(QMainWindow):
    def __init__(self, simulate=False, firmware=False, capture_link=None):
        super().__init__()
        from main import AppController
        self.setWindowTitle("PPG 工作台 · 采集与烧录")
        self.setFont(ui_font())
        self.setStyleSheet(DARK_QSS + """
            QTabWidget::pane { border: none; }
            QTabBar::tab { background: #1A2332; color: #9BAFC3; padding: 7px 22px; margin: 2px; }
            QTabBar::tab:selected { background: #0891B2; color: white; }
            QGroupBox { border: 1px solid #2A3A4E; border-radius: 8px; margin-top: 12px; padding: 18px; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; }
        """)
        self.resize(1360, 900)
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.monitor = MonitorWindow(bind_mac=capture_link != "wired")
        self.monitor.setWindowFlags(Qt.Widget)
        self.monitor.setMinimumSize(1000, 650)
        self.monitor._save_dir = ROOT / "recordings"
        if capture_link is not None:
            self.monitor._save_dir /= capture_link
            label = {'wired': '有线参考', 'wireless': '无线链路'}[capture_link]
            self.setWindowTitle(f"PPG 数据采集 · {label}")
            self.monitor._btn_save_path.setText('有线目录' if capture_link == 'wired' else '无线目录')
        self.monitor._save_dir.mkdir(parents=True, exist_ok=True)
        self.monitor._btn_save_path.setToolTip(str(self.monitor._save_dir))
        self.controller = AppController(self.monitor)
        self.monitor.refresh_ports()
        self.tabs.addTab(self.monitor, "原始数据采集")
        self.firmware = FirmwarePanel(self._prepare_hardware)
        firmware_scroll = QScrollArea()
        firmware_scroll.setWidgetResizable(True)
        firmware_scroll.setFrameShape(QScrollArea.NoFrame)
        firmware_scroll.setWidget(self.firmware)
        self.tabs.addTab(firmware_scroll, "固件配置与烧录")
        self.firmware.busy_changed.connect(lambda busy: self.monitor.setEnabled(not busy))
        self.demo_button = QPushButton("开始模拟")
        self.demo_button.setMinimumHeight(28)
        self.demo_button.clicked.connect(self._toggle_simulation)
        self.monitor._btn_disconnect.clicked.connect(lambda: self.demo_button.setText("开始模拟"))
        self.tabs.setCornerWidget(self.demo_button)
        self.firmware.busy_changed.connect(lambda busy: self.demo_button.setEnabled(not busy))
        if simulate:
            self._toggle_simulation()
        if firmware:
            self.tabs.setCurrentIndex(1)

    def _toggle_simulation(self):
        if self.monitor._raw_panel.is_recording:
            QMessageBox.information(self, "正在录制", "请先停止录制，再切换模拟或真实采集。")
            return
        if self.monitor._simulating:
            self.monitor.stop_simulations()
            self.monitor.set_connected(False)
            self.demo_button.setText("开始模拟")
        else:
            self.controller._disconnect()
            self.monitor._clear_active_panel()
            self.monitor.start_raw_simulation()
            self.demo_button.setText("停止模拟")

    def _prepare_hardware(self):
        if self.monitor._raw_panel.is_recording:
            QMessageBox.information(self, "正在录制", "请先停止数据录制，再连接目标芯片。")
            return False
        self.controller.cleanup()
        self.demo_button.setText("开始模拟")
        return True

    def closeEvent(self, event):
        if self.firmware.busy:
            QMessageBox.information(self, "操作进行中", "请等待当前编译或烧录结束后再关闭工作台。")
            event.ignore()
            return
        self.controller.cleanup()
        self.monitor.close()
        super().closeEvent(event)
