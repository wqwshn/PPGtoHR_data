"""
PPG Heart Rate Monitor - 仪表盘主窗口

暗色主题实时监测仪表盘, 包含:
  - 心率大字显示 + 运动状态
  - 三路径 BPM 对比
  - 实时心率趋势图 (pyqtgraph)
  - 底部状态栏

设计系统 (UI/UX Pro Max):
  风格: Dark Mode + Real-Time Monitoring
  配色: Primary #0891B2, Accent #059669
  背景: #0F1923, 卡片: #1A2332
"""

import time
from collections import deque

from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty, QSize
from PyQt5.QtGui import QFont, QColor, QPalette
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QProgressBar, QStatusBar,
    QFrame, QApplication,
)
import pyqtgraph as pg

from protocol import HRPacket


# ── 配色常量 ──────────────────────────────────────────────
COLOR_BG = "#0F1923"
COLOR_CARD = "#1A2332"
COLOR_CARD_BORDER = "#2A3A4E"
COLOR_PRIMARY = "#0891B2"
COLOR_TEXT = "#E2E8F0"
COLOR_TEXT_DIM = "#8899AA"
COLOR_GREEN = "#22C55E"
COLOR_ORANGE = "#F97316"
COLOR_RED = "#EF4444"
COLOR_BLUE = "#3B82F6"

# 心率颜色阈值
HR_COLOR_LOW = COLOR_BLUE        # < 60 BPM
HR_COLOR_NORMAL = COLOR_GREEN    # 60-100 BPM
HR_COLOR_ELEVATED = COLOR_ORANGE # 100-140 BPM
HR_COLOR_HIGH = COLOR_RED        # > 140 BPM

TREND_SECONDS = 60   # 趋势图显示最近 60 秒

DARK_QSS = f"""
QMainWindow, QWidget {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
}}
QFrame#card {{
    background-color: {COLOR_CARD};
    border: 1px solid {COLOR_CARD_BORDER};
    border-radius: 12px;
}}
QLabel {{
    color: {COLOR_TEXT};
}}
QPushButton {{
    background-color: {COLOR_PRIMARY};
    color: white;
    border: none;
    border-radius: 6px;
    padding: 6px 16px;
    font-size: 13px;
    font-weight: bold;
}}
QPushButton:hover {{
    background-color: #0EA5C9;
}}
QPushButton:disabled {{
    background-color: #2A3A4E;
    color: #556677;
}}
QPushButton#btn_disconnect {{
    background-color: {COLOR_RED};
}}
QPushButton#btn_disconnect:hover {{
    background-color: #DC2626;
}}
QComboBox {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_CARD_BORDER};
    border-radius: 6px;
    padding: 6px 12px;
    min-width: 120px;
}}
QComboBox::drop-down {{
    border: none;
}}
QComboBox QAbstractItemView {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    selection-background-color: {COLOR_PRIMARY};
}}
QStatusBar {{
    background-color: #0A1520;
    color: {COLOR_TEXT_DIM};
    font-size: 12px;
    border-top: 1px solid {COLOR_CARD_BORDER};
}}
QProgressBar {{
    background-color: #1A2332;
    border: none;
    border-radius: 4px;
    text-align: center;
    color: {COLOR_TEXT};
    font-size: 11px;
    min-height: 18px;
    max-height: 18px;
}}
QProgressBar::chunk {{
    background-color: {COLOR_PRIMARY};
    border-radius: 4px;
}}
"""


def hr_color(bpm: float) -> str:
    """根据 BPM 返回对应颜色"""
    if bpm < 60:
        return HR_COLOR_LOW
    elif bpm <= 100:
        return HR_COLOR_NORMAL
    elif bpm <= 140:
        return HR_COLOR_ELEVATED
    else:
        return HR_COLOR_HIGH


class StatusDot(QLabel):
    """带脉冲动画的连接状态指示灯"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self._opacity = 1.0
        self._connected = False
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._pulse)
        self._phase = 0
        self._update_style()

    def set_connected(self, connected: bool):
        self._connected = connected
        if connected:
            self._timer.start(1000)
        else:
            self._timer.stop()
            self._opacity = 1.0
        self._update_style()

    def _pulse(self):
        self._phase = (self._phase + 1) % 2
        self._opacity = 0.5 if self._phase else 1.0
        self._update_style()

    def _update_style(self):
        color = COLOR_GREEN if self._connected else COLOR_RED
        self.setStyleSheet(
            f"background-color: {color}; border-radius: 7px; "
            f"opacity: {int(self._opacity * 255)};"
        )


class Dashboard(QMainWindow):
    """心率监测仪表盘主窗口"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("PPG Heart Rate Monitor")
        self.setMinimumSize(900, 650)
        self.resize(1000, 700)

        # 数据存储
        self._trend_bpm = deque(maxlen=TREND_SECONDS)
        self._trend_time = deque(maxlen=TREND_SECONDS)
        self._start_time = time.time()
        self._pkt_count = 0

        self._init_ui()
        self._init_plot()

        # 模拟定时器 (仅 --simulate 模式使用)
        self._sim_timer = QTimer(self)
        self._sim_timer.timeout.connect(self._sim_tick)
        self._sim_step = 0

    # ── UI 构建 ──────────────────────────────────────────

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 8, 12, 8)

        # 工具栏
        root.addWidget(self._build_toolbar())

        # 上半部分: 心率卡片 + 三路径对比
        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        top_row.addWidget(self._build_hr_card(), 2)
        top_row.addWidget(self._build_paths_card(), 3)
        root.addLayout(top_row, 3)

        # 中间: 趋势图
        root.addWidget(self._build_trend_card(), 4)

        # 状态栏
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_label = QLabel("Disconnected")
        self._status_bar.addPermanentWidget(self._status_label, 1)

        self.setStyleSheet(DARK_QSS)

    def _build_toolbar(self) -> QFrame:
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 4, 0, 4)

        self._status_dot = StatusDot()
        layout.addWidget(self._status_dot)

        self._combo_port = QComboBox()
        layout.addWidget(self._combo_port)

        self._btn_connect = QPushButton("Connect")
        # 按钮信号由 AppController 在 main.py 中连接
        layout.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_disconnect.setObjectName("btn_disconnect")
        self._btn_disconnect.setEnabled(False)
        layout.addWidget(self._btn_disconnect)

        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(self._btn_refresh)

        layout.addStretch()

        self._conn_label = QLabel("Disconnected")
        self._conn_label.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 12px;")
        layout.addWidget(self._conn_label)

        return frame

    def _build_hr_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setAlignment(Qt.AlignCenter)

        # 心率数字
        self._lbl_bpm = QLabel("--.-")
        self._lbl_bpm.setAlignment(Qt.AlignCenter)
        font = QFont("Segoe UI", 72, QFont.Bold)
        self._lbl_bpm.setFont(font)
        self._lbl_bpm.setStyleSheet(f"color: {COLOR_TEXT_DIM};")
        layout.addWidget(self._lbl_bpm)

        # BPM 单位
        lbl_unit = QLabel("BPM")
        lbl_unit.setAlignment(Qt.AlignCenter)
        lbl_unit.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 16px; font-weight: bold;")
        layout.addWidget(lbl_unit)

        # 运动状态
        self._lbl_motion = QLabel("REST")
        self._lbl_motion.setAlignment(Qt.AlignCenter)
        self._lbl_motion.setStyleSheet(
            f"color: {COLOR_GREEN}; font-size: 18px; font-weight: bold; "
            f"background-color: #0D2818; border-radius: 8px; padding: 4px 24px;"
        )
        layout.addWidget(self._lbl_motion)

        return card

    def _build_paths_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 12, 16, 12)

        title = QLabel("Algorithm Paths")
        title.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 13px; font-weight: bold;")
        layout.addWidget(title)

        # 三路径进度条
        self._path_bars = {}
        self._path_labels = {}
        paths = [
            ("LMS-HF", COLOR_PRIMARY),
            ("LMS-ACC", "#8B5CF6"),
            ("FFT", COLOR_ORANGE),
        ]
        for name, color in paths:
            row = QHBoxLayout()
            lbl_name = QLabel(f"{name}:")
            lbl_name.setFixedWidth(70)
            lbl_name.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 13px;")
            row.addWidget(lbl_name)

            bar = QProgressBar()
            bar.setRange(0, 2000)  # x10 BPM, 最大 200 BPM
            bar.setValue(0)
            bar.setFormat("--.- BPM")
            bar.setStyleSheet(f"""
                QProgressBar::chunk {{
                    background-color: {color};
                    border-radius: 4px;
                }}
            """)
            row.addWidget(bar)
            self._path_bars[name] = bar

            layout.addLayout(row)

        layout.addSpacing(12)

        # 右下信息区域
        info_grid = QGridLayout()
        info_grid.setSpacing(6)

        info_items = [
            ("PPG Signal:", "_lbl_ppg", "--"),
            ("Calibration:", "_lbl_calib", "-- / 8"),
            ("Window:", "_lbl_window", "--"),
        ]
        for row_idx, (label_text, attr_name, default) in enumerate(info_items):
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 12px;")
            info_grid.addWidget(lbl, row_idx, 0)
            val = QLabel(default)
            val.setStyleSheet(f"color: {COLOR_TEXT}; font-size: 12px; font-weight: bold;")
            info_grid.addWidget(val, row_idx, 1)
            setattr(self, attr_name, val)

        layout.addLayout(info_grid)
        layout.addStretch()

        return card

    def _build_trend_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)

        # pyqtgraph 暗色配置
        pg.setConfigOptions(
            antialias=True,
            background=COLOR_CARD,
            foreground=COLOR_TEXT_DIM,
        )
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setLabel("left", "Heart Rate", units="BPM")
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.setYRange(40, 160)
        self._plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self._plot_widget.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_widget.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))

        # 趋势曲线
        self._curve = self._plot_widget.plot(
            pen=pg.mkPen(COLOR_PRIMARY, width=2.5),
            symbol=None,
        )

        # 正常心率参考区域 (60-100 BPM)
        normal_region = pg.LinearRegionItem(
            values=[60, 100],
            movable=False,
            brush=pg.mkBrush(34, 197, 94, 25),
        )
        normal_region.lines[0].setPen(pg.mkPen(COLOR_GREEN, style=Qt.DashLine, width=1))
        normal_region.lines[1].setPen(pg.mkPen(COLOR_GREEN, style=Qt.DashLine, width=1))
        self._plot_widget.addItem(normal_region)

        layout.addWidget(self._plot_widget)
        return card

    def _init_plot(self):
        """初始化趋势图数据"""
        pass  # 数据在 update_data 中逐步填充

    # ── 公共接口 ─────────────────────────────────────────

    def refresh_ports(self):
        """刷新串口列表"""
        import serial.tools.list_ports
        self._combo_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self._combo_port.addItems(ports)
        else:
            self._combo_port.addItem("No ports found")

    def set_connected(self, connected: bool):
        """更新连接状态 UI"""
        self._status_dot.set_connected(connected)
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._combo_port.setEnabled(not connected)
        self._conn_label.setText("Connected" if connected else "Disconnected")
        self._conn_label.setStyleSheet(
            f"color: {COLOR_GREEN if connected else COLOR_TEXT_DIM}; font-size: 12px;"
        )

    def update_data(self, pkt: HRPacket):
        """接收并展示一个心率数据包"""
        self._pkt_count += 1
        elapsed = time.time() - self._start_time

        # 心率数字
        bpm = pkt.fused_bpm
        color = hr_color(bpm)
        self._lbl_bpm.setText(f"{bpm:.1f}")
        self._lbl_bpm.setStyleSheet(f"color: {color};")

        # 运动状态
        if pkt.is_motion:
            self._lbl_motion.setText("MOTION")
            self._lbl_motion.setStyleSheet(
                f"color: {COLOR_ORANGE}; font-size: 18px; font-weight: bold; "
                f"background-color: #2D1B0E; border-radius: 8px; padding: 4px 24px;"
            )
        else:
            self._lbl_motion.setText("REST")
            self._lbl_motion.setStyleSheet(
                f"color: {COLOR_GREEN}; font-size: 18px; font-weight: bold; "
                f"background-color: #0D2818; border-radius: 8px; padding: 4px 24px;"
            )

        # 三路径进度条
        path_data = [
            ("LMS-HF", pkt.hr_lms_hf),
            ("LMS-ACC", pkt.hr_lms_acc),
            ("FFT", pkt.hr_fft),
        ]
        for name, val in path_data:
            bar = self._path_bars[name]
            bar.setValue(int(val * 10))
            bar.setFormat(f"{val:.1f} BPM")

        # 信息标签
        self._lbl_ppg.setText(str(pkt.ppg_mean))
        self._lbl_calib.setText(
            f"{pkt.calib_progress} / 8  "
            f"({'Calibrated' if pkt.motion_calibrated else 'Pending'})"
        )
        self._lbl_window.setText("Ready" if pkt.win_filled else "Filling...")

        # 趋势图
        self._trend_bpm.append(bpm)
        self._trend_time.append(elapsed)
        self._curve.setData(list(self._trend_time), list(self._trend_bpm))
        if self._trend_time:
            t_max = self._trend_time[-1]
            t_min = max(0, t_max - TREND_SECONDS)
            self._plot_widget.setXRange(t_min, max(t_max, t_min + 10))

            # 自动调整纵轴: 基于可见数据范围, 上下各留 10 BPM 余量
            bpm_list = list(self._trend_bpm)
            if bpm_list:
                y_min = max(0, min(bpm_list) - 10)
                y_max = min(250, max(bpm_list) + 10)
                # 保证最小显示范围 40-160
                y_min = min(y_min, 40)
                y_max = max(y_max, 160)
                self._plot_widget.setYRange(y_min, y_max)

        # 状态栏
        self._status_label.setText(
            f"Packets: {self._pkt_count}  |  "
            f"Uptime: {int(elapsed)}s  |  "
            f"PPG: {pkt.ppg_mean}  |  "
            f"Calib: {pkt.calib_progress}/8"
        )

    def show_error(self, msg: str):
        """显示错误信息"""
        self._conn_label.setText(msg)
        self._conn_label.setStyleSheet(f"color: {COLOR_RED}; font-size: 12px;")

    # ── 模拟模式 ─────────────────────────────────────────

    def start_simulation(self):
        """启动模拟数据模式"""
        self._sim_step = 0
        self._sim_timer.start(1000)  # 1Hz 模拟
        self.set_connected(True)
        self._conn_label.setText("Simulated")

    def stop_simulation(self):
        """停止模拟"""
        self._sim_timer.stop()

    def _sim_tick(self):
        """生成一个模拟心率包"""
        import math
        self._sim_step += 1
        t = self._sim_step

        # 正弦心率波动 65-80 BPM
        base = 72 + 8 * math.sin(t * 0.05)
        noise = (hash(t * 7) % 20 - 10) / 10.0
        bpm = base + noise

        # 模拟运动: 每 30-50 秒触发一段
        is_motion = 30 < (t % 60) < 45

        pkt = HRPacket(
            fused_bpm=round(bpm, 1),
            is_motion=is_motion,
            win_filled=t >= 8,
            hr_lms_hf=round(bpm + 1.2, 1),
            hr_lms_acc=round(bpm - 0.8, 1),
            hr_fft=round(bpm + 0.3, 1),
            ppg_mean=85000 + (hash(t) % 5000),
            motion_calibrated=t >= 8,
            timestamp=t,
            calib_progress=min(t, 8),
        )
        self.update_data(pkt)

