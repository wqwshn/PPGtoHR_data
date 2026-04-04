"""
PPG Monitor - 原始传感器数据可视化面板

暗色主题实时波形面板, 接收 21 字节原始传感器数据包 (125Hz),
展示 PPG 波形 / 热膜桥压 / 三轴加速度 / SpO2 温度.
"""
from __future__ import annotations

import csv
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QFileDialog, QStackedWidget,
)
import pyqtgraph as pg

from protocol import RawDataPacket

# 复用 dashboard 的配色常量
from dashboard import (
    COLOR_BG, COLOR_CARD, COLOR_CARD_BORDER,
    COLOR_PRIMARY, COLOR_TEXT, COLOR_TEXT_DIM,
    COLOR_GREEN, COLOR_ORANGE, COLOR_RED,
    TRANSLATIONS,
)

# 显示缓冲区大小
PLOT_POINTS = 1000


class RawDataPanel(QWidget):
    """原始传感器数据可视化面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lang = "zh"

        # 数据缓冲区
        self._data_Uc1 = deque(maxlen=PLOT_POINTS)
        self._data_Uc2 = deque(maxlen=PLOT_POINTS)
        self._data_Ut1 = deque(maxlen=PLOT_POINTS)
        self._data_Ut2 = deque(maxlen=PLOT_POINTS)
        self._data_Accx = deque(maxlen=PLOT_POINTS)
        self._data_Accy = deque(maxlen=PLOT_POINTS)
        self._data_Accz = deque(maxlen=PLOT_POINTS)
        self._data_ppg_g = deque(maxlen=PLOT_POINTS)
        self._data_ppg_r = deque(maxlen=PLOT_POINTS)
        self._data_ppg_ir = deque(maxlen=PLOT_POINTS)
        self._data_temp = deque(maxlen=PLOT_POINTS)

        # 状态
        self._current_mode = "hr"
        self._packet_count = 0
        self._sample_count = 0  # 每秒重置, 用于采样率计算
        self._start_time = time.time()

        # 录制
        self._is_recording = False
        self._csv_file = None
        self._csv_writer = None
        self._recording_start_time: Optional[float] = None

        self._init_ui()

        # 波形刷新定时器 (50ms = 20FPS)
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(50)

        # 采样率计算定时器 (1s)
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._update_sample_rate)
        self._rate_timer.start(1000)

        # 模拟定时器
        self._sim_timer = QTimer(self)
        self._sim_timer.timeout.connect(self._sim_tick)
        self._sim_step = 0

    # ── UI 构建 ──────────────────────────────────────────

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 8, 8, 8)

        # 顶部信息条
        layout.addWidget(self._build_info_bar())

        # PPG 波形区 (模式切换)
        self._mode_stack = QStackedWidget()
        self._build_hr_mode_widget()
        self._build_spo2_mode_widget()
        self._mode_stack.setCurrentIndex(0)
        layout.addWidget(self._mode_stack, 3)

        # 桥顶电压 Ut1, Ut2
        ut_widget = QWidget()
        ut_layout = QHBoxLayout(ut_widget)
        ut_layout.setContentsMargins(0, 0, 0, 0)
        ut_layout.setSpacing(6)
        self._plot_Ut1, self._curve_Ut1 = self._make_plot("Ut1 (mV)", COLOR_ORANGE)
        self._plot_Ut2, self._curve_Ut2 = self._make_plot("Ut2 (mV)", "#D946EF")
        ut_layout.addWidget(self._plot_Ut1)
        ut_layout.addWidget(self._plot_Ut2)
        layout.addWidget(ut_widget, 2)

        # 桥中电压 Uc1, Uc2
        uc_widget = QWidget()
        uc_layout = QHBoxLayout(uc_widget)
        uc_layout.setContentsMargins(0, 0, 0, 0)
        uc_layout.setSpacing(6)
        self._plot_Uc1, self._curve_Uc1 = self._make_plot("Uc1 (mV)", COLOR_RED)
        self._plot_Uc2, self._curve_Uc2 = self._make_plot("Uc2 (mV)", "#3B82F6")
        uc_layout.addWidget(self._plot_Uc1)
        uc_layout.addWidget(self._plot_Uc2)
        layout.addWidget(uc_widget, 1)

        # 三轴加速度
        self._plot_acc, self._curve_accx = self._make_plot("ACC (g)", COLOR_RED)
        self._curve_accy = self._plot_acc.plot(pen=pg.mkPen(COLOR_GREEN, width=1.5))
        self._curve_accz = self._plot_acc.plot(pen=pg.mkPen("#3B82F6", width=1.5))
        self._plot_acc.addLegend(offset=(60, 10))
        layout.addWidget(self._plot_acc, 2)

    def _build_info_bar(self) -> QFrame:
        """顶部状态信息条"""
        frame = QFrame()
        frame.setObjectName("card")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 6, 12, 6)

        self._lbl_mode = QLabel("Mode: --")
        self._lbl_mode.setStyleSheet(
            f"color: {COLOR_PRIMARY}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_mode)

        self._lbl_rate = QLabel("Rate: 0 Hz")
        self._lbl_rate.setStyleSheet(
            f"color: {COLOR_ORANGE}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_rate)

        self._lbl_loss = QLabel("Loss: 0.00%")
        self._lbl_loss.setStyleSheet(
            f"color: {COLOR_RED}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_loss)

        layout.addStretch()

        self._lbl_count = QLabel("Packets: 0")
        self._lbl_count.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 12px;"
        )
        layout.addWidget(self._lbl_count)

        return frame

    def _build_hr_mode_widget(self):
        """心率模式: PPG 绿光波形"""
        widget = QWidget()
        hlayout = QHBoxLayout(widget)
        hlayout.setContentsMargins(0, 0, 0, 0)
        hlayout.setSpacing(6)
        self._plot_ppg_g, self._curve_ppg_g = self._make_plot(
            "PPG Green", COLOR_GREEN
        )
        hlayout.addWidget(self._plot_ppg_g)
        self._mode_stack.addWidget(widget)

    def _build_spo2_mode_widget(self):
        """血氧模式: 红光+红外 / 温度+SpO2"""
        widget = QWidget()
        hlayout = QHBoxLayout(widget)
        hlayout.setContentsMargins(0, 0, 0, 0)
        hlayout.setSpacing(6)

        # 左侧: 红光 + 红外 上下排列
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        self._plot_ppg_r, self._curve_ppg_r = self._make_plot("PPG Red", COLOR_RED)
        self._plot_ppg_ir, self._curve_ppg_ir = self._make_plot("PPG IR", "#3B82F6")
        left_layout.addWidget(self._plot_ppg_r)
        left_layout.addWidget(self._plot_ppg_ir)
        hlayout.addWidget(left, 1)

        # 右侧: 温度 + SpO2 预留
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        self._plot_temp, self._curve_temp = self._make_plot(
            "Temp (C)", COLOR_ORANGE
        )
        self._plot_spo2, self._curve_spo2 = self._make_plot(
            "SpO2 (%) - Reserved", "#D946EF"
        )
        self._plot_spo2.setYRange(70, 100)
        right_layout.addWidget(self._plot_temp, 1)
        right_layout.addWidget(self._plot_spo2, 1)
        hlayout.addWidget(right, 1)

        self._mode_stack.addWidget(widget)

    def _make_plot(self, title: str, color: str) -> tuple[pg.PlotWidget, pg.PlotDataItem]:
        """创建一个暗色风格的 pyqtgraph 绘图组件"""
        pw = pg.PlotWidget()
        pw.setTitle(title, color=COLOR_TEXT_DIM, size="10pt")
        pw.showGrid(x=True, y=True, alpha=0.12)
        pw.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        pw.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        pw.setBackground(COLOR_CARD)
        curve = pw.plot(pen=pg.mkPen(color, width=1.5))
        return pw, curve

    # ── 数据处理 ─────────────────────────────────────────

    def handle_raw_data(self, pkt: RawDataPacket):
        """接收并缓存一个原始数据包"""
        self._sample_count += 1
        self._packet_count += 1

        # 模式自动切换
        target_mode = "hr" if pkt.is_hr_mode else "spo2"
        if target_mode != self._current_mode:
            self._current_mode = target_mode
            self._mode_stack.setCurrentIndex(0 if target_mode == "hr" else 1)

        # 追加数据缓冲区
        self._data_Uc1.append(pkt.Uc1)
        self._data_Uc2.append(pkt.Uc2)
        self._data_Ut1.append(pkt.Ut1)
        self._data_Ut2.append(pkt.Ut2)
        self._data_Accx.append(pkt.acc_x)
        self._data_Accy.append(pkt.acc_y)
        self._data_Accz.append(pkt.acc_z)
        self._data_ppg_g.append(pkt.ppg_green)
        self._data_ppg_r.append(pkt.ppg_red)
        self._data_ppg_ir.append(pkt.ppg_ir)
        self._data_temp.append(pkt.temperature)

        # CSV 实时写入
        if self._is_recording and self._csv_writer:
            elapsed = round(time.time() - self._recording_start_time, 3)
            self._csv_writer.writerow([
                elapsed, pkt.mode_str,
                round(pkt.Uc1, 5), round(pkt.Uc2, 5),
                round(pkt.Ut1, 5), round(pkt.Ut2, 5),
                round(pkt.acc_x, 5), round(pkt.acc_y, 5),
                round(pkt.acc_z, 5),
                round(pkt.ppg_green, 5), round(pkt.ppg_red, 5),
                round(pkt.ppg_ir, 5), round(pkt.temperature, 2),
            ])
            self._csv_file.flush()

        # 更新信息条
        self._update_info_bar(pkt)

    def _update_info_bar(self, pkt: RawDataPacket):
        t = TRANSLATIONS[self._lang]
        mode_text = t.get("hr_mode", "HR Mode") if pkt.is_hr_mode else t.get("spo2_mode", "SpO2 Mode")
        self._lbl_mode.setText(f"{t.get('mode', 'Mode')}: {mode_text}")
        self._lbl_count.setText(f"{t.get('pkt_count', 'Packets')}: {self._packet_count}")

    def _update_plots(self):
        """50ms 定时刷新波形"""
        if len(self._data_Uc1) == 0:
            return

        # 模式相关波形
        if self._current_mode == "hr":
            self._curve_ppg_g.setData(list(self._data_ppg_g))
        else:
            self._curve_ppg_r.setData(list(self._data_ppg_r))
            self._curve_ppg_ir.setData(list(self._data_ppg_ir))
            self._curve_temp.setData(list(self._data_temp))

        # 通用波形 (始终更新)
        self._curve_Ut1.setData(list(self._data_Ut1))
        self._curve_Ut2.setData(list(self._data_Ut2))
        self._curve_Uc1.setData(list(self._data_Uc1))
        self._curve_Uc2.setData(list(self._data_Uc2))
        self._curve_accx.setData(list(self._data_Accx))
        self._curve_accy.setData(list(self._data_Accy))
        self._curve_accz.setData(list(self._data_Accz))

    def _update_sample_rate(self):
        """1秒定时器: 刷新采样率"""
        t = TRANSLATIONS[self._lang]
        self._lbl_rate.setText(f"{t.get('sample_rate', 'Rate')}: {self._sample_count} Hz")
        self._sample_count = 0

    # ── 录制 ─────────────────────────────────────────────

    def _toggle_record(self) -> bool:
        """
        切换录制状态.
        Returns: True=正在录制, False=停止录制
        """
        t = TRANSLATIONS[self._lang]
        if not self._is_recording:
            desktop = Path.home() / "Desktop"
            if not desktop.exists():
                desktop = Path.home()
            default_name = str(
                desktop / f"raw_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            path, _ = QFileDialog.getSaveFileName(
                self, t["select_save_path"], default_name, "CSV Files (*.csv)"
            )
            if not path:
                return False
            self._csv_file = open(path, "w", newline="", encoding="utf-8-sig")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "Time(s)", "Mode",
                "Uc1(mV)", "Uc2(mV)", "Ut1(mV)", "Ut2(mV)",
                "AccX(g)", "AccY(g)", "AccZ(g)",
                "PPG_Green", "PPG_Red", "PPG_IR", "Temp(C)",
            ])
            self._recording_start_time = time.time()
            self._is_recording = True
            return True
        else:
            self._stop_recording()
            return False

    def _stop_recording(self):
        """停止录制并关闭文件"""
        self._is_recording = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self._recording_start_time = None

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ── 清屏 ─────────────────────────────────────────────

    def _clear_screen(self):
        """清除所有显示数据和曲线"""
        for d in (
            self._data_Uc1, self._data_Uc2, self._data_Ut1, self._data_Ut2,
            self._data_Accx, self._data_Accy, self._data_Accz,
            self._data_ppg_g, self._data_ppg_r, self._data_ppg_ir,
            self._data_temp,
        ):
            d.clear()

        self._packet_count = 0
        self._sample_count = 0
        self._start_time = time.time()

        # 停止录制
        if self._is_recording:
            self._stop_recording()

        # 清空曲线
        for curve in (
            self._curve_ppg_g, self._curve_ppg_r, self._curve_ppg_ir,
            self._curve_temp, self._curve_spo2,
            self._curve_Ut1, self._curve_Ut2,
            self._curve_Uc1, self._curve_Uc2,
            self._curve_accx, self._curve_accy, self._curve_accz,
        ):
            curve.setData([])

        # 重置信息条
        t = TRANSLATIONS[self._lang]
        self._lbl_mode.setText(f"{t.get('mode', 'Mode')}: --")
        self._lbl_rate.setText(f"{t.get('sample_rate', 'Rate')}: 0 Hz")
        self._lbl_loss.setText(f"{t.get('packet_loss', 'Loss')}: 0.00%")
        self._lbl_count.setText(f"{t.get('pkt_count', 'Packets')}: 0")

    # ── 语言切换 ─────────────────────────────────────────

    def _apply_language(self, lang: str):
        """应用语言到所有 UI 元素"""
        self._lang = lang
        t = TRANSLATIONS[lang]
        self._lbl_mode.setText(f"{t.get('mode', 'Mode')}: --")
        self._lbl_rate.setText(f"{t.get('sample_rate', 'Rate')}: -- Hz")
        self._lbl_loss.setText(f"{t.get('packet_loss', 'Loss')}: --")
        self._lbl_count.setText(f"{t.get('pkt_count', 'Packets')}: 0")

        # 更新图表标题
        self._plot_ppg_g.setTitle(
            t.get("ppg_green", "PPG Green"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_ppg_r.setTitle(
            t.get("ppg_red", "PPG Red"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_ppg_ir.setTitle(
            t.get("ppg_ir", "PPG IR"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_temp.setTitle(
            t.get("temperature", "Temp (C)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Ut1.setTitle(
            t.get("bridge_top", "Ut1 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Ut2.setTitle(
            t.get("bridge_top", "Ut2 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Uc1.setTitle(
            t.get("bridge_mid", "Uc1 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Uc2.setTitle(
            t.get("bridge_mid", "Uc2 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_acc.setTitle(
            t.get("acceleration", "ACC (g)"), color=COLOR_TEXT_DIM, size="10pt"
        )

    # ── 模拟模式 ─────────────────────────────────────────

    def start_simulation(self):
        """启动 125Hz 模拟数据"""
        self._sim_step = 0
        self._sim_timer.start(8)  # ~125Hz

    def stop_simulation(self):
        """停止模拟"""
        self._sim_timer.stop()

    def _sim_tick(self):
        """生成一个模拟原始数据包"""
        self._sim_step += 1
        t = self._sim_step / 125.0

        # 合成 PPG 信号 (~1.2Hz 心率)
        ppg = 1000.0 + 50.0 * math.sin(2 * math.pi * 1.2 * t)

        pkt = RawDataPacket(
            Ut2=1200.0 + 5 * math.sin(0.1 * t),
            Ut1=1198.0 + 5 * math.sin(0.1 * t + 0.5),
            Uc2=600.0 + 2 * math.sin(0.2 * t),
            Uc1=601.0 + 2 * math.cos(0.15 * t),
            acc_x=0.01 * math.sin(2 * t),
            acc_y=0.01 * math.cos(3 * t),
            acc_z=1.0 + 0.005 * math.sin(t),
            ppg_green=ppg,
            ppg_red=0.0,
            ppg_ir=0.0,
            temperature=36.5,
            is_hr_mode=True,
            mode_str="HR",
        )
        self.handle_raw_data(pkt)
