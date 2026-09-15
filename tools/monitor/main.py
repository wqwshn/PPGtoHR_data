"""
PPG Monitor - 程序入口

用法:
  实际串口模式:     python main.py
  原始数据模拟别名:  python main.py --simulate
  原始数据模拟模式: python main.py --raw-simulate
"""
from __future__ import annotations

import sys
import argparse
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from dashboard import MonitorWindow, ui_font
from serial_reader import SerialReader


class AppController:
    """连接 MonitorWindow 和 SerialReader 的控制器"""

    def __init__(self, window: MonitorWindow):
        self._win = window
        self._reader: Optional[SerialReader] = None

        self._win._btn_connect.clicked.connect(self._connect)
        self._win._btn_disconnect.clicked.connect(self._disconnect)

    def _connect(self):
        port = self._win._combo_port.currentData()
        if not port or "No ports" in port:
            self._win.show_error("No valid port selected")
            return

        self._win._raw_panel._rf_last = None
        self._reader = SerialReader(port, baudrate=115200, bind_mac=self._win._bind_mac)
        # 双协议信号连接
        self._reader.raw_packet_received.connect(self._win._raw_panel.handle_raw_data)
        self._reader.raw_parse_stats_received.connect(
            self._win._raw_panel.handle_raw_parse_stats
        )
        self._reader.rf_event_received.connect(self._win._raw_panel.handle_rf_event)
        self._reader.status_packet_received.connect(self._win._raw_panel.handle_status_data)
        self._reader.calib_status_received.connect(self._win._raw_panel.handle_calib_status)
        self._reader.error_occurred.connect(self._on_error)
        self._reader.connection_changed.connect(self._win.set_connected)
        self._reader.mac_configured.connect(self._win.on_mac_configured)
        self._reader.start()

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._win.stop_simulations()
        self._win.set_connected(False)

    def _on_error(self, msg: str):
        self._win.show_error(msg)

    def cleanup(self):
        self._disconnect()
        self._win.stop_simulations()


def main():
    parser = argparse.ArgumentParser(description="PPG Monitor")
    parser.add_argument(
        "--simulate", action="store_true",
        help="Alias for --raw-simulate",
    )
    parser.add_argument(
        "--raw-simulate", action="store_true",
        help="Run with raw sensor simulated data (100Hz)",
    )
    parser.add_argument("--firmware", action="store_true", help="Open firmware configuration page")
    parser.add_argument("--capture-link", choices=("wired", "wireless"),
                        help="Label a capture window and use a separate recording directory")
    args = parser.parse_args()

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    app.setFont(ui_font())
    app.setStyle("Fusion")

    from workbench import Workbench
    win = Workbench(simulate=args.simulate or args.raw_simulate, firmware=args.firmware,
                    capture_link=args.capture_link)
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
