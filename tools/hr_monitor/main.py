"""
PPG Heart Rate Monitor - 程序入口

用法:
  实际串口模式:   python main.py
  模拟数据模式:   python main.py --simulate
"""
from __future__ import annotations

import sys
import argparse
from typing import Optional

from PyQt5.QtWidgets import QApplication

from dashboard import Dashboard
from serial_reader import SerialReader
from protocol import HRPacket


class AppController:
    """连接 Dashboard 和 SerialReader 的控制器"""

    def __init__(self, dashboard: Dashboard):
        self._dash = dashboard
        self._reader: Optional[SerialReader] = None

        # 直接连接 Dashboard 按钮到控制器方法
        self._dash._btn_connect.clicked.connect(self._connect)
        self._dash._btn_disconnect.clicked.connect(self._disconnect)

    def _connect(self):
        port = self._dash._combo_port.currentText()
        if not port or "No ports" in port:
            self._dash.show_error("No valid port selected")
            return

        self._reader = SerialReader(port, baudrate=115200)
        self._reader.packet_received.connect(self._dash.update_data)
        self._reader.error_occurred.connect(self._on_error)
        self._reader.connection_changed.connect(self._dash.set_connected)
        self._reader.start()

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._dash.set_connected(False)

    def _on_error(self, msg: str):
        self._dash.show_error(msg)

    def cleanup(self):
        self._disconnect()


def main():
    parser = argparse.ArgumentParser(description="PPG Heart Rate Monitor")
    parser.add_argument(
        "--simulate", action="store_true",
        help="Run with simulated data (no serial port needed)",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = Dashboard()
    win.refresh_ports()
    win.show()

    if args.simulate:
        win.start_simulation()
    else:
        ctrl = AppController(win)
        app.aboutToQuit.connect(ctrl.cleanup)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
