"""Capture real parser input before decoding; keep device lines readable."""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtWidgets import QApplication

import debug_logging
import serial_reader
from dashboard import MonitorWindow
from debug_logging import DebugSessionLog
from main import AppController
from test_raw_transport_quality import make_raw_packet, make_status_packet


@pytest.mark.parametrize("chunk_size", [1, 7, 73])
def test_capture_log_saves_binary_text_status_and_partial_tail_without_record_button(tmp_path, monkeypatch, chunk_size):
    app = QApplication.instance() or QApplication([])
    session = DebugSessionLog("COM_TEST", {"link": "wireless"}, directory=tmp_path,
                              kind="capture", raw_text=False)
    reader = serial_reader.SerialReader("COM_TEST", bind_mac=False, session_log=session)
    device_line = b"[DEBUG-ppg-live] SCL=1 SDA=1 FAIL reg=0xFF stage=2 value=NA nonempty=0\r\n"
    payload = (b"From    784128c58150: " + make_raw_packet(8, green=0x463C0 / 16)
               + device_line + make_status_packet(mimu_state=3) + b"unterminated tail")
    stream = bytearray(payload)

    class Port:
        is_open = True

        @property
        def in_waiting(self):
            return min(chunk_size, len(stream))

        def read(self, size):
            if not stream:
                reader._running = False
                return b""
            result = bytes(stream[:size])
            del stream[:size]
            return result

        def close(self):
            self.is_open = False

    monkeypatch.setattr(serial_reader.serial, "Serial", lambda **kwargs: Port())
    monkeypatch.setattr(serial_reader.time, "sleep", lambda _: None)
    packets, lines, live_logs = [], [], []
    reader.raw_packet_received.connect(packets.append)
    reader.calib_status_received.connect(lines.append)
    reader.log_received.connect(live_logs.append)
    reader.run()
    assert [item.sequence for item in packets] == [8]
    assert device_line.decode().strip() in lines
    assert session.raw_path.read_bytes() == payload
    text = session.path.read_text(encoding="utf-8")
    assert device_line.decode().strip() in text
    assert '"mimu_state": 3' in text and "DEVICE_PARTIAL" in text and "unterminated tail" in text
    assert "PARSER" in text and "DISCONNECT" in text and "CLOSE" in text
    assert "\\xaa" not in text  # Binary Raw frames belong to .bin, not the readable log.
    assert any("DEBUG-ppg-live" in entry for entry in live_logs)
    latest = json.loads((tmp_path / "latest-capture.json").read_text(encoding="utf-8"))
    assert latest["log_path"] == str(session.path)


def test_capture_ui_can_copy_complete_log_after_display_trim_and_reconnect(tmp_path):
    app = QApplication.instance() or QApplication([])
    win = MonitorWindow()
    first = tmp_path / "first.log"
    first.write_text("first session\n" + "line\n" * 2400, encoding="utf-8")
    second = tmp_path / "second.log"
    second.write_text("second session\n", encoding="utf-8")
    try:
        win.set_capture_log(first)
        assert "first session" not in win._capture_log_display.toPlainText()
        win._capture_log_tools.copy_all()
        assert app.clipboard().text() == first.read_text(encoding="utf-8")
        win.set_capture_log(second)
        win.append_capture_log("new device line\n")
        assert "second session" in win._capture_log_display.toPlainText()
        assert "first session" not in win._capture_log_display.toPlainText()
        assert win._capture_log_tools.path == second
    finally:
        win.close()


def test_capture_log_creation_error_prevents_opening_port(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    blocked = tmp_path / "not-a-directory"
    blocked.write_text("keep", encoding="utf-8")
    monkeypatch.setattr(debug_logging, "LOG_DIR", blocked)
    win = MonitorWindow()
    win._combo_port.clear()
    win._combo_port.addItem("COM_TEST", "COM_TEST")
    controller = AppController(win)
    monkeypatch.setattr(serial_reader.SerialReader, "start", lambda _: pytest.fail("must not start reader"))
    try:
        controller._connect()
        assert "无法创建接收日志" in win._conn_label.text()
        assert controller._reader is None
    finally:
        win.close()


def test_serial_open_error_still_closes_capture_session(tmp_path, monkeypatch):
    session = DebugSessionLog("MISSING", {}, directory=tmp_path, kind="capture", raw_text=False)
    reader = serial_reader.SerialReader("MISSING", bind_mac=False, session_log=session)

    def fail(**kwargs):
        raise serial_reader.serial.SerialException("port missing")

    monkeypatch.setattr(serial_reader.serial, "Serial", fail)
    reader.run()
    text = session.path.read_text(encoding="utf-8")
    assert "port missing" in text and "CLOSE" in text
    assert reader._session_log is None


def test_controller_starts_auto_capture_and_log_failure_does_not_stop_raw_parser(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(debug_logging, "LOG_DIR", tmp_path)
    monkeypatch.setattr(serial_reader.time, "sleep", lambda _: None)
    win = MonitorWindow(bind_mac=False)
    win._combo_port.clear()
    win._combo_port.addItem("COM_TEST", "COM_TEST")
    controller = AppController(win)
    stream = bytearray(make_raw_packet(9))
    errors = []

    class Port:
        is_open = True
        in_waiting = 73

        def read(self, size):
            if not stream:
                controller._reader._running = False
                return b""
            result = bytes(stream[:size])
            del stream[:size]
            return result

        def close(self):
            self.is_open = False

    def start(reader):
        # This file failure must be visible and must not alter received Raw values.
        def fail(data):
            raise OSError("disk full")
        reader._session_log.receive = fail
        reader.error_occurred.connect(errors.append)
        reader.run()

    monkeypatch.setattr(serial_reader.serial, "Serial", lambda **kwargs: Port())
    monkeypatch.setattr(serial_reader.SerialReader, "start", start)
    try:
        controller._connect()
        assert win._raw_panel._packet_count == 1
        assert errors and "disk full" in errors[0]
        assert win._capture_log_tools.path.exists()
        assert (tmp_path / "latest-capture.json").exists()
    finally:
        controller.cleanup()
        win.close()
