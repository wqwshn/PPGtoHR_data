"""USB-TTL reception, report recovery and workbench serial ownership."""
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication
import board_check_panel as module
from board_check_panel import CHECK_ITEMS, BoardCheckParser, BoardCheckPanel, BoardCheckReader
from firmware_build import FirmwareSettings
from firmware_panel import FirmwarePanel
from workbench import Workbench


@pytest.fixture(autouse=True)
def isolated_log_directory(tmp_path, monkeypatch):
    monkeypatch.setattr("debug_logging.LOG_DIR", tmp_path / "logs")


def report(failed=()):
    rows = ["=== BOARD COMMUNICATION CHECK SUMMARY ==="]
    rows.extend(f"[{'FAIL' if name in failed else 'PASS'}] {name}: response-{i}" for i, name in enumerate(CHECK_ITEMS))
    rows.append(f"TOTAL: PASS={5 - len(failed)} FAIL={len(failed)}; PPG1=NOT_TESTED")
    return ("\r\n".join(rows) + "\r\n").encode()


@pytest.mark.parametrize("size", [1, 3, 127])
def test_parser_recovers_from_baud_garbage_and_chunk_boundaries(size):
    parser = BoardCheckParser()
    stream = b"\xff\x00garbage" * 1000 + b"\r\n" + report((CHECK_ITEMS[2],))
    messages = []
    for pos in range(0, len(stream), size):
        messages.extend(parser.feed(stream[pos:pos + size]))
    rows = [message for message in messages if message.kind == "result"]
    assert [row.item for row in rows] == list(CHECK_ITEMS)
    assert [row.status for row in rows] == ["PASS", "PASS", "FAIL", "PASS", "PASS"]
    assert messages[-1].passed == 4 and messages[-1].failed == 1
    assert parser.feed(b"[PASS] unknown: ok\nTOTAL: PASS=5 FAIL=1; PPG1=NOT_TESTED\n") == []


def test_incomplete_repeated_summary_never_reuses_older_passes():
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    parser = BoardCheckParser()
    for message in parser.feed(report()):
        panel._on_message(message)
    assert "通过 5 项，失败 0 项" in panel.status.text()
    partial = report().replace(b"[PASS] MIMU MAG: response-1\r\n", b"")
    for message in parser.feed(partial):
        panel._on_message(message)
    assert "不完整" in panel.status.text()
    assert panel.table.item(1, 1).text() == "待检查"
    for message in parser.feed(b"BOARD COMMUNICATION CHECK / PPG2 / 115200 8N1\r\n"):
        panel._on_message(message)
    assert all(panel.table.item(row, 1).text() == "待检查" for row in range(5))
    panel.close()


def test_skip_results_are_counted_and_old_firmware_reports_remain_readable():
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    payload = report().replace(b"[PASS] MIMU MAG:", b"[SKIP] MIMU MAG:").replace(
        b"[PASS] PPG2 MAX30101:", b"[SKIP] PPG2 MAX30101:").replace(
        b"TOTAL: PASS=5 FAIL=0;", b"TOTAL: PASS=3 FAIL=0 SKIP=2;")
    try:
        for message in BoardCheckParser().feed(payload):
            panel._on_message(message)
        assert "通过 3 项，失败 0 项，跳过 2 项" in panel.status.text()
        assert panel.table.item(1, 1).text() == "跳过"
        assert panel.table.item(3, 1).text() == "跳过"
        for message in BoardCheckParser().feed(report()):
            panel._on_message(message)
        assert "通过 5 项，失败 0 项，跳过 0 项" in panel.status.text()
    finally:
        panel.close()


def test_manual_bluetooth_is_pending_and_not_counted_as_pass_or_failure():
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    payload = report().replace(b"[PASS] BLE HJ-131IMH:", b"[MANUAL] BLE HJ-131IMH:").replace(
        b"TOTAL: PASS=5 FAIL=0;", b"TOTAL: PASS=4 FAIL=0 SKIP=0 MANUAL=1;")
    try:
        messages = BoardCheckParser().feed(payload)
        assert messages[-1].kind == "total" and messages[-1].manual == 1
        for message in messages:
            panel._on_message(message)
        assert panel.table.item(4, 1).text() == "待人工确认"
        assert "通过 4 项，失败 0 项" in panel.status.text()
        assert "待人工确认 1 项" in panel.status.text()
        assert "HJ-380" in panel.status.text()
        assert BoardCheckParser().feed(b"TOTAL: PASS=5 FAIL=0 SKIP=0 MANUAL=1; PPG1=NOT_TESTED\n") == []
    finally:
        panel.close()


@pytest.mark.parametrize("disconnect_error", [False, True])
def test_reader_is_readonly_and_always_closes(monkeypatch, disconnect_error):
    reader = BoardCheckReader("test-port")
    stopped = False
    events, errors, opened = [], [], []
    stream = bytearray(report((CHECK_ITEMS[4],)))
    reader.isInterruptionRequested = lambda: stopped
    reader.message_received.connect(events.append)
    reader.error_occurred.connect(errors.append)

    class Port:
        closed = False

        @property
        def in_waiting(self):
            return min(3, len(stream))

        def read(self, size):
            nonlocal stopped
            if not stream:
                if disconnect_error:
                    raise module.serial.SerialException("device removed")
                stopped = True
                return b""
            result = bytes(stream[:size])
            del stream[:size]
            return result

        def write(self, _):
            pytest.fail("The board-check viewer must never send serial commands")

        def close(self):
            self.closed = True

    port = Port()

    def open_port(**kwargs):
        opened.append(kwargs)
        return port

    monkeypatch.setattr(module.serial, "Serial", open_port)
    reader.run()
    assert opened[0]["port"] == "test-port" and opened[0]["baudrate"] == 115200
    assert opened[0]["timeout"] == 0.1 and not opened[0]["rtscts"]
    assert events[-1].failed == 1 and port.closed
    assert bool(errors) == disconnect_error


def process_until(app, condition):
    deadline = time.monotonic() + 3
    while not condition() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert condition()


@pytest.fixture
def fake_ports(monkeypatch):
    ports = []
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: FirmwareSettings(board_check=1)))
    monkeypatch.setattr(module.serial.tools.list_ports, "comports", lambda: [SimpleNamespace(device="COM_TEST", description="USB-TTL")])

    class Port:
        in_waiting = 0

        def __init__(self):
            self.closed = False

        def read(self, size):
            time.sleep(0.01)
            return b""

        def close(self):
            self.closed = True

    def open_port(**kwargs):
        port = Port()
        ports.append(port)
        return port

    monkeypatch.setattr(module.serial, "Serial", open_port)
    return ports


def test_workbench_locks_capture_and_releases_reader_on_close(fake_ports):
    app = QApplication.instance() or QApplication([])
    win = Workbench()
    try:
        win.firmware.check_panel.start_listening()
        process_until(app, lambda: bool(fake_ports))
        assert not win.monitor.isEnabled() and not win.demo_button.isEnabled()
        assert win.firmware.check_panel.stop_listening()
        assert fake_ports[-1].closed and win.monitor.isEnabled()
        win.firmware.check_panel.start_listening()
        process_until(app, lambda: len(fake_ports) == 2)
    finally:
        win.close()
    assert all(port.closed for port in fake_ports)
    assert not win.firmware.check_panel.listening


def test_failed_open_is_recoverable(monkeypatch):
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    panel.port.clear()
    panel.port.addItem("COM_BAD", "COM_BAD")

    def fail(**kwargs):
        raise module.serial.SerialException("port busy")

    monkeypatch.setattr(module.serial, "Serial", fail)
    try:
        panel.start_listening()
        process_until(app, lambda: not panel.listening)
        assert "port busy" in panel.status.text() and panel.connect_button.isEnabled()
    finally:
        panel.close()


def test_switch_mode_disconnects_and_flash_success_reconnects(fake_ports, tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    try:
        panel.check_panel.start_listening()
        process_until(app, lambda: bool(fake_ports))
        panel.purpose.setCurrentIndex(panel.purpose.findData(0))
        assert fake_ports[-1].closed and not panel.check_panel.listening
        panel.purpose.setCurrentIndex(panel.purpose.findData(1))
        panel.check_panel.start_listening()
        process_until(app, lambda: len(fake_ports) == 2)
        monkeypatch.setattr(panel, "_next", lambda: None)
        monkeypatch.setattr("firmware_panel.ROOT", tmp_path)
        panel._queue = []
        panel._begin()
        assert fake_ports[-1].closed and not panel.check_panel.listening
        panel._settings = FirmwareSettings(board_check=1)
        panel._flash = True
        panel._phase = "烧录与校验"
        panel._finished(0, QProcess.NormalExit)
        process_until(app, lambda: len(fake_ports) == 3)
        assert panel.check_panel.listening and not panel.busy
    finally:
        panel.close()
    assert all(port.closed for port in fake_ports)


def test_flash_failure_does_not_open_debug_port(fake_ports):
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    panel._settings = FirmwareSettings(board_check=1)
    panel._flash = True
    panel._phase = "烧录与校验"
    panel._set_busy(True)
    panel._finished(1, QProcess.NormalExit)
    assert not panel.check_panel.listening and not fake_ports
    panel.close()
