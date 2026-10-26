"""Full diagnostic logs survive display trimming and arriving serial chunks."""
import json
import os
import sys
from pathlib import Path
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QApplication
from board_check_panel import BoardCheckPanel, BoardCheckReader
from firmware_panel import FirmwarePanel


def test_new_output_does_not_destroy_log_selection():
    app = QApplication.instance() or QApplication([])
    panels = [BoardCheckPanel(lambda: True), FirmwarePanel(lambda: True)]
    try:
        for panel in panels:
            panel._append("first line\nsecond line\n")
            cursor = panel.log.textCursor()
            cursor.setPosition(0)
            cursor.setPosition(10, QTextCursor.KeepAnchor)
            panel.log.setTextCursor(cursor)
            selected = panel.log.textCursor().selectedText()
            panel._append("new incoming data\n")
            assert panel.log.textCursor().selectedText() == selected
    finally:
        for panel in panels:
            panel.close()


def test_session_log_preserves_raw_bytes_and_last_unterminated_line(tmp_path, monkeypatch):
    import debug_logging
    from debug_logging import DebugSessionLog
    session = DebugSessionLog("COM_TEST", {"board_check": 1}, directory=tmp_path)
    reader = BoardCheckReader("COM_TEST", session_log=session)
    chunks = [b"boot\r\n", b"BLE: querying ", b"19200\xff\x00"]
    closed = []
    reader.isInterruptionRequested = lambda: not chunks

    class Port:
        in_waiting = 1024

        def read(self, _):
            return chunks.pop(0)

        def close(self):
            closed.append(True)

    monkeypatch.setattr("board_check_panel.serial.Serial", lambda **kwargs: Port())
    # No UI slots are connected: stopping/discarding queued display signals must
    # not discard the end of the serial capture.
    reader.run()
    payload = b"boot\r\nBLE: querying 19200\xff\x00"
    assert closed and session.raw_path.read_bytes() == payload
    text = session.path.read_text(encoding="utf-8")
    assert "BLE: querying 19200\\xff\\x00" in text
    assert "COM_TEST" in text and "CLOSE" in text and f"rx_bytes={len(payload)}" in text
    latest = json.loads((tmp_path / "latest-board-check.json").read_text(encoding="utf-8"))
    assert latest["log_path"] == str(session.path)
    assert latest["raw_path"] == str(session.raw_path)


def test_copy_full_log_uses_file_even_when_display_is_trimmed(tmp_path):
    from debug_logging import LogTools
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    file = tmp_path / "complete.log"
    full = "SESSION BEGIN\n" + "device line\n" * 1200 + "unterminated tail"
    file.write_text(full, encoding="utf-8")
    try:
        panel._append(full)
        assert "SESSION BEGIN" not in panel.log.toPlainText()
        tools = LogTools(panel.log)
        tools.set_path(file)
        tools.copy_all()
        assert app.clipboard().text() == full
        tools.close()
    finally:
        panel.close()


def test_log_creation_failure_does_not_open_serial(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    blocked = tmp_path / "not-a-directory"
    blocked.write_text("keep", encoding="utf-8")
    monkeypatch.setattr("debug_logging.LOG_DIR", blocked)
    panel = BoardCheckPanel(lambda: True)
    panel.port.clear()
    panel.port.addItem("COM_TEST", "COM_TEST")
    monkeypatch.setattr("board_check_panel.serial.Serial", lambda **kwargs: pytest.fail("serial must not open"))
    try:
        panel.start_listening()
        assert "无法创建检查日志" in panel.status.text()
        assert not panel.listening and panel.connect_button.isEnabled()
    finally:
        panel.close()


def test_log_buttons_open_saved_paths(tmp_path, monkeypatch):
    from debug_logging import LogTools
    app = QApplication.instance() or QApplication([])
    panel = BoardCheckPanel(lambda: True)
    path = tmp_path / "board-check.log"
    path.write_text("saved", encoding="utf-8")
    calls = []
    monkeypatch.setattr("debug_logging.QDesktopServices.openUrl", lambda url: calls.append(url.toLocalFile()) or True)
    try:
        panel.log_tools.set_path(path)
        panel.log_tools.open_log()
        panel.log_tools.open_folder()
        assert [Path(value) for value in calls] == [path, tmp_path]
    finally:
        panel.close()
