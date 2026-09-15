import csv
from datetime import date
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "monitor"))

from PyQt5.QtWidgets import QApplication, QStackedWidget
from dashboard import MonitorWindow
from recording_naming import RecordingMetadata, build_recording_paths


def test_toolbar_records_raw_and_close_flushes_file(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MonitorWindow()
    try:
        assert not window.findChildren(QStackedWidget)
        assert not hasattr(window, "_hr_panel")
        window._save_dir = tmp_path
        window._raw_recording_metadata = RecordingMetadata(tmp_path, "kaiji", 1, "TEST", date(2026, 9, 15))
        window._btn_record.click()
        window._raw_panel._sim_tick()
        window.close()
        assert not window._raw_panel.is_recording
        path = build_recording_paths(window._raw_recording_metadata).raw_path
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 1
        assert int(rows[0]["PPG_Green"]) > 0
    finally:
        window.close()


def test_clear_resets_recording_and_language_preserves_simulation(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MonitorWindow()
    try:
        window._save_dir = tmp_path
        window._raw_recording_metadata = RecordingMetadata(tmp_path, "kaiji", 1, "TEST", date(2026, 9, 15))
        window.start_raw_simulation()
        window._btn_record.click()
        window._btn_clear.click()
        assert not window._raw_panel.is_recording
        assert window._btn_record.text() == "录制"
        window._btn_lang.click()
        assert window._btn_record.text() == "Record"
        assert window._conn_label.text() == "Simulated"
    finally:
        window.close()
