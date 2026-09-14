import csv
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "monitor"))

from PyQt5.QtWidgets import QApplication, QStackedWidget
from dashboard import MonitorWindow


def test_toolbar_records_raw_and_close_flushes_file(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MonitorWindow()
    try:
        assert not window.findChildren(QStackedWidget)
        assert not hasattr(window, "_hr_panel")
        window._save_dir = tmp_path
        window._btn_record.click()
        window._raw_panel._sim_tick()
        window.close()
        assert not window._raw_panel.is_recording
        path = next(p for p in tmp_path.glob("raw_data_*.csv") if "_status" not in p.stem)
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
