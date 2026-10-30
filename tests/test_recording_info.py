"""Experiment groups, restored metadata and flattened recording previews."""
import os
import sys
from datetime import date
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtWidgets import QApplication, QWidget
from dashboard import DARK_QSS, RecordingInfoDialog
from recording_naming import RecordingMetadata


@pytest.mark.parametrize("token,group", [
    ("run", 0), ("rest", 0), ("quanji", 0),
    ("kaiji", 1), ("wenpiao", 1), ("PnP_up", 1), ("PnP_down", 1), ("huayi", 1)])
def test_reopening_metadata_selects_its_experiment_group(tmp_path, token, group):
    app = QApplication.instance() or QApplication([])
    metadata = RecordingMetadata(tmp_path, token, 3, "LYX", date(2026, 10, 27))
    dialog = RecordingInfoDialog(tmp_path, metadata, "zh")
    try:
        assert dialog._experiment_type.currentIndex() == group
        assert dialog._scenario.currentData() == token
        assert dialog._trial.value() == 3
        assert dialog._subject.isHidden() == (group == 1)
        assert dialog._directory_preview.text() == str(tmp_path / ("20261027" if group else "1027-LYX"))
        dialog.accept()
        assert dialog.recording_metadata == (replace(metadata, subject="") if group else metadata)
    finally:
        dialog.close()


def test_experiment_groups_filter_scenarios_and_preview_flat_slip_recording(tmp_path):
    app = QApplication.instance() or QApplication([])
    host = QWidget()
    host.setStyleSheet(DARK_QSS)
    dialog = RecordingInfoDialog(tmp_path, None, "zh", host)
    try:
        tabs = dialog._experiment_type
        assert [tabs.tabText(i) for i in range(tabs.count())] == [
            "运动场景采集实验", "信号性能实验"]
        assert dialog._scenario.findData("run") >= 0
        assert dialog._scenario.findData("huayi") == -1
        dialog._subject.setText("lyx")
        dialog._record_date.setDate(date(2026, 10, 27))
        tabs.setCurrentIndex(1)
        assert dialog._scenario.findData("run") == -1
        assert {dialog._scenario.itemData(i) for i in range(dialog._scenario.count())} == {
            None, "kaiji", "wenpiao", "PnP_down", "PnP_up", "huayi"}
        dialog._scenario.setCurrentIndex(dialog._scenario.findData("huayi"))
        assert dialog._subject.isHidden()
        assert dialog._subject_label.isHidden()
        assert dialog._directory_preview.text() == str(tmp_path / "20261027")
        assert dialog._file_preview.text() == "huayi1_20261027.csv"
        dialog.accept()
        assert dialog.recording_metadata.scenario_token == "huayi"
        assert dialog.recording_metadata.subject == ""
        assert not list(tmp_path.iterdir())
        tabs.setCurrentIndex(0)
        assert not dialog._subject.isHidden()
        assert dialog._subject.text() == "LYX"
        assert dialog._scenario.currentData() is None
    finally:
        dialog.close()
        host.close()
