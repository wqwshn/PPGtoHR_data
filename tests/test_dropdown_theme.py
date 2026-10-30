"""Render application dropdowns through their actual parent stylesheets."""
import os
import sys
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtCore import QPoint
from PyQt5.QtGui import QPalette
from PyQt5.QtWidgets import QApplication
from dashboard import RecordingInfoDialog
from firmware_build import FirmwareSettings
from workbench import Workbench


@pytest.mark.parametrize("style", ["Windows", "Fusion"])
@pytest.mark.parametrize("location", ["acquisition-port", "recording-scenario", "recording-signal", "check-port"])
def test_application_dropdowns_match_firmware_dark_theme(monkeypatch, tmp_path, style, location):
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: cls()))
    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        SimpleNamespace(device="COM_TEST1", description="Test port 1"),
        SimpleNamespace(device="COM_TEST2", description="Test port 2")])
    app = QApplication.instance() or QApplication([])
    previous_style = app.style().objectName()
    app.setStyle(style)
    window = Workbench()
    dialog = None
    try:
        window.show()
        if location == "acquisition-port":
            combo = window.monitor._combo_port
        elif location in ("recording-scenario", "recording-signal"):
            dialog = RecordingInfoDialog(tmp_path, None, "zh", window.monitor)
            if location == "recording-signal":
                dialog._experiment_type.setCurrentIndex(1)
            dialog.show()
            combo = dialog._scenario
        else:
            window.tabs.setCurrentIndex(1)
            window.firmware.purpose.setCurrentIndex(1)
            combo = window.firmware.check_panel.port
        app.processEvents()
        combo.showPopup()
        app.processEvents()
        view = combo.view()
        row = 1 if combo.currentIndex() == 0 else 0
        index = combo.model().index(row, 0)
        rect = view.visualRect(index)
        point = view.viewport().mapTo(view.window(),
            QPoint(view.viewport().width() - 12, rect.center().y()))
        background = view.window().grab().toImage().pixelColor(point)
        assert background.alpha() == 255
        assert background.name() == "#1b2a39"
        assert view.palette().color(QPalette.Text).name() == "#e4edf5"
        assert rect.height() >= 34
        view.setCurrentIndex(index)
        app.processEvents()
        selected = view.window().grab().toImage().pixelColor(point)
        assert selected.name() == "#24566c"
        assert view.palette().color(QPalette.HighlightedText).name() == "#ffffff"
        combo.hidePopup()
    finally:
        if dialog is not None:
            dialog.close()
        window.close()
        app.setStyle(previous_style)
