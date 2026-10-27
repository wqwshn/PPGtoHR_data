"""Firmware purpose defaults, legacy choices and isolated build/flash targets."""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtGui import QPalette
from PyQt5.QtCore import QPoint
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from dashboard import DARK_QSS
from firmware_build import BUILD, FirmwareSettings, flash_args
from firmware_panel import FirmwarePanel


@pytest.mark.parametrize("style", ["Windows", "Fusion"])
def test_firmware_dropdown_popup_has_opaque_dark_background(monkeypatch, style):
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: cls()))
    app = QApplication.instance() or QApplication([])
    previous_style = app.style().objectName()
    app.setStyle(style)
    host = QWidget()
    host.setStyleSheet(DARK_QSS)
    panel = FirmwarePanel(lambda: True)
    QVBoxLayout(host).addWidget(panel)
    panel.advanced_toggle.setChecked(True)
    host.resize(1100, 1000)
    host.show()
    app.processEvents()
    try:
        for combo in (panel.purpose, panel.channel, panel.mode, panel.ble,
                      panel.mimu, panel.rf, panel.speed):
            combo.showPopup()
            app.processEvents()
            view = combo.view()
            # Sample blank space within an unselected row in the composited popup.
            # A transparent viewport renders black alone but exposes a white popup.
            row = 1 if combo.currentIndex() == 0 else 0
            rect = view.visualRect(combo.model().index(row, 0))
            point = view.viewport().mapTo(view.window(),
                QPoint(view.viewport().width() - 12, rect.center().y()))
            color = view.window().grab().toImage().pixelColor(point)
            assert color.alpha() == 255
            assert max(color.red(), color.green(), color.blue()) < 100, color.name()
            text = view.palette().color(QPalette.Text)
            assert min(text.red(), text.green(), text.blue()) > 180
            view.setCurrentIndex(combo.model().index(row, 0))
            app.processEvents()
            selected = view.window().grab().toImage().pixelColor(point)
            assert selected != color
            assert max(selected.red(), selected.green(), selected.blue()) < 160
            selected_text = view.palette().color(QPalette.HighlightedText)
            assert min(selected_text.red(), selected_text.green(), selected_text.blue()) > 180
            combo.hidePopup()
    finally:
        host.close()
        app.setStyle(previous_style)


def test_new_and_missing_settings_use_reliable_runtime_defaults(tmp_path):
    path = tmp_path / "settings.json"
    for contents in (None, '{"channel": 2}'):
        if contents is not None:
            path.write_text(contents, encoding="utf-8")
        settings = FirmwareSettings.load(path)
        assert settings.mimu_recovery == 1
        assert settings.ble_rf_disabled == settings.ble_init == settings.acquisition_debug == 0
        assert "-DENABLE_ACQUISITION_DEBUG=0" in settings.cmake_args()
    path.write_text('{"mimu_recovery":0,"ble_rf_disabled":7}', encoding="utf-8")
    old = FirmwareSettings.load(path)
    assert old.mimu_recovery == 0 and old.ble_rf_disabled == 7


def test_purposes_build_and_flash_the_same_isolated_target():
    cases = [(FirmwareSettings(), "runtime"),
             (FirmwareSettings(acquisition_debug=1), "diagnostic"),
             (FirmwareSettings(board_check=1, ble_init=1, acquisition_debug=1,
                               ble_rf_disabled=2), "board-check")]
    for settings, name in cases:
        expected = BUILD / name
        assert settings.build_dir == expected
        args = settings.cmake_args()
        assert args[args.index("-B") + 1] == str(expected)
        assert f"build/desktop/{name}/L452CEU6.elf" in flash_args(settings)[-1]
    checking = cases[-1][0].cmake_args()
    assert "-DENABLE_MIMU_RECOVERY=0" in checking
    assert "-DENABLE_ACQUISITION_DEBUG=0" in checking
    assert "-DBLE_RF_EXPERIMENT=0" in checking
    assert "-DENABLE_BLE_CONFIG=1" in checking


@pytest.mark.parametrize("legacy_rf", range(9))
def test_legacy_rf_modes_never_silently_change_and_advanced_choices_are_visible(monkeypatch, legacy_rf):
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(
        lambda cls: cls(ble_rf_disabled=legacy_rf)))
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    try:
        assert panel.settings().ble_rf_disabled == legacy_rf
        assert panel.advanced_toggle.isChecked() == (legacy_rf != 0)
        assert "runtime" in panel.artifact_note.text()
        panel.debug.setChecked(True)
        assert panel.settings().acquisition_debug == 1
        assert "diagnostic" in panel.artifact_note.text()
        panel.purpose.setCurrentIndex(panel.purpose.findData(1))
        assert panel.settings().acquisition_debug == panel.settings().mimu_recovery == 0
        panel.purpose.setCurrentIndex(panel.purpose.findData(0))
        assert panel.settings().acquisition_debug == 1
    finally:
        panel.close()


def test_invalid_debug_flag_is_rejected():
    with pytest.raises(ValueError):
        FirmwareSettings(acquisition_debug=2)


def test_saving_board_check_preserves_hidden_runtime_preferences(monkeypatch):
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: cls(acquisition_debug=1)))
    saved = []
    monkeypatch.setattr(FirmwareSettings, "save", lambda self: saved.append(self))
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    try:
        panel.purpose.setCurrentIndex(panel.purpose.findData(1))
        panel._save()
        assert saved[-1].mimu_recovery == saved[-1].acquisition_debug == 1
        assert panel.settings().mimu_recovery == panel.settings().acquisition_debug == 0
        panel.purpose.setCurrentIndex(panel.purpose.findData(0))
        assert panel.settings().mimu_recovery == panel.settings().acquisition_debug == 1
    finally:
        panel.close()
