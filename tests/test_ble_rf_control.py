import os
import sys
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'monitor'))
import pytest
from PyQt5.QtWidgets import QApplication
from firmware_build import FirmwareSettings
from firmware_panel import FirmwarePanel


def test_rf_off_overrides_initialization_and_roundtrips(tmp_path):
    settings = FirmwareSettings(channel=0, ble_init=1, ble_rf_disabled=1)
    assert '-DBLE_RF_DISABLED=1' in settings.cmake_args()
    assert '-DENABLE_BLE_CONFIG=0' in settings.cmake_args()
    path = tmp_path / 'settings.json'
    settings.save(path)
    assert FirmwareSettings.load(path) == settings
    with pytest.raises(ValueError):
        FirmwareSettings(ble_rf_disabled=9)


def test_rf_three_cycle_ui_selection_and_saved_settings(tmp_path, monkeypatch):
    path = tmp_path / 'settings.json'
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    panel.rf.setCurrentIndex(panel.rf.findData(0))
    panel.ble.setCurrentIndex(panel.ble.findData(1))
    assert panel.rf.findData(2) >= 0
    panel.rf.setCurrentIndex(panel.rf.findData(2))
    assert not panel.ble.isEnabled()
    settings = panel.settings()
    assert settings.ble_init == 0
    assert settings.ble_rf_disabled == 2
    args = settings.cmake_args()
    assert '-DBLE_RF_EXPERIMENT=1' in args
    for flag in ('BLE_RF_DISABLED', 'ENABLE_BLE_CONFIG', 'BLE_POWER_EXPERIMENT',
                 'BLE_FIXED_MINUS10', 'BLE_BATCH5', 'BLE_SINGLE25'):
        assert f'-D{flag}=0' in args
    settings.save(path)
    load_settings = FirmwareSettings.load
    monkeypatch.setattr(FirmwareSettings, 'load', classmethod(lambda cls: load_settings(path)))
    restored = FirmwarePanel(lambda: True)
    assert restored.settings() == settings
    restored.close()
    panel.close()


def test_standard_ui_uses_explicit_power_configuration():
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    panel.rf.setCurrentIndex(panel.rf.findData(0))
    panel.ble.setCurrentIndex(panel.ble.findData(1))
    panel.rf.setCurrentIndex(panel.rf.findData(8))
    assert not panel.ble.isEnabled()
    assert panel.settings().ble_init == 0
    assert panel.settings().ble_rf_disabled == 8
    panel.rf.setCurrentIndex(panel.rf.findData(0))
    assert panel.ble.isEnabled()
    panel.close()
