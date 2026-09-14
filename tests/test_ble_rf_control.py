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
