import json
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "monitor"))

import pytest
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMessageBox
from firmware_build import FirmwareSettings, flash_args
from firmware_panel import FirmwarePanel


def test_settings_roundtrip_and_explicit_compiler_definitions(tmp_path):
    chosen = FirmwareSettings(channel=1, ble_init=1, work_mode=1, swd_speed=400)
    path = tmp_path / "firmware.json"
    chosen.save(path)
    assert FirmwareSettings.load(path) == chosen
    args = chosen.cmake_args()
    for flag in ("-DPPG_DEFAULT_CHANNEL=1", "-DENABLE_BLE_CONFIG=1",
                 "-DCURRENT_WORK_MODE=1", "-DENABLE_RAW_DATA_PACKET=1", "-DPPG_SAMPLE_RATE=100"):
        assert flag in args
    assert "adapter speed 400" in flash_args(chosen)


@pytest.mark.parametrize("values", [{"channel": 3}, {"ble_init": "1;bad"},
                                    {"work_mode": 2}, {"swd_speed": 999}])
def test_invalid_firmware_settings_are_rejected(values):
    with pytest.raises(ValueError):
        FirmwareSettings(**values)


def test_failed_build_discards_flash_step():
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    panel._set_busy(True)
    panel._phase = "编译固件"
    panel._queue = [("烧录与校验", "openocd", [])]
    panel._finished(1, QProcess.NormalExit)
    assert not panel.busy
    assert not panel._queue
    assert "失败" in panel.status.text()
    assert panel.process.state() == QProcess.NotRunning
    panel.close()


def test_flash_cancel_never_prepares_hardware(monkeypatch):
    app = QApplication.instance() or QApplication([])
    calls = []
    panel = FirmwarePanel(lambda: calls.append("hardware"))
    monkeypatch.setattr(QMessageBox, "question", lambda *a: QMessageBox.No)
    panel.start_build(True)
    assert not calls
    assert not panel.busy
    panel.close()


def test_no_ppg_option_is_saved_and_disables_light_mode(tmp_path):
    settings = FirmwareSettings(channel=0)
    path = tmp_path / "firmware.json"
    settings.save(path)
    assert FirmwareSettings.load(path).channel == 0
    assert "-DPPG_DEFAULT_CHANNEL=0" in settings.cmake_args()
    app = QApplication.instance() or QApplication([])
    panel = FirmwarePanel(lambda: True)
    panel.channel.setCurrentIndex(panel.channel.findData(0))
    assert not panel.mode.isEnabled()
    assert "双 IIC 关闭" in panel.summary.text()
    panel.channel.setCurrentIndex(panel.channel.findData(2))
    assert panel.mode.isEnabled()
    panel.close()
