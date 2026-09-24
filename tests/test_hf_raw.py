import csv, math
from dataclasses import replace
from PyQt5.QtWidgets import QApplication
from raw_data_panel import RawDataPanel
from firmware_build import FirmwareSettings
from protocol import parse_raw_packet
from test_raw_transport_quality import make_raw_packet

def test_raw_recording_display_gap_and_wrap(tmp_path):
    app = QApplication.instance() or QApplication([])
    panel = RawDataPanel()
    panel._toggle_record(tmp_path / "capture.csv")
    base = parse_raw_packet(make_raw_packet(0))
    for seq, value in [(65534, 1), (65535, 99), (0, 2), (1, 3), (3, 4), (4, 100), (5, 5)]:
        panel.handle_raw_data(replace(base, sequence=seq, Ut1=value))
    panel._update_plots()
    assert list(panel._curve_Ut1.getData()[1]) == [1, 99, 2, 3, 4, 100, 5]
    panel._stop_recording()
    with (tmp_path / "capture.csv").open(encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 8
    assert [float(r["Ut1(mV)"]) for r in rows[:4]] == [1, 99, 2, 3]
    assert rows[4]["ValidFlag"] == "0" and math.isnan(float(rows[4]["Ut1(mV)"]))
    assert not any("Median" in key for key in rows[0])
    assert [int(r["SampleIndex"]) for r in rows] == list(range(8))
    panel._clear_screen()
    assert len(panel._data_Ut1) == 0
    panel.close()


def test_default_single25_build_flags():
    args=FirmwareSettings(channel=0,ble_rf_disabled=8).cmake_args()
    for flag in ['-DBLE_SINGLE25=1','-DBLE_BATCH5=0','-DBLE_FIXED_MINUS10=0','-DBLE_POWER_EXPERIMENT=0','-DENABLE_BLE_CONFIG=0']:
        assert flag in args


def test_toolbar_is_raw_label():
    from dashboard import MonitorWindow, ui_font
    from PyQt5.QtWidgets import QLabel
    app = QApplication.instance() or QApplication([])
    win = MonitorWindow()
    assert isinstance(win._page_title, QLabel)
    assert "原始信号" in win._page_title.toolTip()
    assert ui_font().families() == ['Arial', 'SimSun']
    win.close()
