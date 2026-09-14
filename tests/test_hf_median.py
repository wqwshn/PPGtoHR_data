import csv, math
from dataclasses import replace
from PyQt5.QtWidgets import QApplication
from raw_data_panel import RawDataPanel
from firmware_build import FirmwareSettings
from protocol import parse_raw_packet
from test_raw_transport_quality import make_raw_packet

def test_median_recording_toggle_gap_and_wrap(tmp_path):
    app=QApplication.instance() or QApplication([])
    panel=RawDataPanel(); panel._toggle_record(tmp_path)
    base=parse_raw_packet(make_raw_packet(0))
    for seq,value in [(65534,1),(65535,99),(0,2)]:
        panel.handle_raw_data(replace(base,sequence=seq,Ut1=value))
    assert list(panel._hf_smooth['Ut1'])[-1]==2
    panel._toggle_smoothing()
    assert not panel._smooth_display
    panel.handle_raw_data(replace(base,sequence=1,Ut1=3))
    panel.handle_raw_data(replace(base,sequence=3,Ut1=4))
    assert math.isnan(panel._hf_smooth['Ut1'][-1])
    panel.handle_raw_data(replace(base,sequence=4,Ut1=100))
    panel.handle_raw_data(replace(base,sequence=5,Ut1=5))
    panel._stop_recording()
    raw=next(p for p in tmp_path.glob('*.csv') if not p.stem.endswith('_status'))
    with raw.open(encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
    assert [float(r['Ut1(mV)']) for r in rows[:4]]==[1,99,2,3]
    assert rows[2]['Ut1_Median3(mV)']=='2' or float(rows[2]['Ut1_Median3(mV)'])==2
    assert float(rows[3]['Ut1_Median3(mV)'])==3
    assert rows[4]['ValidFlag']=='0' and rows[4]['Median3Valid']=='0'
    assert rows[5]['Median3Valid']=='0' and rows[6]['Median3Valid']=='0'
    assert float(rows[7]['Ut1_Median3(mV)'])==5
    assert len(list(tmp_path.glob('*_processing.json')))==1
    panel._clear_screen();assert len(panel._hf_smooth['Ut1'])==0
    panel.close()

def test_default_single25_build_flags():
    args=FirmwareSettings(channel=0,ble_rf_disabled=8).cmake_args()
    for flag in ['-DBLE_SINGLE25=1','-DBLE_BATCH5=0','-DBLE_FIXED_MINUS10=0','-DBLE_POWER_EXPERIMENT=0','-DENABLE_BLE_CONFIG=0']:
        assert flag in args


def test_toolbar_switch_preserves_recording_state():
    from dashboard import MonitorWindow, ui_font
    app=QApplication.instance() or QApplication([])
    win=MonitorWindow()
    assert not win._page_title.isChecked() and win._raw_panel._smooth_display
    assert win._raw_panel.smoothing_button.isHidden()
    win._page_title.click()
    assert not win._raw_panel._smooth_display
    win._page_title.click()
    assert win._raw_panel._smooth_display
    assert ui_font().families()==['Arial','SimSun']
    win.close()
