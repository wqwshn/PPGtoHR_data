import os,sys,csv
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/monitor'))
from PyQt5.QtWidgets import QApplication
from rf_events import parse_rf_event,RF_PACKET_LEN
from raw_data_panel import RawDataPanel
from firmware_build import FirmwareSettings

def packet(code=6):
    a=bytearray([0xaa,0xee,code,2,3,0,1])
    for v in [0xfffffff0,12500,15,0xfffff000]: a.extend(v.to_bytes(4,'big'))
    a.extend(b'\x00\x00');check=0
    for v in a[2:]:check^=v
    a.extend([check,0x55]);return bytes(a)

def test_event_decode_and_checksum():
    p=packet();assert len(p)==RF_PACKET_LEN
    e=parse_rf_event(p)
    assert (e.code,e.cycle,e.mcu_ms,e.sample_counter,e.event_id)==(6,2,0xfffffff0,12500,15)
    assert parse_rf_event(p[:-1]) is None
    for i in range(2,26):
        bad=bytearray(p);bad[i]^=1;assert parse_rf_event(bytes(bad)) is None

def test_events_recorded_and_flushed_separately(tmp_path):
    app=QApplication.instance() or QApplication([])
    panel=RawDataPanel()
    panel.handle_rf_event(parse_rf_event(packet()))
    panel._toggle_record(tmp_path)
    panel.handle_rf_event(parse_rf_event(packet(7)))
    panel._stop_recording()
    paths=list(tmp_path.glob('*_rf_events.csv'));assert len(paths)==1
    with paths[0].open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    assert rows[0]['RecordMarker']=='recording_start_snapshot'
    assert rows[1]['Event']=='RESET_RELEASE'
    assert rows[1]['McuTime(ms)']==str(0xfffffff0)
    assert panel._rf_csv_file is None
    panel.close()

def test_alternating_mode_never_holds_reset_at_boot_or_reconfigures():
    args=FirmwareSettings(channel=0,ble_init=1,ble_rf_disabled=2).cmake_args()
    assert '-DBLE_RF_EXPERIMENT=1' in args
    assert '-DBLE_RF_DISABLED=0' in args
    assert '-DENABLE_BLE_CONFIG=0' in args

def test_fragmented_serial_stream_preserves_raw_status_and_events(monkeypatch):
    import serial_reader
    from test_raw_transport_quality import make_raw_packet, make_status_packet
    app=QApplication.instance() or QApplication([])
    reader=serial_reader.SerialReader('fake')
    bad=bytearray(packet());bad[7]^=1
    stream=bytearray(make_raw_packet(123)+packet()+bytes(bad)+make_status_packet()+make_raw_packet(124))
    class FakeSerial:
        is_open=True
        @property
        def in_waiting(self):return min(3,len(stream))
        def read(self,n):
            if not stream:reader._running=False;return b''
            data=bytes(stream[:n]);del stream[:n];return data
        def close(self):self.is_open=False
    monkeypatch.setattr(serial_reader.serial,'Serial',lambda **kwargs:FakeSerial())
    raw=[];events=[];status=[]
    reader.raw_packet_received.connect(raw.append)
    reader.rf_event_received.connect(events.append)
    reader.status_packet_received.connect(status.append)
    reader.run()
    assert [p.sequence for p in raw]==[123,124]
    assert len(events)==1 and len(status)==1
    assert reader.get_raw_stats()==(2,0)
