from firmware_build import FirmwareSettings
from test_raw_transport_quality import make_raw_packet, make_status_packet
import serial_reader

def test_batch_mode_excludes_other_experiments():
    args=FirmwareSettings(channel=0, ble_rf_disabled=7).cmake_args()
    assert '-DBLE_BATCH5=1' in args and '-DBLE_FIXED_MINUS10=0' in args
    assert '-DENABLE_BLE_CONFIG=0' in args and '-DBLE_POWER_EXPERIMENT=0' in args
    assert '-DBLE_BATCH5=0' in FirmwareSettings(ble_rf_disabled=6).cmake_args()

def test_five_frame_bursts_fragmented_with_status_and_sequence_wrap(monkeypatch):
    reader=serial_reader.SerialReader('fake')
    seq=[(65532+i)%65536 for i in range(15)]
    stream=bytearray()
    for start in range(0,15,5):
        stream.extend(b''.join(make_raw_packet(n) for n in seq[start:start+5]))
        stream.extend(make_status_packet())
    class FakeSerial:
        def write(self, data): return len(data)
        def flush(self): pass
        is_open=True
        @property
        def in_waiting(self): return min(79,len(stream))
        def read(self,n):
            if not stream: reader._running=False; return b''
            data=bytes(stream[:n]);del stream[:n];return data
        def close(self): self.is_open=False
    monkeypatch.setattr(serial_reader.serial,'Serial',lambda **kwargs: FakeSerial())
    packets=[]
    reader.raw_packet_received.connect(packets.append)
    reader.run()
    assert [p.sequence for p in packets]==seq


def test_experiment_modes_removed_from_workbench():
    from PyQt5.QtWidgets import QApplication
    from firmware_panel import FirmwarePanel
    app=QApplication.instance() or QApplication([])
    panel=FirmwarePanel(lambda: True)
    assert all(panel.rf.findData(v)==-1 for v in range(1,8))
    panel.rf.setCurrentIndex(panel.rf.findData(8))
    assert panel.settings().ble_rf_disabled == 8
    assert not panel.ble.isEnabled()
    assert '-DBLE_SINGLE25=1' in panel.settings().cmake_args()
    panel.close()
