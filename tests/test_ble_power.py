import pytest
import csv
import os
import sys
from datetime import datetime
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/monitor'))
from PyQt5.QtWidgets import QApplication
from firmware_build import FirmwareSettings
from rf_events import parse_rf_event, event_row, RF_CSV_HEADER
from raw_data_panel import RawDataPanel
from rf_events import parse_power_diagnostic

def diagnostic_packet(tail=b'<rd_tx_power=+2.5>', reason=0):
    data=bytearray(53)
    data[:7]=bytes([0xaa,0xef,1,1,16,1,reason])
    for i, value in ((7,10000),(11,1000),(15,20),(19,len(tail))):
        data[i:i+4]=value.to_bytes(4,'big')
    data[25:27]=(1).to_bytes(2,'big')
    data[27]=len(tail); data[28:28+len(tail)]=tail; data[50]=1
    for byte in data[2:51]: data[51]^=byte
    data[52]=0x55
    return bytes(data)

def test_readonly_diagnostics_and_corruption():
    p=diagnostic_packet()
    e=parse_power_diagnostic(p)
    row=dict(zip(RF_CSV_HEADER,event_row(e,0,0)))
    assert row['DiagReason']=='READ_OK' and row['CommandTxDone']==1
    assert row['RxTailAscii']=='<rd_tx_power=+2.5>'
    assert row['PowerVerified']=='' and row['TargetPower(dBm)']==''
    for i in range(2,52):
        bad=bytearray(p); bad[i]^=1
        assert parse_power_diagnostic(bytes(bad)) is None
    e=parse_power_diagnostic(diagnostic_packet(b'',1))
    assert e.diagnostic['RxBytes']==0 and '(无)' in e.summary
    e=parse_power_diagnostic(diagnostic_packet(b'\0\xff',2))
    assert e.diagnostic['RxTailHex']=='00 ff'
    assert e.diagnostic['RxTailAscii']==r'\x00\xff'
    args=FirmwareSettings(ble_rf_disabled=4,ble_init=1).cmake_args()
    assert '-DBLE_POWER_READ_ONLY=1' in args and '-DENABLE_BLE_CONFIG=0' in args

def power_packet(code=15, stage=6, phase=9):
    data = bytearray([0xaa, 0xee, code, stage, phase, 1, 0])
    for value in (400000, 36000, 72, 390000):
        data.extend(value.to_bytes(4, 'big'))
    data.extend(b'\0\0')
    check = 0
    for value in data[2:]: check ^= value
    return bytes(data + bytes([check, 0x55]))

def test_power_decode_and_csv():
    e = parse_rf_event(power_packet())
    assert '-10 dBm' in e.summary
    row = dict(zip(RF_CSV_HEADER, event_row(e, 1.2, 10)))
    assert row['TargetPower(dBm)'] == -10 and row['PowerVerified'] == 1
    assert row['Event'] == 'POWER_START' and row['Experiment'] == 'power'
    for code, phase in ((0, 5), (0, 12), (17, 7), (19, 11), (18, 10)):
        assert parse_rf_event(power_packet(code, 7, phase)) is not None
    assert parse_rf_event(power_packet(15, 8, 9)) is None
    assert parse_rf_event(power_packet(15, 2, 3)) is None
    assert parse_rf_event(power_packet(6, 2, 9)) is None
    bad = bytearray(power_packet()); bad[4] ^= 1
    assert parse_rf_event(bad) is None

def test_power_settings_do_not_reset_other_module_parameters():
    args = FirmwareSettings(channel=0, ble_init=1, ble_rf_disabled=3).cmake_args()
    for expected in ('-DBLE_POWER_EXPERIMENT=1', '-DBLE_RF_EXPERIMENT=0',
                     '-DBLE_RF_DISABLED=0', '-DENABLE_BLE_CONFIG=0'):
        assert expected in args
    assert '-DBLE_POWER_EXPERIMENT=0' in FirmwareSettings().cmake_args()

def test_writeonly_never_labels_power_as_verified():
    args=FirmwareSettings(ble_rf_disabled=5,ble_init=1).cmake_args()
    for flag in ('-DBLE_POWER_EXPERIMENT=1','-DBLE_POWER_READ_ONLY=0',
                 '-DBLE_POWER_WRITE_ONLY=1','-DENABLE_BLE_CONFIG=0'):
        assert flag in args
    for code,phase in ((12,17),(25,18),(15,19),(26,20),(17,21)):
        e=parse_rf_event(power_packet(code,7,phase))
        assert e is not None and '未读回验证' in e.summary
        row=dict(zip(RF_CSV_HEADER,event_row(e,1,100)))
        assert row['PowerVerified']==0 and row['Experiment']=='power_writeonly'
        assert row['TargetPower(dBm)']==2.5

def test_fixed_minus10_is_startup_only_and_excludes_sweeps():
    args=FirmwareSettings(ble_rf_disabled=6,ble_init=1).cmake_args()
    for flag in ('-DBLE_FIXED_MINUS10=1','-DENABLE_BLE_CONFIG=0',
                 '-DBLE_RF_DISABLED=0','-DBLE_RF_EXPERIMENT=0',
                 '-DBLE_POWER_EXPERIMENT=0','-DBLE_POWER_WRITE_ONLY=0','-DBLE_POWER_READ_ONLY=0'):
        assert flag in args
    assert '-DBLE_FIXED_MINUS10=0' in FirmwareSettings(ble_rf_disabled=5).cmake_args()
    source=(Path(__file__).resolve().parents[1]/'Core/Src/main.c').read_text(encoding='utf-8')
    fixed=source.split('#if BLE_FIXED_MINUS10',1)[1].split('PPG_SetChannel',1)[0]
    assert '<ST_TX_POWER=-10>' in fixed and 'ST_FACTORY' not in fixed and 'RD_TX_POWER' not in fixed
    assert fixed.index('BLE_RST_Pin,GPIO_PIN_SET') < fixed.index('HAL_Delay(100)')
    assert fixed.index('HAL_Delay(100)') < fixed.index('BLE_RST_Pin,GPIO_PIN_RESET')
    assert fixed.index('BLE_RST_Pin,GPIO_PIN_RESET') < fixed.index('HAL_Delay(300)')
    assert fixed.index('HAL_Delay(300)') < fixed.index('HAL_UART_Transmit(&huart2,(uint8_t *)wake')
    assert source.index('#if BLE_FIXED_MINUS10') < source.index('HAL_TIM_Base_Start_IT(&htim16)')

def test_two_recorders_do_not_overwrite_and_event_file_closes(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    panels = [RawDataPanel(), RawDataPanel()]
    for i, p in enumerate(panels):
        p._toggle_record(tmp_path / f"capture{i}.csv")
        p.handle_rf_event(parse_rf_event(power_packet()))
    with pytest.raises(FileExistsError):
        RawDataPanel()._toggle_record(tmp_path / "capture0.csv")
    handles = [p._rf_csv_file for p in panels]
    for p in panels: p._stop_recording(); p.close()
    assert all(f.closed for f in handles)
    files = list(tmp_path.glob('*_rf_events.csv'))
    assert len(files) == 2
    for file in files:
        with file.open(encoding='utf-8-sig') as stream:
            assert list(csv.DictReader(stream))[0]['TargetPower(dBm)'] == '-10'

def test_power_command_and_events_interleave_without_losing_raw(monkeypatch):
    import serial_reader
    from test_raw_transport_quality import make_raw_packet, make_status_packet
    reader = serial_reader.SerialReader('fake')
    stream = bytearray(make_raw_packet(50) + b'<ST_TX_POWER=-10>' + power_packet() + diagnostic_packet()
                       + b'<RD_TX_POWER>' + make_status_packet() + make_raw_packet(51))
    class FakeSerial:
        def write(self, data): return len(data)
        def flush(self): pass
        is_open = True
        @property
        def in_waiting(self): return min(2, len(stream))
        def read(self, n):
            if not stream: reader._running=False; return b''
            data=bytes(stream[:n]); del stream[:n]; return data
        def close(self): self.is_open=False
    monkeypatch.setattr(serial_reader.serial, 'Serial', lambda **kwargs: FakeSerial())
    raw=[]; events=[]
    reader.raw_packet_received.connect(raw.append)
    reader.rf_event_received.connect(events.append)
    reader.run()
    assert [p.sequence for p in raw] == [50, 51]
    assert len(events) == 2 and events[0].cycle == 6
    assert events[1].diagnostic['CommandTxDone']==1

def test_dual_capture_launcher_and_ui_configuration(monkeypatch):
    import firmware_panel
    app = QApplication.instance() or QApplication([])
    panel = firmware_panel.FirmwarePanel(lambda: True)
    panel.rf.setCurrentIndex(panel.rf.findData(8))
    assert not panel.ble.isEnabled() and panel.settings().ble_init == 0
    assert '-DBLE_SINGLE25=1' in panel.settings().cmake_args()
    calls = []
    def launch(program, args, cwd):
        calls.append(args)
        return (True, 123)
    monkeypatch.setattr(firmware_panel.QProcess, 'startDetached', launch)
    panel._open_dual_capture()
    assert [args[-1] for args in calls] == ['wired', 'wireless']
    panel.close()
