"""MIMU snapshots work without boot text, including legacy and lost links."""
import os
import sys
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

import pytest
from PyQt5.QtWidgets import QApplication

import protocol
import raw_data_panel
import serial_reader
from test_raw_transport_quality import make_raw_packet, make_status_packet


@pytest.fixture
def panel():
    app = QApplication.instance() or QApplication([])
    widget = raw_data_panel.RawDataPanel()
    yield widget
    widget.close()


@pytest.mark.parametrize("state,label", [(0, "初始化中"), (1, "启动成功"), (2, "仍在重试"),
                                          (3, "后台恢复成功"), (4, "自动重试关闭")])
def test_late_receiver_shows_explicit_firmware_state_without_boot_logs(panel, state, label):
    panel.handle_connection_changed(True)
    assert "等待设备状态" in panel._lbl_mimu.text()
    status = protocol.parse_status_packet(make_status_packet(mimu_state=state, imu_error_counter=8))
    panel.handle_status_data(status)
    assert label in panel._lbl_mimu.text()
    assert "8 次" in panel._lbl_mimu.text()
    panel._apply_language("en")
    assert "Init failures 8" in panel._lbl_mimu.text()
    panel._apply_language("zh")
    assert label in panel._lbl_mimu.text()


def test_detailed_debug_text_never_overwrites_calibration_progress(panel):
    panel.handle_calib_status("GYRO CALIB: collecting...")
    for text in ("DEBUG: System Start Loop...", "[DEBUG-acq] reg=FF FAIL", "[PPG] config VERIFIED",
                 "[SYSTEM] TIM16 start=0", "DBG: before ADC_Init"):
        panel.handle_calib_status(text)
        assert panel._lbl_calib.text() == "GYRO CALIB: collecting..."
    panel.handle_calib_status("GYRO CALIB: OK")
    assert panel._lbl_calib.text() == "GYRO CALIB: OK"


@pytest.mark.parametrize("version", [1, 2])
def test_legacy_firmware_never_claims_success_or_recovery(panel, version):
    status = protocol.parse_status_packet(make_status_packet(protocol_version=version, imu_error_counter=5))
    assert status is not None and status.mimu_state is None
    panel.handle_status_data(status)
    assert "旧固件未提供状态" in panel._lbl_mimu.text()
    assert "成功" not in panel._lbl_mimu.text()


def test_lost_status_disconnect_reconnect_clear_and_simulation_do_not_keep_live_success(panel, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(raw_data_panel.time, "monotonic", lambda: clock[0])
    status = protocol.parse_status_packet(make_status_packet(mimu_state=3))
    panel.handle_status_data(status)
    clock[0] = 104.0
    panel._update_sample_rate()
    assert "状态已过期" in panel._lbl_mimu.text()
    panel.handle_connection_changed(False)
    assert panel._lbl_mimu.text() == "MIMU：未连接"
    panel.handle_connection_changed(True)
    assert "等待设备状态" in panel._lbl_mimu.text()
    panel.handle_status_data(status)
    assert "后台恢复成功" in panel._lbl_mimu.text() and "过期" not in panel._lbl_mimu.text()
    panel._clear_screen()
    assert "等待设备状态" in panel._lbl_mimu.text()
    panel.start_simulation()
    assert "模拟数据" in panel._lbl_mimu.text()
    panel.stop_simulation()
    assert "成功" not in panel._lbl_mimu.text()


def test_status_csv_appends_explicit_state_and_leaves_legacy_empty():
    status = protocol.parse_status_packet(make_status_packet(mimu_state=3))
    snapshot = raw_data_panel.RawQualityStats().observe_status(status)
    row = raw_data_panel.status_packet_to_csv_row(status, snapshot, 1.5)
    assert len(row) == len(raw_data_panel.STATUS_CSV_HEADER)
    assert raw_data_panel.STATUS_CSV_HEADER[-1] == "MimuState" and row[-1] == 3
    legacy_row = raw_data_panel.status_packet_to_csv_row(replace(status, mimu_state=None), snapshot, 1.5)
    assert row[:-1] == legacy_row[:-1] and legacy_row[-1] == ""


@pytest.mark.parametrize("chunk_size", [1, 3, 73])
def test_serial_reader_routes_fragmented_legacy_current_and_corrupt_status(monkeypatch, chunk_size):
    app = QApplication.instance() or QApplication([])
    reader = serial_reader.SerialReader("fake", bind_mac=False)
    corrupt = bytearray(make_status_packet(mimu_state=3))
    corrupt[67] ^= 1
    stream = bytearray(b"From    784128c58150: " + make_status_packet(protocol_version=2)
                       + make_raw_packet(40) + make_status_packet(mimu_state=3)
                       + corrupt + b"\xaa\xdd\x7f" + make_raw_packet(41))

    class FakeSerial:
        is_open = True

        @property
        def in_waiting(self):
            return min(chunk_size, len(stream))

        def read(self, size):
            if not stream:
                reader._running = False
                return b""
            data = bytes(stream[:size])
            del stream[:size]
            return data

        def close(self):
            self.is_open = False

    monkeypatch.setattr(serial_reader.serial, "Serial", lambda **kwargs: FakeSerial())
    monkeypatch.setattr(serial_reader.time, "sleep", lambda _: None)
    statuses, packets = [], []
    reader.status_packet_received.connect(statuses.append)
    reader.raw_packet_received.connect(packets.append)
    reader.run()
    assert [item.mimu_state for item in statuses] == [None, 3]
    assert [item.sequence for item in packets] == [40, 41]


def test_status_version_and_length_must_agree():
    data = bytearray(make_status_packet(protocol_version=2))
    data[2] = 3
    assert protocol.parse_status_packet(bytes(data)) is None
    data = bytearray(make_status_packet())
    data[2] = 2
    assert protocol.parse_status_packet(bytes(data)) is None
