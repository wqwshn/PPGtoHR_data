import sys
import csv
from types import SimpleNamespace
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

import raw_data_panel
import serial_reader
import protocol


class FakeSerial:
    def __init__(self, in_waiting):
        self.in_waiting = in_waiting
        self.read_sizes = []

    def read(self, size):
        self.read_sizes.append(size)
        return b"x" * size


def test_serial_reader_does_not_wait_for_a_full_4096_byte_buffer():
    fake = FakeSerial(in_waiting=4096)

    chunk = serial_reader.read_serial_chunk(fake)

    assert len(chunk) == serial_reader.SERIAL_READ_CHUNK_BYTES
    assert fake.read_sizes == [serial_reader.SERIAL_READ_CHUNK_BYTES]
    assert serial_reader.SERIAL_READ_CHUNK_BYTES == max(
        serial_reader.RAW_PACKET_LEN,
        serial_reader.STATUS_PACKET_LEN,
    ) * 4


def test_serial_reader_blocks_for_only_one_byte_when_no_data_is_buffered():
    fake = FakeSerial(in_waiting=0)

    chunk = serial_reader.read_serial_chunk(fake)

    assert len(chunk) == 1
    assert fake.read_sizes == [1]


def test_raw_recording_time_uses_sample_index_not_ui_wall_clock():
    assert raw_data_panel.sample_index_to_elapsed_seconds(0) == 0.0
    assert raw_data_panel.sample_index_to_elapsed_seconds(1) == 0.01
    assert raw_data_panel.sample_index_to_elapsed_seconds(23999) == 239.99


def test_raw_csv_exports_sequence_and_missing_count():
    pkt = SimpleNamespace(
        sequence=42,
        Uc1=1.0,
        Uc2=2.0,
        Ut1=3.0,
        Ut2=4.0,
        acc_x=0.1,
        acc_y=0.2,
        acc_z=0.3,
        gyro_x=1.1,
        gyro_y=1.2,
        gyro_z=1.3,
        ppg_green=100,
        ppg_red=200,
        ppg_ir=300,
    )

    assert raw_data_panel.RAW_CSV_HEADER[:7] == [
        "Time(s)",
        "SampleIndex",
        "Seq",
        "ValidFlag",
        "InterpFlag",
        "GapLen",
        "MissingBefore",
    ]
    rows = raw_data_panel.timeline_packet_to_csv_rows(pkt, sample_index=5, missing_before=2)
    assert rows[-1][:7] == [0.05, 5, 42, 1, 0, 0, 2]


def test_timeline_csv_expands_missing_samples_as_nan_rows():
    pkt = SimpleNamespace(
        sequence=42,
        Uc1=1.0,
        Uc2=2.0,
        Ut1=3.0,
        Ut2=4.0,
        acc_x=0.1,
        acc_y=0.2,
        acc_z=0.3,
        gyro_x=1.1,
        gyro_y=1.2,
        gyro_z=1.3,
        ppg_green=100,
        ppg_red=200,
        ppg_ir=300,
    )

    rows = raw_data_panel.timeline_packet_to_csv_rows(
        pkt,
        sample_index=5,
        missing_before=2,
    )

    assert raw_data_panel.TIMELINE_CSV_HEADER[:7] == [
        "Time(s)",
        "SampleIndex",
        "Seq",
        "ValidFlag",
        "InterpFlag",
        "GapLen",
        "MissingBefore",
    ]
    assert rows[0][:7] == [0.03, 3, 40, 0, 0, 2, ""]
    assert rows[1][:7] == [0.04, 4, 41, 0, 0, 2, ""]
    assert rows[2][:7] == [0.05, 5, 42, 1, 0, 0, 2]
    assert rows[0][7:] == ["NaN"] * 13
    assert rows[2][7:] == [
        1.0,
        2.0,
        3.0,
        4.0,
        0.1,
        0.2,
        0.3,
        1.1,
        1.2,
        1.3,
        100,
        200,
        300,
    ]


def test_raw_recording_outputs_timeline_status_and_marker_csv():
    raw_path, status_path, marker_path = raw_data_panel.recording_output_paths(
        Path("multi_tiaosheng1.csv")
    )

    assert raw_path == Path("multi_tiaosheng1.csv")
    assert status_path == Path("multi_tiaosheng1_status.csv")
    assert marker_path == Path("multi_tiaosheng1_markers.csv")


def _raw_panel():
    return SimpleNamespace(
        _is_recording=False,
        _csv_file=None,
        _csv_writer=None,
        _status_csv_file=None,
        _status_csv_writer=None,
        _marker_csv_file=None,
        _marker_csv_writer=None,
        _marker_count=0,
        _rf_last=None,
        _rf_csv_file=None,
        _rf_csv_writer=None,
        _recording_start_time=None,
        _recorded_sample_count=0,
        _flush_counter=0,
        _quality=SimpleNamespace(expected_count=0),
        _reset_quality_stats=lambda: None,
    )


def test_raw_recording_defers_marker_file_until_first_marker(tmp_path):
    panel = _raw_panel()
    raw_path = tmp_path / "kaiji1_LYX_0710.csv"

    assert raw_data_panel.RawDataPanel._toggle_record(panel, raw_path) is True
    assert list(tmp_path.glob("*_markers.csv")) == []

    raw_data_panel.RawDataPanel.add_marker(panel)
    raw_data_panel.RawDataPanel.add_marker(panel)
    raw_data_panel.RawDataPanel._stop_recording(panel)

    marker_path = next(tmp_path.glob("*_markers.csv"))
    with marker_path.open(newline="", encoding="utf-8-sig") as marker_file:
        rows = list(csv.reader(marker_file))
    assert rows[0] == ["Elapsed(s)", "SampleIndex", "MarkerNo", "Note"]
    assert [row[2] for row in rows[1:]] == ["1", "2"]


def test_raw_recording_leaves_no_marker_file_without_marker(tmp_path):
    panel = _raw_panel()
    raw_path = tmp_path / "kaiji1_LYX_0710.csv"

    assert raw_data_panel.RawDataPanel._toggle_record(panel, raw_path) is True
    raw_data_panel.RawDataPanel._stop_recording(panel)

    assert not raw_data_panel.recording_output_paths(raw_path)[2].exists()


def test_raw_recording_requires_a_prevalidated_raw_path(tmp_path):
    panel = _raw_panel()

    with pytest.raises(TypeError):
        raw_data_panel.RawDataPanel._toggle_record(panel, save_dir=tmp_path)


def test_raw_recording_uses_prevalidated_metadata_path(tmp_path):
    panel = _raw_panel()
    raw_path = tmp_path / "202607-multiperson" / "0710-LYX" / "kaiji1_LYX_0710.csv"

    assert raw_data_panel.RawDataPanel._toggle_record(panel, raw_path=raw_path) is True
    raw, status, marker = raw_data_panel.recording_output_paths(raw_path)
    assert raw.exists()
    assert status.exists()
    assert not marker.exists()

    raw_data_panel.RawDataPanel._stop_recording(panel)


def test_status_summary_exposes_diagnostic_counters():
    status = protocol.StatusPacket(
        protocol_version=1,
        mcu_time_ms=1234,
        sample_counter=100,
        adc_drdy_counter=400,
        frame_counter=99,
        tx_start_counter=98,
        tx_done_counter=97,
        tx_busy_counter=7,
        tx_error_counter=2,
        adc_error_counter=3,
        imu_error_counter=4,
        ppg_fifo_empty_counter=5,
        ppg_fifo_overflow_counter=6,
        ppg_fifo_sample_total_counter=250,
        ppg_fifo_nonempty_counter=90,
        ppg_fifo_single_sample_counter=10,
        ppg_fifo_multi_sample_counter=80,
    )
    snapshot = raw_data_panel.RawQualityStats().observe_status(status)

    text = raw_data_panel.status_packet_to_summary(status, snapshot)

    assert "Busy 7" in text
    assert "Err 2" in text
    assert "PCGap 0" in text
    assert "FIFO 5/6" in text
    assert "PPGAvg 2.78" in text


def test_status_summary_uses_chinese_labels_in_chinese_mode():
    status = protocol.StatusPacket(
        protocol_version=1,
        mcu_time_ms=1234,
        sample_counter=100,
        adc_drdy_counter=400,
        frame_counter=99,
        tx_start_counter=98,
        tx_done_counter=97,
        tx_busy_counter=7,
        tx_error_counter=2,
        adc_error_counter=3,
        imu_error_counter=4,
        ppg_fifo_empty_counter=5,
        ppg_fifo_overflow_counter=6,
        ppg_fifo_sample_total_counter=250,
        ppg_fifo_nonempty_counter=90,
        ppg_fifo_single_sample_counter=10,
        ppg_fifo_multi_sample_counter=80,
    )
    snapshot = raw_data_panel.RawQualityStats().observe_status(status)

    text = raw_data_panel.status_packet_to_summary(status, snapshot, lang="zh")

    assert "发送忙 7" in text
    assert "发送错误 2" in text
    assert "发送后缺口 0" in text
    assert "FIFO空/溢出 5/6" in text
    assert "PPG均值 2.78" in text
    assert "Busy" not in text


def test_status_csv_row_exports_diagnostic_snapshot():
    status = protocol.StatusPacket(
        protocol_version=1,
        mcu_time_ms=1234,
        sample_counter=100,
        adc_drdy_counter=400,
        frame_counter=99,
        tx_start_counter=98,
        tx_done_counter=97,
        tx_busy_counter=7,
        tx_error_counter=2,
        adc_error_counter=3,
        imu_error_counter=4,
        ppg_fifo_empty_counter=5,
        ppg_fifo_overflow_counter=6,
        ppg_fifo_sample_total_counter=250,
        ppg_fifo_nonempty_counter=90,
        ppg_fifo_single_sample_counter=10,
        ppg_fifo_multi_sample_counter=80,
    )
    stats = raw_data_panel.RawQualityStats()
    stats.observe_parser_stats(raw_total=120, raw_invalid=3)
    snapshot = stats.observe_status(status)

    assert raw_data_panel.STATUS_CSV_HEADER[:4] == [
        "RxTime(s)",
        "McuTime(ms)",
        "SampleCounter",
        "FrameCounter",
    ]
    assert raw_data_panel.status_packet_to_csv_row(status, snapshot, 1.25)[:4] == [
        1.25,
        1234,
        100,
        99,
    ]
    assert raw_data_panel.STATUS_CSV_HEADER[-3:] == [
        "PcRawTotalCandidates",
        "PcRawInvalidCandidates",
        "PcRawInvalidDelta",
    ]
    assert raw_data_panel.status_packet_to_csv_row(status, snapshot, 1.25)[-3:] == [
        120,
        3,
        3,
    ]
    assert "PpgFifoSampleTotalCounter" in raw_data_panel.STATUS_CSV_HEADER
    row = raw_data_panel.status_packet_to_csv_row(status, snapshot, 1.25)
    total_idx = raw_data_panel.STATUS_CSV_HEADER.index("PpgFifoSampleTotalCounter")
    multi_idx = raw_data_panel.STATUS_CSV_HEADER.index("PpgFifoMultiSampleCounter")
    assert row[total_idx] == 250
    assert row[multi_idx] == 80


def test_serial_reader_exposes_raw_parse_stats_signal():
    assert hasattr(serial_reader.SerialReader, "raw_parse_stats_received")
