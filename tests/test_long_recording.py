"""Long Raw recordings: sequence rollover, bounded display and elapsed clock."""
import csv
from dataclasses import replace

from PyQt5.QtWidgets import QApplication
import raw_data_panel
from raw_data_panel import RawDataPanel, PLOT_BUFFER, sample_index_to_elapsed_seconds
from raw_quality import RawQualityStats
from protocol import parse_raw_packet
from test_raw_transport_quality import make_raw_packet


def test_all_24_hour_sequences_survive_131_wraps():
    stats = RawQualityStats()
    count = 24 * 3600 * 100
    for i in range(count):
        stats.observe(i & 0xFFFF)
    assert stats.received_count == stats.expected_count == count
    assert stats.missing_count == stats.duplicate_count == stats.out_of_order_count == 0
    assert stats.last_sequence == (count - 1) & 0xFFFF
    assert sample_index_to_elapsed_seconds(count - 1) == 86399.99


def test_recording_past_24_hours_retains_raw_and_bounded_buffers(tmp_path):
    app = QApplication.instance() or QApplication([])
    panel = RawDataPanel()
    panel._toggle_record(tmp_path / "day.csv")
    base = parse_raw_packet(make_raw_packet(0))
    count = 24 * 3600 * 100
    panel._quality.expected_count = panel._quality.received_count = count
    panel._quality.last_sequence = (count - 1) & 0xFFFF
    try:
        for i in range(PLOT_BUFFER + 100):
            panel.handle_raw_data(replace(base, sequence=(count + i) & 0xFFFF, Ut1=1234.56789))
        assert len(panel._data_Ut1) == len(panel._data_ppg_g) == PLOT_BUFFER
        panel._stop_recording()
        with (tmp_path / "day.csv").open(encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == PLOT_BUFFER + 100
        assert rows[0]["Time(s)"] == "86400.0"
        assert int(rows[-1]["SampleIndex"]) == count + PLOT_BUFFER + 99
        assert all(float(row["Ut1(mV)"]) == 1234.56789 for row in rows)
    finally:
        panel._stop_recording()
        panel.close()


def test_recording_elapsed_ignores_wall_clock_adjustment(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    now = [100.0]
    monkeypatch.setattr(raw_data_panel.time, "monotonic", lambda: now[0])
    panel = RawDataPanel()
    panel._toggle_record(tmp_path / "clock.csv")
    try:
        monkeypatch.setattr(raw_data_panel.time, "time", lambda: -100000.0)
        now[0] += 86400.0
        panel.add_marker("24 h")
        panel._stop_recording()
        with (tmp_path / "clock_markers.csv").open(encoding="utf-8-sig") as stream:
            row = next(csv.DictReader(stream))
        assert float(row["Elapsed(s)"]) == 86400.0
    finally:
        panel._stop_recording()
        panel.close()
