"""Regression coverage for local recording/SNR and the remote workbench together."""
import csv
import json
import os
import sys
from datetime import date
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/monitor"))

from PyQt5.QtWidgets import QApplication
from recording_naming import RecordingMetadata, build_recording_paths, ensure_recording_available
from workbench import Workbench
from test_raw_transport_quality import make_raw_packet, make_status_packet
from test_rf_events import packet as rf_packet
from rf_events import parse_rf_event
import serial_reader


def test_recording_keeps_snr_raw_median_marker_and_rf_together(tmp_path):
    app = QApplication.instance() or QApplication([])
    win = Workbench()
    panel = win.monitor._raw_panel
    metadata = RecordingMetadata(tmp_path, "kaiji", 1, "TEST", date(2026, 9, 15))
    win.monitor._raw_recording_metadata = metadata
    paths = build_recording_paths(metadata)
    try:
        win.monitor._btn_record.click()
        assert panel.is_recording
        for _ in range(900):
            panel._sim_tick()
        win.monitor._btn_marker.click()
        panel.handle_rf_event(parse_rf_event(rf_packet()))
        panel._update_realtime_hr()
        panel._update_plots()
        assert "SNR --" not in panel._lbl_realtime_hr.text()
        assert "SNR" in panel._lbl_realtime_hr.text() and "dB" in panel._lbl_realtime_hr.text()
        np.testing.assert_allclose(panel._curve_ppg_g.getData()[1], np.array(panel._data_ppg_g)[-800:] / 16)

        # Display and language changes must not end recording or alter raw values.
        win.monitor._page_title.click()
        win.monitor._btn_lang.click()
        assert panel.is_recording and not panel._smooth_display
        assert "nA" in panel._plot_ppg_g._base_title
        assert win.monitor._btn_marker.text().endswith("(1)")
        win.close()

        with paths.raw_path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 900
        assert float(rows[-1]["PPG_Green"]) == panel._data_ppg_g[-1]
        assert rows[0]["Median3Valid"] == "0" and rows[2]["Median3Valid"] == "1"
        assert "Ut1_Median3(mV)" in rows[0]
        with paths.marker_path.open(encoding="utf-8-sig", newline="") as stream:
            marker = next(csv.DictReader(stream))
        assert marker["SampleIndex"] == "899"
        settings = json.loads(paths.raw_path.with_name(paths.raw_path.stem + "_processing.json").read_text())
        assert settings["display_toggle_affects_recording"] is False
        assert paths.raw_path.with_name(paths.raw_path.stem + "_rf_events.csv").exists()
        assert panel._csv_file is None and panel._rf_csv_file is None and panel._marker_csv_file is None
    finally:
        win.close()


@pytest.mark.parametrize("suffix", ["_processing.json", "_rf_events.csv"])
def test_new_companion_files_also_prevent_overwriting(tmp_path, suffix):
    paths = build_recording_paths(RecordingMetadata(tmp_path, "kaiji", 1, "TEST", date(2026, 9, 15)))
    paths.directory.mkdir(parents=True)
    paths.raw_path.with_name(paths.raw_path.stem + suffix).write_text("existing", encoding="utf-8")
    with pytest.raises(FileExistsError):
        ensure_recording_available(paths)


@pytest.mark.parametrize("wired", [False, True])
def test_prefix_payload_and_experiment_packets_coexist(monkeypatch, wired):
    app = QApplication.instance() or QApplication([])
    reader = serial_reader.SerialReader("fake", bind_mac=not wired)
    # Q4 payload includes both 'F' and '<', which must not start ASCII parsing.
    raw_bytes = make_raw_packet(70, green=0x463C0 / 16)
    prefix = b"" if wired else b"From    784128c58150: "
    stream = bytearray(prefix + raw_bytes + rf_packet() + make_status_packet())
    writes = []

    class FakeSerial:
        is_open = True

        @property
        def in_waiting(self):
            return min(3, len(stream))

        def read(self, size):
            if not stream:
                reader._running = False
                return b""
            chunk = bytes(stream[:size])
            del stream[:size]
            return chunk

        def write(self, data):
            writes.append(data)
            return len(data)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    monkeypatch.setattr(serial_reader.serial, "Serial", lambda **kwargs: FakeSerial())
    monkeypatch.setattr(serial_reader.time, "sleep", lambda _: None)
    raw, events, statuses = [], [], []
    reader.raw_packet_received.connect(raw.append)
    reader.rf_event_received.connect(events.append)
    reader.status_packet_received.connect(statuses.append)
    reader.run()
    assert len(raw) == len(events) == len(statuses) == 1
    assert raw[0].ppg_green == 0x463C0 / 16
    assert raw[0].sequence == 70 and reader.get_raw_stats() == (1, 0)
    if wired:
        assert writes == []
    else:
        assert writes == [b"<ST_CENTER_LINK=ALL>", b"<ST_CON_MAC=784128c58150>"]
