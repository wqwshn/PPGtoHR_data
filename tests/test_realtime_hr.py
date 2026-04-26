import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

from realtime_hr import estimate_green_fft_hr, timed_estimate_green_fft_hr


def _sine_ppg(bpm: float, seconds: float = 8.0, sample_rate_hz: float = 100.0) -> list[float]:
    freq_hz = bpm / 60.0
    count = int(seconds * sample_rate_hz)
    return [
        50000.0
        + 7000.0 * math.sin(2.0 * math.pi * freq_hz * idx / sample_rate_hz)
        + 400.0 * math.sin(2.0 * math.pi * 0.2 * idx / sample_rate_hz)
        for idx in range(count)
    ]


def test_estimate_green_fft_hr_returns_resting_bpm_from_clean_green_ppg():
    estimate = estimate_green_fft_hr(_sine_ppg(72.0))

    assert estimate.ready is True
    assert estimate.bpm is not None
    assert abs(estimate.bpm - 72.0) < 1.0
    assert estimate.window_seconds == 8.0


def test_estimate_green_fft_hr_waits_for_full_window():
    estimate = estimate_green_fft_hr(_sine_ppg(72.0, seconds=4.0))

    assert estimate.ready is False
    assert estimate.bpm is None
    assert estimate.status == "filling"


def test_estimate_green_fft_hr_rejects_flat_signal():
    estimate = estimate_green_fft_hr([50000.0] * 800)

    assert estimate.ready is False
    assert estimate.bpm is None
    assert estimate.status == "weak"


def test_timed_estimate_green_fft_hr_reports_small_runtime():
    estimate = timed_estimate_green_fft_hr(_sine_ppg(72.0))

    assert estimate.ready is True
    assert estimate.elapsed_ms is not None
    assert estimate.elapsed_ms < 10.0
