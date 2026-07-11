"""原始数据面板使用的轻量实时绿光 PPG 心率估计。"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np


DEFAULT_SAMPLE_RATE_HZ = 100.0
DEFAULT_WINDOW_SECONDS = 8.0
DEFAULT_MIN_HZ = 0.7
DEFAULT_MAX_HZ = 4.0
DEFAULT_FFT_LEN = 8192
MIN_AC_AMPLITUDE = 50.0
SNR_PEAK_GUARD_BINS = 2


@dataclass(frozen=True)
class RealtimeHrEstimate:
    bpm: float | None
    ready: bool
    status: str
    window_seconds: float
    elapsed_ms: float | None = None
    snr_db: float | None = None


def _spectral_snr_db(power: np.ndarray, peak_index: int) -> float | None:
    """Return the peak-to-median-noise-floor ratio in dB."""
    low = max(0, peak_index - SNR_PEAK_GUARD_BINS)
    high = min(power.size, peak_index + SNR_PEAK_GUARD_BINS + 1)
    residual = np.concatenate((power[:low], power[high:]))
    if residual.size == 0:
        return None
    noise_floor = float(np.median(residual))
    peak_power = float(power[peak_index])
    if peak_power <= 0.0 or noise_floor <= 0.0:
        return None
    return round(10.0 * float(np.log10(peak_power / noise_floor)), 1)


def estimate_green_fft_hr(
    samples: Sequence[float],
    sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
    min_hz: float = DEFAULT_MIN_HZ,
    max_hz: float = DEFAULT_MAX_HZ,
    fft_len: int = DEFAULT_FFT_LEN,
) -> RealtimeHrEstimate:
    """使用最近一段绿光 PPG 窗口做静息纯 FFT 心率估计。"""
    needed = int(round(sample_rate_hz * window_seconds))
    if len(samples) < needed:
        return RealtimeHrEstimate(None, False, "filling", window_seconds)

    values = np.asarray(samples[-needed:], dtype=float)
    values = values - float(np.mean(values))
    amplitude_is_low = float(np.ptp(values)) < MIN_AC_AMPLITUDE

    windowed = values * np.hamming(values.size)
    spectrum = np.abs(np.fft.rfft(windowed, n=fft_len)) / values.size
    freqs = np.fft.rfftfreq(fft_len, d=1.0 / sample_rate_hz)
    mask = (freqs >= min_hz) & (freqs <= max_hz)
    if not np.any(mask):
        return RealtimeHrEstimate(None, False, "weak", window_seconds)

    band = spectrum[mask]
    power = band ** 2
    if power.size == 0 or float(np.max(power)) <= 0.0:
        return RealtimeHrEstimate(None, False, "weak", window_seconds)

    peak_index = int(np.argmax(power))
    snr_db = _spectral_snr_db(power, peak_index)
    if amplitude_is_low:
        return RealtimeHrEstimate(None, False, "weak", window_seconds, snr_db=snr_db)

    bpm = float(freqs[mask][peak_index] * 60.0)
    return RealtimeHrEstimate(round(bpm, 1), True, "ok", window_seconds, snr_db=snr_db)


def timed_estimate_green_fft_hr(
    samples: Sequence[float],
    sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
) -> RealtimeHrEstimate:
    start = time.perf_counter()
    estimate = estimate_green_fft_hr(samples, sample_rate_hz, window_seconds)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return RealtimeHrEstimate(
        bpm=estimate.bpm,
        ready=estimate.ready,
        status=estimate.status,
        window_seconds=estimate.window_seconds,
        elapsed_ms=round(elapsed_ms, 3),
        snr_db=estimate.snr_db,
    )
