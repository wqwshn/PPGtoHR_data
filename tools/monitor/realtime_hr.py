"""原始数据面板使用的轻量实时绿光 PPG 心率估计。"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np


DEFAULT_SAMPLE_RATE_HZ = 100.0
DEFAULT_WINDOW_SECONDS = 8.0
DEFAULT_MIN_HZ = 1.0
DEFAULT_MAX_HZ = 4.0
DEFAULT_FFT_LEN = 8192
MIN_AC_AMPLITUDE = 100.0


@dataclass(frozen=True)
class RealtimeHrEstimate:
    bpm: float | None
    ready: bool
    status: str
    window_seconds: float
    elapsed_ms: float | None = None


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
    if float(np.ptp(values)) < MIN_AC_AMPLITUDE:
        return RealtimeHrEstimate(None, False, "weak", window_seconds)

    windowed = values * np.hamming(values.size)
    spectrum = np.abs(np.fft.rfft(windowed, n=fft_len)) / values.size
    freqs = np.fft.rfftfreq(fft_len, d=1.0 / sample_rate_hz)
    mask = (freqs >= min_hz) & (freqs <= max_hz)
    if not np.any(mask):
        return RealtimeHrEstimate(None, False, "weak", window_seconds)

    band = spectrum[mask]
    if band.size == 0 or float(np.max(band)) <= 0.0:
        return RealtimeHrEstimate(None, False, "weak", window_seconds)

    bpm = float(freqs[mask][int(np.argmax(band))] * 60.0)
    return RealtimeHrEstimate(round(bpm, 1), True, "ok", window_seconds)


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
    )
