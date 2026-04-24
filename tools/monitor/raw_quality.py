from __future__ import annotations

from dataclasses import dataclass


UINT16_MODULO = 0x10000


@dataclass
class RawQualityStats:
    """Track device-sequence gaps for the fixed-rate Raw packet stream."""

    received_count: int = 0
    expected_count: int = 0
    missing_count: int = 0
    last_sequence: int | None = None

    def reset(self) -> None:
        self.received_count = 0
        self.expected_count = 0
        self.missing_count = 0
        self.last_sequence = None

    def observe(self, sequence: int) -> int:
        sequence &= 0xFFFF
        self.received_count += 1

        if self.last_sequence is None:
            self.last_sequence = sequence
            self.expected_count = 1
            return 0

        delta = (sequence - self.last_sequence) % UINT16_MODULO
        self.last_sequence = sequence

        if delta == 0:
            return 0

        missing_before = delta - 1
        self.expected_count += delta
        self.missing_count += missing_before
        return missing_before

    @property
    def loss_rate(self) -> float:
        if self.expected_count == 0:
            return 0.0
        return self.missing_count / self.expected_count
