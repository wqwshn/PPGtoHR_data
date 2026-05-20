from __future__ import annotations

from dataclasses import dataclass, field

from protocol import StatusPacket


UINT16_MODULO = 0x10000
UINT16_HALF_RANGE = UINT16_MODULO // 2


@dataclass
class SequenceObservation:
    """Result of classifying one Raw sequence number."""

    sequence: int = 0
    missing_before: int = 0
    accepted: bool = False
    reason: str = ""


@dataclass
class DiagnosticSnapshot:
    """PC-side interpretation of the latest MCU STATUS counters."""

    pc_missing_after_tx_done: int = 0
    tx_inflight: int = 0
    pc_received_raw: int = 0
    pc_expected_raw: int = 0
    pc_missing_raw: int = 0
    pc_raw_total_candidates: int = 0
    pc_raw_invalid_candidates: int = 0
    pc_raw_invalid_delta: int = 0


@dataclass
class RawQualityStats:
    """Track device-sequence gaps for the fixed-rate Raw packet stream."""

    received_count: int = 0
    expected_count: int = 0
    missing_count: int = 0
    last_sequence: int | None = None
    duplicate_count: int = 0
    out_of_order_count: int = 0
    latest_sequence_observation: SequenceObservation = field(
        default_factory=SequenceObservation
    )
    latest_status: StatusPacket | None = None
    latest_diagnostic: DiagnosticSnapshot = field(default_factory=DiagnosticSnapshot)
    _status_baseline_tx_done: int | None = None
    _status_baseline_received: int | None = None
    _pc_raw_total_candidates: int = 0
    _pc_raw_invalid_candidates: int = 0
    _last_snapshot_raw_invalid: int = 0

    def reset(self) -> None:
        self.received_count = 0
        self.expected_count = 0
        self.missing_count = 0
        self.last_sequence = None
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.latest_sequence_observation = SequenceObservation()
        self.latest_status = None
        self.latest_diagnostic = DiagnosticSnapshot()
        self._status_baseline_tx_done = None
        self._status_baseline_received = None
        self._pc_raw_total_candidates = 0
        self._pc_raw_invalid_candidates = 0
        self._last_snapshot_raw_invalid = 0

    def observe(self, sequence: int) -> int:
        sequence &= 0xFFFF
        self.received_count += 1

        if self.last_sequence is None:
            self.last_sequence = sequence
            self.expected_count = 1
            self.latest_sequence_observation = SequenceObservation(
                sequence=sequence,
                accepted=True,
            )
            return 0

        delta = (sequence - self.last_sequence) % UINT16_MODULO

        if delta == 0:
            self.duplicate_count += 1
            self.latest_sequence_observation = SequenceObservation(
                sequence=sequence,
                accepted=False,
                reason="duplicate",
            )
            return 0

        if delta > UINT16_HALF_RANGE:
            self.out_of_order_count += 1
            self.latest_sequence_observation = SequenceObservation(
                sequence=sequence,
                accepted=False,
                reason="out_of_order",
            )
            return 0

        missing_before = delta - 1
        self.last_sequence = sequence
        self.expected_count += delta
        self.missing_count += missing_before
        self.latest_sequence_observation = SequenceObservation(
            sequence=sequence,
            missing_before=missing_before,
            accepted=True,
        )
        return missing_before

    def observe_parser_stats(self, raw_total: int, raw_invalid: int) -> None:
        self._pc_raw_total_candidates = max(int(raw_total), 0)
        self._pc_raw_invalid_candidates = max(int(raw_invalid), 0)

    def observe_status(self, status: StatusPacket) -> DiagnosticSnapshot:
        self.latest_status = status
        if self._status_baseline_tx_done is None:
            self._status_baseline_tx_done = status.tx_done_counter
            self._status_baseline_received = self.received_count

        tx_done_delta = status.tx_done_counter - self._status_baseline_tx_done
        pc_received_delta = self.received_count - (self._status_baseline_received or 0)
        pc_missing_after_tx_done = max(tx_done_delta - pc_received_delta, 0)
        tx_inflight = max(status.tx_start_counter - status.tx_done_counter, 0)
        pc_raw_invalid_delta = max(
            self._pc_raw_invalid_candidates - self._last_snapshot_raw_invalid,
            0,
        )
        self._last_snapshot_raw_invalid = self._pc_raw_invalid_candidates
        self.latest_diagnostic = DiagnosticSnapshot(
            pc_missing_after_tx_done=pc_missing_after_tx_done,
            tx_inflight=tx_inflight,
            pc_received_raw=self.received_count,
            pc_expected_raw=self.expected_count,
            pc_missing_raw=self.missing_count,
            pc_raw_total_candidates=self._pc_raw_total_candidates,
            pc_raw_invalid_candidates=self._pc_raw_invalid_candidates,
            pc_raw_invalid_delta=pc_raw_invalid_delta,
        )
        return self.latest_diagnostic

    @property
    def loss_rate(self) -> float:
        if self.expected_count == 0:
            return 0.0
        return self.missing_count / self.expected_count
