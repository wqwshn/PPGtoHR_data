"""PPG Raw CSV quality diagnostics.

This module is intentionally independent from PyQt so it can be used after a
recording session to quantify whether PPG precision and continuity improved.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PPG_CHANNELS = ("PPG_Green", "PPG_Red", "PPG_IR")
UINT16_MODULO = 65536


@dataclass(frozen=True)
class ChannelQuality:
    name: str
    rows: int
    fractional_rows: int
    zero_diff_count: int
    zero_diff_ratio: float
    max_run: int
    unique_values: int


@dataclass(frozen=True)
class PpgQualityReport:
    path: Path
    total_rows: int
    valid_rows: int
    sequence_missing_count: int
    sequence_gap_count: int
    triple_zero_diff_count: int
    triple_zero_diff_ratio: float
    triple_max_run: int
    channels: dict[str, ChannelQuality]


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    last_error: UnicodeDecodeError | None = None
    for encoding in ("utf-8-sig", "gbk", "latin1"):
        try:
            with path.open("r", encoding=encoding, newline="") as f:
                return list(csv.DictReader(f))
        except UnicodeDecodeError as exc:
            last_error = exc
    if last_error is not None:
        raise last_error
    return []


def _is_valid_row(row: dict[str, str]) -> bool:
    return row.get("ValidFlag", "1") != "0"


def _parse_float(text: str | None) -> float | None:
    if text is None:
        return None
    stripped = text.strip()
    if not stripped or stripped.lower() == "nan":
        return None
    return float(stripped)


def _has_fraction(value: float) -> bool:
    return abs(value - round(value)) > 1e-9


def _run_lengths(values: Iterable[object]) -> list[int]:
    iterator = iter(values)
    try:
        current = next(iterator)
    except StopIteration:
        return []

    lengths: list[int] = []
    length = 1
    for value in iterator:
        if value == current:
            length += 1
            continue
        lengths.append(length)
        current = value
        length = 1
    lengths.append(length)
    return lengths


def _channel_quality(name: str, values: list[float]) -> ChannelQuality:
    if not values:
        return ChannelQuality(
            name=name,
            rows=0,
            fractional_rows=0,
            zero_diff_count=0,
            zero_diff_ratio=0.0,
            max_run=0,
            unique_values=0,
        )

    zero_diff_count = sum(1 for left, right in zip(values, values[1:]) if left == right)
    transition_count = max(len(values) - 1, 1)
    runs = _run_lengths(values)
    return ChannelQuality(
        name=name,
        rows=len(values),
        fractional_rows=sum(1 for value in values if _has_fraction(value)),
        zero_diff_count=zero_diff_count,
        zero_diff_ratio=zero_diff_count / transition_count,
        max_run=max(runs) if runs else 0,
        unique_values=len(set(values)),
    )


def _sequence_quality(rows: list[dict[str, str]]) -> tuple[int, int]:
    sequences: list[int] = []
    for row in rows:
        text = row.get("Seq")
        if text is None or not text.strip():
            continue
        sequences.append(int(text))

    missing_count = 0
    gap_count = 0
    for left, right in zip(sequences, sequences[1:]):
        delta = (right - left) % UINT16_MODULO
        if delta > 1:
            missing_count += delta - 1
            gap_count += 1
    return missing_count, gap_count


def analyze_raw_csv(path: str | Path) -> PpgQualityReport:
    csv_path = Path(path)
    rows = _read_csv_rows(csv_path)
    valid_rows = [row for row in rows if _is_valid_row(row)]

    channel_values: dict[str, list[float]] = {}
    for channel in PPG_CHANNELS:
        values: list[float] = []
        for row in valid_rows:
            value = _parse_float(row.get(channel))
            if value is not None:
                values.append(value)
        channel_values[channel] = values

    triples = list(zip(*(channel_values[channel] for channel in PPG_CHANNELS)))
    triple_zero_diff_count = sum(
        1 for left, right in zip(triples, triples[1:]) if left == right
    )
    triple_transition_count = max(len(triples) - 1, 1)
    triple_runs = _run_lengths(triples)
    sequence_missing_count, sequence_gap_count = _sequence_quality(valid_rows)

    return PpgQualityReport(
        path=csv_path,
        total_rows=len(rows),
        valid_rows=len(valid_rows),
        sequence_missing_count=sequence_missing_count,
        sequence_gap_count=sequence_gap_count,
        triple_zero_diff_count=triple_zero_diff_count,
        triple_zero_diff_ratio=triple_zero_diff_count / triple_transition_count,
        triple_max_run=max(triple_runs) if triple_runs else 0,
        channels={
            channel: _channel_quality(channel, channel_values[channel])
            for channel in PPG_CHANNELS
        },
    )


def _pct(value: float) -> str:
    return f"{value * 100:.1f}%"


def format_text_report(report: PpgQualityReport) -> str:
    lines = [
        f"path={report.path}",
        f"valid_rows={report.valid_rows}/{report.total_rows}",
        f"sequence_missing_count={report.sequence_missing_count}",
        f"sequence_gap_count={report.sequence_gap_count}",
        f"triple_zero_diff_ratio={_pct(report.triple_zero_diff_ratio)}",
        f"triple_zero_diff_count={report.triple_zero_diff_count}",
        f"triple_max_run={report.triple_max_run}",
    ]
    for channel in PPG_CHANNELS:
        quality = report.channels[channel]
        lines.append(
            f"{channel}: fractional_rows={quality.fractional_rows}/{quality.rows}, "
            f"zero_diff_ratio={_pct(quality.zero_diff_ratio)}, "
            f"zero_diff_count={quality.zero_diff_count}, "
            f"max_run={quality.max_run}, unique_values={quality.unique_values}"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze PPG Raw CSV quality")
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args(argv)

    print(format_text_report(analyze_raw_csv(args.csv_path)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
