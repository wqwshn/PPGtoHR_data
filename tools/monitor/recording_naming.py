"""Raw recording metadata validation and algorithm-compatible file naming."""
from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import date
from pathlib import Path


SCENARIO_OPTIONS: tuple[tuple[str, str], ...] = (
    ("波比跳", "bobi"),
    ("敲键盘", "jianpan"),
    ("开合跳", "kaihe"),
    ("拳击", "quanji"),
    ("跑步", "run"),
    ("跳绳", "tiaosheng"),
    ("握力", "woli"),
    ("写字", "xiezi"),
    ("俯卧撑", "fuwo"),
    ("弯举", "wanju"),
    ("静息", "rest"),
    ("仰卧起坐", "yangwo"),
    ("快速出拳", "quanji"),
    ("高抬腿", "gaotai"),
    ("开机", "kaiji"),
)
_SCENARIO_TOKENS = frozenset(token for _, token in SCENARIO_OPTIONS)
_SUBJECT_RE = re.compile(r"^[A-Z]{1,8}$")


@dataclass(frozen=True)
class RecordingMetadata:
    """The user-selected metadata for one Raw recording."""

    save_root: Path
    scenario_token: str
    trial: int
    subject: str
    record_date: date


@dataclass(frozen=True)
class RecordingPaths:
    """All paths derived from one recording metadata value."""

    directory: Path
    raw_path: Path
    status_path: Path
    marker_path: Path


def validate_metadata(metadata: RecordingMetadata) -> None:
    """Raise ValueError when metadata cannot produce a safe record name."""
    if not metadata.save_root:
        raise ValueError("保存根目录不能为空")
    if metadata.scenario_token not in _SCENARIO_TOKENS:
        raise ValueError("实验场景无效")
    if not isinstance(metadata.trial, int) or isinstance(metadata.trial, bool) or metadata.trial < 1:
        raise ValueError("实验编号必须是正整数")
    if not _SUBJECT_RE.fullmatch(metadata.subject):
        raise ValueError("受试者缩写必须为 1–8 位大写 ASCII 字母")
    if not isinstance(metadata.record_date, date):
        raise ValueError("实验日期无效")


def build_recording_paths(metadata: RecordingMetadata) -> RecordingPaths:
    """Build the Raw CSV family paths without creating any filesystem entries."""
    validate_metadata(metadata)
    month = metadata.record_date.strftime("%Y%m")
    day = metadata.record_date.strftime("%m%d")
    directory = Path(metadata.save_root) / f"{month}-multiperson" / f"{day}-{metadata.subject}"
    stem = f"{metadata.scenario_token}{metadata.trial}_{metadata.subject}_{day}"
    raw_path = directory / f"{stem}.csv"
    return RecordingPaths(
        directory=directory,
        raw_path=raw_path,
        status_path=directory / f"{stem}_status.csv",
        marker_path=directory / f"{stem}_markers.csv",
    )


def suggest_next_trial(metadata: RecordingMetadata) -> int:
    """Return one greater than the highest matching main Raw file trial."""
    paths = build_recording_paths(metadata)
    if not paths.directory.is_dir():
        return 1
    day = metadata.record_date.strftime("%m%d")
    pattern = re.compile(
        rf"^{re.escape(metadata.scenario_token)}(\d+)_"
        rf"{re.escape(metadata.subject)}_{day}\.csv$"
    )
    trials = [
        int(match.group(1))
        for path in paths.directory.iterdir()
        if path.is_file() and (match := pattern.fullmatch(path.name))
    ]
    return max(trials, default=0) + 1


def ensure_recording_available(paths: RecordingPaths) -> None:
    """Raise FileExistsError if any member of the recording family exists."""
    for path in (paths.raw_path, paths.status_path, paths.marker_path):
        if path.exists():
            raise FileExistsError(f"录制文件已存在: {path}")
