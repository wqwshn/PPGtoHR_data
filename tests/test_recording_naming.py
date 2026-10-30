import sys
from datetime import date
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

from recording_naming import (
    SCENARIO_OPTIONS,
    RecordingMetadata,
    build_recording_paths,
    ensure_recording_available,
    suggest_next_trial,
)


def _metadata(root: Path, **overrides) -> RecordingMetadata:
    values = {
        "save_root": root,
        "scenario_token": "run",
        "trial": 1,
        "subject": "LYX",
        "record_date": date(2026, 7, 10),
    }
    values.update(overrides)
    return RecordingMetadata(**values)


def test_motion_metadata_preserves_filename_with_one_less_directory(tmp_path):
    paths = build_recording_paths(_metadata(tmp_path))

    assert ("开机", "kaiji") in SCENARIO_OPTIONS
    assert ("快速出拳", "quanji") in SCENARIO_OPTIONS
    assert paths.directory == tmp_path / "0710-LYX"
    assert paths.raw_path == paths.directory / "run1_LYX_0710.csv"
    assert paths.status_path == paths.directory / "run1_LYX_0710_status.csv"
    assert paths.marker_path == paths.directory / "run1_LYX_0710_markers.csv"


@pytest.mark.parametrize(
    ("label", "token", "expected_filename"),
    [
        ("开机", "kaiji", "kaiji1_20260710.csv"),
        ("滑移测试", "huayi", "huayi1_20260710.csv"),
        ("温漂测试", "wenpiao", "wenpiao1_20260710.csv"),
        (
            "接近度/接触压力测试（上到下：接触压力→接近度）",
            "PnP_down",
            "PnP_down1_20260710.csv",
        ),
        (
            "接近度/接触压力测试（下到上：接近度→接触压力）",
            "PnP_up",
            "PnP_up1_20260710.csv",
        ),
    ],
)
def test_new_scenarios_are_selectable_and_used_in_recording_names(
    tmp_path, label, token, expected_filename
):
    assert (label, token) in SCENARIO_OPTIONS

    paths = build_recording_paths(_metadata(tmp_path, scenario_token=token, subject=""))

    assert paths.raw_path.name == expected_filename
    assert paths.directory == tmp_path / "20260710"


@pytest.mark.parametrize(
    ("subject", "trial"),
    [
        ("", 1),
        ("lower", 1),
        ("TOO_LONG9", 1),
        ("A-1", 1),
        ("LYX", 0),
    ],
)
def test_metadata_rejects_invalid_subject_or_trial(tmp_path, subject, trial):
    with pytest.raises(ValueError):
        build_recording_paths(_metadata(tmp_path, subject=subject, trial=trial))


def test_suggest_next_trial_uses_only_exact_main_files(tmp_path):
    metadata = _metadata(tmp_path)
    paths = build_recording_paths(metadata)
    paths.directory.mkdir(parents=True)
    for filename in [
        "run1_LYX_0710.csv",
        "run3_LYX_0710.csv",
        "run2_LYX_0710_status.csv",
        "run9_OTHER_0710.csv",
        "running7_LYX_0710.csv",
    ]:
        (paths.directory / filename).touch()

    assert suggest_next_trial(metadata) == 4


def test_signal_trial_suggestion_has_no_subject_and_ignores_companion_files(tmp_path):
    metadata = _metadata(tmp_path, scenario_token="huayi", subject="")
    paths = build_recording_paths(metadata)
    paths.directory.mkdir()
    for filename in ("huayi1_20260710.csv", "huayi3_20260710.csv",
                     "huayi9_20260710_status.csv", "huayi8_20260711.csv",
                     "wenpiao7_20260710.csv"):
        (paths.directory / filename).touch()
    assert suggest_next_trial(metadata) == 4


@pytest.mark.parametrize("path_name", ["raw_path", "status_path", "marker_path"])
def test_existing_companion_file_blocks_recording_start(tmp_path, path_name):
    paths = build_recording_paths(_metadata(tmp_path))
    paths.directory.mkdir(parents=True)
    getattr(paths, path_name).touch()

    with pytest.raises(FileExistsError):
        ensure_recording_available(paths)
