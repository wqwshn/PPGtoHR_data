from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs" / "蓝牙数据缺失问题" / "raw_data_file_structure.md"


def test_raw_data_file_structure_doc_describes_all_recording_outputs():
    text = DOC.read_text(encoding="utf-8")

    for filename in [
        "raw_data_YYYYMMDD_HHMMSS.csv",
        "raw_data_YYYYMMDD_HHMMSS_status.csv",
        "ValidFlag",
        "GapLen",
    ]:
        assert filename in text


def test_raw_data_file_structure_doc_explains_nan_placeholders():
    text = DOC.read_text(encoding="utf-8")

    assert "NaN" in text
    assert "ValidFlag=0" in text
    assert "InterpFlag=0" in text
    assert "30-50" in text
    assert "No automatic interpolation" in text
