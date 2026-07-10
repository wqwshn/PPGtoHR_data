from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs" / "原始数据录制文件结构说明.md"


def test_raw_data_file_structure_doc_describes_metadata_based_recording_outputs():
    text = DOC.read_text(encoding="utf-8")

    for filename in [
        "<scenario><trial>_<SUBJECT>_<MMDD>.csv",
        "kaiji1_LYX_0710.csv",
        "_status.csv",
        "至少一次",
        "ValidFlag",
        "GapLen",
    ]:
        assert filename in text


def test_raw_data_file_structure_doc_explains_nan_placeholders():
    text = DOC.read_text(encoding="utf-8")

    assert "NaN" in text
    assert "ValidFlag=0" in text
    assert "InterpFlag" in text
    assert "30-50" in text
    assert "丢包" in text
    assert "缺失占位符" in text
