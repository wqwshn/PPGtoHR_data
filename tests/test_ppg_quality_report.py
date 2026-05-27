import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

from ppg_quality_report import analyze_raw_csv, format_text_report


def test_analyze_raw_csv_reports_fractional_ppg_and_repeated_triples(tmp_path):
    csv_path = tmp_path / "raw_data.csv"
    csv_path.write_text(
        "\n".join([
            "Time(s),SampleIndex,Seq,ValidFlag,InterpFlag,GapLen,MissingBefore,"
            "PPG_Green,PPG_Red,PPG_IR",
            "0.00,0,10,1,0,0,0,100,200,300",
            "0.01,1,11,1,0,0,0,100,200,300",
            "0.02,2,12,1,0,0,0,100.0625,200,300.5",
            "0.03,3,13,0,0,1,,NaN,NaN,NaN",
            "0.04,4,14,1,0,0,1,101.125,200.25,300.5",
        ]),
        encoding="utf-8",
    )

    report = analyze_raw_csv(csv_path)

    assert report.total_rows == 5
    assert report.valid_rows == 4
    assert report.sequence_missing_count == 1
    assert report.triple_zero_diff_count == 1
    assert report.triple_zero_diff_ratio == 1 / 3
    assert report.channels["PPG_Green"].fractional_rows == 2
    assert report.channels["PPG_Green"].zero_diff_count == 1
    assert report.channels["PPG_Red"].fractional_rows == 1
    assert report.channels["PPG_IR"].fractional_rows == 2


def test_format_text_report_includes_validation_metrics(tmp_path):
    csv_path = tmp_path / "raw_data.csv"
    csv_path.write_text(
        "\n".join([
            "Time(s),SampleIndex,Seq,ValidFlag,PPG_Green,PPG_Red,PPG_IR",
            "0.00,0,1,1,10,20,30",
            "0.01,1,2,1,10.0625,20,30",
        ]),
        encoding="utf-8",
    )

    text = format_text_report(analyze_raw_csv(csv_path))

    assert "valid_rows=2/2" in text
    assert "sequence_missing_count=0" in text
    assert "triple_zero_diff_ratio=0.0%" in text
    assert "PPG_Green: fractional_rows=1/2" in text
