from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SAMPLE_RATE_CONFIG_H = (
    ROOT / "Core" / "Inc" / "sample_rate_config.h"
).read_text(encoding="utf-8")
PROJECT_CONFIG_DOC = (
    ROOT / "docs" / "MAX30101项目配置说明.md"
).read_text(encoding="utf-8")


def _section_between(start: str, end: str) -> str:
    return SAMPLE_RATE_CONFIG_H.split(start, 1)[1].split(end, 1)[0]


def _rate_100_branch(section: str) -> str:
    return section.split("#elif (PPG_SAMPLE_RATE == 100)", 1)[1].split(
        "#elif (PPG_SAMPLE_RATE == 125)", 1
    )[0]


def _rate_100_branch_until_else(section: str) -> str:
    return section.split("#elif (PPG_SAMPLE_RATE == 100)", 1)[1].split(
        "#else", 1
    )[0]


def test_100hz_triple_led_uses_400sps_18bit_with_2x_average():
    spo2_branch = _rate_100_branch(
        _section_between(
            "MAX30101 SPO2_CONFIG_REG",
            "MAX30101 FIFO_CONFIG_REG",
        )
    )
    fifo_branch = _rate_100_branch(
        _section_between(
            "MAX30101 FIFO_CONFIG_REG",
            "采样率信息字符串",
        )
    )
    decode_branch = _rate_100_branch_until_else(
        _section_between(
            "多光路时隙配置",
            "#endif /* __SAMPLE_RATE_CONFIG_H */",
        )
    )

    assert "#define MAX30101_SPO2_CONFIG_VAL    0x6F" in spo2_branch
    assert "SR=400sps" in spo2_branch
    assert "PW=411us" in spo2_branch
    assert "#define MAX30101_FIFO_CONFIG_VAL    0x3F" in fifo_branch
    assert "SMP_AVE=2x" in fifo_branch
    assert "#define MAX30101_PPG_RIGHT_SHIFT       0U" in decode_branch
    assert "#define MAX30101_PPG_VALID_MASK        0x03FFFFU" in decode_branch


def test_project_documentation_matches_deployed_100hz_triple_led_config():
    assert "| SPO2_CONFIG | 0x0A | **0x6F**" in PROJECT_CONFIG_DOC
    assert "| FIFO_CONFIG | 0x08 | **0x3F**" in PROJECT_CONFIG_DOC
    assert "400sps / 2x" in PROJECT_CONFIG_DOC
