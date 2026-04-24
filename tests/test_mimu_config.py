from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MIMU_C = (ROOT / "Core" / "Src" / "MIMU.c").read_text(encoding="utf-8")
LSM9DS1_DOC = (ROOT / "docs" / "LSM9DS1配置说明.md").read_text(encoding="utf-8")


def test_acc_uses_dc_preserving_antialias_filtering():
    assert "ACC_GYRO_Write(CTRL_REG6_XL, 0x77)" in MIMU_C
    assert "ACC_GYRO_Write(CTRL_REG7_XL, 0x00)" in MIMU_C


def test_gyro_precision_registers_are_unchanged():
    assert "ACC_GYRO_Write(CTRL_REG1_G, 0x68)" in MIMU_C
    assert "ACC_GYRO_Write(CTRL_REG3_G, 0x46)" in MIMU_C


def test_documentation_matches_acc_filter_strategy():
    assert "CTRL_REG6_XL (0x20) = 0x77" in LSM9DS1_DOC
    assert "CTRL_REG7_XL (0x21) = 0x00" in LSM9DS1_DOC
    assert "FDS=1" in LSM9DS1_DOC
    assert "Z轴静息趋近0" in LSM9DS1_DOC
