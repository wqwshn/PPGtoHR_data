"""Validated build settings and argument lists for the desktop workbench."""
from __future__ import annotations

import json
import os
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "desktop"
SETTINGS = ROOT / "config" / "firmware.json"
OPENOCD_CONFIG = ROOT / "tools" / "firmware" / "openocd-stlink.cfg"


@dataclass(frozen=True)
class FirmwareSettings:
    channel: int = 2
    ble_init: int = 0
    work_mode: int = 0
    swd_speed: int = 1000
    ble_rf_disabled: int = 0

    def __post_init__(self):
        for name, allowed in {"channel": (0, 1, 2), "ble_init": (0, 1),
                              "ble_rf_disabled": (0, 1, 2, 3, 4, 5, 6, 7, 8), "work_mode": (0, 1), "swd_speed": (100, 400, 1000, 1800)}.items():
            value = getattr(self, name)
            if type(value) is not int or value not in allowed:
                raise ValueError(f"无效配置 {name}: {value}")

    def summary(self) -> str:
        channel = f"IIC{self.channel}" if self.channel else "不使用 PPG（双 IIC 关闭，PPG 填 0）"
        light = ("红光 / 红外" if self.work_mode else "绿光 / 红光 / 红外") if self.channel else "光模式不生效"
        return (f"STM32L452CEU6 · {channel} · "
                f"蓝牙射频{('正常运行', '关闭（保持硬件复位）', '自动三轮：连接30秒/复位30秒', '功率七段：+2.5/0/+2.5/-5/+2.5/-10/+2.5 dBm', '只读功率自检（不设置功率）', '只写功率七段（未读回验证）', '固定 -10 dBm（启动复位后只写一次）', '批量5帧/50ms，启动复位配置 +2.5 dBm（未读回）', '单帧100Hz，启动复位配置 +2.5 dBm（未读回）')[self.ble_rf_disabled]} · "
                f"蓝牙初始化{'开启' if self.ble_init else '关闭'} · "
                f"{light} · "
                f"Raw 100 Hz · SWD {self.swd_speed} kHz")

    def cmake_args(self) -> list[str]:
        return ["-S", str(ROOT), "-B", str(BUILD), "-G", "Ninja",
                "-DCMAKE_BUILD_TYPE=Debug", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                "-DPPG_SAMPLE_RATE=100", "-DENABLE_RAW_DATA_PACKET=1",
                f"-DPPG_DEFAULT_CHANNEL={self.channel}",
                f"-DENABLE_BLE_CONFIG={0 if self.ble_rf_disabled else self.ble_init}",
                f"-DBLE_RF_DISABLED={int(self.ble_rf_disabled == 1)}",
                f"-DBLE_RF_EXPERIMENT={int(self.ble_rf_disabled == 2)}",
                f"-DBLE_POWER_EXPERIMENT={int(self.ble_rf_disabled in (3, 4, 5))}",
                f"-DBLE_POWER_READ_ONLY={int(self.ble_rf_disabled == 4)}",
                f"-DBLE_POWER_WRITE_ONLY={int(self.ble_rf_disabled == 5)}",
                f"-DBLE_FIXED_MINUS10={int(self.ble_rf_disabled == 6)}",
                f"-DBLE_BATCH5={int(self.ble_rf_disabled == 7)}",
                f"-DBLE_SINGLE25={int(self.ble_rf_disabled == 8)}",
                f"-DCURRENT_WORK_MODE={self.work_mode}"]

    def save(self, path: Path = SETTINGS):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path: Path = SETTINGS):
        if not path.exists():
            return cls(ble_rf_disabled=8)
        return cls(**json.loads(path.read_text(encoding="utf-8")))


def tool_environment() -> dict[str, str]:
    env = dict(os.environ)
    runtime = ROOT
    if not (runtime / ".venv").exists():
        runtime = next((p for p in ROOT.parents if (p / ".venv" / "Scripts" / "python.exe").exists()), ROOT)
    bins = [runtime / ".venv" / "Scripts"]
    for pattern in ("xpack-arm-none-eabi-gcc-*/bin", "xpack-openocd-*/bin"):
        bins.extend(sorted((runtime / ".local-tools").glob(pattern), reverse=True))
    env["PATH"] = os.pathsep.join(str(p) for p in bins) + os.pathsep + env.get("PATH", "")
    env["PYTHONUTF8"] = "1"
    return env


def find_tools(env: dict[str, str]) -> dict[str, str]:
    tools = {}
    for name in ("cmake", "ninja", "arm-none-eabi-gcc", "openocd"):
        path = shutil.which(name, path=env["PATH"])
        if not path:
            raise FileNotFoundError(f"缺少 {name}，请查看 docs/统一工作台使用说明.md 的环境说明。")
        tools[name] = path
    return tools


def flash_args(settings: FirmwareSettings) -> list[str]:
    # Relative firmware path keeps OpenOCD's Tcl command independent of spaces in ROOT.
    return ["-f", str(OPENOCD_CONFIG), "-c", f"adapter speed {settings.swd_speed}",
            "-c", "program {build/desktop/L452CEU6.elf} verify reset exit"]
