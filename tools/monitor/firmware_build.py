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

    def __post_init__(self):
        for name, allowed in {"channel": (0, 1, 2), "ble_init": (0, 1),
                              "work_mode": (0, 1), "swd_speed": (100, 400, 1000, 1800)}.items():
            value = getattr(self, name)
            if type(value) is not int or value not in allowed:
                raise ValueError(f"无效配置 {name}: {value}")

    def summary(self) -> str:
        channel = f"IIC{self.channel}" if self.channel else "不使用 PPG（双 IIC 关闭，PPG 填 0）"
        light = ("红光 / 红外" if self.work_mode else "绿光") if self.channel else "光模式不生效"
        return (f"STM32L452CEU6 · {channel} · "
                f"蓝牙初始化{'开启' if self.ble_init else '关闭'} · "
                f"{light} · "
                f"Raw 100 Hz · SWD {self.swd_speed} kHz")

    def cmake_args(self) -> list[str]:
        return ["-S", str(ROOT), "-B", str(BUILD), "-G", "Ninja",
                "-DCMAKE_BUILD_TYPE=Debug", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                "-DPPG_SAMPLE_RATE=100", "-DENABLE_RAW_DATA_PACKET=1",
                f"-DPPG_DEFAULT_CHANNEL={self.channel}",
                f"-DENABLE_BLE_CONFIG={self.ble_init}",
                f"-DCURRENT_WORK_MODE={self.work_mode}"]

    def save(self, path: Path = SETTINGS):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path: Path = SETTINGS):
        if not path.exists():
            return cls()
        return cls(**json.loads(path.read_text(encoding="utf-8")))


def tool_environment() -> dict[str, str]:
    env = dict(os.environ)
    bins = [ROOT / ".venv" / "Scripts"]
    for pattern in ("xpack-arm-none-eabi-gcc-*/bin", "xpack-openocd-*/bin"):
        bins.extend(sorted((ROOT / ".local-tools").glob(pattern), reverse=True))
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
