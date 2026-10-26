"""Check the monitor environment without opening serial ports or using ST-Link."""
from __future__ import annotations

import argparse
import importlib
from importlib import metadata
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "monitor"))
RUNTIME_MODULES = (("PyQt5.QtCore", "PyQt5"), ("pyqtgraph", "pyqtgraph"),
                   ("serial", "pyserial"), ("numpy", "numpy"))


def collect_report(*, firmware: bool = False) -> dict:
    checks = [{"name": "Python 3.10-3.12 / 64-bit", "ok": (3, 10) <= sys.version_info[:2] <= (3, 12) and struct.calcsize("P") == 8,
               "required": True, "detail": f"{sys.executable} ({sys.version.split()[0]})"}]
    for module, package in RUNTIME_MODULES:
        try:
            importlib.import_module(module)
            detail, ok = metadata.version(package), True
        except Exception as exc:
            detail, ok = str(exc), False
        checks.append({"name": package, "ok": ok, "required": True, "detail": detail})
    from firmware_build import tool_environment
    env = tool_environment()
    for tool in ("cmake", "ninja", "arm-none-eabi-gcc", "openocd"):
        executable = shutil.which(tool, path=env["PATH"])
        checks.append({"name": tool, "ok": executable is not None,
                       "required": firmware, "detail": executable or "not found; optional for acquisition"})
    return {"ok": all(check["ok"] for check in checks if check["required"]),
            "project": str(ROOT), "checks": checks}


def gui_worker() -> int:
    from PyQt5.QtWidgets import QApplication
    from workbench import Workbench
    app = QApplication([])
    window = Workbench(simulate=True)
    try:
        panel = window.monitor._raw_panel
        for _ in range(100):
            panel._sim_tick()
        app.processEvents()
        count = panel._quality.received_count
        print(json.dumps({"simulated_samples": count, "ok": count >= 100}))
        return 0 if count >= 100 else 1
    finally:
        window.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", action="store_true", help="Also require the four firmware build/flash tools")
    parser.add_argument("--gui-smoke-test", action="store_true", help="Create the workbench and process simulated samples offscreen")
    parser.add_argument("--json", action="store_true", help="Print a machine-readable report")
    parser.add_argument("--gui-worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.gui_worker:
        return gui_worker()
    report = collect_report(firmware=args.firmware)
    if args.gui_smoke_test and report["ok"]:
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen", PYTHONUTF8="1")
        try:
            result = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--gui-worker"],
                                    cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", timeout=45)
            check = {"name": "offscreen simulation", "required": True, "ok": result.returncode == 0,
                     "detail": (result.stdout + result.stderr)[-2000:].strip()}
        except (OSError, subprocess.TimeoutExpired) as exc:
            check = {"name": "offscreen simulation", "required": True, "ok": False, "detail": str(exc)}
        report["checks"].append(check)
        report["ok"] &= check["ok"]
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        for check in report["checks"]:
            status = "PASS" if check["ok"] else "FAIL" if check["required"] else "OPTIONAL"
            print(f"[{status}] {check['name']}: {check['detail']}")
        print("Environment ready" if report["ok"] else "Environment incomplete")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
