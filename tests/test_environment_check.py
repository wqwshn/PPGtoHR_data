"""Missing firmware tools must not block acquisition; missing runtime packages must."""
import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location("monitor_environment_check",
    Path(__file__).resolve().parents[1] / "scripts/check_environment.py")
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)


def test_acquisition_can_run_without_firmware_tools(monkeypatch):
    monkeypatch.setattr(check.shutil, "which", lambda *a, **kw: None)
    assert check.collect_report()["ok"]
    assert not check.collect_report(firmware=True)["ok"]


def test_missing_runtime_package_has_a_readable_failure(monkeypatch):
    original = check.importlib.import_module
    def import_module(name):
        if name == "serial":
            raise ImportError("pyserial is missing")
        return original(name)
    monkeypatch.setattr(check.importlib, "import_module", import_module)
    report = check.collect_report()
    assert not report["ok"]
    failure = next(item for item in report["checks"] if item["name"] == "pyserial")
    assert not failure["ok"] and "missing" in failure["detail"]
