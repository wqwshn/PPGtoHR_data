"""Build both RF diagnostic images without accessing hardware."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools' / 'monitor'))
from firmware_build import FirmwareSettings, tool_environment, find_tools

env = tool_environment()
tools = find_tools(env)
manifest = {}
for disabled in (0, 1, 2):
    name = ('rf-on', 'rf-off', 'rf-alternating')[disabled]
    build = ROOT / 'build' / name
    settings = FirmwareSettings(channel=0, ble_init=0, ble_rf_disabled=disabled)
    args = settings.cmake_args()
    args[args.index('-B') + 1] = str(build)
    build.mkdir(parents=True, exist_ok=True)
    with (build / 'build.log').open('w', encoding='utf-8') as log:
        subprocess.run([tools['cmake'], *args], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run([tools['cmake'], '--build', str(build), '-j4'], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    manifest[name] = {'settings': settings.__dict__, 'artifacts': {}}
    for ext in ('elf', 'hex', 'bin'):
        p = build / ('L452CEU6.' + ext)
        manifest[name]['artifacts'][p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    print(name, 'build passed', flush=True)
(ROOT / 'build' / 'rf-pair-manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
