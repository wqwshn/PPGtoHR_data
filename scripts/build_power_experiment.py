"""Build the continuous seven-stage power experiment; never access hardware."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/monitor'))
from firmware_build import FirmwareSettings, tool_environment, find_tools

if __name__ == '__main__':
    env = tool_environment()
    tools = find_tools(env)
    read_only = '--read-only' in sys.argv
    write_only = '--write-only' in sys.argv
    fixed = '--fixed-minus10' in sys.argv
    batch = '--batch5' in sys.argv
    single = '--single25' in sys.argv
    if sum((read_only,write_only,fixed,batch,single))>1: raise ValueError('Choose only one mode')
    build = ROOT / ('build/ble-single25' if single else 'build/ble-batch5' if batch else 'build/ble-fixed-minus10' if fixed else 'build/ble-power-writeonly' if write_only else 'build/ble-power-readonly' if read_only else 'build/ble-power')
    build.mkdir(parents=True, exist_ok=True)
    settings = FirmwareSettings(channel=0, ble_init=0, ble_rf_disabled=8 if single else 7 if batch else 6 if fixed else 5 if write_only else 4 if read_only else 3)
    args = settings.cmake_args()
    args[args.index('-B') + 1] = str(build)
    with (build / 'build.log').open('w', encoding='utf-8') as log:
        for command in ([tools['cmake'], *args],
                        [tools['cmake'], '--build', str(build), '-j4']):
            subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                           stderr=subprocess.STDOUT, check=True)
    manifest = {'settings': settings.__dict__, 'stages_dbm': [2.5] if (batch or single) else [-10] if fixed else [] if read_only else [2.5, 0, 2.5, -5, 2.5, -10, 2.5],
                'prepare_s': 0 if (fixed or batch or single) else 10 if read_only else 60, 'settle_s': 0 if (read_only or fixed or batch or single) else 10, 'measure_s': None if (fixed or batch or single) else 0 if read_only else 60,
                'readback_verified': False if (write_only or fixed or batch or single) else None, 'batch_frames': 5 if batch else 1, 'sample_rate_hz': 100, 'artifacts': {}}
    for ext in ('elf', 'hex', 'bin'):
        p = build / ('L452CEU6.' + ext)
        manifest['artifacts'][p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    (build / 'firmware-manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print('Power experiment build passed; not flashed:', build)
