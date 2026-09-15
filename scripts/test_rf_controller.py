"""Run compiled ARM controller without accessing target hardware."""
import sys, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/monitor'))
sys.path.insert(0,str(ROOT/'.local/test-deps'))
from firmware_build import tool_environment,find_tools
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0
env=tool_environment();gcc=find_tools(env)['arm-none-eabi-gcc']
power='--power' in sys.argv
batch='--batch' in sys.argv
out=ROOT/('build/batch-controller-test' if batch else 'build/power-controller-test' if power else 'build/controller-test');out.mkdir(parents=True,exist_ok=True)
subprocess.run([gcc,'-mcpu=cortex-m4','-mthumb','-O1','-ffreestanding','-nostdlib',
 '-I'+str(ROOT/'Core/Inc'),str(ROOT/('tests/batch_controller_harness.c' if batch else 'tests/power_controller_harness.c' if power else 'tests/rf_controller_harness.c')),
 '-Wl,-Ttext=0x1000,-e,rf_controller_test','-o',str(out/'test.elf')],env=env,check=True)
nm=str(Path(gcc).with_name('arm-none-eabi-nm.exe'))
symbols=subprocess.check_output([nm,str(out/'test.elf')],text=True)
entry=int(next(line.split()[0] for line in symbols.splitlines() if line.endswith(' rf_controller_test')),16)
objcopy=str(Path(gcc).with_name('arm-none-eabi-objcopy.exe'))
subprocess.run([objcopy,'-O','binary',str(out/'test.elf'),str(out/'test.bin')],check=True)
emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);emu.mem_map(0,0x40000)
emu.mem_write(0x1000,(out/'test.bin').read_bytes());emu.reg_write(UC_ARM_REG_SP,0x30000)
emu.reg_write(UC_ARM_REG_LR,0x38001)
emu.emu_start(entry|1,0x38000,count=1000000)
result=emu.reg_read(UC_ARM_REG_R0)
assert result==0,f'Controller assertion failed at C line {result}'
print('ARM batch buffering tests passed.' if batch else 'ARM power controller tests passed: seven stages, readback, recovery, disconnect, tick wrap.' if power else
      'ARM controller tests passed: three rounds, reconnect waits, early disconnect, stuck pin, tick wrap.')
