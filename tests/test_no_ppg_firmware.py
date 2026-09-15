"""Verify the compiler-selected packet path for both optical modes."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GCC = next((ROOT / ".local-tools").glob("xpack-arm-none-eabi-gcc-*/bin/arm-none-eabi-gcc.exe"), None)
GCC = str(GCC) if GCC else shutil.which("arm-none-eabi-gcc")


@pytest.mark.skipif(not GCC, reason="ARM GCC is required for firmware preprocessing")
@pytest.mark.parametrize("mode", [0, 1])
def test_no_ppg_packet_path_keeps_other_data_and_zeroes_ppg(mode):
    includes = ["Core/Inc", "Drivers/CMSIS/Include", "Drivers/CMSIS/DSP/Include",
                "Drivers/CMSIS/Device/ST/STM32L4xx/Include", "Drivers/STM32L4xx_HAL_Driver/Inc"]
    result = subprocess.run([GCC, "-E", "-P", "-DSTM32L452xx", "-DUSE_HAL_DRIVER",
                             "-DPPG_DEFAULT_CHANNEL=0", f"-DCURRENT_WORK_MODE={mode}",
                             *[f"-I{ROOT / path}" for path in includes],
                             str(ROOT / "Core/Src/main.c")], capture_output=True, check=True)
    source = result.stdout.decode("utf-8", errors="replace")
    main_body = source.split("int main(void)", 1)[1].split("void SystemClock_Config(void)", 1)[0]
    compact = re.sub(r"\s+", "", main_body)
    assert "memset(&allData[22],0,9);" in compact
    assert "HAL_UART_Transmit_DMA(&huart2,allData,35)" in compact
    assert "CheckXOR(&allData[2],31)" in compact
    assert "PPG_Read" not in main_body
    assert "PPG_Init();" not in main_body
    assert "PPG_Check()" not in main_body
    assert "raw_diag_ppg_fifo_empty_counter++" not in main_body
