"""Execute actual MIMU initialization and its actual main.c reporting block."""
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def function_source(source, name):
    match = re.search(rf"(?:void|uint8_t) {name}\([^)]*\)\s*\{{", source)
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


@pytest.fixture(scope="module")
def startup_wasm(tmp_path_factory):
    return build_startup_wasm(tmp_path_factory, recovery=1)


@pytest.fixture(scope="module")
def standard_startup_wasm(tmp_path_factory):
    return build_startup_wasm(tmp_path_factory, recovery=0)


def build_startup_wasm(tmp_path_factory, recovery):
    clang, node = shutil.which("clang"), shutil.which("node")
    if not clang or not node:
        pytest.skip("Clang wasm32 and Node are required")
    folder = tmp_path_factory.mktemp("mimu-startup")
    driver = (ROOT / "Core/Src/MIMU.c").read_text(encoding="utf-8")
    main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    start = main.index("  /* 2. MIMU 初始化 */")
    end = main.index("#if (PPG_DEFAULT_CHANNEL != 0)", start)
    defines = "\n".join(re.findall(r"^#define\s+\w+\s+0x[0-9A-Fa-f]+", (ROOT / "Core/Inc/MIMU.h").read_text(encoding="utf-8"), re.M))
    header = (ROOT / "Core/Inc/main.h").read_text(encoding="utf-8")
    status_defines = "\n".join(re.findall(r"^#define (?:MIMU_STATE_\w+|STATUS_\w+|RAW_DIAG_PROTOCOL_VERSION)\s+\S+", header, re.M))
    source = status_defines + r'''
#include <stdint.h>
#define HAL_OK 0
#define ENABLE_BOARD_CHECK 0
#define HAL_UART_STATE_READY 0
#define DRDY_Pin 2048
#define GPIOA 1
#define SPI1_NSS_Pin 16
#define START_CONV_Pin 1024
#define CS_A_G_GPIO_Port 2
#define CS_A_G_Pin 128
#define PPG_SAMPLE_RATE 100
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define ADC_MUX_REG 2
int snprintf(char *, __SIZE_TYPE__, const char *, ...);
__SIZE_TYPE__ strlen(const char *);
struct {int gState;} huart2;
uint8_t ADC_1to4Voltage_flag, Utop_times1, Utop_times2, allData[50];
uint32_t raw_diag_adc_drdy_counter, raw_diag_adc_error_counter, raw_diag_imu_error_counter;
uint32_t raw_diag_sample_counter;
uint32_t raw_diag_frame_counter, raw_diag_tx_start_counter, raw_diag_tx_done_counter;
uint32_t raw_diag_tx_busy_counter, raw_diag_tx_error_counter, raw_diag_ppg_fifo_empty_counter;
uint32_t raw_diag_ppg_fifo_overflow_counter, raw_diag_ppg_fifo_sample_total_counter;
uint32_t raw_diag_ppg_fifo_nonempty_counter, raw_diag_ppg_fifo_single_sample_counter, raw_diag_ppg_fifo_multi_sample_counter;
static uint8_t statusData[STATUS_PACKET_LEN];
static uint8_t mimu_state = MIMU_STATE_INITIALIZING;
uint8_t raw_diag_status_pending;
typedef struct {int unused;} TIM_HandleTypeDef;
TIM_HandleTypeDef htim16;
static volatile uint8_t mimu_ready;
static uint8_t mimu_recovery_warming;
static uint32_t mimu_last_retry_ms;
void ACC_GYRO_Write(uint8_t, uint8_t);
void MAG_Write(uint8_t, uint8_t);
uint8_t ACC_GYRO_Read(uint8_t);
int ACC_GYRO_ReadChecked(uint8_t, uint8_t *);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(int, int, int);
void ADC_RDATA(uint8_t *);
void ADC_WREG(uint8_t, uint8_t);
void ACC_6BytesRead(void);
void GYRO_6BytesRead(void);
int HAL_UART_Transmit(void *, uint8_t *, uint16_t, uint32_t);
void MIMU_LoadGyroOffset(void);
void MIMU_GyroCalibrate(void);
uint8_t CheckXOR(uint8_t *data, uint8_t length) {uint8_t result=0; while(length--) result^=*data++; return result;}
'''
    source += defines + "\n" + function_source(driver, "MIMU_Init")
    source += "\n" + function_source(main, "WriteU32BE")
    source += "\n" + function_source(main, "BuildStatusFrame")
    for name in ("MIMU_Startup", "MIMU_PollRecovery"):
        if re.search(rf"void {name}\(void\)\s*\{{", main):
            source += "\n" + function_source(main, name)
    source += "\n" + function_source(main, "HAL_GPIO_EXTI_Callback")
    source += "\n" + function_source(main, "HAL_TIM_PeriodElapsedCallback")
    source += "\nvoid Test_Start(void) {\n" + main[start:end] + "\n}\n"
    poll = "MIMU_PollRecovery();" if re.search(r"void MIMU_PollRecovery\(void\)\s*\{", source) else ""
    source += f"\nvoid Test_Poll(void) {{{poll}}}\n"
    source += "\nuint8_t Test_Ready(void) {return mimu_ready;}\n"
    source += "\nuint8_t Test_State(void) {return mimu_state;}\n"
    source += "\nvoid Test_BuildStatus(void) {BuildStatusFrame();}\n"
    source += "\nuint8_t Test_StatusByte(unsigned index) {return statusData[index];}\n"
    source += "\nvoid Test_DataISR(void) {ADC_1to4Voltage_flag=4; HAL_GPIO_EXTI_Callback(DRDY_Pin);}\n"
    source += "\nvoid Test_UartBusy(void) {huart2.gState=1;}\n"
    source += "\nvoid Test_TimerISR(void) {HAL_TIM_PeriodElapsedCallback(&htim16);}\n"
    path = folder / "startup.c"
    path.write_text(source, encoding="utf-8")
    wasm = folder / "startup.wasm"
    subprocess.run([clang, "--target=wasm32", "-nostdlib", "-O0", str(path),
                    f"-DENABLE_MIMU_RECOVERY={recovery}",
                    "-Wl,--no-entry", "-Wl,--allow-undefined", "-Wl,--export-all",
                    "-o", str(wasm)], check=True, capture_output=True)
    return node, wasm


@pytest.mark.parametrize("scenario,healthy", [({}, True), ({"identity": 255}, False),
                                            ({"identity": 0}, False), ({"bad_config": True}, False),
                                            ({"spi_error": True}, False)])
def test_mimu_ok_requires_identity_and_configuration_and_failure_skips_calibration(startup_wasm, scenario, healthy):
    node, wasm = startup_wasm
    result = subprocess.run([node, str(ROOT / "tests/mimu_startup_sim.cjs"), str(wasm), json.dumps(scenario)],
                            check=True, capture_output=True, text=True, timeout=10)
    data = json.loads(result.stdout)
    assert ("MIMU Init OK" in data["output"]) == healthy
    assert data["calibrated"] == int(healthy)
    assert data["loaded"] == int(healthy)
    if not healthy:
        assert "FAIL" in data["output"]


def simulate(startup_wasm, scenario):
    node, wasm = startup_wasm
    result = subprocess.run([node, str(ROOT / "tests/mimu_startup_sim.cjs"), str(wasm), json.dumps(scenario)],
                            check=True, capture_output=True, text=True, timeout=10)
    return json.loads(result.stdout)


def test_cold_start_retries_until_late_identity_then_calibrates_once(startup_wasm):
    data = simulate(startup_wasm, {"ready_at_ms": 1800})
    assert data["ready"] == 1
    assert data["state"] == 1
    assert data["calibrated"] == data["loaded"] == 1
    assert data["identity_reads"] >= 4
    assert "retry" in data["output"].lower()


def test_initial_failures_do_not_read_unconfigured_device_in_adc_isr(startup_wasm):
    data = simulate(startup_wasm, {"identity": 255, "read_after_start": True})
    assert data["ready"] == 0
    assert data["data_reads"] == 0


def test_background_recovery_uses_saved_offset_without_blocking_calibration_or_spi_collision(startup_wasm):
    data = simulate(startup_wasm, {"ready_at_ms": 4800, "poll_at_ms": [3500, 5000, 5100, 5400, 7400],
                                  "interrupt_during_delay": True, "read_after_polls": True})
    assert data["ready_history"] == [0, 0, 0, 1, 1]
    assert data["state_history"] == [2, 2, 2, 3, 3]
    assert data["calibrated"] == 0
    assert data["loaded"] == 1
    assert data["irq_reads_during_delay"] == 0
    assert data["irq_cs_writes_during_delay"] == 0
    assert data["data_reads"] >= 2
    assert "recovered" in data["output"].lower()


def test_bad_configuration_never_recovers_or_loads_offsets(startup_wasm):
    data = simulate(startup_wasm, {"bad_config": True, "poll_at_ms": [6000, 9000, 12000]})
    assert data["ready"] == 0
    assert data["calibrated"] == data["loaded"] == 0


def test_busy_uart_defers_background_attempt(startup_wasm):
    data = simulate(startup_wasm, {"ready_at_ms": 4800, "poll_at_ms": [6000], "uart_busy": True})
    assert data["ready"] == 0
    assert data["loaded"] == 0
    assert data["configurations"] == 0


def test_standard_startup_detects_once_and_never_recovers(standard_startup_wasm):
    data = simulate(standard_startup_wasm, {"ready_at_ms": 1800,
                                           "poll_at_ms": [5000, 8000, 11000]})
    assert data["ready"] == 0
    assert data["identity_reads"] == 1
    assert data["state"] == 4
    assert data["calibrated"] == data["loaded"] == 0
    assert "power settling" not in data["output"]
    assert "recovery disabled" in data["output"]


def test_standard_healthy_startup_still_calibrates(standard_startup_wasm):
    data = simulate(standard_startup_wasm, {})
    assert data["ready"] == 1
    assert data["calibrated"] == data["loaded"] == 1
    assert "power settling" not in data["output"]


@pytest.mark.parametrize("scenario,state,failures", [
    ({}, 1, 0),
    ({"ready_at_ms": 1800}, 1, 2),
    ({"identity": 255}, 2, 5),
    ({"ready_at_ms": 4800, "poll_at_ms": [6000, 6300]}, 3, 5),
])
def test_firmware_status_snapshot_preserves_init_result_for_late_receiver(startup_wasm, scenario, state, failures):
    import sys
    sys.path.insert(0, str(ROOT / "tools/monitor"))
    from protocol import parse_status_packet
    data = simulate(startup_wasm, scenario)
    status = parse_status_packet(bytes.fromhex(data["status"]))
    assert status is not None
    assert status.mimu_state == state
    assert status.imu_error_counter == failures
