"""Workbench transitions and execution of the actual checker with simulated HAL."""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "monitor"))
from firmware_build import FirmwareSettings
from firmware_panel import FirmwarePanel
from PyQt5.QtWidgets import QApplication


def test_board_check_can_reuse_first_boot_bluetooth_initialization(monkeypatch):
    settings = FirmwareSettings(board_check=1, ble_init=1)
    assert "-DENABLE_BLE_CONFIG=1" in settings.cmake_args()
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: FirmwareSettings(board_check=1)))
    panel = FirmwarePanel(lambda: True)
    try:
        assert panel.ble.isEnabled() and not panel.ble.isHidden()
        panel.ble.setCurrentIndex(panel.ble.findData(1))
        assert panel.settings().ble_init == 1
    finally:
        panel.close()


def test_board_check_leaves_bluetooth_available_for_connection(checker_wasm):
    node, wasm = checker_wasm
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), "{}"],
                            check=True, capture_output=True, text=True, timeout=15)
    assert json.loads(result.stdout)["ble_reset"] == 0


def test_shared_rx_requires_external_confirmation_without_querying(checker_wasm):
    node, wasm = checker_wasm
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), '{"ble":"none"}'],
                            check=True, capture_output=True, text=True, timeout=15)
    data = json.loads(result.stdout)
    assert "[MANUAL] BLE HJ-131IMH" in data["output"]
    assert "HJ-380" in data["output"]
    assert data["commands"] == []
    assert "TX disconnected" not in data["output"]


def test_check_settings_override_acquisition_and_old_settings_still_load(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text('{"channel": 0, "ble_rf_disabled": 8}', encoding="utf-8")
    assert FirmwareSettings.load(path).board_check == 0
    settings = FirmwareSettings(channel=0, ble_init=1, ble_rf_disabled=8, board_check=1)
    settings.save(path)
    assert FirmwareSettings.load(path) == settings
    args = settings.cmake_args()
    for flag in ("ENABLE_BOARD_CHECK=1", "PPG_DEFAULT_CHANNEL=0", "ENABLE_BLE_CONFIG=1",
                 "BLE_RF_DISABLED=0", "BLE_RF_EXPERIMENT=0", "BLE_POWER_EXPERIMENT=0",
                 "BLE_SINGLE25=0", "BLE_BATCH5=0", "BLE_FIXED_MINUS10=0"):
        assert f"-D{flag}" in args
    assert "-DENABLE_BOARD_CHECK=0" in FirmwareSettings().cmake_args()
    with pytest.raises(ValueError):
        FirmwareSettings(board_check=2)


def test_check_ui_forces_ppg2_and_can_return_to_capture(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: FirmwareSettings(channel=0, ble_rf_disabled=8)))
    panel = FirmwarePanel(lambda: True)
    panel.purpose.setCurrentIndex(panel.purpose.findData(1))
    assert panel.settings() == FirmwareSettings(channel=0, board_check=1, mimu_recovery=0)
    assert panel.channel.isEnabled() and not panel.rf.isEnabled()
    assert not panel.mode.isEnabled() and panel.ble.isEnabled()
    assert "跳过 PPG" in panel.summary.text() and "115200" in panel.output_note.text()
    panel.purpose.setCurrentIndex(panel.purpose.findData(0))
    assert panel.settings().channel == 0 and panel.settings().ble_rf_disabled == 8
    assert panel.channel.isEnabled() and panel.rf.isEnabled()
    assert "-DENABLE_BOARD_CHECK=0" in panel.settings().cmake_args()
    panel.close()


@pytest.fixture(scope="module")
def checker_wasm(tmp_path_factory):
    return compile_checker(tmp_path_factory.mktemp("board-check"))


@pytest.fixture(scope="module")
def checker_with_init(tmp_path_factory):
    return compile_checker(tmp_path_factory.mktemp("board-check-init"), init=1)


@pytest.fixture(scope="module")
def checker_no_ppg(tmp_path_factory):
    return compile_checker(tmp_path_factory.mktemp("board-check-no-ppg"), channel=0)


def compile_checker(folder, init=0, channel=2):
    clang, node = shutil.which("clang"), shutil.which("node")
    if not clang or not node:
        pytest.skip("Clang wasm32 and Node are required for simulated HAL execution")
    for name in ("spi.h", "usart.h", "MyIIC_2.h"):
        (folder / name).write_text("/* HAL supplied by mock.h. */\n", encoding="utf-8")
    (folder / "stdio.h").write_text("int snprintf(char *, __SIZE_TYPE__, const char *, ...);\n", encoding="utf-8")
    (folder / "string.h").write_text("__SIZE_TYPE__ strlen(const char *);\nint strncmp(const char *, const char *, __SIZE_TYPE__);\n", encoding="utf-8")
    (folder / "mock.h").write_text(r'''
#include <stdint.h>
#define __MAIN_H
#define ENABLE_BOARD_CHECK 1
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef uint32_t GPIO_PinState;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
typedef struct { uint32_t id; } SPI_HandleTypeDef;
typedef struct { struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
SPI_HandleTypeDef hspi1={1}, hspi2={2};
UART_HandleTypeDef huart2={{115200}};
#define CS_A_G_GPIO_Port 2
#define CS_M_GPIO_Port 2
#define CS_A_G_Pin 128
#define CS_M_Pin 256
#define SPI1_NSS_GPIO_Port 1
#define SPI1_NSS_Pin 16
#define START_CONV_GPIO_Port 1
#define START_CONV_Pin 1024
#define AD_RESET_GPIO_Port 1
#define AD_RESET_Pin 4096
#define BLE_RST_GPIO_Port 2
#define BLE_RST_Pin 8192
#define V5_0_CE_GPIO_Port 1
#define V5_0_CE_Pin 256
#define V1_8_CE_GPIO_Port 1
#define V1_8_CE_Pin 512
#define I2C_SCL2_GPIO_Port 2
#define I2C_SCL2_Pin 1
#define I2C_SDA2_GPIO_Port 2
#define I2C_SDA2_Pin 2
#define EXTI15_10_IRQn 40
#define USART2_IRQn 38
#define __HAL_UART_CLEAR_OREFLAG(x) Mock_Clear()
#define __HAL_UART_FLUSH_DRREGISTER(x) Mock_Clear()
void Mock_Clear(void);
void HAL_NVIC_DisableIRQ(int);
void HAL_GPIO_WritePin(uint32_t, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(uint32_t, uint16_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *);
void Error_Handler(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
void IIC2_Start(void);
void IIC2_Stop(void);
void IIC2_Send_Byte(uint8_t);
uint8_t IIC2_Wait_Ack(void);
uint8_t IIC2_Read_Byte(uint8_t);
''', encoding="utf-8")
    with (folder / "mock.h").open("a", encoding="utf-8") as stream:
        stream.write(f'\n#define ENABLE_BLE_CONFIG {init}\n#define PPG_DEFAULT_CHANNEL {channel}\n#define BLE_CUSTOM_MAC "784128c58150"\n')
    main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    start = main.index("static void BLE_Init(void)\n{")
    end = main.index("#endif /* ENABLE_BLE_CONFIG */", start)
    helper = ("#if ENABLE_BLE_CONFIG\nstatic void BLE_ResetModule(void);\n"
              "static void BLE_SendConfigCommand(const uint8_t *, uint16_t);\n"
              "static void BLE_SetUartBaud(uint32_t);\n" + main[start:end] + "\n#endif\n")
    start = main.index("#if ENABLE_BOARD_CHECK", main.index("/* USER CODE BEGIN 2 */"))
    end = main.index("  BoardCheck_Run();", start) + len("  BoardCheck_Run();")
    runner = folder / "runner.c"
    runner.write_text(f'#include "{(ROOT / "Core/Src/board_check.c").as_posix()}"\n' + helper +
                      "void Test_Run(void) {\n" + main[start:end] + "\n#endif\n}\n", encoding="utf-8")
    wasm = folder / "checker.wasm"
    subprocess.run([clang, "--target=wasm32", "-nostdlib", "-O0", "-fno-builtin",
                    "-I", str(folder), "-I", str(ROOT / "Core/Inc"),
                    "-include", str(folder / "mock.h"), str(runner),
                    "-Wl,--no-entry", "-Wl,--allow-undefined", "-Wl,--export=Test_Run",
                    "-Wl,--export=BoardCheck_PrintSummary", "-o", str(wasm)], check=True, capture_output=True)
    return node, wasm


@pytest.mark.parametrize("scenario,expected", [
    ({}, [1, 1, 1, 1, 1]),
    ({"adc": 0xFF, "baud": 115200}, [1, 1, 0, 1, 1]),
    ({"adc": 0xF9}, [1, 1, 1, 1, 1]),
    ({"spi_error": True, "ack_fail": 1, "ble": "none"}, [0, 0, 0, 0, 0]),
    ({"ack_fail": 2}, [1, 1, 1, 0, 1]),
    ({"ack_fail": 3}, [1, 1, 1, 0, 1]),
    ({"bus_low": True, "ble": "echo"}, [1, 1, 1, 0, 0]),
    ({"part": 0xFF, "ble": "partial"}, [1, 1, 1, 0, 0]),
    ({"ble": "invalid"}, [1, 1, 1, 1, 0]),
    ({"ble": "noise_then_valid"}, [1, 1, 1, 1, 1]),
    ({"ble": "uart_error"}, [1, 1, 1, 1, 0]),
])
def test_actual_checker_reports_failures_and_continues(checker_wasm, scenario, expected):
    node, wasm = checker_wasm
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), json.dumps(scenario)],
                            check=True, capture_output=True, text=True, timeout=15)
    data = json.loads(result.stdout)
    summary = data["output"].split("=== BOARD COMMUNICATION CHECK SUMMARY ===")[-1]
    rows = [line for line in summary.splitlines() if line.startswith(("[PASS]", "[FAIL]", "[SKIP]", "[MANUAL]"))]
    expected = [expected[0], 2, *expected[2:4], 3]  # MAG skipped; BLE requires external confirmation.
    assert [3 if line.startswith("[MANUAL]") else 2 if line.startswith("[SKIP]") else int(line.startswith("[PASS]")) for line in rows] == expected
    assert f"TOTAL: PASS={expected.count(1)} FAIL={expected.count(0)} SKIP=1 MANUAL=1" in summary
    assert data["elapsed"] < 15000
    assert data["baud"] == 115200 and data["ble_reset"] == 0
    assert data["spi_cs_released"] and data["disabled"] == [40, 38]
    assert data["commands"] == [] and data["uart_reads"] == 0
    assert data["iic_reg"] in (None, 0xFE, 0xFF)


@pytest.mark.parametrize("baud", [19200, 115200])
def test_first_board_setup_reuses_working_sequence_without_requiring_readback(checker_with_init, baud):
    node, wasm = checker_with_init
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), json.dumps({"baud": baud})],
                            check=True, capture_output=True, text=True, timeout=15)
    data = json.loads(result.stdout)
    assert "[MANUAL] BLE HJ-131IMH: SETUP_UNVERIFIED" in data["output"]
    assert "<RD_BLE_MAC>" not in data["commands"] and data["uart_reads"] == 0
    assert data["ble_reset"] == 0 and data["module_baud"] == 115200
    commands = data["command_trace"]
    assert any(item == [19200, "<ST_FACTORY>"] for item in commands)
    project = [command for _, command in commands if command.startswith("<ST_") and command != "<ST_FACTORY>"]
    assert project == ["<ST_CLEAR_SECRET>", "<ST_WAKE=FOREVER>", "<ST_TX_POWER=+2.5>",
                       "<ST_NAME=HJ-131-LYX>", "<ST_CON_MIN_GAP=75>",
                       "<ST_OWN_MAC=784128c58150>", "<ST_BAUD=115200>"]
    assert data["elapsed"] < 20000


@pytest.mark.parametrize("fault", ["ignore_mac_write", "ignore_baud_write"])
def test_setup_commands_alone_cannot_report_configuration_verified(checker_with_init, fault):
    node, wasm = checker_with_init
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), json.dumps({fault: True})],
                            check=True, capture_output=True, text=True, timeout=15)
    text = json.loads(result.stdout)["output"]
    assert "[MANUAL] BLE HJ-131IMH: SETUP_UNVERIFIED" in text
    assert "[PASS] BLE" not in text and "[FAIL] BLE" not in text


def test_unconnected_ppg_can_be_skipped_in_firmware(checker_no_ppg):
    node, wasm = checker_no_ppg
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), "{}"],
                            check=True, capture_output=True, text=True, timeout=15)
    data = json.loads(result.stdout)
    assert "[SKIP] PPG2 MAX30101" in data["output"]
    assert data["iic_reg"] is None
    assert "PASS=2 FAIL=0 SKIP=2 MANUAL=1" in data["output"]


@pytest.mark.parametrize("ids,passed", [([255, 255, 255], False), ([104, 255, 104], False), ([104, 104, 104], True)])
def test_imu_identity_repeated_reads_expose_absent_and_intermittent_response(checker_wasm, ids, passed):
    node, wasm = checker_wasm
    result = subprocess.run([node, str(ROOT / "tests/board_check_hal_sim.cjs"), str(wasm), json.dumps({"ag_ids": ids})],
                            check=True, capture_output=True, text=True, timeout=15)
    text = json.loads(result.stdout)["output"]
    assert f"[{'PASS' if passed else 'FAIL'}] MIMU ACC/GYRO" in text
    assert "WHO_AM_I=" + "/".join(f"0x{value:02X}" for value in ids) in text


def test_check_initialization_choice_survives_settings_restore(monkeypatch):
    app = QApplication.instance() or QApplication([])
    expected = FirmwareSettings(board_check=1, ble_init=1, channel=0, mimu_recovery=0)
    monkeypatch.setattr(FirmwareSettings, "load", classmethod(lambda cls: expected))
    panel = FirmwarePanel(lambda: True)
    try:
        assert panel.settings() == expected
    finally:
        panel.close()
