"""Execute production PPG probes, stall logging and Raw packet gate with mocked I/O.

This cannot reproduce electrical faults or ISR lockups on the physical board.
"""
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def acquisition_sim(tmp_path_factory):
    return compile_acquisition(tmp_path_factory.mktemp("acquisition-debug"))


@pytest.fixture(scope="module")
def quiet_acquisition_sim(tmp_path_factory):
    return compile_acquisition(tmp_path_factory.mktemp("acquisition-quiet"), debug=0)


def compile_acquisition(folder, debug=1):
    clang, node = shutil.which("clang"), shutil.which("node")
    if not clang or not node:
        pytest.skip("Clang wasm32 and Node are required")
    main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    diagnostics = main.split("/* 新板取证：", 1)[1].split("// PPG 血氧模式硬件配置", 1)[0]
    diagnostics = diagnostics.split("*/", 1)[1]
    packet = main.split("    /* ---- 数据采集: ADC 完成后触发 ---- */", 1)[1].split("    /* USER CODE END WHILE */", 1)[0]
    registers = (ROOT / "Core/Inc/MAX30101.h").read_text(encoding="utf-8")
    defines = "\n".join(re.findall(r"^#define\s+\w+\s+0[xX][0-9A-Fa-f]+", registers, re.M))
    source = r'''
#include <stdint.h>
#define PPG_DEFAULT_CHANNEL 2
#define PPG_CH2 2
#define CURRENT_WORK_MODE 0
#define MODE_SPO2 1
#define ENABLE_RAW_DATA_PACKET 1
#define HAL_OK 0
#define HAL_BUSY 2
#define HAL_UART_STATE_READY 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define PACKET_LEN 35
#define XOR_CHECK_LEN 31
#define GYRO_START_INDEX 16
#define PPG_START_INDEX 22
#define RAW_SEQUENCE_START_INDEX 31
#define PPG_AVG_FRAC_BITS 4
typedef struct {uint32_t ODR, IDR, MODER, OTYPER;} GPIO_TypeDef;
static GPIO_TypeDef port1={3,3,5,3}, port2={3,3,5,3};
typedef int HAL_StatusTypeDef;
struct { unsigned gState, ErrorCode; } huart2;
int hspi1;
#define I2C_SCL1_GPIO_Port (&port1)
#define I2C_SCL2_GPIO_Port (&port2)
#define I2C_SDA2_GPIO_Port (&port2)
#define I2C_SCL1_Pin 1
#define I2C_SCL2_Pin 1
#define I2C_SDA1_Pin 2
#define I2C_SDA2_Pin 2
#define SPI1_NSS_GPIO_Port (&port1)
#define SPI1_NSS_Pin 16
#define DRDY_GPIO_Port (&port1)
#define DRDY_Pin 2048
#define GPIOA (&port1)
#define GPIO_PIN_6 64
#define AD_RESET_GPIO_Port (&port1)
#define AD_RESET_Pin 4096
#define START_CONV_GPIO_Port (&port1)
#define START_CONV_Pin 1024
#define V5_0_CE_GPIO_Port (&port1)
#define V5_0_CE_Pin 256
#define V1_8_CE_GPIO_Port (&port1)
#define V1_8_CE_Pin 512
#define ADC_ID_REG 0
#define ADC_STATUS_REG 1
#define ADC_MUX_REG 2
#define ADC_GSET_REG 3
#define ADC_RATE_REG 4
#define ADC_RCON_REG 5
#define ADC_SCON_REG 9
int snprintf(char *, __SIZE_TYPE__, const char *, ...);
__SIZE_TYPE__ strlen(const char *);
int HAL_UART_Transmit(void *, uint8_t *, uint16_t, uint32_t);
uint32_t raw_diag_sample_counter=200, raw_diag_adc_drdy_counter;
uint32_t raw_diag_frame_counter, raw_diag_tx_start_counter, raw_diag_tx_done_counter;
uint32_t raw_diag_tx_busy_counter, raw_diag_tx_error_counter, raw_diag_ppg_fifo_empty_counter;
uint32_t raw_diag_ppg_fifo_sample_total_counter, raw_diag_ppg_fifo_nonempty_counter;
uint32_t raw_diag_ppg_fifo_single_sample_counter, raw_diag_ppg_fifo_multi_sample_counter;
uint32_t raw_diag_ppg_fifo_overflow_counter, raw_diag_dma_tx_kind;
uint8_t ADC_1to4Voltage_flag, allData[50], ACC_XYZ[6], GYRO_XYZ[6];
int16_t gyro_offset[3];
uint16_t raw_packet_seq;
static unsigned tick, channel, fail_ack, ack_index, bus_low, bad_reg, selected_reg, tx_count, transactions;
static unsigned adc_case, driver_probe, external_scl_low;
uint32_t HAL_GetTick(void) {return tick;}
unsigned PPG_GetChannel(void) {return channel;}
int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    if(driver_probe && port==&port2) {
        if(pin==I2C_SCL2_Pin) return !external_scl_low && (port->ODR & pin)!=0;
        return 1; /* 未接器件：SDA始终高，地址不应答。 */
    }
    return !bus_low;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int value) {
    if(value)port->ODR|=pin;else port->ODR&=~pin;
}
void HAL_Delay(unsigned ms) {tick+=ms;}
int HAL_SPI_TransmitReceive(void *spi, uint8_t *tx, uint8_t *rx, unsigned n, unsigned timeout) {
    const uint8_t values[]={0x09,0x00,0x3C,0x00,0x2C,0x3A,0,0,0,0x10};
    if(adc_case==0)return 0;
    if(adc_case==3)return 1;
    rx[adc_case==2 ? 1 : 2]=values[tx[0]&0x0F];
    return 0;
}
void IIC_Start(void) {transactions |= 1;}
void IIC2_Start(void) {transactions |= 2;}
void IIC_Stop(void) {}
void IIC2_Stop(void) {}
void IIC_Send_Byte(uint8_t b) {if(b!=0xAE && b!=0xAF) selected_reg=b;}
void IIC2_Send_Byte(uint8_t b) {IIC_Send_Byte(b);}
uint8_t IIC_Wait_Ack(void) {return ++ack_index==fail_ack;}
uint8_t IIC2_Wait_Ack(void) {return IIC_Wait_Ack();}
uint8_t IIC_Read_Byte(uint8_t ack) {
    if(selected_reg==bad_reg) return 0;
    switch(selected_reg) {
        case 0xFF:return 0x15; case 0xFE:return 0x42;
        case 0x09:return 0x07; case 0x0A:return 0x6B; case 0x08:return 0x3F;
        case 0x0C:case 0x0D:return 0xC0; case 0x0E:return 0xE0;
        case 0x11:return 0x13; case 0x12:return 0x02; default:return 0;
    }
}
uint8_t IIC2_Read_Byte(uint8_t ack) {return IIC_Read_Byte(ack);}
uint8_t PPG_ReadOneByte(unsigned reg) {return 255;}
void PPG_ReadFIFO_Burst(uint8_t *buf, unsigned n) {while(n--) *buf++=0;}
uint8_t CheckXOR(uint8_t *buf, uint8_t n) {return 0;}
int HAL_UART_Transmit_DMA(void *uart, uint8_t *data, unsigned n) {tx_count++; return 0;}
'''
    source += '\n#include "sample_rate_config.h"\n' + defines + "\n" + diagnostics
    source += r'''
unsigned Test_PPG(unsigned ch, unsigned fail, unsigned low, unsigned bad) {
    channel=ch; fail_ack=fail; ack_index=0; bus_low=low; bad_reg=bad; transactions=0;
    return Acquisition_DebugPPG(1);
}
unsigned Test_Bus(void) {return transactions;}
void Test_ADC(unsigned scenario) {adc_case=scenario; Acquisition_DebugADC("post-init");}
void Test_Poll(unsigned ms, unsigned frames, unsigned busy, unsigned done) {
    tick=ms; raw_diag_frame_counter=frames; huart2.gState=busy; raw_diag_tx_done_counter=done;
    Acquisition_DebugPoll();
}
unsigned Test_Packet(unsigned state) {
    tx_count=0; ADC_1to4Voltage_flag=state;
'''
    source += packet + "\nreturn tx_count;\n}\n"
    # 真实软件IIC与MAX2写路径：证明NACK分支最终是否释放SCL。
    iic = (ROOT / "Core/Src/MyIIC_2.c").read_text(encoding="utf-8", errors="replace")
    iic = re.sub(r'^#include[^\n]*', '', iic, flags=re.M)
    iic = iic.replace("IIC2_", "Probe_").replace("Delay2_us", "Probe_Delay")
    source += "\nvoid Probe_Stop(void);\n" + iic
    driver = (ROOT / "Core/Src/MAX30101_2.c").read_text(encoding="utf-8")
    write = re.search(r"void MAX2_WriteOneByte\([^}]+\}", driver).group()
    source += "\n" + write.replace("IIC2_", "Probe_")
    source += r'''
unsigned Test_Idle(unsigned held) {
    driver_probe=1; external_scl_low=held; port2.ODR=3;
    MAX2_WriteOneByte(0x06,0);
    unsigned result=((port2.ODR&1)!=0)*2+HAL_GPIO_ReadPin(&port2,1);
    driver_probe=0;
    return result;
}
void Test_Live(unsigned ms, unsigned failed, unsigned low, unsigned busy, unsigned seen) {
    channel=2; fail_ack=failed; ack_index=0; bus_low=low; huart2.gState=busy;
    raw_diag_ppg_fifo_nonempty_counter=seen;
    Acquisition_DebugPPGLive(ms);
}
'''
    path, wasm = folder / "probe.c", folder / "probe.wasm"
    path.write_text(source, encoding="utf-8")
    subprocess.run([clang, "--target=wasm32", "-nostdlib", "-O0", str(path),
                    f"-I{ROOT / 'Core/Inc'}", f"-DENABLE_ACQUISITION_DEBUG={debug}", "-Wl,--no-entry", "-Wl,--allow-undefined",
                    "-Wl,--export-all", "-o", str(wasm)], check=True, capture_output=True)
    runner = folder / "run.cjs"
    runner.write_text(r'''
const fs = require('fs');
let instance, output='';
const memory=()=>new Uint8Array(instance.exports.memory.buffer);
const read=p=>{let end=p; while(memory()[end]) end++; return Buffer.from(memory().slice(p,end)).toString();};
const env={
 strlen(p){return Buffer.byteLength(read(p));},
 HAL_UART_Transmit(u,p,n){output+=Buffer.from(memory().slice(p,p+n)).toString();return 0;},
 snprintf(dest,capacity,format,args){
  let offset=args;
  const text=read(format).replace(/%(?:0[248])?l?[usX]/g,spec=>{
   const value=new DataView(memory().buffer).getUint32(offset,true);offset+=4;
   if(spec==='%s')return read(value);
   if(spec.endsWith('X'))return value.toString(16).toUpperCase().padStart(Number(spec.match(/0([248])/)?.[1]??0),'0');
   return String(value);
  });
  const bytes=Buffer.from(text);memory().set(bytes.subarray(0,capacity-1),dest);
  memory()[dest+Math.min(bytes.length,capacity-1)]=0;return bytes.length;
 }
};
(async()=>{
 ({instance}=await WebAssembly.instantiate(fs.readFileSync(process.argv[2]),{env}));
 const results=JSON.parse(process.argv[3]).map(([name,...args])=>instance.exports[name](...args));
 process.stdout.write(JSON.stringify({results,output}));
})().catch(e=>{console.error(e);process.exit(1);});
''', encoding="utf-8")

    def run(*calls):
        result = subprocess.run([node, str(runner), str(wasm), json.dumps(calls)],
                                check=True, capture_output=True, text=True, timeout=10)
        return json.loads(result.stdout)
    return run


@pytest.mark.parametrize("channel", [1, 2])
@pytest.mark.parametrize("fail,low,stage", [(0, 1, 1), (1, 0, 2), (2, 0, 3), (3, 0, 4)])
def test_ppg_failure_reports_bus_or_ack_stage_without_false_success(acquisition_sim, channel, fail, low, stage):
    data = acquisition_sim(["Test_PPG", channel, fail, low, 256], ["Test_Bus"])
    assert data["results"] == [0, 0 if low else channel]
    assert f"FAIL stage={stage}" in data["output"]
    assert "value=NA" in data["output"]
    assert "VERIFIED" not in data["output"]


@pytest.mark.parametrize("bad", [256, 0xFF, 0x09, 0x0E, 0x11])
def test_ppg_config_requires_identity_and_register_readback(acquisition_sim, bad):
    data = acquisition_sim(["Test_PPG", 2, 0, 0, bad])
    assert data["results"] == [int(bad == 256)]
    assert ("config VERIFIED" in data["output"]) == (bad == 256)


def test_no_drdy_reports_stall_but_active_frames_or_dma_stay_quiet(acquisition_sim):
    data = acquisition_sim(["Test_Poll", 0, 0, 0, 0], ["Test_Poll", 1999, 0, 0, 0],
                           ["Test_Poll", 2000, 0, 0, 0], ["Test_Poll", 4000, 100, 0, 100],
                           ["Test_Poll", 6000, 100, 1, 100], ["Test_Poll", 8000, 100, 0, 100],
                           ["Test_Poll", 10000, 200, 0, 100])
    assert data["output"].count("NO_FRAME") == 2
    assert data["output"].count("NO_TX_DONE") == 1
    assert "tick=200 drdy=0 adc_state=0" in data["output"]


def test_adc_completion_gates_raw_sending_even_if_ppg_reads_ff(acquisition_sim):
    data = acquisition_sim(*[["Test_Packet", state] for state in range(6)])
    assert data["results"] == [0, 0, 0, 0, 1, 0]


@pytest.mark.parametrize("case,healthy", [(0, False), (1, True), (2, False), (3, False)])
def test_adc_all_zero_shifted_and_spi_error_are_not_identity_or_config_success(acquisition_sim, case, healthy):
    data = acquisition_sim(["Test_ADC", case])
    assert ("identity=PASS config=PASS" in data["output"]) == healthy
    assert "transport_ok=" in data["output"]
    if case == 2:
        assert "RX=00/09/00 value=0x00" in data["output"]


def test_real_ppg_write_releases_scl_after_nack_and_distinguishes_external_low(acquisition_sim):
    data = acquisition_sim(["Test_Idle", 0], ["Test_Idle", 1])
    assert data["results"] == [3, 2]  # ODR=1 always; sampled SCL depends on external line.


def test_active_adc_with_no_ppg_samples_reports_live_ppg_state_for_late_bluetooth(acquisition_sim):
    data = acquisition_sim(["Test_Poll", 0, 0, 0, 0], ["Test_Poll", 10000, 1000, 0, 1000])
    assert "[DEBUG-ppg-live]" in data["output"]
    assert "FF=15" in data["output"]
    assert "09=07" in data["output"]
    assert "nonempty=0" in data["output"]


def test_main_poll_retries_pending_ppg_report_as_soon_as_uart_becomes_idle(acquisition_sim):
    data = acquisition_sim(["Test_Poll", 0, 0, 0, 0],
                           ["Test_Poll", 10000, 1000, 1, 1000],
                           ["Test_Poll", 10001, 1000, 0, 1000])
    assert data["output"].count("[DEBUG-ppg-live]") == 1


@pytest.mark.parametrize("failed,low,stage", [(1, 0, 2), (0, 1, 1)])
def test_live_ppg_probe_keeps_nack_and_bus_low_distinct_without_fake_id(acquisition_sim, failed, low, stage):
    data = acquisition_sim(["Test_Live", 10000, failed, low, 0, 0])
    assert f"FAIL reg=0xFF stage={stage} value=NA" in data["output"]
    assert "FF=15" not in data["output"]


def test_live_probe_is_rate_limited_defers_for_uart_and_stops_after_ppg_samples(acquisition_sim):
    data = acquisition_sim(["Test_Live", 10000, 0, 0, 1, 0],
                           ["Test_Live", 10001, 0, 0, 0, 0],
                           ["Test_Live", 11000, 0, 0, 0, 0],
                           ["Test_Live", 20001, 0, 0, 0, 1])
    assert data["output"].count("[DEBUG-ppg-live]") == 1


def test_quiet_runtime_still_checks_identity_ack_and_configuration(quiet_acquisition_sim):
    for bad in (256, 0xFF, 0x09, 0x0A):
        data = quiet_acquisition_sim(["Test_PPG", 2, 0, 0, bad])
        assert data["results"] == [int(bad == 256)]
        assert "GPIO" not in data["output"]
        if bad != 256:
            assert "MISMATCH" in data["output"]
    data = quiet_acquisition_sim(["Test_PPG", 2, 1, 0, 256])
    assert data["results"] == [0]
    assert "FAIL stage=2" in data["output"]


def test_quiet_runtime_disables_periodic_diagnostics_but_keeps_adc_verification(quiet_acquisition_sim):
    data = quiet_acquisition_sim(["Test_Poll", 0, 0, 0, 0], ["Test_Poll", 20000, 0, 0, 0],
                                ["Test_Live", 30000, 0, 0, 0, 0], ["Test_ADC", 0])
    assert "DEBUG-ppg-live" not in data["output"] and "NO_FRAME" not in data["output"]
    assert "identity=FAIL config=FAIL" in data["output"]
    assert "RX=" not in data["output"]
