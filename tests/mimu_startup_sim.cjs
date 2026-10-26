const fs = require('fs');
const scenario = JSON.parse(process.argv[3]);
let instance, output = '', calibrated = 0, loaded = 0;
let tick = 0, identityReads = 0, configurations = 0, dataReads = 0, inDelay = false, irqReadsDuringDelay = 0;
let irqCsWritesDuringDelay = 0;
const readyHistory = [];
const stateHistory = [];
const registers = new Map();
const memory = () => new Uint8Array(instance.exports.memory.buffer);
const read = p => { let end=p; while(memory()[end]) end++; return Buffer.from(memory().slice(p,end)).toString(); };
const imports = {
    strlen(p) { return Buffer.byteLength(read(p)); },
    snprintf(dest, capacity, format, args) {
        let offset=args;
        const line=read(format).replace(/%u/g, () => {
            const value=new DataView(memory().buffer).getUint32(offset,true); offset+=4; return String(value);
        });
        const bytes=Buffer.from(line); memory().set(bytes.subarray(0,capacity-1),dest);
        memory()[dest+Math.min(bytes.length,capacity-1)]=0; return bytes.length;
    },
    ACC_GYRO_Write(reg, value) { registers.set(reg, value); if(reg === 0x22 && value === 5) configurations++; },
    MAG_Write() {},
    ACC_GYRO_Read(reg) {
        if (reg === 0x0F) {
            identityReads++;
            return tick < (scenario.ready_at_ms ?? 0) ? 255 : scenario.identity ?? 0x68;
        }
        if (scenario.bad_config && reg === 0x20) return 0;
        return registers.get(reg) ?? 0;
    },
    ACC_GYRO_ReadChecked(reg, ptr) {
        new Uint8Array(instance.exports.memory.buffer)[ptr] = imports.ACC_GYRO_Read(reg);
        return scenario.spi_error ? 3 : 0;
    },
    HAL_Delay(ms) {
        tick += ms;
        if (scenario.interrupt_during_delay) {
            inDelay = true;
            instance.exports.Test_DataISR();
            instance.exports.Test_TimerISR();
            inDelay = false;
        }
    },
    HAL_GetTick() { return tick; },
    HAL_GPIO_WritePin(port, pin) {if(inDelay && port===2 && pin===128) irqCsWritesDuringDelay++;},
    ADC_RDATA() {},
    ADC_WREG() {},
    ACC_6BytesRead() { dataReads++; if(inDelay) irqReadsDuringDelay++; },
    GYRO_6BytesRead() { dataReads++; if(inDelay) irqReadsDuringDelay++; },
    HAL_UART_Transmit(uart, ptr, size) {
        output += Buffer.from(new Uint8Array(instance.exports.memory.buffer, ptr, size)).toString();
        return 0;
    },
    MIMU_LoadGyroOffset() { loaded++; },
    MIMU_GyroCalibrate() { calibrated++; },
};
(async () => {
    ({instance} = await WebAssembly.instantiate(fs.readFileSync(process.argv[2]), {env: imports}));
    instance.exports.Test_Start();
    if (scenario.read_after_start) instance.exports.Test_DataISR();
    if (scenario.uart_busy) instance.exports.Test_UartBusy();
    for (const ms of scenario.poll_at_ms ?? []) {
        tick = ms;
        instance.exports.Test_Poll();
        readyHistory.push(instance.exports.Test_Ready());
        stateHistory.push(instance.exports.Test_State());
    }
    if (scenario.read_after_polls) instance.exports.Test_DataISR();
    instance.exports.Test_BuildStatus();
    const status=Buffer.from(Array.from({length:73}, (_,index) => instance.exports.Test_StatusByte(index))).toString('hex');
    process.stdout.write(JSON.stringify({output, calibrated, loaded, ready: instance.exports.Test_Ready(),
        identity_reads: identityReads, configurations, data_reads: dataReads, irq_reads_during_delay: irqReadsDuringDelay,
        irq_cs_writes_during_delay: irqCsWritesDuringDelay,
        ready_history: readyHistory, state_history: stateHistory, state: instance.exports.Test_State(), status, elapsed: tick}));
})().catch(error => { console.error(error); process.exit(1); });
