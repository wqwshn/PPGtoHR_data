// Execute the production communication checker with a deterministic HAL.
const fs = require('fs');
const scenario = JSON.parse(process.argv[3]);
let instance, tick = 0, baud = 115200, output = '', rx = [], disabled = [], commands = [];
let iicReg = null, iicBytes = [], ackIndex = 0;
let moduleBaud = scenario.baud ?? 19200, moduleMac = '123456789abc', commandTrace = [], factoryAt = null;
let imuRead = 0, uartReads = 0;
const pins = new Map();
const memory = () => new Uint8Array(instance.exports.memory.buffer);
const view = () => new DataView(instance.exports.memory.buffer);
const read = p => { let end = p; while (memory()[end]) end++; return Buffer.from(memory().slice(p, end)).toString(); };
const imports = {
    Mock_Clear() {},
    HAL_NVIC_DisableIRQ(irq) { disabled.push(irq); },
    HAL_GPIO_WritePin(port, mask, state) {
        for (let bit = 1; bit <= 32768; bit *= 2) if (mask & bit) pins.set(`${port}:${bit}`, state);
    },
    HAL_GPIO_ReadPin() { return scenario.bus_low ? 0 : 1; },
    HAL_Delay(ms) {
        tick += ms;
        if (factoryAt !== null && tick >= factoryAt) { moduleBaud = 19200; factoryAt = null; }
    },
    HAL_GetTick() { return tick++; },
    HAL_SPI_TransmitReceive(spi, tx, dest, length, timeout) {
        if (timeout > 50) throw new Error('Unbounded SPI check');
        const adc = view().getUint32(spi, true) === 1;
        if (adc && (memory()[tx] !== 0x20 || length !== 3 || pins.get('1:16') !== 0)) throw new Error('ADC transaction');
        if (!adc && (memory()[tx] !== 0x8F || length !== 2)) throw new Error('IMU transaction');
        const id = adc ? (scenario.adc ?? 1) : pins.get('2:128') === 0 ?
            (scenario.ag_ids ? scenario.ag_ids[imuRead++] : 0x68) : 0x3D;
        memory()[dest + length - 1] = id;
        if (scenario.spi_error) { tick += timeout; return 3; }
        return 0;
    },
    HAL_UART_Init(uart) { baud = view().getUint32(uart, true); return 0; },
    HAL_UART_DeInit() { rx = []; return 0; },
    Error_Handler() { throw new Error('Firmware Error_Handler invoked'); },
    HAL_UART_Transmit(uart, source, length, timeout) {
        if (timeout > 1000) throw new Error('Unbounded UART TX');
        const bytes = memory().slice(source, source + length);
        const text = Buffer.from(bytes).toString();
        if (bytes[0] === 0xAA) return 0;
        if (text.startsWith('<ST_')) {
            commandTrace.push([baud, text]);
            if (baud === moduleBaud) {
                if (text === '<ST_FACTORY>') { factoryAt = tick + 500; moduleMac = '123456789abc'; }
                if (text.startsWith('<ST_OWN_MAC=') && !scenario.ignore_mac_write) {
                    moduleMac = text.slice(12, -1);
                    moduleBaud = 19200; // MAC update restarts the module before the final baud command.
                }
                if (text === '<ST_BAUD=115200>' && !scenario.ignore_baud_write) moduleBaud = 115200;
            }
            return 0;
        }
        if (text === '<RD_BLE_MAC>') {
            commands.push(text);
            if (pins.get('2:8192') !== 0) throw new Error('BLE queried in reset');
            let reply = '';
            if (baud === moduleBaud) {
                reply = ({none: '', echo: text, partial: '<rd_ble_mac=123456',
                    invalid: '<rd_ble_mac=123456789abZ>', noise_then_valid: 'junk<rd_name=bad><rd_ble_mac=123456789abc>'})[scenario.ble];
                if (reply === undefined) reply = `<rd_ble_mac=${moduleMac}>`;
            }
            rx.push(...Buffer.from(reply));
        } else {
            if (baud !== 115200) throw new Error('Log baud changed');
            output += text;
        }
        return 0;
    },
    HAL_UART_Receive(uart, dest, length, timeout) {
        ++uartReads;
        if (timeout > 10 || length !== 1) throw new Error('Unbounded UART RX');
        tick += timeout;
        if (scenario.ble === 'uart_error') return 1;
        if (!rx.length) return 3;
        memory()[dest] = rx.shift(); return 0;
    },
    IIC2_Start() { iicBytes = []; },
    IIC2_Stop() {},
    IIC2_Send_Byte(byte) { iicBytes.push(byte); if (iicBytes.length === 2) iicReg = byte; },
    IIC2_Wait_Ack() { return ++ackIndex === scenario.ack_fail ? 1 : 0; },
    IIC2_Read_Byte(ack) {
        if (ack !== 0 || iicBytes[0] !== 0xAF) throw new Error('PPG transaction');
        return iicReg === 0xFF ? (scenario.part ?? 0x15) : 0xFF; // Revision is not a fixed identity.
    },
    strlen(p) { return Buffer.byteLength(read(p)); },
    strncmp(a, b, n) {
        for (let i = 0; i < n; ++i) {
            const x = memory()[a + i], y = memory()[b + i];
            if (x !== y) return x - y;
            if (!x) return 0;
        }
        return 0;
    },
    snprintf(dest, capacity, format, args) {
        let offset = args;
        const text = read(format).replace(/%(?:02)?l?[usX]/g, spec => {
            const value = view().getUint32(offset, true); offset += 4;
            if (spec === '%s') return read(value);
            if (spec === '%02X') return value.toString(16).toUpperCase().padStart(2, '0');
            return String(value);
        });
        const bytes = Buffer.from(text);
        memory().set(bytes.subarray(0, capacity - 1), dest);
        memory()[dest + Math.min(bytes.length, capacity - 1)] = 0;
        return bytes.length;
    },
};
(async () => {
    ({instance} = await WebAssembly.instantiate(fs.readFileSync(process.argv[2]), {env: imports}));
    instance.exports.Test_Run();
    instance.exports.BoardCheck_PrintSummary();
    process.stdout.write(JSON.stringify({output, elapsed: tick, baud, disabled, commands, uart_reads: uartReads, module_baud: moduleBaud, command_trace: commandTrace,
        ble_reset: pins.get('2:8192'), iic_reg: iicReg,
        spi_cs_released: [pins.get('1:16'), pins.get('2:128'), pins.get('2:256')].every(x => x === 1)}));
})().catch(error => { console.error(error); process.exit(1); });
