"""RF event wire format: AA EE, 23 payload bytes, XOR, 55 (27 bytes)."""
from dataclasses import dataclass

RF_PACKET_LEN = 27
RF_HEADER = 0xEE
POWER_DIAG_HEADER = 0xEF
POWER_DIAG_LEN = 53
EVENTS = ('SNAPSHOT', 'BOOT', 'LINK_HIGH', 'LINK_LOW', 'NORMAL_START',
          'NORMAL_ABORT', 'RESET_ASSERT', 'RESET_RELEASE', 'COMPLETE',
          'WAIT_WARNING', 'RESET_WARNING', 'POWER_BOOT', 'POWER_SET', 'POWER_QUERY',
          'POWER_VERIFIED', 'POWER_START', 'POWER_END', 'POWER_FAILED',
          'POWER_COMPLETE', 'POWER_RECOVERED', 'POWER_LINK_HIGH', 'POWER_LINK_LOW',
          'POWER_WAIT_WARNING', 'RX_DIAGNOSTIC', 'POWER_READ_OK',
          'POWER_WRITE_DONE', 'POWER_UNVERIFIED_COMPLETE')
PHASES = ('PREPARE', 'WAIT_LINK', 'NORMAL', 'RESET_HELD', 'DONE',
          'POWER_PREPARE', 'POWER_SET', 'POWER_QUERY', 'POWER_SETTLE',
          'POWER_MEASURE', 'POWER_DONE', 'POWER_ERROR', 'POWER_WAIT_LINK',
          'READ_DONE', 'READ_ERROR', 'READ_PREPARE', 'READ_QUERY',
          'WRITE_WAIT_TX', 'WRITE_SETTLE', 'WRITE_MEASURE', 'WRITE_DONE', 'WRITE_ERROR')
POWER_STAGES = (2.5, 0, 2.5, -5, 2.5, -10, 2.5)

@dataclass(frozen=True)
class RFEvent:
    code: int
    cycle: int
    phase: int
    connected: int
    reset_command: int
    mcu_ms: int
    sample_counter: int
    event_id: int
    phase_start_ms: int
    queue_dropped: int
    diagnostic: dict | None = None

    @property
    def summary(self):
        elapsed = ((self.mcu_ms-self.phase_start_ms) & 0xffffffff)/1000
        if self.diagnostic is not None:
            d = self.diagnostic
            return (f"功率应答自检 · {d['DiagReason']} · RX={d['RxBytes']}字节 · "
                    f"接收错误={d['RxErrors']} · 指令发送完成={d['CommandTxDone']} · "
                    f"末尾应答={d['RxTailAscii'] or '(无)'}")
        if self.phase >= 17:
            phase = ('等待指令发送完成', '稳定等待', '测量中',
                     '七段结束 · 已发送 +2.5 dBm 恢复指令', '本地UART发送未完成，已停止')[self.phase-17]
            duration = {18:10, 19:60}.get(self.phase)
            remaining = f' · 剩余 {max(0,duration-elapsed):.1f}s' if duration else ''
            return (f'只写功率 · 第{self.cycle}/7段 · 目标 {POWER_STAGES[self.cycle-1]:g} dBm · '
                    f'{phase}{remaining} · 未读回验证 · 连接脚={self.connected} · {EVENTS[self.code]}')
        if self.phase >= 13:
            phase = ('查询成功 · 未修改功率', '查询未通过 · 请查看 RX_DIAGNOSTIC',
                     f'准备只读查询 · 剩余 {max(0,10-elapsed):.1f}s', '等待只读应答')[self.phase-13]
            return f'只读功率自检 · {phase} · {EVENTS[self.code]}'
        if self.phase >= 5:
            phase = ('准备双路录制', '设置功率', '等待功率读回', '稳定等待',
                     '功率测量', '完成 · 已回到 +2.5 dBm',
                     '实验失败 · 请查看事件日志确认恢复结果', '等待初次连接')[self.phase-5]
            duration = {5: 60, 8: 10, 9: 60}.get(self.phase)
            remaining = f' · 剩余 {max(0, duration-elapsed):.1f}s' if duration else ''
            return (f'功率实验 · 第 {self.cycle}/7 段 · 目标 {POWER_STAGES[self.cycle-1]:g} dBm · '
                    f'{phase}{remaining} · 连接脚={self.connected} · '
                    f'{EVENTS[self.code]} · 队列丢失={self.queue_dropped}')
        phase = ('准备录制', '等待实际连接', '正常采集', '保持复位', '三轮完成')[self.phase]
        duration = 60 if self.phase == 0 else 30 if self.phase in (2, 3) else None
        remaining = f' · 剩余 {max(0, duration-elapsed):.1f}s' if duration else ''
        return (f'RF 实验 · 第 {self.cycle}/3 轮 · {phase}{remaining} · '
                f'连接脚={self.connected} · 复位命令={self.reset_command} · '
                f'{EVENTS[self.code]} · 队列丢失={self.queue_dropped}')

def parse_rf_event(data: bytes):
    if len(data) != RF_PACKET_LEN or data[:2] != b'\xaa\xee' or data[-1] != 0x55:
        return None
    check = 0
    for byte in data[2:25]:
        check ^= byte
    if check != data[25] or data[2] >= len(EVENTS) or data[2] == 23 or data[4] >= len(PHASES):
        return None
    power = data[4] >= 5
    if data[3] not in (range(1, 8) if power else range(1, 4)) or data[5] not in (0, 1) or data[6] not in (0, 1):
        return None
    if power and (data[6] != 0 or 0 < data[2] < 11):
        return None
    if not power and data[2] >= 11:
        return None
    return RFEvent(*data[2:7], *(int.from_bytes(data[i:i+4], 'big') for i in (7,11,15,19)),
                   int.from_bytes(data[23:25], 'big'))

def parse_power_diagnostic(data: bytes):
    if len(data) != POWER_DIAG_LEN or data[:3] != b'\xaa\xef\x01' or data[-1] != 0x55:
        return None
    check = 0
    for byte in data[2:51]: check ^= byte
    if check != data[51] or data[3] not in range(1,8) or data[4] not in range(5,22):
        return None
    if data[5] not in (0,1) or data[6] not in range(4) or data[27] > 22 or data[50] not in (0,1):
        return None
    tail = data[28:28+data[27]]
    diagnostic = dict(DiagReason=('READ_OK', 'QUERY_TIMEOUT', 'UNEXPECTED_REPLY', 'RX_ERROR')[data[6]],
        RxBytes=int.from_bytes(data[19:23], 'big'), RxErrors=int.from_bytes(data[23:25], 'big'),
        CommandTxDone=int.from_bytes(data[25:27], 'big'), RxTailHex=tail.hex(' '),
        RxTailAscii=''.join(chr(b) if 32<=b<=126 else f'\\x{b:02x}' for b in tail),
        ReadOnly=data[50])
    mcu = int.from_bytes(data[7:11], 'big')
    return RFEvent(23, data[3], data[4], data[5], 0, mcu,
        int.from_bytes(data[11:15], 'big'), int.from_bytes(data[15:19], 'big'), mcu, 0, diagnostic)

RF_CSV_HEADER = ['PcRxTime(s)', 'RecordMarker', 'EventId', 'Event', 'Cycle', 'Phase',
                 'McuTime(ms)', 'SampleCounter', 'PhaseStart(ms)', 'ConnectedPin',
                 'ResetCommand', 'QueueDropped', 'LastRecordedSampleIndex',
                 'Experiment', 'TargetPower(dBm)', 'PowerVerified']
DIAG_FIELDS = ['DiagReason', 'RxBytes', 'RxErrors', 'CommandTxDone', 'RxTailHex', 'RxTailAscii', 'ReadOnly']
RF_CSV_HEADER += DIAG_FIELDS

def event_row(event, rx_seconds, sample_index, marker='live'):
    return [round(rx_seconds, 6), marker, event.event_id, EVENTS[event.code], event.cycle,
            PHASES[event.phase], event.mcu_ms, event.sample_counter, event.phase_start_ms,
            event.connected, event.reset_command, event.queue_dropped, sample_index,
            'power_writeonly' if event.phase >= 17 else 'power_readonly' if event.phase >= 13 else 'power' if event.phase >= 5 else 'reset',
            POWER_STAGES[event.cycle-1] if 5 <= event.phase < 13 or event.phase >= 17 else '',
            0 if event.phase >= 17 else int(event.phase in (8, 9, 10) or event.code in (14, 19)) if 5 <= event.phase < 13 and not event.diagnostic else ''] + [
                (event.diagnostic or {}).get(key, '') for key in DIAG_FIELDS]
