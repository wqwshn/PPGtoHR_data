# Raw Transport Quality Design

## Goal

Add device-side sequence accounting to the 100Hz Raw data link so the PC can evaluate real transport quality, not only packet checksum validity.

## Selected Approach

Extend the Raw packet from 33 bytes to 35 bytes and append a big-endian `uint16 sequence` after the three PPG channels. Keep all existing ADC, ACC, GYRO, and PPG fields unchanged. Move XOR to byte 33 and footer to byte 34.

## Packet Layout

| Offset | Field | Type |
| --- | --- | --- |
| 0-1 | Header | `0xAA 0xBB` |
| 2-9 | Bridge ADC values | Existing format |
| 10-15 | ACC X/Y/Z | `int16` big-endian |
| 16-21 | GYRO X/Y/Z | `int16` big-endian |
| 22-30 | PPG Green/Red/IR | 3 bytes each |
| 31-32 | Sequence | `uint16` big-endian |
| 33 | XOR | XOR of bytes 2..32 |
| 34 | Footer | `0xCC` |

## Quality Metrics

- `rx_count`: valid packets parsed by the PC.
- `invalid_count`: frames with correct length/type but failed parser validation.
- `expected_count`: sequence span from first received sequence to latest received sequence, modulo `uint16`.
- `missing_count`: sum of sequence gaps.
- `loss_rate`: `missing_count / expected_count`.
- `rx_hz`: packets received per UI sample-rate window.
- `device_hz`: sequence advance per UI sample-rate window.

## Firmware Semantics

The firmware sequence counter represents generated Raw sample periods. It increments once per 100Hz Raw sample cycle, even if `HAL_UART_Transmit_DMA` returns busy/error and the frame is not sent. This makes sequence gaps visible on the PC side.

## Compatibility

The 35-byte Raw protocol requires firmware and PC software to be upgraded together. HR result packets remain unchanged. IMU scaling and gyro precision are not changed.
